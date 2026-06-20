#!/usr/bin/env python3
"""
xmflash -- remote CLI for the XMOS JTAG web flasher.

Drives the HTTP API served by the example firmware (example/main/main.c) over
the network, so the whole identify / scan / load / flash flow is scriptable
without the browser UI.

  xmflash identify                         # uses the default mDNS host
  xmflash --host 192.168.1.23 scan
  xmflash --host flasher.local load-ram firmware.xe
  xmflash write-flash image.bin
  xmflash load-cram bitstream.bin          # iCE40
  xmflash svf program.svf

Host resolution
  The --host value may be an IP, a DNS hostname, or an mDNS ".local" name; it
  is resolved through the operating system's resolver (getaddrinfo), so mDNS
  works wherever the OS supports it (natively on macOS, via Avahi/nss-mdns on
  Linux).  The firmware advertises itself as "xmflash.local", which is the
  built-in default when no host is given.

Configuration precedence (highest first)
  1. command-line flag        (--host / --timeout / --poll)
  2. environment variable     (XMFLASH_HOST / XMFLASH_TIMEOUT / XMFLASH_POLL)
  3. config file              ([xmflash] section; see `xmflash config`)
  4. built-in default

No third-party dependencies -- standard library only.
"""

import argparse
import configparser
import json
import os
import socket
import sys
import time
import urllib.error
import urllib.request

DEFAULT_HOST = "xmflash.local"        # matches the firmware's mDNS hostname
DEFAULT_TIMEOUT = 30                  # seconds, per simple request
DEFAULT_POLL = 0.5                    # seconds, status poll interval
UPLOAD_TIMEOUT = 300                 # seconds, large firmware over WiFi
OP_TIMEOUT = 900                     # seconds, flash erase+program can be slow


class FlasherError(Exception):
    pass


# --------------------------------------------------------------------------- #
# Configuration: config file + environment + CLI, in that order of increasing
# priority.
# --------------------------------------------------------------------------- #

def _config_search_paths(explicit):
    """Ordered candidate config locations; first existing one wins."""
    if explicit:
        return [explicit]
    paths = []
    env = os.environ.get("XMFLASH_CONFIG")
    if env:
        paths.append(env)
    paths.append(os.path.join(os.getcwd(), ".xmflashrc"))
    xdg = os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config")
    paths.append(os.path.join(xdg, "xmflash", "config"))
    paths.append(os.path.join(xdg, "xmflash.ini"))
    paths.append(os.path.expanduser("~/.xmflashrc"))
    return paths


def load_config(explicit):
    """Return (settings_dict, path_used).  Accepts an INI file with an
    [xmflash] section, or a bare key=value file (a section is synthesised)."""
    for path in _config_search_paths(explicit):
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path) as f:
                text = f.read()
        except OSError as e:
            raise FlasherError("cannot read config %s: %s" % (path, e))
        if not text.lstrip().startswith("["):
            text = "[xmflash]\n" + text
        cp = configparser.ConfigParser()
        try:
            cp.read_string(text)
        except configparser.Error as e:
            raise FlasherError("invalid config %s: %s" % (path, e))
        sect = cp["xmflash"] if cp.has_section("xmflash") else cp[cp.default_section]
        out = {k: sect[k] for k in ("host", "timeout", "poll") if k in sect}
        return out, path
    return {}, None


def resolve_settings(args):
    """Apply precedence CLI > env > config > default. Returns a dict with
    host/timeout/poll plus diagnostic info about where each came from."""
    cfg, cfg_path = load_config(getattr(args, "config", None))

    def pick(cli_val, env_name, cfg_key):
        if cli_val is not None:
            return cli_val, "cli"
        if os.environ.get(env_name):
            return os.environ[env_name], "env (%s)" % env_name
        if cfg_key in cfg:
            return cfg[cfg_key], "config (%s)" % cfg_path
        return None, None

    host, host_src = pick(args.host, "XMFLASH_HOST", "host")
    if host is None:
        host, host_src = DEFAULT_HOST, "default"
    timeout, t_src = pick(args.timeout, "XMFLASH_TIMEOUT", "timeout")
    poll, p_src = pick(args.poll, "XMFLASH_POLL", "poll")

    try:
        timeout = float(timeout) if timeout is not None else DEFAULT_TIMEOUT
        poll = float(poll) if poll is not None else DEFAULT_POLL
    except ValueError as e:
        raise FlasherError("bad numeric setting: %s" % e)

    return {
        "host": host, "host_src": host_src or "default",
        "timeout": timeout, "timeout_src": t_src or "default",
        "poll": poll, "poll_src": p_src or "default",
        "config_path": cfg_path,
    }


# --------------------------------------------------------------------------- #
# HTTP client
# --------------------------------------------------------------------------- #

class Flasher:
    def __init__(self, host, timeout=DEFAULT_TIMEOUT, verbose=False):
        if not host:
            raise FlasherError("no host given")
        self.host_arg = host
        if not host.startswith(("http://", "https://")):
            host = "http://" + host
        self.base = host.rstrip("/")
        self.timeout = timeout
        self.verbose = verbose
        self._resolved = False

    def _hostname_port(self):
        netloc = self.base.split("://", 1)[1]
        netloc = netloc.split("/", 1)[0]
        if netloc.startswith("[") and "]" in netloc:           # IPv6 literal
            host = netloc[1:netloc.index("]")]
            rest = netloc[netloc.index("]") + 1:]
            port = int(rest[1:]) if rest.startswith(":") else 80
        elif ":" in netloc:
            host, port = netloc.rsplit(":", 1)
            port = int(port)
        else:
            host, port = netloc, 80
        return host, port

    def resolve(self):
        """Resolve the host through the system resolver (handles DNS and mDNS
        .local names).  Returns a list of IPs; raises a helpful error if the
        name cannot be resolved."""
        host, port = self._hostname_port()
        try:
            infos = socket.getaddrinfo(host, port, proto=socket.IPPROTO_TCP)
        except socket.gaierror as e:
            hint = ""
            if host.endswith(".local"):
                hint = ("  (mDNS name -- needs mDNS resolution: native on "
                        "macOS, Avahi/nss-mdns on Linux. Is the device powered "
                        "and on this network?)")
            raise FlasherError("cannot resolve '%s': %s%s" % (host, e, hint))
        ips = []
        for fam, _, _, _, sockaddr in infos:
            ip = sockaddr[0]
            if ip not in ips:
                ips.append(ip)
        return ips

    def _request(self, path, data=None, method="GET", ctype=None, timeout=None):
        if self.verbose and not self._resolved:
            host, _ = self._hostname_port()
            print("Resolving %s -> %s" % (host, ", ".join(self.resolve())),
                  file=sys.stderr)
            self._resolved = True
        url = self.base + path
        headers = {"Content-Type": ctype} if ctype else {}
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as resp:
                return resp.read()
        except urllib.error.HTTPError as e:
            body = e.read().decode("utf-8", "replace").strip()
            raise FlasherError("HTTP %d from %s: %s" % (e.code, path, body or e.reason))
        except urllib.error.URLError as e:
            if isinstance(e.reason, socket.gaierror):
                # Surface the friendly resolution error.
                self.resolve()
            raise FlasherError("cannot reach %s: %s" % (url, e.reason))
        except TimeoutError:
            raise FlasherError("request to %s timed out" % url)

    def _json(self, path, **kw):
        raw = self._request(path, **kw)
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            raise FlasherError("unexpected (non-JSON) reply from %s: %r"
                               % (path, raw[:200]))

    def identify(self):
        return self._json("/api/identify", timeout=max(self.timeout, 60))

    def diag(self):
        return self._json("/api/diag", timeout=max(self.timeout, 60))

    def chain(self):
        return self._json("/api/chain", timeout=max(self.timeout, 60))

    def bscan(self):
        return self._json("/api/bscan", timeout=max(self.timeout, 60))

    def status(self):
        return self._json("/api/status")

    def pins(self):
        return self._json("/api/pins")

    def autopins(self):
        return self._json("/api/autopins", data=b"", method="POST",
                          timeout=max(self.timeout, 60))

    def upload(self, data):
        return self._json("/api/upload", data=data, method="POST",
                          ctype="application/octet-stream", timeout=UPLOAD_TIMEOUT)

    def start_flash(self, mode):
        return self._json("/api/flash?mode=%s" % mode, data=b"", method="POST")

    def start_svf(self):
        return self._json("/api/svf", data=b"", method="POST")

    def wait(self, poll=DEFAULT_POLL, timeout=OP_TIMEOUT, on_update=None):
        deadline = time.monotonic() + timeout
        last = None
        while True:
            st = self.status()
            prog = st.get("progress", -1)
            text = st.get("status", "")
            if on_update is not None and (prog, text) != last:
                on_update(prog, text)
                last = (prog, text)
            if prog == 100 and st.get("result", 0) == 0:
                return st
            if prog == -1:
                raise FlasherError("operation failed: %s" % text)
            if time.monotonic() > deadline:
                raise FlasherError("timed out after %ds waiting for completion" % timeout)
            time.sleep(poll)


# --------------------------------------------------------------------------- #
# Presentation helpers
# --------------------------------------------------------------------------- #

def _bar(prog):
    prog = max(0, min(100, prog))
    filled = prog * 24 // 100
    return "[" + "#" * filled + "." * (24 - filled) + "] %3d%%" % prog


def progress_printer():
    state = {"last": None}

    def cb(prog, text):
        line = ("      %s" % text) if prog < 0 else ("%s  %s" % (_bar(prog), text))
        if sys.stdout.isatty():
            sys.stdout.write("\r\033[K" + line)
            sys.stdout.flush()
            if prog in (100, -1):
                sys.stdout.write("\n")
        elif (prog, text) != state["last"]:
            print(line)
            state["last"] = (prog, text)
    return cb


def _read_file(path):
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError as e:
        raise FlasherError("cannot read %s: %s" % (path, e))


def _print_kv(d, keys=None):
    items = [(k, d[k]) for k in (keys or d)] if keys else list(d.items())
    width = max((len(str(k)) for k, _ in items), default=0)
    for k, v in items:
        print("  %-*s : %s" % (width, k, v))


# --------------------------------------------------------------------------- #
# Commands
# --------------------------------------------------------------------------- #

def cmd_identify(fl, args):
    info = fl.identify()
    if not info.get("ok"):
        print("No device identified (%s)" % info.get("family", "?"))
        return 1
    print("Device identified:")
    _print_kv(info, ["family", "idcode", "tiles", "revision", "bsr_len"])
    return 0


def cmd_diag(fl, args):
    print("JTAG diagnostic:")
    _print_kv(fl.diag())
    return 0


def cmd_scan(fl, args):
    res = fl.chain()
    if res.get("error"):
        raise FlasherError("chain scan failed: %s" % res["error"])
    devs = res.get("devices", [])
    if not devs:
        print("No devices on the JTAG chain.")
        return 1
    print("%d device(s) on the chain:" % len(devs))
    for i, d in enumerate(devs):
        print("  [%d] %s  %s  (mfg %s, part %s, rev %s, IR %s)" % (
            i, d.get("idcode"), d.get("name"), d.get("manufacturer"),
            d.get("part"), d.get("version"), d.get("ir_len")))
    return 0


def cmd_bscan(fl, args):
    res = fl.bscan()
    print("Boundary scan (%s bits):" % res.get("bsr_len"))
    print("  hex : %s" % res.get("hex"))
    if args.bits:
        print("  bits: %s" % res.get("bits"))
    return 0


def cmd_status(fl, args):
    _print_kv(fl.status())
    return 0


def cmd_pins(fl, args):
    p = fl.pins()
    print("Current pin assignment:")
    for k in ('tck', 'tms', 'tdi', 'tdo', 'trst', 'srst'):
        v = p.get(k)
        print("  %-4s : %s" % (k.upper(), 'NC' if v in (None, -1) else 'GPIO%d' % v))
    return 0


def cmd_autopins(fl, args):
    print("Auto-detecting JTAG pins (briefly drives candidate pins)...")
    d = fl.autopins()
    if not d.get('ok'):
        print("No working pin mapping found in %s permutations." % d.get('trials'))
        return 1
    extra = ' (TDI by elimination)' if d.get('tdi_assumed') else ('' if d.get('tdi_found') else ' (TDI unconfirmed)')
    print("Found in %s permutations (IDCODE %s):" % (d.get('trials'), d.get('idcode')))
    print("  TCK=GPIO%s  TMS=GPIO%s  TDI=GPIO%s  TDO=GPIO%s%s"
          % (d.get('tck'), d.get('tms'), d.get('tdi'), d.get('tdo'), extra))
    print("Applied to the live session. Set PIN_* in main.c to persist.")
    return 0


def cmd_resolve(fl, args):
    host, port = fl._hostname_port()
    ips = fl.resolve()
    print("%s -> %s  (port %d)" % (host, ", ".join(ips), port))
    return 0


def _upload_and_run(fl, args, mode, op_name):
    data = _read_file(args.file)
    print("Uploading %s (%d bytes)..." % (args.file, len(data)))
    up = fl.upload(data)
    ftype = up.get("type", "?")
    line = "  type=%s size=%s" % (ftype, up.get("size"))
    if ftype in ("xe", "elf"):
        line += " tiles=%s segments=%s entry0=%s" % (
            up.get("tiles"), up.get("segments"), up.get("entry0"))
    print(line)

    if not args.skip_identify and mode in ("ram", "flash"):
        info = fl.identify()
        if info.get("ok"):
            print("Target: %s (%s, %s tile(s))" % (
                info.get("family"), info.get("idcode"), info.get("tiles")))
        else:
            print("Warning: no XMOS device identified; '%s' may take the wrong "
                  "path." % op_name)

    print("Starting %s..." % op_name)
    fl.start_flash(mode)
    fl.wait(poll=args._poll, on_update=progress_printer())
    print("%s complete." % op_name.capitalize())
    return 0


def cmd_load_ram(fl, args):
    return _upload_and_run(fl, args, "ram", "load-to-RAM")


def cmd_write_flash(fl, args):
    return _upload_and_run(fl, args, "flash", "flash write")


def cmd_load_cram(fl, args):
    return _upload_and_run(fl, args, "cram", "iCE40 CRAM load")


def cmd_svf(fl, args):
    data = _read_file(args.file)
    print("Uploading %s (%d bytes)..." % (args.file, len(data)))
    fl.upload(data)
    print("Playing SVF...")
    fl.start_svf()
    fl.wait(poll=args._poll, on_update=progress_printer())
    print("SVF playback complete.")
    return 0


def cmd_config(settings, args):
    """Show the resolved configuration and where each value came from."""
    print("Resolved configuration:")
    print("  host    : %s   [%s]" % (settings["host"], settings["host_src"]))
    print("  timeout : %s   [%s]" % (settings["timeout"], settings["timeout_src"]))
    print("  poll    : %s   [%s]" % (settings["poll"], settings["poll_src"]))
    print("  config file : %s" % (settings["config_path"] or "(none found)"))
    return 0


# --------------------------------------------------------------------------- #
# Argument parsing
# --------------------------------------------------------------------------- #

def build_parser():
    p = argparse.ArgumentParser(
        prog="xmflash",
        description="Remote CLI for the XMOS JTAG web flasher (ESP32).",
        epilog="Host/timeout/poll resolve from CLI > env (XMFLASH_*) > config "
               "file > default. Run `xmflash config` to see the resolved values.")
    p.add_argument("--host",
                   help="flasher address: IP, hostname, or mDNS .local name "
                        "(default %s)" % DEFAULT_HOST)
    p.add_argument("--config", help="path to a config file (overrides search)")
    p.add_argument("--timeout", type=float, help="per-request timeout in seconds")
    p.add_argument("--poll", type=float, help="status poll interval in seconds")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="print the resolved IP before connecting")

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("identify", help="identify the attached XMOS device").set_defaults(fn=cmd_identify)
    sub.add_parser("diag", help="low-level JTAG pin/IDCODE diagnostic").set_defaults(fn=cmd_diag)
    sub.add_parser("scan", help="scan and list the JTAG chain").set_defaults(fn=cmd_scan)
    sub.add_parser("status", help="show current operation status").set_defaults(fn=cmd_status)
    sub.add_parser("pins", help="show the current JTAG pin assignment").set_defaults(fn=cmd_pins)
    sub.add_parser("autopins", help="auto-detect the JTAG pin mapping").set_defaults(fn=cmd_autopins)
    sub.add_parser("resolve", help="resolve the host via the system resolver and exit").set_defaults(fn=cmd_resolve)
    sub.add_parser("config", help="show resolved configuration and exit").set_defaults(fn=cmd_config, _no_host=True)

    b = sub.add_parser("bscan", help="capture the boundary-scan register")
    b.add_argument("--bits", action="store_true", help="also print the raw bit string")
    b.set_defaults(fn=cmd_bscan)

    for name, fn, helptext in (
        ("load-ram",   cmd_load_ram,   "load a .xe/.elf/.bin into xCORE RAM and run it"),
        ("write-flash", cmd_write_flash, "write a boot image to the XMOS boot flash"),
        ("load-cram",  cmd_load_cram,  "load an iCE40 bitstream into CRAM (SPI)"),
        ("svf",        cmd_svf,        "play an SVF file over JTAG"),
    ):
        s = sub.add_parser(name, help=helptext)
        s.add_argument("file", help="path to the file to send")
        s.add_argument("--skip-identify", action="store_true",
                       help="don't auto-identify before the operation")
        s.set_defaults(fn=fn)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        settings = resolve_settings(args)
        # `config` is informational and needs no connection.
        if getattr(args, "_no_host", False):
            return args.fn(settings, args)
        args._poll = settings["poll"]
        fl = Flasher(settings["host"], timeout=settings["timeout"],
                     verbose=args.verbose)
        return args.fn(fl, args)
    except FlasherError as e:
        print("\nerror: %s" % e, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
