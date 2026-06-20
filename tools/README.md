# xmflash -- remote CLI for the web flasher

`xmflash.py` drives the HTTP API served by the example firmware
(`example/main/`) over the network, so the whole identify / scan / load / flash
flow is scriptable without the browser UI. Pure Python standard library, no
dependencies.

```sh
tools/xmflash.py identify                 # uses the default mDNS host
tools/xmflash.py --host 192.168.1.23 scan
tools/xmflash.py write-flash image.bin    # program the XMOS boot flash via JTAG
tools/xmflash.py load-ram firmware.xe     # load + run from RAM
tools/xmflash.py load-cram bitstream.bin  # iCE40 CRAM
tools/xmflash.py svf program.svf          # play an SVF file
```

## Commands

| Command | What it does |
|---------|--------------|
| `identify` | Identify the attached XMOS device (family, IDCODE, tiles) |
| `scan` | Scan and list the JTAG chain |
| `diag` | Low-level JTAG pin / IDCODE diagnostic |
| `bscan [--bits]` | Capture the boundary-scan register |
| `status` | Show the current operation status |
| `resolve` | Resolve the host through the system resolver and print the IP |
| `config` | Show the resolved configuration and where each value came from |
| `load-ram FILE` | Load a `.xe`/`.elf`/`.bin` into xCORE RAM and run it |
| `write-flash FILE` | Write a boot image to the XMOS boot flash (via the JTAG stub) |
| `load-cram FILE` | Load an iCE40 bitstream into CRAM over SPI |
| `svf FILE` | Play an SVF file over JTAG |

`load-*` / `write-flash` auto-run `identify` first (so the firmware routes to
the correct path); pass `--skip-identify` to suppress that.

## Host, hostnames and mDNS

`--host` accepts an IP, a DNS hostname, or an mDNS `.local` name. Names are
resolved through the operating system's resolver (`getaddrinfo`), so mDNS works
wherever the OS supports it -- natively on macOS, via Avahi / `nss-mdns` on
Linux.

The firmware advertises itself over mDNS as **`xmflash.local`**, which is the
built-in default when no host is given. Check resolution with:

```sh
tools/xmflash.py resolve
tools/xmflash.py --host xmflash.local -v identify   # -v prints the resolved IP
```

## Configuration

Settings resolve with this precedence (highest first):

1. command-line flag -- `--host`, `--timeout`, `--poll`
2. environment variable -- `XMFLASH_HOST`, `XMFLASH_TIMEOUT`, `XMFLASH_POLL`
3. config file
4. built-in default (`host = xmflash.local`, `timeout = 30`, `poll = 0.5`)

The config file is an INI file with an `[xmflash]` section (a bare
`key = value` file also works -- the section is added automatically):

```ini
[xmflash]
host = 192.168.1.23
timeout = 30
poll = 0.5
```

Searched in order, first match wins: `--config PATH`, `$XMFLASH_CONFIG`,
`./.xmflashrc`, `$XDG_CONFIG_HOME/xmflash/config`, `~/.config/xmflash.ini`,
`~/.xmflashrc`. Run `xmflash config` to see what resolved and from where.

## Requirements

Python 3.6+ (standard library only). Make it executable with
`chmod +x tools/xmflash.py`, or run it as `python3 tools/xmflash.py ...`.
