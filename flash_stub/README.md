# XMOS JTAG flash stub

This directory holds the **on-chip flash programmer** used by
`xmos_jtag_program_flash()` to write a boot image to the XMOS device's own
QSPI/SPI flash over JTAG — the same approach `xflash` uses (load a small
program into the xCORE's RAM, run it, and stream flash data to it).

> **Status: the XS2 stub is embedded in the example, pending on-silicon
> validation; XS3 must be rebuilt.** Building an xCORE program needs the XMOS
> toolchain (`xcc`), so the component itself ships no binary, but the example
> app (`example/main/`) embeds the prebuilt `flash_stub_xs2.xe` and drives it
> from the "Write Flash" action when an XS2 device is detected. The stub is
> correct-by-construction against the documented `libquadflash` API and the
> host protocol below; the one part that can only be confirmed on hardware is
> whether the running core's `getps`/`setps` alias the same PSWITCH scratch
> registers the host writes over JTAG. If you only need to recover a device
> and can reach the flash chip's pins, the **direct-SPI** path
> (`xmos_spi_flash_program()`, fully implemented) is simpler — see the
> component README.
>
> **XS3/xcore.ai is not bundled.** xcore.ai maps SRAM at `0x80000`, so the
> shared data buffer must sit at `0x90000`, not the XS2 value `0x50000`.
> `flash_stub.xc` now selects this automatically (`#if defined(__XS3A__)`),
> but you must compile it for `xs3a` yourself and drop the resulting `.xe`
> into `example/main/` (and re-enable the XS3 branch); the example refuses
> the XS3 flash path until then rather than risk writing garbage.
>
> **Page alignment is required.** This stub's only write primitive is
> `quad_spi_flash_write_page()` (full 256-byte page), so every `WRITE` must
> target a page-aligned flash offset; `xmos_jtag_program_flash()` already
> chunks the image on 256-byte boundaries and the stub pads any short final
> chunk with `0xFF`.

## Why a stub is needed

The boot flash is wired to the xCORE's QSPI port, not to the ESP32. To
write it "via the XMOS" the chip must drive its own flash pins, so a
program has to run on the xCORE. That program is what `xflash` injects;
here we load an equivalent program and talk to it through the debug
scratch-register mailbox. For **XUF** parts (internal/embedded flash)
this is the *only* way to write the flash — there is no external chip to
clip onto.

## Supported parts (one stub, same ports)

The QSPI boot ports are identical across the whole family, so a single
stub source covers every part — only the build architecture changes:

| Part(s)            | Arch  | Build target | Flash         |
|--------------------|-------|--------------|---------------|
| XU208              | xs2a  | xcore-200    | external QSPI |
| XUF208, XUF216     | xs2a  | xcore-200    | internal QSPI |
| XU316 (xcore.ai)   | xs3a  | xcore.ai     | external QSPI |

Ports (all parts): `CS = XS1_PORT_1B`, `SCLK = XS1_PORT_1C`,
`SIO = XS1_PORT_4B`, flash on `tile[0]` — verified against XMOS's
`xk-audio-316-mc` (xcore.ai) and XUF216 board XN files.

> There is **no XUF316**. xcore.ai/XS3 is external-flash only; the "UF"
> internal-flash suffix exists only on xCORE-200/XS2 (XUF208/XUF216/
> XEF216). Don't look for one.

**xcore.ai (XS3) caveat:** the low-level QSPI driver's read sample point
was tuned for XS2's clocking. It compiles for `xs3a`, but keep `SCLK`
conservative (≈25 MHz — `libquadflash`'s default class) and re-validate
reads on silicon; erase/program timing is forgiving, only high-speed
reads need the `lib_qspi_fast_read` calibration dance (not used here).

## Host <-> stub protocol (authoritative)

The mailbox is the per-tile PSWITCH debug scratch registers. The host
writes them over JTAG while the core is briefly halted, then resumes; the
running stub polls them. Register numbers are the `XMOS_PSWITCH_DBG_*`
values in `components/xmos_jtag/src/xmos_regs.h`.

| Mailbox slot | Reg | Direction | Meaning |
|---|---|---|---|
| STATUS  | scratch[0] (0x20) | stub → host | 1=READY 2=BUSY 3=OK ≥0x80=ERROR |
| COMMAND | scratch[1] (0x21) | host → stub | 0=NONE 1=WRITE 2=ERASE 0xFF=DONE |
| ARG0    | scratch[2] (0x22) | host → stub | flash address (sector / write offset) |
| ARG1    | scratch[3] (0x23) | host → stub | byte count (WRITE) |

Data buffer: the host writes each chunk's bytes to tile RAM at
`RAM_BASE + 0x10000` (`STUB_DATA_BUF_OFFSET`) via the debug memory
interface before issuing `WRITE`. Keep this region out of the stub's
own linker map.

Handshake (race-free): the host halts the core, sets `STATUS=BUSY`, writes
`ARG0/ARG1/COMMAND`, then resumes. The stub performs the op, sets
`STATUS=OK` (or `≥ERROR`) and clears `COMMAND`. The host polls `STATUS`.
On `DONE` the stub sets the flash Quad-Enable bit (so the boot ROM's `0xEB`
quad read works) and halts.

## Boot image

The bytes streamed must be a complete **boot image**, i.e. the output of
`xflash --factory app.xe -o image.bin` (flash loader + factory image with
descriptor/header/CRC). This tool writes them verbatim; it does not build
the container. Write from flash offset 0.

## Building (with the XMOS XTC tools)

Put the `sc_flash` `module_quad_spi_flash/src/quad_spi_flash.{xc,h}` next
to this file (or add the module to `USED_MODULES`), supply an XN for your
board, then build for the matching architecture:

```sh
# xCORE-200 -- build for the SMALLEST SRAM variant you will run on
# (XU208-128 = 128 KB), so the linker keeps the stack inside the target's
# RAM.  A stub built for -256 puts its stack at the top of 256 KB (0x80000),
# which is OUTSIDE a 128 KB part's RAM (ends at 0x60000) -- it then faults on
# the first stack access and never reaches the mailbox.
xcc flash_stub.xc quad_spi_flash.xc -target=XU208-128-TQ64-C10 -o flash_stub_xs2.xe

# xcore.ai (XU316)
xcc flash_stub.xc quad_spi_flash.xc -target=XK-EVK-XU316 -o flash_stub_xs3.xe
```

(Use the XN that matches your board's `PORT_SQI_*` assignment; the
defaults above are the standard `CS=1B / SCLK=1C / SIO=4B` map.) The bundled
`example/main/flash_stub_xs2.xe` is built for `XU208-128` so it runs on both
128 KB and 256 KB parts. Then
embed the resulting `.xe` (e.g. via `EMBED_FILES`) and pass its bytes to
`xmos_jtag_program_flash()` — it loads the same way regardless of arch.

## Open-source reference

The flash command sequences and QSPI driver come from XMOS's
permissively-licensed `xcore/sc_flash`
(`module_quad_spi_flash/src/quad_spi_flash.xc`,
`module_flash/specs/*.spispec`). `libquadflash` (shipped with the tools,
built for both `xs2a` and `xs3a`) exposes the higher-level `fl_*` API; we
use the low-level driver here because writing a complete pre-built boot
image to flash offset 0 needs raw sector-erase / page-program access,
which the image-oriented `fl_*` API does not expose.

## Note on the boot image / IDCODE

- The bytes written must be a complete boot image (`xflash -o`), stored
  with the tools-15.x **per-word nibble-swap** encoding — that's part of
  the image `xflash` emits, so writing its `-o` output verbatim is
  correct; don't re-order bytes yourself.
- XU316's JTAG IDCODE is documented as `0x...6633` but, like XU208
  (documented `6633`, silicon `5633`), the real value must be confirmed on
  hardware — the host's detection table treats the XS3 ID as unverified.
