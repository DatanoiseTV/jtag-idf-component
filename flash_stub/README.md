# XMOS JTAG flash stub

This directory holds the **on-chip flash programmer** used by
`xmos_jtag_program_flash()` to write a boot image to the XMOS device's own
QSPI/SPI flash over JTAG — the same approach `xflash` uses (load a small
program into the xCORE's RAM, run it, and stream flash data to it).

> **Status: reference implementation, not prebuilt.** The component cannot
> ship a binary because building an xCORE program requires the XMOS
> toolchain (`xcc`). `flash_stub.xc` is a correct-by-construction reference
> against the documented `libquadflash` API and the host protocol below;
> it must be compiled and validated on hardware. Until a stub is supplied,
> `xmos_jtag_program_flash()` returns `ESP_ERR_NOT_SUPPORTED`. If you only
> need to recover a device and can reach the flash chip's pins, the
> **direct-SPI** path (`xmos_spi_flash_program()`, fully implemented) is
> simpler — see the component README.

## Why a stub is needed

The XU208 boot flash is wired to the xCORE's QSPI port, not to the ESP32.
To write it "via the XMOS" the chip must drive its own flash pins, so a
program has to run on the xCORE. That program is what `xflash` injects;
here we load an equivalent program and talk to it through the debug
scratch-register mailbox.

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

```sh
# Provide an XN file describing your board's QSPI flash ports, then:
xcc flash_stub.xc -target=XCORE-200-EXPLORER -lquadflash -o flash_stub.xe
# (use the XN that matches your target's PORT_SQI_* assignment)
```

Then embed `flash_stub.xe` (e.g. via `target_add_binary_data` / an
`EMBED_FILES` entry) and pass its bytes to `xmos_jtag_program_flash()`.

## Open-source reference

The flash command sequences and QSPI driver come from XMOS's
permissively-licensed `xcore/sc_flash`
(`module_quad_spi_flash/src/quad_spi_flash.xc`,
`module_flash/specs/*.spispec`). `libquadflash` (shipped with the tools)
exposes the same `fl_*` API used below.
