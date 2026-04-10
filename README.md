# xmos-jtag-idf-component

ESP-IDF component for programming XMOS xCORE-200 (XU21x) and xCORE.ai (XU3xx) devices via JTAG from an ESP32.

## Features

- **Load firmware to xCORE RAM** via JTAG boot mode (parse `.xe` or raw binaries)
- **Program SPI flash** via a JTAG-loaded stub running on the xCORE
- **Direct SPI flash programming** by holding XMOS in reset and bit-banging SPI
- **Two JTAG transport backends:**
  - **GPIO bit-bang** -- works on any ESP32 variant (S2, S3, C3, C6, H2, ...)
  - **PARLIO with DMA** -- high-speed on ESP32-P4, up to ~40 MHz TCK

## Supported Devices

| XMOS Family | Chips | JTAG IDCODE |
|---|---|---|
| xCORE-200 (XS2) | XU208, XU216 | `0x00005633` |
| xCORE.ai (XS3) | XU316 | `0x00006633` (verify) |
| XS1 (legacy) | XS1-G1, XS1-G4, XS1-SU | `0x00002633`, `0x00104731`, `0x00003633` |

## Wiring

Connect your ESP32 to the XMOS JTAG header:

```
ESP32           XMOS JTAG
─────           ─────────
GPIO (TCK)  ──> TCK
GPIO (TMS)  ──> TMS
GPIO (TDI)  ──> TDI
GPIO (TDO)  <── TDO
GPIO (TRST) ──> TRST_N   (optional, active low)
GPIO (SRST) ──> RST_N    (optional, active low, open-drain)
GND         ──> GND
```

> **Note:** TRST and SRST should be configured as open-drain with external pull-ups to the XMOS I/O voltage (typically 3.3V). If your ESP32 runs at 3.3V I/O, direct connection is fine. For 1.8V XMOS I/O, use level shifters.

For **direct SPI flash programming**, also connect to the XMOS SPI flash chip while XMOS is held in reset:

```
ESP32           SPI Flash
─────           ─────────
GPIO (CS)   ──> CS
GPIO (CLK)  ──> CLK
GPIO (MOSI) ──> MOSI/DI
GPIO (MISO) <── MISO/DO
```

## Usage

### Add as a component

```
cd your-project
mkdir -p components
cd components
git clone https://github.com/DatanoiseTV/xmos-jtag-idf-component.git xmos_jtag
```

Or add to your `idf_component.yml`:

```yaml
dependencies:
  xmos_jtag:
    git: https://github.com/DatanoiseTV/xmos-jtag-idf-component.git
    path: components/xmos_jtag
```

### Configuration

Run `idf.py menuconfig` and navigate to **XMOS JTAG Programmer**:

| Option | Default | Description |
|---|---|---|
| `XMOS_JTAG_BACKEND` | GPIO | GPIO bit-bang or PARLIO (ESP32-P4 only) |
| `XMOS_JTAG_TCK_FREQ_KHZ` | 1000 / 10000 | JTAG clock frequency |
| `XMOS_JTAG_PARLIO_DMA_BUF_SIZE` | 4096 | DMA buffer size (PARLIO only) |

### Example: Identify a device

```c
#include "xmos_jtag.h"

xmos_jtag_pins_t pins = {
    .tck = GPIO_NUM_12,
    .tms = GPIO_NUM_13,
    .tdi = GPIO_NUM_14,
    .tdo = GPIO_NUM_15,
    .trst_n = GPIO_NUM_NC,
    .srst_n = GPIO_NUM_NC,
};

xmos_jtag_handle_t jtag;
ESP_ERROR_CHECK(xmos_jtag_init(&pins, &jtag));

xmos_chip_info_t info;
ESP_ERROR_CHECK(xmos_jtag_identify(jtag, &info));
// info.family == XMOS_FAMILY_XS2, info.idcode == 0x00005633, etc.
```

### Example: Load XE firmware to RAM and run

```c
// xe_data / xe_len loaded from flash, SPIFFS, HTTP, etc.
extern const uint8_t firmware_xe[] asm("_binary_firmware_xe_start");
extern const uint8_t firmware_xe_end[] asm("_binary_firmware_xe_end");

ESP_ERROR_CHECK(xmos_jtag_load_xe(
    jtag,
    firmware_xe,
    firmware_xe_end - firmware_xe,
    true  // start execution
));
```

### Example: Load raw binary to RAM

```c
ESP_ERROR_CHECK(xmos_jtag_load_raw(
    jtag,
    0,              // tile 0
    my_binary,      // firmware bytes
    my_binary_len,
    0x00040000,     // load address (RAM base)
    0x00080000      // entry point (or 0 to leave halted)
));
```

### Example: Program SPI flash directly

```c
// Requires srst_n pin to hold XMOS in reset
xmos_spi_pins_t spi = {
    .cs   = GPIO_NUM_5,
    .clk  = GPIO_NUM_18,
    .mosi = GPIO_NUM_23,
    .miso = GPIO_NUM_19,
    .wp   = GPIO_NUM_NC,
    .hold = GPIO_NUM_NC,
};

// flash_image created with: xflash --factory app.xe -o image.bin
ESP_ERROR_CHECK(xmos_spi_flash_program(
    jtag, &spi,
    flash_image, flash_image_len,
    0  // flash offset
));
```

### Example: Program SPI flash via JTAG stub

```c
// Requires a flash programmer stub compiled for xCORE
// (see test/flash_stub/ for source)
ESP_ERROR_CHECK(xmos_jtag_program_flash(
    jtag,
    flash_image, flash_image_len,
    stub_binary, stub_binary_len
));
```

## Architecture

```
include/xmos_jtag.h         Public API
src/
  jtag_transport.h           Abstract transport vtable (shift_ir, shift_dr, reset, idle)
  jtag_gpio.c                GPIO bit-bang backend
  jtag_parlio.c              PARLIO DMA backend (ESP32-P4)
  xmos_regs.h                XMOS TAP, MUX, PSWITCH/SSWITCH, debug register definitions
  xmos_jtag.c                XMOS JTAG protocol: chain scan, MUX, register R/W,
                             debug mode, memory access, boot sequence, flash programming
  xmos_xe.c / xmos_xe.h     XE file format + ELF32 segment parser
```

### JTAG Transport Backends

**GPIO bit-bang** (`jtag_gpio.c`):
Direct register writes (`out_w1ts`/`out_w1tc`) for maximum toggle speed. Typical 1-5 MHz TCK depending on CPU clock. No DMA, no peripheral dependencies.

**PARLIO DMA** (`jtag_parlio.c`):
Uses the ESP32-P4 Parallel IO peripheral:
- **TX**: `data_width=2` (TMS on bit 0, TDI on bit 1), `clk_out` = TCK
- **RX**: `data_width=1` (TDO), clocked from TCK via GPIO matrix loopback
- Each TX byte encodes 4 JTAG clock cycles (2 bits per cycle, interleaved)
- Pre-computes entire TAP navigation + data shift into one DMA buffer
- TX and RX run on independent DMA channels simultaneously
- Up to ~40 MHz TCK from 160 MHz PLL with fractional divider

### XMOS JTAG Protocol

The XMOS JTAG interface uses a multiplexed TAP chain:

1. **Top-level TAP** (boundary scan): 4-bit IR, always present
2. **SETMUX** instruction opens the internal chain
3. **Internal chain** (when MUX open): OTP(2b) + XCORE(10b) + CHIP(4b) + BSCAN(4b) = 20-bit IR
4. **Register access**: xCORE TAP IR encodes `(reg_index << 2) | read_write_flag`
5. **Debug mode**: Write to `PSWITCH_DBG_INT` register, memory access via scratch register mailbox

### XE File Format

The parser handles the XMOS XE container format (verified against `tool_axe` reference):
- Sector types: ELF (0x02), BINARY (0x01), GOTO (0x05), CALL (0x06), XN (0x08), CONFIG (0x03)
- 12-byte sector header with 64-bit length field
- Per-tile ELF binaries with standard ELF32 PT_LOAD segments
- Accepts both `.xe` files and raw ELF binaries

## Protocol Notes

Some register addresses and debug command codes were derived from reverse-engineering the `sc_jtag` open-source JTAG master and XMOS forum posts. The following may need adjustment on specific silicon revisions:

- **PSWITCH debug scratch registers** (0x20-0x27 as command mailbox)
- **Debug command codes** (1=READ, 2=WRITE, ..., 9=RFDBG)
- **XS3 IDCODE** (assumed 0x00006633)
- **MUX DR encoding** when internal chain is already open

Tested with xCORE-200 (XU208/XU216). XS3 support is structural but unverified.

## License

MIT
