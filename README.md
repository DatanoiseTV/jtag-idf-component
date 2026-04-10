# xmos-jtag-idf-component

ESP-IDF component for programming XMOS xCORE-200 (XU21x) and xCORE.ai (XU3xx) devices via JTAG from an ESP32.

Builds with **ESP-IDF v6.0** (also compatible with v5.x). Verified compiling for ESP32-S3 and ESP32-P4.


## WebUI Example

<img width="3360" height="1892" alt="image" src="https://github.com/user-attachments/assets/013fa2a7-c98d-4f61-907a-104fcb167ac7" />

## Protocol Verification

<img width="1728" height="1030" alt="Screenshot 2026-04-10 at 21 36 38" src="https://github.com/user-attachments/assets/e37209d1-0885-47a6-9ec6-7d385134b64d" />CORE
- **Direct SPI flash programming** by holding XMOS in reset and bit-banging SPI
- **Two JTAG transport backends:**
  - **GPIO bit-bang** -- works on any ESP32 variant (S2, S3, C3, C6, H2, P4, ...)
  - **PARLIO with DMA** -- high-speed on ESP32-P4, up to ~40 MHz TCK
- **Web flasher example** -- WiFi AP with browser UI for uploading and flashing firmware

## Supported XMOS Devices

| XMOS Family | Chips | JTAG IDCODE | Tiles |
|---|---|---|---|
| xCORE.ai (XS3) | XU316 | `0x00006633` | 2 |
| xCORE-200 (XS2) | XU208, XU216 | `0x00005633` | 1-2 |
| XS1 (legacy) | XS1-G1, XS1-G4, XS1-SU | `0x00002633`, `0x00104731`, `0x00003633` | 1-4 |

> **Note:** All XMOS JTAG signals use **1.8V** levels. Use level shifters if your ESP32 runs at 3.3V I/O. The JTAG module can be reset by holding TMS high for five clock cycles.

## Wiring

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

**Waveshare ESP32-P4-NANO** suggested pin assignment (right header, rows 7-10):

| Signal | GPIO | Header Pin |
|---|---|---|
| TCK | 47 | R-Row 8 inner |
| TMS | 48 | R-Row 8 outer |
| TDI | 46 | R-Row 9 inner |
| TDO | 45 | R-Row 10 inner |
| TRST (opt) | 53 | R-Row 7 outer |
| SRST (opt) | 54 | R-Row 7 inner |

For **direct SPI flash programming**, also connect to the XMOS SPI flash chip:

```
ESP32           SPI Flash
─────           ─────────
GPIO (CS)   ──> CS
GPIO (CLK)  ──> CLK
GPIO (MOSI) ──> MOSI/DI
GPIO (MISO) <── MISO/DO
```

## Quick Start

### Add to your project

```sh
cd your-project/components
git clone https://github.com/DatanoiseTV/xmos-jtag-idf-component.git xmos_jtag
```

### Menuconfig

`idf.py menuconfig` -> **XMOS JTAG Programmer**:

| Option | Default | Description |
|---|---|---|
| `XMOS_JTAG_BACKEND` | GPIO | GPIO bit-bang or PARLIO (ESP32-P4) |
| `XMOS_JTAG_TCK_FREQ_KHZ` | 1000 / 10000 | JTAG clock frequency |
| `XMOS_JTAG_PARLIO_DMA_BUF_SIZE` | 4096 | DMA buffer size (PARLIO only) |

### Identify a device

```c
#include "xmos_jtag.h"

xmos_jtag_pins_t pins = {
    .tck = GPIO_NUM_12, .tms = GPIO_NUM_13,
    .tdi = GPIO_NUM_14, .tdo = GPIO_NUM_11,
    .trst_n = GPIO_NUM_NC, .srst_n = GPIO_NUM_NC,
};

xmos_jtag_handle_t jtag;
ESP_ERROR_CHECK(xmos_jtag_init(&pins, &jtag));

xmos_chip_info_t info;
ESP_ERROR_CHECK(xmos_jtag_identify(jtag, &info));
// info.family, info.idcode, info.num_tiles, info.revision
```

### Load XE firmware to RAM

```c
extern const uint8_t fw_xe[] asm("_binary_firmware_xe_start");
extern const uint8_t fw_xe_end[] asm("_binary_firmware_xe_end");

ESP_ERROR_CHECK(xmos_jtag_load_xe(jtag, fw_xe, fw_xe_end - fw_xe, true));
```

### Program SPI flash directly

```c
xmos_spi_pins_t spi = {
    .cs = GPIO_NUM_5, .clk = GPIO_NUM_18,
    .mosi = GPIO_NUM_23, .miso = GPIO_NUM_19,
    .wp = GPIO_NUM_NC, .hold = GPIO_NUM_NC,
};
// image created with: xflash --factory app.xe -o image.bin
ESP_ERROR_CHECK(xmos_spi_flash_program(jtag, &spi, image, image_len, 0));
```

## Web Flasher Example

The `example/` directory contains a complete ESP-IDF project with a web UI for flashing XMOS firmware over WiFi.

```sh
cd example
idf.py set-target esp32s3   # or esp32, esp32c3, etc.
idf.py build flash monitor
```

Connect to WiFi AP **XMOS-Flasher** (password: `xmosjtag`), open http://192.168.4.1.

The web UI lets you:
- Identify the connected XMOS device (IDCODE, family, tile count)
- Upload `.xe` or `.bin` firmware files and inspect their contents (segments, entry points, tile mapping)
- Flash firmware to xCORE RAM or SPI flash with progress tracking

> **ESP32-P4 note:** The P4 has no native WiFi. For P4, use Ethernet or the ESP32-C6 coprocessor with `esp_wifi_remote`. The xmos_jtag component itself builds fine for P4 -- only the example's WiFi code needs adaptation.

## Architecture

```
components/xmos_jtag/
  include/
    xmos_jtag.h             Public API
    xmos_xe.h               XE/ELF parser API
  src/
    jtag_transport.h         Abstract transport vtable
    jtag_gpio.c              GPIO bit-bang backend (all ESP32 variants)
    jtag_parlio.c            PARLIO DMA backend (ESP32-P4)
    xmos_regs.h              XMOS register definitions (TAP, MUX, PSWITCH, debug)
    xmos_jtag.c              XMOS JTAG protocol layer
    xmos_xe.c                XE file format + ELF32 parser
```

### JTAG Backends

**GPIO bit-bang** -- uses `gpio_set_level`/`gpio_get_level`, portable across all ESP32 chips. ~1-5 MHz TCK.

**PARLIO DMA** (ESP32-P4) -- uses the Parallel IO peripheral with DMA:
- TX: `data_width=2` (TMS + TDI), `clk_out` = TCK
- RX: `data_width=1` (TDO), clocked from TCK via GPIO matrix
- Each TX byte encodes 4 JTAG cycles; entire shift sequences are DMA'd in one shot
- Up to ~40 MHz TCK from 160 MHz PLL

### XMOS JTAG Protocol

1. **Top-level TAP**: 4-bit IR, IEEE 1149.1. IDCODE at IR=1, SET_TEST_MODE at IR=8.
2. **SETMUX** (IR=4): opens internal chain to OTP + XCORE + CHIP TAPs (20-bit IR total)
3. **Register access**: xCORE TAP IR = `(reg_index << 2) | rw_flag`, DR = 32/33 bits
4. **Debug mode**: write `DBG_INT` register, then memory R/W via scratch register mailbox
5. **JTAG boot**: SET_TEST_MODE with bit 29 (BOOT_FROM_JTAG), load code, set PC, resume

### XE File Format

Verified against the `tool_axe` reference parser and real satellite firmware `.xe` files:
- 12-byte sector headers with 64-bit length field and padding descriptor
- Sector types: ELF (0x02), BINARY (0x01), GOTO (0x05), CALL (0x06), XN (0x08)
- Per-tile ELF32 binaries with PT_LOAD segments
- Also accepts raw ELF files directly

## Tests

56 host-side tests covering XE parsing, ELF edge cases, JTAG chain IR encoding, MUX constants, PARLIO bit packing, TAP state machine navigation, and real firmware file validation.

```sh
cd test
make test
```

## Protocol Notes

Some register addresses and debug commands were derived from the open-source `sc_jtag` JTAG master and XMOS forum posts. These may need adjustment for specific silicon:

- PSWITCH debug scratch registers (0x20-0x27 as command mailbox)
- Debug command codes (1=READ, 2=WRITE, ..., 9=RFDBG)
- MUX DR encoding when internal chain is already open
- XS3 IDCODE confirmed as `0x00006633` from XU316 datasheet

## License

MIT
