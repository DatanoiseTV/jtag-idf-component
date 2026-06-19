# jtag-idf-component

A general-purpose JTAG programmer component for ESP-IDF. Scan chains, program FPGAs, flash microcontrollers, play SVF files -- all from an ESP32 with a browser UI.

> Built for **ESP-IDF v6.0**. Tested on **ESP32-S3** (WiFi) and **ESP32-P4** (Ethernet + DMA-accelerated JTAG).

---

<img width="766" height="579" alt="Screenshot 2026-04-10 at 22 47 39" src="https://github.com/user-attachments/assets/7fd63ede-c594-4aef-853a-fe4208c1809c" />

---

## At a Glance

| | |
|---|---|
| **Chain scan** | Auto-detect every device on the JTAG chain -- XMOS, Lattice, Xilinx, Espressif, ARM DAP and more |
| **SVF player** | Play Serial Vector Format files to program any JTAG-capable device *(coming soon)* |
| **XMOS support** | Full xCORE programming: load `.xe` to RAM, program SPI flash, debug register access |
| **iCE40 support** | Program Lattice iCE40 FPGAs via SPI (CRAM and flash) *(coming soon)* |
| **Boundary scan** | Capture all I/O pin states, auto-detect BSR length, live refresh from the browser |
| **Two backends** | GPIO bit-bang on any ESP32 (1-5 MHz) **or** PARLIO DMA on ESP32-P4 (up to 40 MHz) |
| **Web UI** | Visual chain diagram, firmware inspector, one-click flash -- no tools to install |

## Supported Targets

### XMOS (fully implemented)

| Family | Chips | IDCODE | Tiles |
|---|---|---|---|
| **xCORE.ai** (XS3) | XU316 | `0x00006633` *(unverified)* | 2 |
| **xCORE-200** (XS2) | XU208, XU216 | `0x00005633` | 1--2 |
| **XS1** (legacy) | XS1-G1, G4, SU | `0x00002633` `0x00104731` `0x00003633` | 1--4 |

- Load firmware to RAM via JTAG boot (`.xe` or raw ELF)
- Program SPI flash (direct or via JTAG-loaded stub)
- Debug register access (PSWITCH/SSWITCH)
- XE file parser verified against real multi-tile satellite firmware

### FPGAs & Other Devices (chain scan + SVF)

The chain scanner recognises **Lattice ECP5/iCE40**, **Xilinx 7-Series**, **Espressif SoCs**, and **ARM CoreSight DAPs** by IDCODE. Any JTAG device can be driven via SVF file playback.

| Device | Programming Method | Status |
|---|---|---|
| Lattice iCE40UP5K | SPI slave (CRAM/flash) | *planned* |
| Lattice ECP5 | JTAG (SVF) | *via SVF player* |
| Xilinx 7-Series | JTAG (SVF) | *via SVF player* |
| Any JTAG device | SVF file playback | *planned* |

> **I/O voltage** -- XMOS xCORE-200/xCORE.ai JTAG pins sit on the 3.3 V `VDDIO` rail (the 1.8 V rails are core/PLL supplies only), so they connect to ESP32 GPIOs directly. Only targets whose JTAG bank actually runs below 3.3 V (check the datasheet's I/O bank supply) need a level shifter; on the ESP32-P4-NANO the `LDO_VO4` header pin can supply 1.8 V for the shifter's low-voltage rail in that case.

---

## Use Cases

**Production & Manufacturing**
- Flash firmware on the assembly line without a PC -- just an ESP32, a browser, and a JTAG cable
- Boundary scan for board-level continuity testing before functional test
- Chain scan to verify correct device population and orientation

**CI / CD & Automated Testing**
- Drive the ESP32 flasher from a test script over HTTP (`curl -X POST /api/upload ...`)
- Load nightly builds to RAM, run tests, repeat -- no flash wear, instant boot
- Gate firmware releases on automated JTAG connectivity checks

**Field Updates & OTA**
- ESP32 pulls new firmware over WiFi/Ethernet, then JTAG-boots the target -- true over-the-air updates for devices with no native network stack
- Dual-bank strategy: load to RAM for validation, then commit to SPI flash only if self-test passes

**Development & Debugging**
- Rapid edit-compile-load cycle without an XTAG adapter or expensive JTAG probe
- Inspect I/O pin states in real time via boundary scan from any browser
- Identify unknown or mislabelled parts via IDCODE + chain scan

**Multi-Device Systems**
- Single ESP32 manages the JTAG chain of an entire board (XMOS + FPGA + MCU)
- Chain visualization shows every device -- useful for bring-up of complex PCBs

---

## Getting Started

### 1. Add the component

```sh
cd your-project/components
git clone https://github.com/DatanoiseTV/jtag-idf-component.git jtag_prog
```

### 2. Configure

```sh
idf.py menuconfig   # -> JTAG Programmer
```

| Option | Default | |
|---|---|---|
| `XMOS_JTAG_BACKEND` | GPIO | GPIO bit-bang **or** PARLIO DMA (ESP32-P4) |
| `XMOS_JTAG_TCK_FREQ_KHZ` | 1 000 / 10 000 | Clock frequency |
| `XMOS_JTAG_PARLIO_DMA_BUF_SIZE` | 4 096 | DMA buffer (PARLIO only) |

### 3. Use the API

```c
#include "xmos_jtag.h"

/* Initialise */
xmos_jtag_pins_t pins = {
    .tck = GPIO_NUM_47, .tms = GPIO_NUM_48,
    .tdi = GPIO_NUM_46, .tdo = GPIO_NUM_45,
    .trst_n = GPIO_NUM_53, .srst_n = GPIO_NUM_54,
};
xmos_jtag_handle_t jtag;
ESP_ERROR_CHECK(xmos_jtag_init(&pins, &jtag));

/* Scan the chain -- works with any JTAG device */
jtag_chain_t chain;
xmos_jtag_scan_chain(jtag, &chain);
for (int i = 0; i < chain.num_devices; i++)
    printf("[%d] %s  0x%08lx\n", i, chain.devices[i].name,
           (unsigned long)chain.devices[i].idcode);

/* Identify XMOS device specifically */
xmos_chip_info_t info;
xmos_jtag_identify(jtag, &info);

/* Load XMOS firmware */
extern const uint8_t fw[] asm("_binary_firmware_xe_start");
extern const uint8_t fw_end[] asm("_binary_firmware_xe_end");
xmos_jtag_load_xe(jtag, fw, fw_end - fw, true);
```

### Program an iCE40 FPGA

```c
#include "jtag_ice40.h"

/* SPI + control pins for the iCE40 */
ice40_pins_t ice_pins = {
    .spi_cs   = GPIO_NUM_20,   /* directly to iCE40 SPI_SS */
    .spi_clk  = GPIO_NUM_23,   /* SPI clock */
    .spi_mosi = GPIO_NUM_5,    /* data to iCE40 */
    .spi_miso = GPIO_NUM_4,    /* data from iCE40 (or GPIO_NUM_NC) */
    .creset   = GPIO_NUM_21,   /* CRESET_B -- hold low to reset FPGA */
    .cdone    = GPIO_NUM_22,   /* CDONE -- goes high when configured */
};

/* Embed the .bin bitstream (output of icepack) */
extern const uint8_t bitstream[] asm("_binary_top_bin_start");
extern const uint8_t bitstream_end[] asm("_binary_top_bin_end");
size_t bitstream_len = bitstream_end - bitstream;

/*
 * Load to CRAM -- volatile, bitstream lost on power cycle.
 * Fastest option for development (no flash erase/write).
 *
 * ice40_program_cram(pins, data, length, timeout_ms)
 *   timeout_ms: how long to wait for CDONE to go high (3000 = 3 sec)
 *               set to 0 to skip the CDONE check
 */
ESP_ERROR_CHECK(ice40_program_cram(&ice_pins, bitstream, bitstream_len, 3000));

/*
 * Write to SPI flash -- persistent, FPGA boots from flash on power-up.
 *
 * ice40_program_flash(pins, data, length, offset)
 *   offset: flash start address (usually 0)
 */
ESP_ERROR_CHECK(ice40_program_flash(&ice_pins, bitstream, bitstream_len, 0));

/*
 * Reset the FPGA (toggle CRESET, wait for CDONE).
 *
 * ice40_reset(pins, timeout_ms)
 *   timeout_ms: max wait for CDONE (0 to skip)
 */
ESP_ERROR_CHECK(ice40_reset(&ice_pins, 3000));
```

### Play an SVF file

SVF (Serial Vector Format) is a standard ASCII format for JTAG operations.
Tools like Lattice Diamond, Xilinx Vivado, and openFPGALoader can export `.svf` files.
The player streams through the file and executes each command via the JTAG transport.

```c
#include "jtag_svf.h"

/* SVF data -- from embedded binary, SPIFFS, HTTP download, PSRAM, etc. */
extern const char svf_data[] asm("_binary_program_svf_start");
extern const char svf_data_end[] asm("_binary_program_svf_end");

/* Optional progress callback -- called every ~100 commands */
void on_progress(size_t bytes_done, size_t bytes_total,
                 size_t commands_done, void *user_ctx) {
    int pct = (int)(bytes_done * 100 / bytes_total);
    printf("SVF: %d%% (%zu commands)\n", pct, commands_done);
}

svf_config_t cfg = {
    .progress_cb      = on_progress,   /* NULL to disable */
    .user_ctx         = NULL,          /* passed to callback */
    .stop_on_mismatch = false,         /* true = abort if TDO != expected */
};
svf_result_t result;

/*
 * svf_play(jtag_handle, svf_text, length, config, result)
 *   jtag_handle: from xmos_jtag_init() -- works with any JTAG device
 *   svf_text:    raw SVF file content (ASCII)
 *   length:      file size in bytes
 *   config:      playback options (NULL for defaults)
 *   result:      output stats (NULL to ignore)
 */
ESP_ERROR_CHECK(svf_play(
    jtag,
    svf_data, svf_data_end - svf_data,
    &cfg, &result
));

printf("SVF complete: %zu commands executed, %zu TDO mismatches\n",
       result.commands_executed, result.tdo_mismatches);
```

---

## Web Flasher Example

A ready-to-flash example with a browser UI lives in `example/`.

```sh
cd example

# Classic ESP32 (WROOM-32) -- WiFi AP
idf.py set-target esp32 && idf.py build flash monitor
# Connect to "XMOS-Flasher" (pw: xmosjtag), open http://192.168.4.1

# ESP32-S3 -- WiFi AP
idf.py set-target esp32s3 && idf.py build flash monitor

# ESP32-P4-NANO -- Ethernet (IP101GRI)
idf.py set-target esp32p4 && idf.py build flash monitor
# Plug in Ethernet, check serial log for DHCP address
```

Pin assignments are per target in [`example/main/main.c`](example/main/main.c)
(the `PIN_*` defines). Edit them there to match your wiring.

**What the UI does:**

| Section | |
|---|---|
| **Device** | One-click identify -- IDCODE, family, tiles, revision |
| **JTAG Chain** | Visual diagram: `TDI -> [XU316] -> [ECP5] -> TDO` |
| **Boundary Scan** | Live pin-state capture with hex + bit view, auto-refresh |
| **Firmware** | Drag-and-drop `.xe` / `.bin` / `.svf`, shows file details |
| **Program** | Select target (XMOS / iCE40 / SVF) and method (RAM / Flash / CRAM), one-click go |

---

## Wiring

```
ESP32              Target JTAG
──────             ───────────
GPIO  (TCK)   ──>  TCK
GPIO  (TMS)   ──>  TMS
GPIO  (TDI)   ──>  TDI         Use a level shifter if
GPIO  (TDO)   <──  TDO         target I/O != 3.3 V
GPIO  (TRST)  ──>  TRST_N      (active low, optional)
GPIO  (SRST)  ──>  RST_N       (open-drain, optional)
GND           ──>  GND
```

### Wiring to an XMOS xSYS connector (XU208, XU316, etc.)

XMOS dev boards and most XMOS audio products expose a 20-pin **xSYS** IDC
header (the connector the xTAG debugger plugs into). The naming in the
xTAG/xSYS manual is the confusing part, so here it is decoded.

**The one thing that trips everyone up:** XMOS calls the JTAG data lines
**TDSRC** and **TDSNK** (data *source* / *sink*), not TDI/TDO:

| xSYS name | = standard JTAG | Direction | Connect to ESP32 |
|---|---|---|---|
| **TDSRC** (Test Data *Source*) | **TDI** | host → target | ESP32 **TDI** (output) |
| **TDSNK** (Test Data *Sink*)   | **TDO** | target → host | ESP32 **TDO** (input)  |

There is **no TRST pin** on the xSYS connector — the XMOS TAP is reset by
clocking TMS, so leave the ESP32 `TRST` pin unconnected (`GPIO_NUM_NC`).
`RST_N` is the system reset (our `SRST`). `MSEL` selects boot-from-JTAG
(active low) and is well worth wiring for recovery — see the note below.

**xSYS 20-pin header pinout** (XM006125 xTAG v3.0 manual §3). Odd pins on
one row, even on the other; pin 1 is usually marked with a triangle/square
pad:

```
        ┌───────────────┐
   5V   1 ● ● 2   NC
 *MSEL  3 ● ● 4   GND
TDSRC   5 ● ● 6   XL1_UP1     ● = connect for JTAG
 (TDI)            (xlink, skip)
  TMS   7 ● ● 8   GND
  TCK   9 ● ● 10  XL1_UP0
DEBUG  11 ● ● 12  GND
TDSNK  13 ● ● 14  XL1_DN0
 (TDO)
 RST_N 15 ● ● 16  GND
UART_RX 17 ● ● 18 XL1_DN1
UART_TX 19 ● ● 20 GND
        └───────────────┘
```

Minimum connection set (six wires): **TCK (9), TMS (7), TDSRC/TDI (5),
TDSNK/TDO (13), RST_N (15), and one GND** (4/8/12/16/20 — tie at least one,
ideally several for signal return). Ignore the `XL1_*` xCONNECT-link and
`UART_*` pins for JTAG. Pin 1 (5V) is *target-to-host* power — do **not**
drive 5V into it from the ESP32.

> **MSEL (pin 3) for recovery:** tie it **low** (to a GND pin, or drive it
> low from a spare GPIO) to force the xCORE to boot-from-JTAG instead of
> running its (corrupt) flash. The core then parks in the boot ROM waiting
> for JTAG, which is far easier to halt than a crashed application — useful
> when "enter debug mode" fails. Release/float it for normal flash boot.

> XMOS JTAG is **3.3 V** (VDDIO) — wire straight to the ESP32, no level
> shifter (see the I/O-voltage note above).

### Classic ESP32 (WROOM-32) -- default pins

| Signal | GPIO | Notes |
|---|---|---|
| TCK | 23 | |
| TMS | 17 | WROVER uses 16/17 for PSRAM -- move if PSRAM is on |
| TDI | 16 | |
| TDO | 18 | input from target |
| TRST | 4 | optional, but tie high if your header exposes it |
| SRST | 19 | optional |

> Don't use GPIO6-11 (module SPI flash -- driving them crashes the chip)
> or GPIO12 (boot strapping pin) for JTAG on the classic ESP32. The
> defaults above avoid both.

### Waveshare ESP32-P4-NANO

<img src="docs/jtag_pinout.svg" alt="ESP32-P4-NANO Pinout" />

JTAG and SPI use separate headers so both can be wired simultaneously.

**JTAG -- Right Header** (rows 7-10)

| Signal | P4-NANO GPIO | Position | XMOS xSYS pin |
|---|---|---|---|
| TCK | 47 | Row 8 inner | 9 (TCK) |
| TMS | 48 | Row 8 outer | 7 (TMS) |
| TDI | 46 | Row 9 inner | **5 (TDSRC)** |
| TDO | 45 | Row 10 inner | **13 (TDSNK)** |
| TRST | 53 | Row 7 outer | *(none on xSYS — leave unconnected)* |
| SRST | 54 | Row 7 inner | 15 (RST_N) |
| GND | GND | — | 4/8/12/16/20 (≥1) |

So for a P4-NANO ↔ XMOS xSYS target, the six wires are:
GPIO47→pin9, GPIO48→pin7, GPIO46→pin5, GPIO45←pin13, GPIO54→pin15, and a
GND-to-GND. (GPIO53/TRST stays unconnected — XMOS has no TRST.) Optionally
tie xSYS pin 3 (MSEL) low for boot-from-JTAG recovery — see the xSYS note
above. The `PIN_*` defines are in
[`example/main/main.c`](example/main/main.c) if you wire differently.

**SPI / iCE40 -- Left Header** (rows 4, 6-8)

| Signal | GPIO | Position |
|---|---|---|
| SPI_CLK | 23 | Row 4 outer |
| SPI_MOSI | 5 | Row 6 outer |
| SPI_MISO | 4 | Row 6 inner |
| SPI_CS | 20 | Row 7 outer |
| ICE40_CRESET | 21 | Row 8 outer |
| ICE40_CDONE | 22 | Row 8 inner |

Ethernet (IP101GRI over RMII) handles networking on internal GPIOs. The **LDO_VO4** pin (Right Row 1, outer) can provide **1.8 V** for a JTAG level shifter in the rare case the target's JTAG bank runs at 1.8 V (XMOS parts do not -- their JTAG is on 3.3 V VDDIO).

---

## Architecture

```
components/xmos_jtag/
 include/
   xmos_jtag.h            XMOS API (chain scan, identify, bscan, load, flash)
   xmos_xe.h              XE / ELF parser
   jtag_svf.h             SVF player API
   jtag_ice40.h           iCE40 SPI programmer API
 src/
   jtag_transport.h        Backend vtable (shift_ir, shift_dr, reset, idle)
   jtag_gpio.c             GPIO bit-bang   -- any ESP32
   jtag_parlio.c           PARLIO + DMA    -- ESP32-P4
   svf_player.c            SVF file parser and executor
   ice40.c                 iCE40 CRAM + SPI flash programmer
   xmos_regs.h             XMOS TAP, MUX, PSWITCH, debug register map
   xmos_jtag.c             XMOS protocol layer + known-device database
   xmos_xe.c               XE container + ELF32 segment parser
```

### JTAG Backends

**GPIO bit-bang** -- portable `gpio_set_level` / `gpio_get_level`. Runs on every ESP32 variant at 1--5 MHz TCK.

**PARLIO DMA** (ESP32-P4) -- the Parallel IO peripheral clocks out TMS + TDI on two data lines while a second DMA channel captures TDO, all in hardware:

- TX `data_width = 2`, one byte = 4 JTAG cycles
- RX `data_width = 1`, clocked from TCK via GPIO matrix loopback
- Up to **40 MHz TCK** from the 160 MHz PLL

### Protocol Stack

The component is layered so device-specific logic sits on top of a generic JTAG transport:

```
 ┌─────────────┐  ┌──────────────┐  ┌─────────────┐
 │  XMOS xCORE │  │  SVF Player  │  │ iCE40 (SPI) │   Device-specific
 │  (MUX, DBG) │  │  (generic)   │  │             │
 └──────┬──────┘  └──────┬───────┘  └──────┬──────┘
        │                │                  │
 ┌──────┴────────────────┴──────────────────┘
 │          Generic JTAG Transport
 │    (shift_ir, shift_dr, reset, idle)
 ├─────────────────┬─────────────────┐
 │  GPIO bit-bang  │   PARLIO DMA    │               Hardware backends
 └─────────────────┴─────────────────┘
```

### XMOS Protocol

1. **Top-level TAP** -- 4-bit IR (IEEE 1149.1). IDCODE = `0x1`, SET_TEST_MODE = `0x8`.
2. **MUX** (IR `0x4`) -- opens the internal chain: OTP + xCORE + CHIP + BSCAN = 20-bit IR.
3. **Register access** -- xCORE TAP IR = `(reg << 2) | rw`. DR = 32/33 bits.
4. **Debug mode** -- enter via `PSWITCH_DBG_INT`, memory R/W through scratch-register mailbox.
5. **JTAG boot** -- SET_TEST_MODE bit 29, upload code, set PC, resume.

### XE File Format

Verified against `tool_axe` reference parser **and** real multi-tile satellite firmware:

- 12-byte sector header with uint64 length + padding descriptor
- Sector types: ELF `0x02`, BINARY `0x01`, GOTO `0x05`, CALL `0x06`, XN `0x08`
- Per-tile ELF32 with PT_LOAD segments; raw ELF accepted too

---

## Tests


<img width="1728" height="1030" alt="Logic analyzer capture" src="https://github.com/user-attachments/assets/e37209d1-0885-47a6-9ec6-7d385134b64d" />

**56 host-side tests** -- run on your dev machine, no hardware needed:

```sh
cd test && make test
```

Covers XE parsing (synthetic + real `.xe` files), ELF edge cases, JTAG chain-IR encoding (all 256 registers verified to fit 20 bits), MUX constants, PARLIO bit packing, TAP state-machine simulation, and structural validation of real firmware images.

---

## Known Limitations

- **XMOS debug commands** and PSWITCH register addresses were reverse-engineered from `sc_jtag` and forum posts -- may need tuning on untested silicon.
- **SVF player** and **iCE40 SPI programmer** are planned but not yet implemented.
- **XS3** IDCODE `0x00006633` confirmed from datasheet but end-to-end protocol not tested on hardware yet.

---

## License

MIT
