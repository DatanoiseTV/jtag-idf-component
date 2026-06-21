/*
 * SWD (Serial Wire Debug) -- ARM ADIv5 two-wire debug for Cortex-M targets.
 *
 * Bit-bangs SWCLK/SWDIO from an ESP32 to talk to STM32, GD32, RP2040, nRF,
 * SAMD and other Cortex-M devices: line reset + JTAG-to-SWD switch, DP/AP
 * register access (ACK + parity + turnaround), MEM-AP memory read/write, core
 * halt/run/reset, and device identification (DPIDR + CPUID + vendor IDCODE).
 *
 * Flashing is target-specific and lives in the per-family flash drivers
 * (swd_stm32.* etc.); this header is the transport + debug core they build on.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Pins / handle
 * ---------------------------------------------------------------------- */
typedef struct {
    gpio_num_t swclk;
    gpio_num_t swdio;
    gpio_num_t srst_n;   /* optional hardware reset (active low); GPIO_NUM_NC if unused */
} swd_pins_t;

typedef struct swd_ctx *swd_handle_t;

/* -------------------------------------------------------------------------
 * DP / AP register addresses (ADIv5)
 * ---------------------------------------------------------------------- */
#define SWD_DP_DPIDR     0x00   /* RO: debug port ID */
#define SWD_DP_ABORT     0x00   /* WO */
#define SWD_DP_CTRLSTAT  0x04   /* R/W (bank 0) */
#define SWD_DP_SELECT    0x08   /* WO: APSEL/APBANKSEL/DPBANKSEL */
#define SWD_DP_RDBUFF    0x0C   /* RO */

#define SWD_AP_CSW       0x00   /* MEM-AP control/status word */
#define SWD_AP_TAR       0x04   /* MEM-AP transfer address */
#define SWD_AP_DRW       0x0C   /* MEM-AP data read/write */
#define SWD_AP_IDR       0xFC   /* AP ID (bank 0xF) */

/* ACK responses */
#define SWD_ACK_OK       0x1
#define SWD_ACK_WAIT     0x2
#define SWD_ACK_FAULT    0x4

/* -------------------------------------------------------------------------
 * Identification
 * ---------------------------------------------------------------------- */
typedef struct {
    uint32_t    dpidr;        /* raw DPIDR */
    uint8_t     dp_version;   /* DPv0/1/2 */
    uint16_t    designer;     /* JEP106 designer of the DP (0x23B = ARM) */
    uint8_t     dp_partno;
    uint32_t    ap_idr;       /* MEM-AP IDR (0 if none found) */
    uint32_t    cpuid;        /* SCB CPUID (0xE000ED00) */
    uint16_t    core_partno;  /* CPUID PARTNO field */
    const char *core;         /* "Cortex-M0+", ... or "unknown" */
    uint32_t    dbg_idcode;   /* vendor debug IDCODE (e.g. STM32 DBGMCU 0xE0042000) */
    const char *vendor;       /* best-effort vendor/family name */
} swd_info_t;

/* -------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------- */
esp_err_t swd_init(const swd_pins_t *pins, swd_handle_t *out);
void      swd_deinit(swd_handle_t h);

/**
 * Line reset + JTAG-to-SWD switch + read DPIDR + power up the debug domain.
 * @param dpidr  receives the DPIDR (may be NULL)
 */
esp_err_t swd_connect(swd_handle_t h, uint32_t *dpidr);

/* -------------------------------------------------------------------------
 * Low-level DP / AP access
 * ---------------------------------------------------------------------- */
esp_err_t swd_dp_read(swd_handle_t h, uint8_t addr, uint32_t *val);
esp_err_t swd_dp_write(swd_handle_t h, uint8_t addr, uint32_t val);
esp_err_t swd_ap_read(swd_handle_t h, uint8_t ap, uint8_t addr, uint32_t *val);
esp_err_t swd_ap_write(swd_handle_t h, uint8_t ap, uint8_t addr, uint32_t val);

/* -------------------------------------------------------------------------
 * MEM-AP memory access (uses AP 0 unless changed via swd_set_ap)
 * ---------------------------------------------------------------------- */
void      swd_set_ap(swd_handle_t h, uint8_t ap);
esp_err_t swd_mem_read32(swd_handle_t h, uint32_t addr, uint32_t *val);
esp_err_t swd_mem_write32(swd_handle_t h, uint32_t addr, uint32_t val);
esp_err_t swd_mem_read(swd_handle_t h, uint32_t addr, void *buf, size_t len);
esp_err_t swd_mem_write(swd_handle_t h, uint32_t addr, const void *buf, size_t len);

/* -------------------------------------------------------------------------
 * Cortex-M core control (via the debug registers in the System Control Space)
 * ---------------------------------------------------------------------- */
esp_err_t swd_halt(swd_handle_t h);
esp_err_t swd_run(swd_handle_t h);
esp_err_t swd_reset_halt(swd_handle_t h);   /* system reset, halt at the reset vector */
bool      swd_is_halted(swd_handle_t h);

/* -------------------------------------------------------------------------
 * Identify
 * ---------------------------------------------------------------------- */
esp_err_t swd_identify(swd_handle_t h, swd_info_t *info);

#ifdef __cplusplus
}
#endif
