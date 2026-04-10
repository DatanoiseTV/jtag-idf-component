/*
 * XMOS JTAG Programmer for ESP-IDF
 *
 * Loads firmware to XMOS xCORE-200 (XU21x) and xCORE.ai (XU3xx) devices
 * via JTAG from an ESP32.
 *
 * Backends:
 *   - GPIO bit-bang  : works on any ESP32 variant (ESP32, S2, S3, C3, ...)
 *   - PARLIO with DMA: high-speed on ESP32-P4 (up to ~40 MHz TCK)
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
 * Chip family
 * ---------------------------------------------------------------------- */
typedef enum {
    XMOS_FAMILY_UNKNOWN = 0,
    XMOS_FAMILY_XS1,
    XMOS_FAMILY_XS2,       /* xCORE-200  (XU21x) */
    XMOS_FAMILY_XS3,       /* xCORE.ai   (XU3xx) */
} xmos_family_t;

/* -------------------------------------------------------------------------
 * Pin configuration
 * ---------------------------------------------------------------------- */
typedef struct {
    gpio_num_t tck;
    gpio_num_t tms;
    gpio_num_t tdi;         /* ESP32 -> XMOS */
    gpio_num_t tdo;         /* XMOS -> ESP32 */
    gpio_num_t trst_n;      /* TAP reset, active low.  GPIO_NUM_NC if unused */
    gpio_num_t srst_n;      /* System reset, active low. GPIO_NUM_NC if unused */
} xmos_jtag_pins_t;

/* -------------------------------------------------------------------------
 * Handle
 * ---------------------------------------------------------------------- */
typedef struct xmos_jtag_ctx *xmos_jtag_handle_t;

/* -------------------------------------------------------------------------
 * Chip information
 * ---------------------------------------------------------------------- */
typedef struct {
    uint32_t      idcode;
    xmos_family_t family;
    uint8_t       num_tiles;
    uint8_t       revision;
} xmos_chip_info_t;

/* -------------------------------------------------------------------------
 * Initialisation / teardown
 * ---------------------------------------------------------------------- */

/**
 * Initialise the XMOS JTAG programmer.
 *
 * Selects the backend configured in Kconfig (GPIO or PARLIO).
 */
esp_err_t xmos_jtag_init(const xmos_jtag_pins_t *pins,
                         xmos_jtag_handle_t *out_handle);

void xmos_jtag_deinit(xmos_jtag_handle_t handle);

/* -------------------------------------------------------------------------
 * Device identification
 * ---------------------------------------------------------------------- */

/**
 * Reset the TAP, read IDCODE, and identify the XMOS device.
 */
esp_err_t xmos_jtag_identify(xmos_jtag_handle_t handle,
                             xmos_chip_info_t *chip_info);

/* -------------------------------------------------------------------------
 * Register access
 * ---------------------------------------------------------------------- */

/**
 * Read a PSWITCH/SSWITCH register.
 * @param tile  0-based tile index, or -1 for system switch
 */
esp_err_t xmos_jtag_read_reg(xmos_jtag_handle_t handle,
                             int tile, uint8_t reg, uint32_t *value);

esp_err_t xmos_jtag_write_reg(xmos_jtag_handle_t handle,
                              int tile, uint8_t reg, uint32_t value);

/* -------------------------------------------------------------------------
 * Memory access (core must be in debug mode)
 * ---------------------------------------------------------------------- */

/**
 * Write a block to xCORE RAM via JTAG debug interface.
 * @param addr  Word-aligned target address
 * @param len   Byte count (rounded up to words internally)
 */
esp_err_t xmos_jtag_mem_write(xmos_jtag_handle_t handle,
                              int tile, uint32_t addr,
                              const void *data, size_t len);

esp_err_t xmos_jtag_mem_read(xmos_jtag_handle_t handle,
                             int tile, uint32_t addr,
                             void *data, size_t len);

/* -------------------------------------------------------------------------
 * High-level firmware loading
 * ---------------------------------------------------------------------- */

/**
 * Load a raw binary into xCORE RAM and optionally start execution.
 *
 * Full JTAG boot sequence: reset, set JTAG boot mode, enter debug,
 * write image, set PC, resume.
 *
 * @param entry_point  Entry point address, or 0 to leave core halted
 */
esp_err_t xmos_jtag_load_raw(xmos_jtag_handle_t handle,
                             int tile,
                             const uint8_t *image, size_t image_len,
                             uint32_t load_addr, uint32_t entry_point);

/**
 * Parse an XE file and load all segments to the appropriate tiles.
 */
esp_err_t xmos_jtag_load_xe(xmos_jtag_handle_t handle,
                            const uint8_t *xe_data, size_t xe_len,
                            bool run);

/* -------------------------------------------------------------------------
 * SPI flash programming via JTAG-loaded stub
 * ---------------------------------------------------------------------- */

/**
 * Program the XMOS SPI flash with a pre-formatted boot image.
 *
 * Uploads a flash-programmer stub to xCORE RAM, runs it, then streams
 * flash data via the JTAG debug mailbox.
 *
 * The image should be created with:
 *   xflash --factory <app.xe> -o image.bin
 *
 * @param stub      User-supplied stub binary, or NULL for built-in stub
 * @param stub_len  Length of stub (ignored if stub == NULL)
 */
esp_err_t xmos_jtag_program_flash(xmos_jtag_handle_t handle,
                                  const uint8_t *flash_image,
                                  size_t flash_image_len,
                                  const uint8_t *stub, size_t stub_len);

/* -------------------------------------------------------------------------
 * Direct SPI flash programming (ESP32 drives SPI bus directly)
 * ---------------------------------------------------------------------- */

typedef struct {
    gpio_num_t cs;
    gpio_num_t clk;
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t wp;          /* GPIO_NUM_NC if unused */
    gpio_num_t hold;        /* GPIO_NUM_NC if unused */
} xmos_spi_pins_t;

/**
 * Program SPI flash directly while holding XMOS in reset.
 * Board must allow shared SPI bus access.
 *
 * @param offset  Flash address to write at (usually 0)
 */
esp_err_t xmos_spi_flash_program(xmos_jtag_handle_t handle,
                                 const xmos_spi_pins_t *spi_pins,
                                 const uint8_t *image, size_t image_len,
                                 uint32_t offset);

#ifdef __cplusplus
}
#endif
