/*
 * Lattice iCE40 FPGA programmer -- SPI slave CRAM and flash programming.
 *
 * Reference: Lattice TN1248 "iCE40 Programming and Configuration"
 * Also informed by openFPGALoader ice40.cpp (Apache-2.0).
 */

#include "jtag_ice40.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "ice40";

/* -------------------------------------------------------------------------
 * SPI bit-bang helpers
 * ---------------------------------------------------------------------- */

static void spi_init(const ice40_pins_t *p)
{
    gpio_num_t outputs[] = { p->spi_cs, p->spi_clk, p->spi_mosi, p->creset };
    for (int i = 0; i < 4; i++) {
        if (outputs[i] == GPIO_NUM_NC) continue;
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << outputs[i],
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&cfg);
    }
    gpio_set_level(p->spi_cs, 1);
    gpio_set_level(p->spi_clk, 0);
    gpio_set_level(p->spi_mosi, 0);

    if (p->cdone != GPIO_NUM_NC) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << p->cdone,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&cfg);
    }

    if (p->spi_miso != GPIO_NUM_NC) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << p->spi_miso,
            .mode = GPIO_MODE_INPUT,
        };
        gpio_config(&cfg);
    }
}

/*
 * iCE40 slave configuration samples SI on the RISING edge of SCLK,
 * MSB first (TN1248).  Present the bit while the clock is low, then
 * generate the rising edge.
 */
static void spi_send_byte(const ice40_pins_t *p, uint8_t byte)
{
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(p->spi_mosi, (byte >> bit) & 1);
        gpio_set_level(p->spi_clk, 0);  /* ensure low phase, bit set up */
        gpio_set_level(p->spi_clk, 1);  /* rising edge -- device samples SI */
    }
}

static void spi_send_clocks(const ice40_pins_t *p, int count)
{
    gpio_set_level(p->spi_mosi, 0);
    for (int i = 0; i < count; i++) {
        gpio_set_level(p->spi_clk, 0);
        gpio_set_level(p->spi_clk, 1);
    }
}

static bool cdone_is_high(const ice40_pins_t *p)
{
    if (p->cdone == GPIO_NUM_NC) return true;  /* assume success */
    return gpio_get_level(p->cdone) != 0;
}

/* -------------------------------------------------------------------------
 * SPI flash helpers (for flash programming mode)
 * ---------------------------------------------------------------------- */

#define FLASH_CMD_WRITE_ENABLE    0x06
#define FLASH_CMD_READ_STATUS     0x05
#define FLASH_CMD_PAGE_PROGRAM    0x02
#define FLASH_CMD_SECTOR_ERASE    0x20
#define FLASH_CMD_READ_JEDEC      0x9F
#define FLASH_STATUS_WIP          0x01

/*
 * SPI mode 0 transfer: MOSI set while SCLK low, slave latches MOSI on the
 * rising edge.  MISO is stable from the previous falling edge through the
 * rising edge, so sample it after the rising edge -- sampling right after
 * our own falling edge would race the flash's clock-to-output delay.
 */
static uint8_t spi_xfer_byte(const ice40_pins_t *p, uint8_t out)
{
    uint8_t in = 0;
    for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(p->spi_mosi, (out >> bit) & 1);
        gpio_set_level(p->spi_clk, 1);
        if (p->spi_miso != GPIO_NUM_NC)
            in |= (gpio_get_level(p->spi_miso) << bit);
        gpio_set_level(p->spi_clk, 0);
    }
    return in;
}

static void flash_cs_low(const ice40_pins_t *p) { gpio_set_level(p->spi_cs, 0); }
static void flash_cs_high(const ice40_pins_t *p) { gpio_set_level(p->spi_cs, 1); }

/* Bounded poll of the flash status register (a missing flash would
 * otherwise spin forever). Returns false on timeout. */
static bool flash_wait_ready(const ice40_pins_t *p, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    bool ok = true;
    flash_cs_low(p);
    spi_xfer_byte(p, FLASH_CMD_READ_STATUS);
    while (spi_xfer_byte(p, 0) & FLASH_STATUS_WIP) {
        if (esp_timer_get_time() > deadline) { ok = false; break; }
    }
    flash_cs_high(p);
    return ok;
}

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

esp_err_t ice40_program_cram(const ice40_pins_t *pins,
                             const uint8_t *bitstream, size_t length,
                             uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Programming iCE40 CRAM: %zu bytes", length);

    spi_init(pins);

    /* 1. Assert CRESET low + SPI CS low */
    gpio_set_level(pins->spi_cs, 0);
    gpio_set_level(pins->creset, 0);
    esp_rom_delay_us(200);  /* min 200 ns */

    /* 2. Release CRESET, wait for CRAM clear */
    gpio_set_level(pins->creset, 1);
    esp_rom_delay_us(1200);  /* 800-1200 us for CRAM clear */

    /* 3. Send bitstream (MSB first per byte) */
    for (size_t i = 0; i < length; i++) {
        spi_send_byte(pins, bitstream[i]);

        if ((i & 0xFFF) == 0 && i > 0) {
            ESP_LOGD(TAG, "CRAM: %zu / %zu bytes", i, length);
        }
    }

    /* 4. Keep CS low and send dummy clocks until CDONE goes high.
     * TN1248: >= 49 SCLK cycles after the bitstream for CDONE, then
     * >= 49 additional cycles to complete user-mode wake-up.  CDONE only
     * advances while the device is being clocked, so clock while polling. */
    spi_send_clocks(pins, 56);

    if (timeout_ms > 0 && pins->cdone != GPIO_NUM_NC) {
        uint32_t elapsed = 0;
        while (!cdone_is_high(pins) && elapsed < timeout_ms) {
            spi_send_clocks(pins, 8);
            esp_rom_delay_us(1000);
            elapsed++;
        }
        if (!cdone_is_high(pins)) {
            gpio_set_level(pins->spi_cs, 1);
            ESP_LOGE(TAG, "CDONE did not go high after %lu ms", (unsigned long)timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
    }

    /* 5. Wake-up: >= 49 more clocks after CDONE, then release CS */
    spi_send_clocks(pins, 56);
    gpio_set_level(pins->spi_cs, 1);

    ESP_LOGI(TAG, "iCE40 CRAM programmed OK (%zu bytes)", length);
    return ESP_OK;
}

esp_err_t ice40_program_flash(const ice40_pins_t *pins,
                              const uint8_t *bitstream, size_t length,
                              uint32_t offset)
{
    ESP_LOGI(TAG, "Programming iCE40 SPI flash: %zu bytes at offset 0x%lx",
             length, (unsigned long)offset);

    spi_init(pins);

    /* Hold iCE40 in reset so it releases the SPI bus */
    gpio_set_level(pins->creset, 0);
    esp_rom_delay_us(10000);

    /* Verify flash is accessible */
    flash_cs_low(pins);
    spi_xfer_byte(pins, FLASH_CMD_READ_JEDEC);
    uint8_t mfg = spi_xfer_byte(pins, 0);
    uint8_t type = spi_xfer_byte(pins, 0);
    uint8_t cap = spi_xfer_byte(pins, 0);
    flash_cs_high(pins);

    ESP_LOGI(TAG, "Flash JEDEC: mfg=0x%02x type=0x%02x cap=0x%02x", mfg, type, cap);
    if (mfg == 0xFF || mfg == 0x00) {
        ESP_LOGE(TAG, "No SPI flash detected");
        gpio_set_level(pins->creset, 1);
        return ESP_ERR_NOT_FOUND;
    }

    /* Erase sectors */
    uint32_t erase_start = offset & ~0xFFF;
    uint32_t erase_end = (offset + length + 0xFFF) & ~0xFFF;
    for (uint32_t addr = erase_start; addr < erase_end; addr += 4096) {
        /* Write enable */
        flash_cs_low(pins); spi_xfer_byte(pins, FLASH_CMD_WRITE_ENABLE); flash_cs_high(pins);
        /* Sector erase */
        flash_cs_low(pins);
        spi_xfer_byte(pins, FLASH_CMD_SECTOR_ERASE);
        spi_xfer_byte(pins, (addr >> 16) & 0xFF);
        spi_xfer_byte(pins, (addr >> 8) & 0xFF);
        spi_xfer_byte(pins, addr & 0xFF);
        flash_cs_high(pins);
        if (!flash_wait_ready(pins, 2000)) {
            ESP_LOGE(TAG, "Flash erase timed out at 0x%lx", (unsigned long)addr);
            gpio_set_level(pins->creset, 1);
            return ESP_ERR_TIMEOUT;
        }
    }

    /* Program pages (256 bytes each) */
    for (size_t off = 0; off < length; off += 256) {
        size_t page_len = (length - off > 256) ? 256 : (length - off);
        uint32_t addr = offset + off;

        flash_cs_low(pins); spi_xfer_byte(pins, FLASH_CMD_WRITE_ENABLE); flash_cs_high(pins);

        flash_cs_low(pins);
        spi_xfer_byte(pins, FLASH_CMD_PAGE_PROGRAM);
        spi_xfer_byte(pins, (addr >> 16) & 0xFF);
        spi_xfer_byte(pins, (addr >> 8) & 0xFF);
        spi_xfer_byte(pins, addr & 0xFF);
        for (size_t i = 0; i < page_len; i++)
            spi_xfer_byte(pins, bitstream[off + i]);
        flash_cs_high(pins);
        if (!flash_wait_ready(pins, 500)) {
            ESP_LOGE(TAG, "Flash page write timed out at 0x%lx", (unsigned long)addr);
            gpio_set_level(pins->creset, 1);
            return ESP_ERR_TIMEOUT;
        }

        if ((off & 0xFFFF) == 0 && off > 0)
            ESP_LOGI(TAG, "Flash: %zu / %zu bytes", off, length);
    }

    /* Release iCE40 from reset -- it will boot from flash */
    gpio_set_level(pins->creset, 1);
    esp_rom_delay_us(12000);

    /* Check CDONE */
    if (pins->cdone != GPIO_NUM_NC) {
        esp_rom_delay_us(50000);
        if (!cdone_is_high(pins)) {
            ESP_LOGW(TAG, "CDONE not high after flash programming (device may need manual reset)");
        } else {
            ESP_LOGI(TAG, "iCE40 booted from flash OK");
        }
    }

    ESP_LOGI(TAG, "iCE40 SPI flash programmed OK (%zu bytes)", length);
    return ESP_OK;
}

esp_err_t ice40_reset(const ice40_pins_t *pins, uint32_t timeout_ms)
{
    spi_init(pins);
    gpio_set_level(pins->creset, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(pins->creset, 1);
    esp_rom_delay_us(12000);

    if (timeout_ms > 0 && pins->cdone != GPIO_NUM_NC) {
        uint32_t elapsed = 0;
        while (!cdone_is_high(pins) && elapsed < timeout_ms) {
            esp_rom_delay_us(1000);
            elapsed++;
        }
        if (!cdone_is_high(pins)) return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
