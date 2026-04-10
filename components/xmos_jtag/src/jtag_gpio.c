/*
 * GPIO bit-bang JTAG transport backend.
 *
 * Works on all ESP32 variants. Uses standard gpio_set_level / gpio_get_level
 * which are fast enough for JTAG bit-bang (1-5 MHz TCK).
 */

#include "jtag_transport.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "jtag_gpio";

/* -------------------------------------------------------------------------
 * Private context
 * ---------------------------------------------------------------------- */
typedef struct {
    jtag_transport_t base;
    xmos_jtag_pins_t pins;
} jtag_gpio_ctx_t;

/* -------------------------------------------------------------------------
 * One JTAG clock cycle
 *
 * JTAG spec: TMS and TDI are sampled on rising edge of TCK.
 * TDO changes on falling edge and is valid before next rising edge.
 * ---------------------------------------------------------------------- */
static inline int jtag_clock(jtag_gpio_ctx_t *ctx, int tms, int tdi)
{
    gpio_set_level(ctx->pins.tms, tms);
    gpio_set_level(ctx->pins.tdi, tdi);

    /* Rising edge -- device samples TMS/TDI */
    gpio_set_level(ctx->pins.tck, 1);

    /* Falling edge -- device updates TDO */
    gpio_set_level(ctx->pins.tck, 0);

    return gpio_get_level(ctx->pins.tdo);
}

/* -------------------------------------------------------------------------
 * TAP state navigation
 * ---------------------------------------------------------------------- */

static esp_err_t gpio_reset(jtag_transport_t *self)
{
    jtag_gpio_ctx_t *ctx = (jtag_gpio_ctx_t *)self;

    if (ctx->pins.trst_n != GPIO_NUM_NC) {
        gpio_set_level(ctx->pins.trst_n, 0);
        esp_rom_delay_us(10);
        gpio_set_level(ctx->pins.trst_n, 1);
        esp_rom_delay_us(10);
    }

    /* TMS=1 for 6 cycles -> Test-Logic-Reset from any state */
    for (int i = 0; i < 6; i++)
        jtag_clock(ctx, 1, 0);

    /* TMS=0 -> Run-Test/Idle */
    jtag_clock(ctx, 0, 0);

    return ESP_OK;
}

/**
 * Shift bits through IR or DR.
 *
 * Navigation: RTI -> Select-xR -> Capture-xR -> Shift-xR (shift bits)
 *             -> Exit1-xR -> Update-xR -> RTI
 */
static esp_err_t gpio_shift(jtag_gpio_ctx_t *ctx, bool is_ir,
                            const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    if (bits == 0)
        return ESP_OK;

    /* RTI -> Select-DR-Scan */
    jtag_clock(ctx, 1, 0);

    if (is_ir) {
        /* Select-DR -> Select-IR-Scan */
        jtag_clock(ctx, 1, 0);
    }

    /* Capture (TMS=0) */
    jtag_clock(ctx, 0, 0);

    if (tdo)
        memset(tdo, 0, (bits + 7) / 8);

    for (size_t i = 0; i < bits; i++) {
        int is_last = (i == bits - 1);
        int di = (tdi[i / 8] >> (i % 8)) & 1;
        int dout = jtag_clock(ctx, is_last ? 1 : 0, di);

        if (tdo)
            tdo[i / 8] |= (dout << (i % 8));
    }

    /* Exit1-xR -> Update-xR (TMS=1) -> RTI (TMS=0) */
    jtag_clock(ctx, 1, 0);
    jtag_clock(ctx, 0, 0);

    return ESP_OK;
}

static esp_err_t gpio_shift_ir(jtag_transport_t *self,
                               const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    return gpio_shift((jtag_gpio_ctx_t *)self, true, tdi, tdo, bits);
}

static esp_err_t gpio_shift_dr(jtag_transport_t *self,
                               const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    return gpio_shift((jtag_gpio_ctx_t *)self, false, tdi, tdo, bits);
}

static esp_err_t gpio_idle(jtag_transport_t *self, unsigned cycles)
{
    jtag_gpio_ctx_t *ctx = (jtag_gpio_ctx_t *)self;
    for (unsigned i = 0; i < cycles; i++)
        jtag_clock(ctx, 0, 0);
    return ESP_OK;
}

static void gpio_free(jtag_transport_t *self)
{
    jtag_gpio_ctx_t *ctx = (jtag_gpio_ctx_t *)self;
    gpio_reset_pin(ctx->pins.tck);
    gpio_reset_pin(ctx->pins.tms);
    gpio_reset_pin(ctx->pins.tdi);
    gpio_reset_pin(ctx->pins.tdo);
    if (ctx->pins.trst_n != GPIO_NUM_NC)
        gpio_reset_pin(ctx->pins.trst_n);
    if (ctx->pins.srst_n != GPIO_NUM_NC)
        gpio_reset_pin(ctx->pins.srst_n);
    free(ctx);
}

/* -------------------------------------------------------------------------
 * Constructor
 * ---------------------------------------------------------------------- */
esp_err_t jtag_transport_gpio_create(const xmos_jtag_pins_t *pins,
                                     jtag_transport_t **out)
{
    jtag_gpio_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    ctx->pins = *pins;

    /* Configure output pins */
    gpio_num_t outputs[] = { pins->tck, pins->tms, pins->tdi };
    for (int i = 0; i < 3; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << outputs[i],
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) {
            free(ctx);
            return err;
        }
        gpio_set_level(outputs[i], 0);
    }

    /* TDO as input */
    gpio_config_t tdo_cfg = {
        .pin_bit_mask = 1ULL << pins->tdo,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&tdo_cfg);
    if (err != ESP_OK) {
        free(ctx);
        return err;
    }

    /* Optional reset pins (open-drain) */
    gpio_num_t rst_pins[] = { pins->trst_n, pins->srst_n };
    for (int i = 0; i < 2; i++) {
        if (rst_pins[i] != GPIO_NUM_NC) {
            gpio_config_t cfg = {
                .pin_bit_mask = 1ULL << rst_pins[i],
                .mode = GPIO_MODE_OUTPUT_OD,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            gpio_config(&cfg);
            gpio_set_level(rst_pins[i], 1);
        }
    }

    ctx->base.reset    = gpio_reset;
    ctx->base.shift_ir = gpio_shift_ir;
    ctx->base.shift_dr = gpio_shift_dr;
    ctx->base.idle     = gpio_idle;
    ctx->base.free     = gpio_free;

    ESP_LOGI(TAG, "GPIO JTAG: TCK=%d TMS=%d TDI=%d TDO=%d",
             pins->tck, pins->tms, pins->tdi, pins->tdo);

    *out = &ctx->base;
    return ESP_OK;
}
