/*
 * GPIO bit-bang JTAG transport backend.
 *
 * Works on all ESP32 variants.  Typical throughput 1-5 MHz TCK depending
 * on CPU clock.  Uses direct GPIO register writes for speed.
 */

#include "jtag_transport.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "jtag_gpio";

/* -------------------------------------------------------------------------
 * Private context
 * ---------------------------------------------------------------------- */
typedef struct {
    jtag_transport_t base;
    xmos_jtag_pins_t pins;
    /* Cached register pointers for fast GPIO writes */
    volatile uint32_t *tck_set;
    volatile uint32_t *tck_clr;
    volatile uint32_t *tms_set;
    volatile uint32_t *tms_clr;
    volatile uint32_t *tdi_set;
    volatile uint32_t *tdi_clr;
    uint32_t tck_mask;
    uint32_t tms_mask;
    uint32_t tdi_mask;
    uint32_t tdo_mask;
    uint32_t tck_delay_ns;  /* inter-edge delay for frequency control */
} jtag_gpio_ctx_t;

/* -------------------------------------------------------------------------
 * Fast GPIO helpers
 * ---------------------------------------------------------------------- */
static inline void tck_low(jtag_gpio_ctx_t *ctx)
{
    *ctx->tck_clr = ctx->tck_mask;
}

static inline void tck_high(jtag_gpio_ctx_t *ctx)
{
    *ctx->tck_set = ctx->tck_mask;
}

static inline void tms_set(jtag_gpio_ctx_t *ctx, int val)
{
    if (val)
        *ctx->tms_set = ctx->tms_mask;
    else
        *ctx->tms_clr = ctx->tms_mask;
}

static inline void tdi_set(jtag_gpio_ctx_t *ctx, int val)
{
    if (val)
        *ctx->tdi_set = ctx->tdi_mask;
    else
        *ctx->tdi_clr = ctx->tdi_mask;
}

static inline int tdo_read(jtag_gpio_ctx_t *ctx)
{
    return (GPIO.in.val >> ctx->pins.tdo) & 1;
}

#if SOC_GPIO_PIN_COUNT > 32
static inline int tdo_read_high(jtag_gpio_ctx_t *ctx)
{
    return (GPIO.in1.val >> (ctx->pins.tdo - 32)) & 1;
}
#endif

static inline int read_tdo(jtag_gpio_ctx_t *ctx)
{
#if SOC_GPIO_PIN_COUNT > 32
    if (ctx->pins.tdo >= 32)
        return tdo_read_high(ctx);
#endif
    return tdo_read(ctx);
}

/**
 * Clock one JTAG cycle: set TMS and TDI, pulse TCK, return TDO.
 *
 * JTAG spec: TMS and TDI are sampled on rising edge of TCK.
 * TDO changes on falling edge and is read before the next rising edge.
 */
static inline int jtag_clock(jtag_gpio_ctx_t *ctx, int tms, int tdi)
{
    tms_set(ctx, tms);
    tdi_set(ctx, tdi);

    /* Rising edge -- device samples TMS/TDI */
    tck_high(ctx);

    /* Brief delay (the loop overhead is usually enough for <5 MHz) */

    /* Falling edge -- device updates TDO */
    tck_low(ctx);

    return read_tdo(ctx);
}

/* -------------------------------------------------------------------------
 * TAP state navigation
 * ---------------------------------------------------------------------- */

static esp_err_t gpio_reset(jtag_transport_t *self)
{
    jtag_gpio_ctx_t *ctx = (jtag_gpio_ctx_t *)self;

    /* If TRST is available, pulse it */
    if (ctx->pins.trst_n != GPIO_NUM_NC) {
        gpio_set_level(ctx->pins.trst_n, 0);
        esp_rom_delay_us(10);
        gpio_set_level(ctx->pins.trst_n, 1);
        esp_rom_delay_us(10);
    }

    /* Drive TMS=1 for 6 cycles to reach Test-Logic-Reset from any state */
    for (int i = 0; i < 6; i++) {
        jtag_clock(ctx, 1, 0);
    }

    /* Move to Run-Test/Idle: TMS=0 for 1 cycle */
    jtag_clock(ctx, 0, 0);

    return ESP_OK;
}

/**
 * Shift bits through IR or DR.
 *
 * @param is_ir   true = IR, false = DR
 * @param tdi     Input data (LSB-first), ceil(bits/8) bytes
 * @param tdo     Output buffer or NULL
 * @param bits    Number of bits to shift
 *
 * Navigation: RTI -> Select-xR -> Capture-xR -> Shift-xR (shift bits)
 *             -> Exit1-xR -> Update-xR -> RTI
 */
static esp_err_t gpio_shift(jtag_gpio_ctx_t *ctx, bool is_ir,
                            const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    if (bits == 0)
        return ESP_OK;

    /* RTI -> Select-DR-Scan (TMS=1) */
    jtag_clock(ctx, 1, 0);

    if (is_ir) {
        /* Select-DR -> Select-IR-Scan (TMS=1) */
        jtag_clock(ctx, 1, 0);
    }

    /* Capture (TMS=0) */
    jtag_clock(ctx, 0, 0);

    /* Shift (TMS=0 for all but last bit) */
    if (tdo)
        memset(tdo, 0, (bits + 7) / 8);

    for (size_t i = 0; i < bits; i++) {
        int is_last = (i == bits - 1);
        int di = (tdi[i / 8] >> (i % 8)) & 1;
        int dout = jtag_clock(ctx, is_last ? 1 : 0, di);

        if (tdo)
            tdo[i / 8] |= (dout << (i % 8));
    }

    /* Now in Exit1-xR. Go to Update-xR (TMS=1), then RTI (TMS=0) */
    jtag_clock(ctx, 1, 0);  /* Update */
    jtag_clock(ctx, 0, 0);  /* RTI */

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
    for (unsigned i = 0; i < cycles; i++) {
        jtag_clock(ctx, 0, 0);
    }
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
 * GPIO set/clear register resolution
 * ---------------------------------------------------------------------- */
static void resolve_gpio_regs(gpio_num_t pin,
                              volatile uint32_t **set_reg,
                              volatile uint32_t **clr_reg,
                              uint32_t *mask)
{
#if SOC_GPIO_PIN_COUNT > 32
    if (pin >= 32) {
        *set_reg = (volatile uint32_t *)&GPIO.out1_w1ts.val;
        *clr_reg = (volatile uint32_t *)&GPIO.out1_w1tc.val;
        *mask = 1u << (pin - 32);
        return;
    }
#endif
    *set_reg = (volatile uint32_t *)&GPIO.out_w1ts;
    *clr_reg = (volatile uint32_t *)&GPIO.out_w1tc;
    *mask = 1u << pin;
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

    /* Configure TDO as input */
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

    /* Optional reset pins */
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
            gpio_set_level(rst_pins[i], 1);  /* De-assert (active low) */
        }
    }

    /* Cache GPIO register pointers for fast toggling */
    resolve_gpio_regs(pins->tck, &ctx->tck_set, &ctx->tck_clr, &ctx->tck_mask);
    resolve_gpio_regs(pins->tms, &ctx->tms_set, &ctx->tms_clr, &ctx->tms_mask);
    resolve_gpio_regs(pins->tdi, &ctx->tdi_set, &ctx->tdi_clr, &ctx->tdi_mask);
    ctx->tdo_mask = 1u << (pins->tdo % 32);

    /* Vtable */
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
