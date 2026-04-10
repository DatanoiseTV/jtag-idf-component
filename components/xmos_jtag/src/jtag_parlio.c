/*
 * PARLIO DMA JTAG transport backend for ESP32-P4.
 *
 * Uses the Parallel IO peripheral to blast JTAG data via DMA:
 *   TX: data_width=2 (TMS on bit 0, TDI on bit 1), clk_out = TCK
 *   RX: data_width=1 (TDO), clocked from TCK via GPIO matrix loopback
 *
 * Bit packing (LSB first, data_width=2):
 *   Each byte in the TX DMA buffer encodes 4 JTAG clock cycles:
 *     bit 0 = TMS cycle 0,  bit 1 = TDI cycle 0
 *     bit 2 = TMS cycle 1,  bit 3 = TDI cycle 1
 *     bit 4 = TMS cycle 2,  bit 5 = TDI cycle 2
 *     bit 6 = TMS cycle 3,  bit 7 = TDI cycle 3
 *
 * RX (data_width=1): each byte = 8 consecutive TDO samples (LSB first).
 *
 * payload_bits for parlio_tx_unit_transmit = total bits in buffer
 *   = num_cycles * data_width = num_cycles * 2
 * payload_bits must be a multiple of data_width.
 */

#include "sdkconfig.h"

#if CONFIG_XMOS_JTAG_BACKEND_PARLIO

#include "jtag_transport.h"
#include "driver/parlio_tx.h"
#include "driver/parlio_rx.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "jtag_parlio";

/* -------------------------------------------------------------------------
 * Context
 * ---------------------------------------------------------------------- */
typedef struct {
    jtag_transport_t       base;
    xmos_jtag_pins_t       pins;
    parlio_tx_unit_handle_t tx;
    parlio_rx_unit_handle_t rx;
    parlio_rx_delimiter_handle_t rx_delim;
    uint32_t               tck_freq_hz;
    uint8_t               *tx_buf;
    uint8_t               *rx_buf;
    size_t                 buf_size;     /* bytes, DMA buffer size */
} jtag_parlio_ctx_t;

/* -------------------------------------------------------------------------
 * TX buffer encoding
 * ---------------------------------------------------------------------- */
static inline uint8_t pack_cycle(int tms, int tdi)
{
    return (uint8_t)((tms & 1) | ((tdi & 1) << 1));
}

/**
 * Build a JTAG shift sequence into the TX buffer.
 * Returns total TCK cycles written.
 */
static size_t build_shift_sequence(bool is_ir, const uint8_t *tdi,
                                   size_t bits, uint8_t *tx_buf)
{
    size_t cycle = 0;

    #define EMIT(tms_val, tdi_val) do {                         \
        size_t byte_idx = cycle / 4;                            \
        size_t shift = (cycle % 4) * 2;                         \
        if (shift == 0) tx_buf[byte_idx] = 0;                  \
        tx_buf[byte_idx] |= pack_cycle(tms_val, tdi_val) << shift; \
        cycle++;                                                \
    } while (0)

    /* RTI -> Select-DR-Scan (TMS=1) */
    EMIT(1, 0);
    if (is_ir) {
        /* Select-DR -> Select-IR-Scan (TMS=1) */
        EMIT(1, 0);
    }
    /* Capture-xR (TMS=0) */
    EMIT(0, 0);
    /* Shift data bits: TMS=0 for all but last, TMS=1 for last */
    for (size_t i = 0; i < bits; i++) {
        int is_last = (i == bits - 1);
        int di = tdi ? ((tdi[i / 8] >> (i % 8)) & 1) : 0;
        EMIT(is_last ? 1 : 0, di);
    }
    /* Exit1-xR -> Update-xR (TMS=1) */
    EMIT(1, 0);
    /* Update-xR -> RTI (TMS=0) */
    EMIT(0, 0);

    #undef EMIT
    return cycle;
}

/**
 * Extract TDO data from the RX buffer.
 * RX captures one TDO bit per cycle (8 per byte, LSB first).
 */
static void extract_tdo(const uint8_t *rx_buf, size_t nav_cycles,
                        uint8_t *tdo, size_t bits)
{
    memset(tdo, 0, (bits + 7) / 8);
    for (size_t i = 0; i < bits; i++) {
        size_t rx_cycle = nav_cycles + i;
        int bit = (rx_buf[rx_cycle / 8] >> (rx_cycle % 8)) & 1;
        tdo[i / 8] |= (bit << (i % 8));
    }
}

/* -------------------------------------------------------------------------
 * Transport interface
 * ---------------------------------------------------------------------- */

static esp_err_t parlio_reset(jtag_transport_t *self)
{
    jtag_parlio_ctx_t *ctx = (jtag_parlio_ctx_t *)self;

    if (ctx->pins.trst_n != GPIO_NUM_NC) {
        gpio_set_level(ctx->pins.trst_n, 0);
        esp_rom_delay_us(10);
        gpio_set_level(ctx->pins.trst_n, 1);
        esp_rom_delay_us(10);
    }

    /* Build 7 cycles: 6x TMS=1 (reset), 1x TMS=0 (RTI) */
    memset(ctx->tx_buf, 0, 4);
    size_t cycle = 0;
    for (int i = 0; i < 6; i++) {
        size_t bi = cycle / 4, sh = (cycle % 4) * 2;
        if (sh == 0) ctx->tx_buf[bi] = 0;
        ctx->tx_buf[bi] |= pack_cycle(1, 0) << sh;
        cycle++;
    }
    {
        size_t bi = cycle / 4, sh = (cycle % 4) * 2;
        if (sh == 0) ctx->tx_buf[bi] = 0;
        ctx->tx_buf[bi] |= pack_cycle(0, 0) << sh;
        cycle++;
    }

    /* payload_bits = total_cycles * data_width = 7 * 2 = 14
     * Must be multiple of data_width (2) -- 14 is fine */
    parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
    esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                            cycle * 2, &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reset TX failed: %s", esp_err_to_name(err));
        return err;
    }
    return parlio_tx_unit_wait_all_done(ctx->tx, 1000);
}

static esp_err_t parlio_do_shift(jtag_parlio_ctx_t *ctx, bool is_ir,
                                 const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    if (bits == 0) return ESP_OK;

    size_t nav_pre = is_ir ? 3 : 2;  /* cycles before first data bit */
    size_t total_cycles = nav_pre + bits + 2;  /* +2 for update+RTI */
    size_t tx_bytes = (total_cycles + 3) / 4;  /* 4 cycles per byte */
    size_t rx_bytes = (total_cycles + 7) / 8;  /* 8 samples per byte */

    if (tx_bytes > ctx->buf_size) {
        ESP_LOGE(TAG, "TX too large: %zu bytes, buf %zu", tx_bytes, ctx->buf_size);
        return ESP_ERR_NO_MEM;
    }

    /* Build TX buffer */
    memset(ctx->tx_buf, 0, tx_bytes);
    build_shift_sequence(is_ir, tdi, bits, ctx->tx_buf);

    /* Set up RX before TX (RX must be ready when clock starts) */
    if (tdo) {
        memset(ctx->rx_buf, 0, ctx->buf_size);

        /* Start soft delimiter first */
        esp_err_t err = parlio_rx_soft_delimiter_start_stop(
            ctx->rx, ctx->rx_delim, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RX delim start failed: %s", esp_err_to_name(err));
            return err;
        }

        /* Queue RX receive -- buffer must be >= eof_data_len */
        parlio_receive_config_t rx_cfg = {
            .delimiter = ctx->rx_delim,
        };
        err = parlio_rx_unit_receive(ctx->rx, ctx->rx_buf,
                                     ctx->buf_size, &rx_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "RX receive failed: %s", esp_err_to_name(err));
            parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, false);
            return err;
        }
    }

    /* Fire TX -- this generates the clock and data */
    parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
    size_t payload_bits = total_cycles * 2;  /* data_width=2 */
    esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                            payload_bits, &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX transmit failed: %s", esp_err_to_name(err));
        if (tdo) parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, false);
        return err;
    }

    /* Wait for TX to finish */
    err = parlio_tx_unit_wait_all_done(ctx->tx, 5000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX wait failed: %s", esp_err_to_name(err));
        if (tdo) parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, false);
        return err;
    }

    if (tdo) {
        /* Stop RX and wait for any remaining data */
        parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, false);
        parlio_rx_unit_wait_all_done(ctx->rx, 1000);

        /* Extract data bits, skipping navigation cycles */
        extract_tdo(ctx->rx_buf, nav_pre, tdo, bits);
    }

    return ESP_OK;
}

static esp_err_t parlio_shift_ir(jtag_transport_t *self,
                                 const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    return parlio_do_shift((jtag_parlio_ctx_t *)self, true, tdi, tdo, bits);
}

static esp_err_t parlio_shift_dr(jtag_transport_t *self,
                                 const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    return parlio_do_shift((jtag_parlio_ctx_t *)self, false, tdi, tdo, bits);
}

static esp_err_t parlio_idle(jtag_transport_t *self, unsigned cycles)
{
    jtag_parlio_ctx_t *ctx = (jtag_parlio_ctx_t *)self;
    if (cycles == 0) return ESP_OK;

    /* Clamp to buffer capacity (4 cycles per byte) */
    while (cycles > 0) {
        unsigned chunk = cycles;
        if (chunk > ctx->buf_size * 4)
            chunk = ctx->buf_size * 4;

        size_t tx_bytes = (chunk + 3) / 4;
        memset(ctx->tx_buf, 0, tx_bytes);  /* TMS=0, TDI=0 for all cycles */

        parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
        esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                                chunk * 2, &tx_cfg);
        if (err != ESP_OK) return err;
        err = parlio_tx_unit_wait_all_done(ctx->tx, 5000);
        if (err != ESP_OK) return err;

        cycles -= chunk;
    }
    return ESP_OK;
}

static void parlio_free_transport(jtag_transport_t *self)
{
    jtag_parlio_ctx_t *ctx = (jtag_parlio_ctx_t *)self;
    if (ctx->rx)
        parlio_del_rx_unit(ctx->rx);
    if (ctx->rx_delim)
        parlio_del_rx_delimiter(ctx->rx_delim);
    if (ctx->tx)
        parlio_del_tx_unit(ctx->tx);
    if (ctx->tx_buf) heap_caps_free(ctx->tx_buf);
    if (ctx->rx_buf) heap_caps_free(ctx->rx_buf);

    gpio_num_t rst[] = { ctx->pins.trst_n, ctx->pins.srst_n };
    for (int i = 0; i < 2; i++) {
        if (rst[i] != GPIO_NUM_NC) gpio_reset_pin(rst[i]);
    }
    free(ctx);
}

/* -------------------------------------------------------------------------
 * Constructor
 * ---------------------------------------------------------------------- */
esp_err_t jtag_transport_parlio_create(const xmos_jtag_pins_t *pins,
                                       uint32_t tck_freq_hz,
                                       jtag_transport_t **out)
{
    esp_err_t err;
    jtag_parlio_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) return ESP_ERR_NO_MEM;

    ctx->pins = *pins;
    ctx->tck_freq_hz = tck_freq_hz;
    ctx->buf_size = CONFIG_XMOS_JTAG_PARLIO_DMA_BUF_SIZE;

    /* Allocate DMA-capable buffers (internal RAM required for DMA) */
    ctx->tx_buf = heap_caps_calloc(1, ctx->buf_size,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    ctx->rx_buf = heap_caps_calloc(1, ctx->buf_size,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
    if (!ctx->tx_buf || !ctx->rx_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers (%zu bytes each)", ctx->buf_size);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    /* ----- TX unit: TCK clock + 2 data lines (TMS, TDI) ----- */
    parlio_tx_unit_config_t tx_cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = tck_freq_hz,
        .data_width = 2,
        .clk_out_gpio_num = pins->tck,
        .valid_gpio_num = -1,
        .data_gpio_nums = { [0] = pins->tms, [1] = pins->tdi },
        .trans_queue_depth = 4,
        .max_transfer_size = ctx->buf_size,
        .sample_edge = PARLIO_SAMPLE_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (int i = 2; i < 16; i++)
        tx_cfg.data_gpio_nums[i] = -1;

    err = parlio_new_tx_unit(&tx_cfg, &ctx->tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX unit create failed: %s", esp_err_to_name(err));
        goto fail;
    }

    /* ----- RX unit: TDO input, clocked from TCK (GPIO matrix loopback) --- */
    parlio_rx_unit_config_t rx_cfg = {
        .clk_src = PARLIO_CLK_SRC_EXTERNAL,
        .ext_clk_freq_hz = tck_freq_hz,
        .exp_clk_freq_hz = tck_freq_hz,
        .clk_in_gpio_num = pins->tck,    /* Loopback from TX clock output */
        .clk_out_gpio_num = -1,
        .data_width = 1,
        .data_gpio_nums = { [0] = pins->tdo },
        .valid_gpio_num = -1,
        .trans_queue_depth = 4,
        .max_recv_size = ctx->buf_size,
        .flags = {
            .free_clk = false,  /* Clock only runs during TX transmission */
        },
    };
    for (int i = 1; i < 16; i++)
        rx_cfg.data_gpio_nums[i] = -1;

    err = parlio_new_rx_unit(&rx_cfg, &ctx->rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RX unit create failed: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Soft delimiter: eof_data_len triggers end-of-frame after N bytes.
     * Set to buf_size so it captures the full buffer, then we stop manually. */
    parlio_rx_soft_delimiter_config_t delim_cfg = {
        .sample_edge = PARLIO_SAMPLE_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = ctx->buf_size,
        .timeout_ticks = tck_freq_hz / 10,  /* 100ms timeout safety */
    };
    err = parlio_new_rx_soft_delimiter(&delim_cfg, &ctx->rx_delim);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RX delimiter create failed: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Enable both units */
    err = parlio_tx_unit_enable(ctx->tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX enable failed: %s", esp_err_to_name(err));
        goto fail;
    }
    err = parlio_rx_unit_enable(ctx->rx, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RX enable failed: %s", esp_err_to_name(err));
        goto fail;
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

    ctx->base.reset    = parlio_reset;
    ctx->base.shift_ir = parlio_shift_ir;
    ctx->base.shift_dr = parlio_shift_dr;
    ctx->base.idle     = parlio_idle;
    ctx->base.free     = parlio_free_transport;

    ESP_LOGI(TAG, "PARLIO JTAG: TCK=%d(%lu Hz) TMS=%d TDI=%d TDO=%d",
             pins->tck, (unsigned long)tck_freq_hz,
             pins->tms, pins->tdi, pins->tdo);

    *out = &ctx->base;
    return ESP_OK;

fail:
    parlio_free_transport(&ctx->base);
    return err;
}

#endif /* CONFIG_XMOS_JTAG_BACKEND_PARLIO */
