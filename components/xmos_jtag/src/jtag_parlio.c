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
    /* DMA buffers (DMA-capable, word-aligned) */
    uint8_t               *tx_buf;
    uint8_t               *rx_buf;
    size_t                 buf_size;
} jtag_parlio_ctx_t;

/* -------------------------------------------------------------------------
 * TX buffer encoding helpers
 *
 * With data_width=2 and LSB pack order, each byte packs 4 JTAG cycles:
 *   byte = (tms0 | tdi0<<1) | (tms1 | tdi1<<1)<<2 | ...
 * ---------------------------------------------------------------------- */

/* Encode a single TMS+TDI pair into 2 bits */
static inline uint8_t pack_cycle(int tms, int tdi)
{
    return (uint8_t)((tms & 1) | ((tdi & 1) << 1));
}

/**
 * Build a JTAG shift sequence into the TX buffer.
 *
 * Returns total TCK cycles written (including TAP navigation overhead).
 * The caller must ensure tx_buf has enough space.
 *
 * @param is_ir  true for IR scan, false for DR scan
 * @param tdi    Data to shift (LSB-first), may be NULL for all-zeros
 * @param bits   Number of data bits to shift
 * @param tx_buf Output DMA buffer
 * @return       Total number of TCK cycles in the buffer
 */
static size_t build_shift_sequence(bool is_ir, const uint8_t *tdi,
                                   size_t bits, uint8_t *tx_buf)
{
    size_t cycle = 0;

    /* Helper to pack a cycle into the buffer at position `cycle` */
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

    /* Shift-xR: shift bits-1 with TMS=0, last bit with TMS=1 */
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
 *
 * The RX buffer captures one TDO bit per cycle (8 bits per byte, LSB first).
 * We need to skip the TAP navigation cycles and extract only the data bits.
 *
 * @param rx_buf     RX DMA buffer
 * @param nav_cycles Number of navigation cycles before data starts
 *                   (3 for DR, 4 for IR: select + [select-IR] + capture + first_shift)
 * @param tdo        Output buffer (LSB-first)
 * @param bits       Number of data bits to extract
 */
static void extract_tdo(const uint8_t *rx_buf, size_t nav_cycles,
                        uint8_t *tdo, size_t bits)
{
    memset(tdo, 0, (bits + 7) / 8);

    /* Data bits start at cycle `nav_cycles` in the RX stream.
     * The capture cycle is nav_cycles-1, shift starts at nav_cycles. */
    for (size_t i = 0; i < bits; i++) {
        size_t rx_cycle = nav_cycles + i;
        int bit = (rx_buf[rx_cycle / 8] >> (rx_cycle % 8)) & 1;
        tdo[i / 8] |= (bit << (i % 8));
    }
}

/* -------------------------------------------------------------------------
 * Transport interface implementation
 * ---------------------------------------------------------------------- */

static esp_err_t parlio_reset(jtag_transport_t *self)
{
    jtag_parlio_ctx_t *ctx = (jtag_parlio_ctx_t *)self;

    /* Pulse TRST if available */
    if (ctx->pins.trst_n != GPIO_NUM_NC) {
        gpio_set_level(ctx->pins.trst_n, 0);
        esp_rom_delay_us(10);
        gpio_set_level(ctx->pins.trst_n, 1);
        esp_rom_delay_us(10);
    }

    /* Send 6 cycles of TMS=1, then 1 cycle TMS=0 to reach RTI */
    size_t cycle = 0;
    memset(ctx->tx_buf, 0, 4);
    for (int i = 0; i < 6; i++) {
        size_t byte_idx = cycle / 4;
        size_t shift = (cycle % 4) * 2;
        if (shift == 0) ctx->tx_buf[byte_idx] = 0;
        ctx->tx_buf[byte_idx] |= pack_cycle(1, 0) << shift;
        cycle++;
    }
    /* TMS=0 for RTI */
    {
        size_t byte_idx = cycle / 4;
        size_t shift = (cycle % 4) * 2;
        if (shift == 0) ctx->tx_buf[byte_idx] = 0;
        ctx->tx_buf[byte_idx] |= pack_cycle(0, 0) << shift;
        cycle++;
    }

    parlio_transmit_config_t tx_cfg = {
        .idle_value = 0,
    };
    esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                            cycle * 2,  /* bits on bus */
                                            &tx_cfg);
    if (err != ESP_OK) return err;
    return parlio_tx_unit_wait_all_done(ctx->tx, 1000);
}

static esp_err_t parlio_do_shift(jtag_parlio_ctx_t *ctx, bool is_ir,
                                 const uint8_t *tdi, uint8_t *tdo, size_t bits)
{
    if (bits == 0) return ESP_OK;

    /* Navigation overhead: 2 cycles for DR (select + capture),
     * 3 for IR (select + select-IR + capture).
     * Plus 2 trailing cycles (update + RTI). */
    size_t nav_pre = is_ir ? 3 : 2;
    size_t total_cycles = nav_pre + bits + 2;
    size_t tx_bytes = (total_cycles + 3) / 4;
    size_t rx_bytes = (total_cycles + 7) / 8;

    if (tx_bytes > ctx->buf_size || rx_bytes > ctx->buf_size) {
        ESP_LOGE(TAG, "Shift too large: %zu cycles, buf %zu", total_cycles, ctx->buf_size);
        return ESP_ERR_NO_MEM;
    }

    /* Build TX buffer */
    size_t actual_cycles = build_shift_sequence(is_ir, tdi, bits, ctx->tx_buf);
    (void)actual_cycles;

    /* Set up RX if TDO capture needed.
     * Buffer size must be >= eof_data_len (set to ctx->buf_size). */
    if (tdo) {
        memset(ctx->rx_buf, 0, ctx->buf_size);
        parlio_receive_config_t rx_cfg = {
            .delimiter = ctx->rx_delim,
        };
        esp_err_t err = parlio_rx_unit_receive(ctx->rx, ctx->rx_buf,
                                               ctx->buf_size, &rx_cfg);
        if (err != ESP_OK) return err;

        /* Start the soft delimiter so RX begins capturing */
        parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, true);
    }

    /* Fire TX */
    parlio_transmit_config_t tx_cfg = {
        .idle_value = 0,
    };
    esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                            total_cycles * 2,  /* 2 bits/cycle */
                                            &tx_cfg);
    if (err != ESP_OK) return err;

    /* Wait for completion */
    err = parlio_tx_unit_wait_all_done(ctx->tx, 5000);
    if (err != ESP_OK) return err;

    if (tdo) {
        /* Stop RX delimiter and wait */
        parlio_rx_soft_delimiter_start_stop(ctx->rx, ctx->rx_delim, false);
        parlio_rx_unit_wait_all_done(ctx->rx, 1000);

        /* Extract data bits from RX buffer, skipping navigation cycles */
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

    /* Fill TX buffer with TMS=0, TDI=0 */
    size_t tx_bytes = (cycles + 3) / 4;
    if (tx_bytes > ctx->buf_size) {
        /* For very long idles, loop in chunks */
        while (cycles > 0) {
            unsigned chunk = (ctx->buf_size * 4 < cycles) ?
                             (unsigned)(ctx->buf_size * 4) : cycles;
            esp_err_t err = parlio_idle(self, chunk);
            if (err != ESP_OK) return err;
            cycles -= chunk;
        }
        return ESP_OK;
    }

    memset(ctx->tx_buf, 0, tx_bytes);

    parlio_transmit_config_t tx_cfg = { .idle_value = 0 };
    esp_err_t err = parlio_tx_unit_transmit(ctx->tx, ctx->tx_buf,
                                            cycles * 2, &tx_cfg);
    if (err != ESP_OK) return err;
    return parlio_tx_unit_wait_all_done(ctx->tx, 5000);
}

static void parlio_free_transport(jtag_transport_t *self)
{
    jtag_parlio_ctx_t *ctx = (jtag_parlio_ctx_t *)self;
    if (ctx->rx) {
        parlio_rx_unit_disable(ctx->rx);  /* OK if already disabled */
        parlio_del_rx_unit(ctx->rx);
    }
    if (ctx->rx_delim)
        parlio_del_rx_delimiter(ctx->rx_delim);
    if (ctx->tx) {
        parlio_tx_unit_disable(ctx->tx);  /* OK if already disabled */
        parlio_del_tx_unit(ctx->tx);
    }
    if (ctx->tx_buf) heap_caps_free(ctx->tx_buf);
    if (ctx->rx_buf) heap_caps_free(ctx->rx_buf);

    /* Reset pins */
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

    /* Allocate DMA-capable buffers */
    ctx->tx_buf = heap_caps_calloc(1, ctx->buf_size, MALLOC_CAP_DMA);
    ctx->rx_buf = heap_caps_calloc(1, ctx->buf_size, MALLOC_CAP_DMA);
    if (!ctx->tx_buf || !ctx->rx_buf) {
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
    /* Fill remaining data_gpio_nums with -1 */
    for (int i = 2; i < 16; i++)
        tx_cfg.data_gpio_nums[i] = -1;

    err = parlio_new_tx_unit(&tx_cfg, &ctx->tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PARLIO TX: %s", esp_err_to_name(err));
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
    };
    for (int i = 1; i < 16; i++)
        rx_cfg.data_gpio_nums[i] = -1;

    err = parlio_new_rx_unit(&rx_cfg, &ctx->rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PARLIO RX: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Soft delimiter: we control start/stop manually, sample on falling edge
     * (TDO updates on TCK falling edge, is stable by next rising edge,
     *  but we sample on falling to maximize setup time) */
    parlio_rx_soft_delimiter_config_t delim_cfg = {
        .sample_edge = PARLIO_SAMPLE_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len = ctx->buf_size,       /* Max capture per transaction */
        .timeout_ticks = tck_freq_hz / 100,  /* 10ms timeout as safety net */
    };
    err = parlio_new_rx_soft_delimiter(&delim_cfg, &ctx->rx_delim);
    if (err != ESP_OK) goto fail;

    /* Enable both units */
    err = parlio_tx_unit_enable(ctx->tx);
    if (err != ESP_OK) goto fail;
    err = parlio_rx_unit_enable(ctx->rx, true);
    if (err != ESP_OK) goto fail;

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
            gpio_set_level(rst_pins[i], 1);
        }
    }

    /* Vtable */
    ctx->base.reset    = parlio_reset;
    ctx->base.shift_ir = parlio_shift_ir;
    ctx->base.shift_dr = parlio_shift_dr;
    ctx->base.idle     = parlio_idle;
    ctx->base.free     = parlio_free_transport;

    ESP_LOGI(TAG, "PARLIO JTAG: TCK=%d(%u Hz) TMS=%d TDI=%d TDO=%d",
             pins->tck, tck_freq_hz, pins->tms, pins->tdi, pins->tdo);

    *out = &ctx->base;
    return ESP_OK;

fail:
    parlio_free_transport(&ctx->base);
    return err;
}

#endif /* CONFIG_XMOS_JTAG_BACKEND_PARLIO */
