/*
 * XMOS JTAG protocol implementation.
 *
 * Implements the XMOS-specific JTAG protocol on top of the abstract
 * transport layer (GPIO bit-bang or PARLIO DMA).
 *
 * Protocol flow for register access:
 *   1. Select MUX target (SSWITCH, XCOREn) via BSCAN SETMUX IR
 *   2. After MUX is open, the internal chain is:
 *      OTP(2b) + XCORE(10b) + CHIP(4b) + BSCAN(4b) = 20-bit IR
 *   3. Register read/write is encoded in the xCORE TAP IR:
 *      IR[9:2] = register index, IR[1:0] = 1(read) or 2(write)
 *   4. DR is 32 bits for SSWITCH, 33 bits for xCORE (data << 1)
 */

#include "xmos_jtag.h"
#include "xmos_regs.h"
#include "xmos_xe.h"
#include "jtag_transport.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "xmos_jtag";

/* =========================================================================
 * Internal context
 * ======================================================================= */
struct xmos_jtag_ctx {
    jtag_transport_t *transport;
    xmos_jtag_pins_t  pins;
    xmos_chip_info_t  chip;
    int               mux_state;    /* Current MUX target, -1 = unknown */
    bool              mux_open;     /* Is the internal chain exposed? */
};

/* =========================================================================
 * Helpers
 * ======================================================================= */

/** Shift a small value into IR (up to 32 bits). */
static esp_err_t shift_ir_val(xmos_jtag_handle_t h, uint32_t val, size_t bits)
{
    uint8_t buf[4];
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
    return h->transport->shift_ir(h->transport, buf, NULL, bits);
}

/** Shift a small value into DR and optionally capture output. */
static esp_err_t shift_dr_val(xmos_jtag_handle_t h, uint64_t val,
                              uint64_t *out, size_t bits)
{
    uint8_t tdi[8] = {0}, tdo[8] = {0};
    for (int i = 0; i < 8; i++)
        tdi[i] = (val >> (i * 8)) & 0xFF;

    esp_err_t err = h->transport->shift_dr(h->transport, tdi,
                                           out ? tdo : NULL, bits);
    if (err != ESP_OK) return err;

    if (out) {
        *out = 0;
        for (int i = 0; i < 8; i++)
            *out |= (uint64_t)tdo[i] << (i * 8);
    }
    return ESP_OK;
}

/* =========================================================================
 * MUX management
 *
 * Opening the MUX adds the internal TAPs (OTP, XCORE, CHIP) to the
 * scan chain.  Closing it (MUX_NC) removes them.
 * ======================================================================= */

/**
 * Select a MUX target.  After this call, if target != MUX_NC, the
 * JTAG chain includes the full internal chain (20-bit IR).
 */
static esp_err_t mux_select(xmos_jtag_handle_t h, uint32_t target)
{
    /* SETMUX via the top-level (boundary scan) TAP */
    esp_err_t err;

    if (!h->mux_open) {
        /* MUX is closed -- chain is just the BSCAN TAP (4-bit IR) */
        err = shift_ir_val(h, XMOS_BSCAN_IR_SETMUX, XMOS_BSCAN_IR_LEN);
        if (err != ESP_OK) return err;

        err = shift_dr_val(h, target, NULL, 4);
        if (err != ESP_OK) return err;

        if (target != XMOS_MUX_NC)
            h->mux_open = true;
    } else {
        /* MUX already open -- chain has 20-bit IR.
         * SETMUX is still on the BSCAN TAP, which is at the MSB end.
         * Other TAPs get BYPASS. */
        uint32_t ir = (uint32_t)XMOS_OTP_TAP_BYPASS
                    | ((uint32_t)0x3FF << 2)            /* XCORE bypass (10 bits) */
                    | ((uint32_t)XMOS_CHIP_TAP_BYPASS << 12)
                    | ((uint32_t)XMOS_BSCAN_IR_SETMUX << 16);
        err = shift_ir_val(h, ir, XMOS_MUX_TOTAL_IR_LEN);
        if (err != ESP_OK) return err;

        /* DR: bypass bits from OTP(1), XCORE(1), CHIP(1) + MUX value.
         * The MUX value goes into the BSCAN DR (4 bits).
         * Total DR = 1 + 1 + 1 + 4 = 7 bits */
        uint64_t dr = (uint64_t)target << 3;  /* 3 bypass bits, then target */
        err = shift_dr_val(h, dr, NULL, 7);
        if (err != ESP_OK) return err;

        if (target == XMOS_MUX_NC)
            h->mux_open = false;
    }

    h->mux_state = (int)target;

    /* Allow the MUX to settle */
    h->transport->idle(h->transport, 4);

    return ESP_OK;
}

/* =========================================================================
 * Register access through the internal chain
 *
 * When MUX is open, IR is 20 bits:
 *   [1:0]   OTP    (bypass = 0x3)
 *   [11:2]  XCORE  (reg_op)
 *   [15:12] CHIP   (bypass = 0xF)
 *   [19:16] BSCAN  (bypass = 0xF)
 *
 * DR for xCORE: 33 bits (data << 1), shifted through OTP bypass(1),
 *   CHIP bypass(1), BSCAN bypass(1) = total 36 bits.
 * DR for SSWITCH: 32 bits, total 35 bits.
 * ======================================================================= */

static esp_err_t reg_access(xmos_jtag_handle_t h, int tile, uint8_t reg,
                            uint32_t wr_val, uint32_t *rd_val, bool is_write)
{
    bool is_sswitch = (tile < 0);
    uint32_t mux_target = is_sswitch ? XMOS_MUX_SSWITCH : xmos_tile_to_mux(tile);

    /* Select the right MUX target if not already there */
    if (h->mux_state != (int)mux_target) {
        esp_err_t err = mux_select(h, mux_target);
        if (err != ESP_OK) return err;
    }

    /* Build the 20-bit chain IR */
    uint16_t xcore_ir;
    if (is_write)
        xcore_ir = (uint16_t)((reg << 2) | XMOS_REG_OP_WRITE);
    else
        xcore_ir = (uint16_t)((reg << 2) | XMOS_REG_OP_READ);

    uint32_t chain_ir = (uint32_t)XMOS_OTP_TAP_BYPASS
                      | ((uint32_t)xcore_ir << 2)
                      | ((uint32_t)XMOS_CHIP_TAP_BYPASS << 12)
                      | ((uint32_t)XMOS_BSCAN_IR_BYPASS << 16);

    esp_err_t err = shift_ir_val(h, chain_ir, XMOS_MUX_TOTAL_IR_LEN);
    if (err != ESP_OK) return err;

    /* DR scan */
    if (is_sswitch) {
        /* SSWITCH: 32-bit DR, + 3 bypass bits = 35 total */
        uint64_t dr_in = (uint64_t)wr_val << 1;  /* 1 bypass bit (OTP), then data */
        /* Actually: OTP bypass(1 bit at LSB) + SSWITCH DR(32 bits) + CHIP bypass(1) + BSCAN bypass(1) */
        dr_in = ((uint64_t)wr_val << 1);  /* OTP bypass absorbs 1 bit */
        uint64_t dr_out = 0;
        err = shift_dr_val(h, dr_in, rd_val ? &dr_out : NULL, 35);
        if (err != ESP_OK) return err;

        if (rd_val) {
            /* Extract 32-bit value after OTP bypass bit */
            *rd_val = (uint32_t)(dr_out >> 1) & 0xFFFFFFFF;
        }
    } else {
        /* xCORE: 33-bit DR (data<<1 + status), + 3 bypass = 36 total */
        uint64_t dr_in = (uint64_t)(wr_val) << 2;  /* OTP bypass(1) + xCORE shift(1) */
        uint64_t dr_out = 0;
        err = shift_dr_val(h, dr_in, rd_val ? &dr_out : NULL, 36);
        if (err != ESP_OK) return err;

        if (rd_val) {
            /* Skip OTP bypass bit, then xCORE DR is: (value << 1) | status */
            uint64_t xcore_dr = (dr_out >> 1) & 0x1FFFFFFFF;
            *rd_val = (uint32_t)(xcore_dr >> 1);
        }
    }

    return ESP_OK;
}

/* =========================================================================
 * Debug mode management
 * ======================================================================= */

static esp_err_t enter_debug(xmos_jtag_handle_t h, int tile)
{
    /* Request debug interrupt */
    esp_err_t err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT,
                               XMOS_DBG_INT_REQ, NULL, true);
    if (err != ESP_OK) return err;

    /* Poll for debug mode entry */
    for (int attempt = 0; attempt < 100; attempt++) {
        uint32_t val = 0;
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT, 0, &val, false);
        if (err != ESP_OK) return err;

        if (val & XMOS_DBG_INT_IN_DBG) {
            ESP_LOGD(TAG, "Tile %d entered debug mode (attempt %d)", tile, attempt);
            return ESP_OK;
        }

        h->transport->idle(h->transport, 100);
    }

    ESP_LOGE(TAG, "Tile %d failed to enter debug mode", tile);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t exit_debug(xmos_jtag_handle_t h, int tile)
{
    /* Clear debug interrupt request */
    esp_err_t err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT, 0, NULL, true);
    if (err != ESP_OK) return err;

    /* Issue RFDBG (return from debug) command */
    return reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                      XMOS_DBG_CMD_RFDBG, NULL, true);
}

/* =========================================================================
 * Debug memory access
 *
 * While in debug mode, memory is accessed through the debug command
 * mailbox (scratch registers):
 *   - Write address to ARG0
 *   - Write/read data via ARG2
 *   - Trigger via COMMAND register
 * ======================================================================= */

static esp_err_t dbg_mem_write_word(xmos_jtag_handle_t h, int tile,
                                    uint32_t addr, uint32_t value)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, addr, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, value, NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                      XMOS_DBG_CMD_WRITE, NULL, true);
}

static esp_err_t dbg_mem_read_word(xmos_jtag_handle_t h, int tile,
                                   uint32_t addr, uint32_t *value)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, addr, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                     XMOS_DBG_CMD_READ, NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, 0, value, false);
}

/**
 * Write 4 words at once using quad-write (READ4PI/WRITE4PI).
 * Address auto-increments by 16 bytes.
 */
static esp_err_t dbg_mem_write_quad(xmos_jtag_handle_t h, int tile,
                                    uint32_t addr, const uint32_t *values)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, addr, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, values[0], NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG3, values[1], NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG4, values[2], NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG5, values[3], NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                      XMOS_DBG_CMD_WRITE4PI, NULL, true);
}

/** Set a processor state register via SETPS debug command. */
static esp_err_t dbg_setps(xmos_jtag_handle_t h, int tile,
                           uint8_t ps_reg, uint32_t value)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, ps_reg, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, value, NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                      XMOS_DBG_CMD_SETPS, NULL, true);
}

/** Read a processor state register via GETPS debug command. */
static esp_err_t dbg_getps(xmos_jtag_handle_t h, int tile,
                           uint8_t ps_reg, uint32_t *value)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, ps_reg, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                     XMOS_DBG_CMD_GETPS, NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, 0, value, false);
}

/* =========================================================================
 * Public API: Init / Deinit
 * ======================================================================= */

esp_err_t xmos_jtag_init(const xmos_jtag_pins_t *pins,
                         xmos_jtag_handle_t *out_handle)
{
    xmos_jtag_handle_t h = calloc(1, sizeof(struct xmos_jtag_ctx));
    if (!h) return ESP_ERR_NO_MEM;

    h->pins = *pins;
    h->mux_state = -1;
    h->mux_open = false;

    esp_err_t err;

#if CONFIG_XMOS_JTAG_BACKEND_PARLIO
    err = jtag_transport_parlio_create(pins,
            CONFIG_XMOS_JTAG_TCK_FREQ_KHZ * 1000u, &h->transport);
#else
    err = jtag_transport_gpio_create(pins, &h->transport);
#endif

    if (err != ESP_OK) {
        free(h);
        return err;
    }

    *out_handle = h;
    return ESP_OK;
}

void xmos_jtag_deinit(xmos_jtag_handle_t handle)
{
    if (!handle) return;
    if (handle->transport) {
        handle->transport->free(handle->transport);
    }
    free(handle);
}

/* =========================================================================
 * Known JTAG manufacturer/part database (for chain display)
 * ======================================================================= */

typedef struct {
    uint32_t id_mask;    /* IDCODE & mask */
    uint32_t id_match;   /* Expected value */
    const char *name;
    uint8_t ir_len;
} jtag_known_device_t;

static const jtag_known_device_t s_known_devices[] = {
    /* XMOS */
    { 0x0FFFFFFF, 0x00005633, "XMOS xCORE-200 (XU21x)", 4 },
    { 0x0FFFFFFF, 0x00006633, "XMOS xCORE.ai (XU316)",  4 },
    { 0x0FFFFFFF, 0x00002633, "XMOS XS1-G1",            4 },
    { 0x0FFFFFFF, 0x00104731, "XMOS XS1-G4",            4 },
    { 0x0FFFFFFF, 0x00003633, "XMOS XS1-SU",            4 },
    /* Lattice */
    { 0x0FFFFFFF, 0x0012B043, "Lattice iCE40UP5K",      8 },
    { 0x0FFFFFFF, 0x0021111B, "Lattice ECP5-25",        8 },
    { 0x0FFFFFFF, 0x0041111B, "Lattice ECP5-45",        8 },
    { 0x0FFFFFFF, 0x0081111B, "Lattice ECP5-85",        8 },
    /* Xilinx / AMD */
    { 0x0FFFFFFF, 0x03631093, "Xilinx XC7A35T",         6 },
    { 0x0FFFFFFF, 0x03636093, "Xilinx XC7A100T",        6 },
    { 0x0FFFFFFF, 0x13631093, "Xilinx XC7A35T",         6 },
    /* Espressif (JTAG debug) */
    { 0x0FFF0FFF, 0x0000120F, "Espressif ESP32",        5 },
    { 0x0FFF0FFF, 0x0000220F, "Espressif ESP32-S2",     5 },
    { 0x0FFF0FFF, 0x0000320F, "Espressif ESP32-S3",     5 },
    /* ARM DAP */
    { 0x0F000FFF, 0x0B000477, "ARM CoreSight DAP",      4 },
    /* Sentinel */
    { 0, 0, NULL, 0 },
};

static const jtag_known_device_t *lookup_device(uint32_t idcode)
{
    for (const jtag_known_device_t *d = s_known_devices; d->name; d++) {
        if ((idcode & d->id_mask) == d->id_match)
            return d;
    }
    return NULL;
}

/* JEDEC JEP106 manufacturer lookup (common ones) */
static const char *lookup_manufacturer(uint16_t mfg_id)
{
    switch (mfg_id) {
        case 0x049: return "XMOS";
        case 0x093: return "Xilinx/AMD";
        case 0x06E: return "Lattice";
        case 0x00F: return "Espressif"; /* custom bank */
        case 0x23B: return "ARM";
        case 0x04F: return "Atmel/Microchip";
        case 0x0E5: return "Intel/Altera";
        case 0x01F: return "Atmel";
        case 0x020: return "ST";
        case 0x04A: return "GigaDevice";
        default:    return NULL;
    }
}

/* =========================================================================
 * Public API: Chain scan
 * ======================================================================= */

esp_err_t xmos_jtag_scan_chain(xmos_jtag_handle_t h, jtag_chain_t *chain)
{
    esp_err_t err;
    memset(chain, 0, sizeof(*chain));

    /* Close MUX if open */
    if (h->mux_open) {
        mux_select(h, XMOS_MUX_NC);
    }

    /* Reset TAP -- after reset, all devices load IDCODE (or BYPASS) into DR */
    h->mux_open = false;
    h->mux_state = -1;
    err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;

    /*
     * Read IDCODEs from the chain.
     *
     * After TAP reset, DR is loaded with IDCODE for each device.
     * Shift out 32 bits per device. An IDCODE has bit 0 = 1.
     * A BYPASS register has 1 bit = 0.
     * All-1s (0xFFFFFFFF) means end of chain.
     */
    size_t total_bits = JTAG_CHAIN_MAX_DEVICES * 32;
    size_t total_bytes = total_bits / 8;
    uint8_t *tdi = calloc(1, total_bytes);
    uint8_t *tdo = calloc(1, total_bytes);
    if (!tdi || !tdo) { free(tdi); free(tdo); return ESP_ERR_NO_MEM; }

    /* Shift zeros in, capture IDCODEs out */
    err = h->transport->shift_dr(h->transport, tdi, tdo, total_bits);
    free(tdi);
    if (err != ESP_OK) { free(tdo); return err; }

    /* Parse IDCODEs from the captured data */
    size_t bit_pos = 0;
    while (chain->num_devices < JTAG_CHAIN_MAX_DEVICES && bit_pos < total_bits) {
        /* Check bit 0: if 1 = IDCODE (32 bits), if 0 = BYPASS (1 bit) */
        int bit0 = (tdo[bit_pos / 8] >> (bit_pos % 8)) & 1;

        if (!bit0) {
            /* BYPASS device -- skip 1 bit. We can't determine IDCODE. */
            bit_pos++;
            continue;
        }

        /* Extract 32-bit IDCODE */
        if (bit_pos + 32 > total_bits) break;

        uint32_t idcode = 0;
        for (int b = 0; b < 32; b++) {
            size_t p = bit_pos + b;
            idcode |= (uint32_t)((tdo[p / 8] >> (p % 8)) & 1) << b;
        }
        bit_pos += 32;

        /* All-1s = end of chain */
        if (idcode == 0xFFFFFFFF) break;

        jtag_chain_device_t *dev = &chain->devices[chain->num_devices];
        dev->idcode = idcode;
        dev->manufacturer = (idcode >> 1) & 0x7FF;
        dev->part_number = (idcode >> 12) & 0xFFFF;
        dev->version = (idcode >> 28) & 0xF;

        /* Look up in known device table */
        const jtag_known_device_t *known = lookup_device(idcode);
        if (known) {
            dev->name = known->name;
            dev->ir_len = known->ir_len;
        } else {
            const char *mfg_name = lookup_manufacturer(dev->manufacturer);
            dev->name = mfg_name ? mfg_name : "Unknown";
            dev->ir_len = 0;
        }

        ESP_LOGI(TAG, "Chain[%zu]: IDCODE=0x%08lx %s (mfg=0x%03x part=0x%04x ver=%d)",
                 chain->num_devices, (unsigned long)idcode, dev->name,
                 dev->manufacturer, dev->part_number, dev->version);

        chain->num_devices++;
    }

    free(tdo);
    ESP_LOGI(TAG, "Chain scan: %zu device(s) found", chain->num_devices);
    return ESP_OK;
}

/* =========================================================================
 * Public API: Identify (XMOS-specific)
 * ======================================================================= */

esp_err_t xmos_jtag_identify(xmos_jtag_handle_t h,
                             xmos_chip_info_t *chip_info)
{
    esp_err_t err;

    /* Reset TAP -- back to just the BSCAN TAP */
    h->mux_open = false;
    h->mux_state = -1;
    err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;

    /* Read IDCODE: IR=IDCODE, then scan 32-bit DR */
    err = shift_ir_val(h, XMOS_BSCAN_IR_IDCODE, XMOS_BSCAN_IR_LEN);
    if (err != ESP_OK) return err;

    uint64_t idcode_raw = 0;
    err = shift_dr_val(h, 0, &idcode_raw, 32);
    if (err != ESP_OK) return err;

    uint32_t idcode = (uint32_t)idcode_raw;
    ESP_LOGI(TAG, "JTAG IDCODE: 0x%08lx", (unsigned long)idcode);

    memset(chip_info, 0, sizeof(*chip_info));
    chip_info->idcode = idcode;
    chip_info->revision = (idcode >> 28) & 0xF;

    uint32_t masked = idcode & XMOS_IDCODE_MASK;
    if (masked == (XMOS_IDCODE_XS2 & XMOS_IDCODE_MASK)) {
        chip_info->family = XMOS_FAMILY_XS2;
        chip_info->num_tiles = 2;  /* XU21x: up to 2 tiles */
        ESP_LOGI(TAG, "Identified: xCORE-200 (XS2), rev %d", chip_info->revision);
    } else if (masked == (XMOS_IDCODE_XS3 & XMOS_IDCODE_MASK)) {
        chip_info->family = XMOS_FAMILY_XS3;
        chip_info->num_tiles = 2;  /* XU316: 2 tiles */
        ESP_LOGI(TAG, "Identified: xCORE.ai (XS3), rev %d", chip_info->revision);
    } else if (masked == (XMOS_IDCODE_XS1_G4 & XMOS_IDCODE_MASK)) {
        chip_info->family = XMOS_FAMILY_XS1;
        chip_info->num_tiles = 4;
        ESP_LOGI(TAG, "Identified: XS1-G4 (quad tile), rev %d", chip_info->revision);
    } else if (masked == (XMOS_IDCODE_XS1_G1 & XMOS_IDCODE_MASK)) {
        chip_info->family = XMOS_FAMILY_XS1;
        chip_info->num_tiles = 1;
        ESP_LOGI(TAG, "Identified: XS1-G1 (single tile), rev %d", chip_info->revision);
    } else if (masked == (XMOS_IDCODE_XS1_SU & XMOS_IDCODE_MASK)) {
        chip_info->family = XMOS_FAMILY_XS1;
        chip_info->num_tiles = 1;
        ESP_LOGI(TAG, "Identified: XS1-SU (USB), rev %d", chip_info->revision);
    } else {
        chip_info->family = XMOS_FAMILY_UNKNOWN;
        ESP_LOGW(TAG, "Unknown IDCODE: 0x%08lx", (unsigned long)idcode);
    }

    h->chip = *chip_info;
    return ESP_OK;
}

/* =========================================================================
 * Public API: Boundary scan
 * ======================================================================= */

esp_err_t xmos_jtag_bscan_detect(xmos_jtag_handle_t h, size_t *bsr_len)
{
    esp_err_t err;
    *bsr_len = 0;

    /* Ensure MUX is closed so we're talking to the top-level BSCAN TAP only */
    if (h->mux_open) {
        err = mux_select(h, XMOS_MUX_NC);
        if (err != ESP_OK) return err;
    }

    /* Reset TAP to known state */
    err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;
    h->mux_open = false;
    h->mux_state = -1;

    /* Select SAMPLE/PRELOAD -- this connects the BSR as the DR */
    err = shift_ir_val(h, XMOS_BSCAN_IR_SAMPLE, XMOS_BSCAN_IR_LEN);
    if (err != ESP_OK) return err;

    /*
     * Auto-detect BSR length:
     *   1. Shift all-zeros into DR to flush
     *   2. Shift a single 1-bit followed by zeros
     *   3. Count how many clocks until the 1-bit appears on TDO
     *
     * We try up to 2048 bits (generous upper bound for any XMOS package).
     */
    #define BSR_MAX_PROBE 2048
    size_t probe_bytes = (BSR_MAX_PROBE + 7) / 8;
    uint8_t *tdi = calloc(1, probe_bytes);
    uint8_t *tdo = calloc(1, probe_bytes);
    if (!tdi || !tdo) {
        free(tdi); free(tdo);
        return ESP_ERR_NO_MEM;
    }

    /* Set bit 0 = 1 (the marker), rest zeros */
    tdi[0] = 0x01;

    err = h->transport->shift_dr(h->transport, tdi, tdo, BSR_MAX_PROBE);
    free(tdi);

    if (err != ESP_OK) {
        free(tdo);
        return err;
    }

    /* Find the first 1-bit in the output -- that's the BSR length */
    for (size_t i = 1; i < BSR_MAX_PROBE; i++) {
        if ((tdo[i / 8] >> (i % 8)) & 1) {
            *bsr_len = i;
            break;
        }
    }
    free(tdo);

    if (*bsr_len == 0) {
        ESP_LOGW(TAG, "BSR length detection failed (no marker found in %d bits)", BSR_MAX_PROBE);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "BSR length: %zu bits", *bsr_len);
    return ESP_OK;
    #undef BSR_MAX_PROBE
}

esp_err_t xmos_jtag_bscan_sample(xmos_jtag_handle_t h,
                                 uint8_t *bsr_data, size_t bsr_len)
{
    esp_err_t err;

    if (h->mux_open) {
        err = mux_select(h, XMOS_MUX_NC);
        if (err != ESP_OK) return err;
    }

    /* SAMPLE/PRELOAD captures current pin states into BSR */
    err = shift_ir_val(h, XMOS_BSCAN_IR_SAMPLE, XMOS_BSCAN_IR_LEN);
    if (err != ESP_OK) return err;

    /* Shift out BSR contents (shift in zeros -- doesn't affect pins) */
    size_t bytes = (bsr_len + 7) / 8;
    uint8_t *tdi = calloc(1, bytes);
    if (!tdi) return ESP_ERR_NO_MEM;

    err = h->transport->shift_dr(h->transport, tdi, bsr_data, bsr_len);
    free(tdi);
    return err;
}

esp_err_t xmos_jtag_bscan_extest(xmos_jtag_handle_t h,
                                 const uint8_t *bsr_data, size_t bsr_len)
{
    esp_err_t err;

    if (h->mux_open) {
        err = mux_select(h, XMOS_MUX_NC);
        if (err != ESP_OK) return err;
    }

    /* EXTEST drives physical pins from BSR contents */
    err = shift_ir_val(h, XMOS_BSCAN_IR_EXTEST, XMOS_BSCAN_IR_LEN);
    if (err != ESP_OK) return err;

    return h->transport->shift_dr(h->transport, bsr_data, NULL, bsr_len);
}

/* =========================================================================
 * Public API: Register access
 * ======================================================================= */

esp_err_t xmos_jtag_read_reg(xmos_jtag_handle_t h,
                             int tile, uint8_t reg, uint32_t *value)
{
    return reg_access(h, tile, reg, 0, value, false);
}

esp_err_t xmos_jtag_write_reg(xmos_jtag_handle_t h,
                              int tile, uint8_t reg, uint32_t value)
{
    return reg_access(h, tile, reg, value, NULL, true);
}

/* =========================================================================
 * Public API: Memory access
 * ======================================================================= */

esp_err_t xmos_jtag_mem_write(xmos_jtag_handle_t h,
                              int tile, uint32_t addr,
                              const void *data, size_t len)
{
    const uint8_t *src = (const uint8_t *)data;
    size_t words = (len + 3) / 4;
    uint32_t cur_addr = addr;

    ESP_LOGD(TAG, "mem_write: tile=%d addr=0x%08lx len=%zu (%zu words)",
             tile, (unsigned long)addr, len, words);

    int64_t start = esp_timer_get_time();

    /* Use quad-write for bulk, single-write for remainder */
    size_t i = 0;
    while (i + 4 <= words) {
        uint32_t vals[4];
        memcpy(vals, src + i * 4, 16);
        esp_err_t err = dbg_mem_write_quad(h, tile, cur_addr, vals);
        if (err != ESP_OK) return err;
        cur_addr += 16;
        i += 4;
    }
    while (i < words) {
        uint32_t val = 0;
        size_t remaining = len - i * 4;
        memcpy(&val, src + i * 4, remaining < 4 ? remaining : 4);
        esp_err_t err = dbg_mem_write_word(h, tile, cur_addr, val);
        if (err != ESP_OK) return err;
        cur_addr += 4;
        i++;
    }

    int64_t elapsed_us = esp_timer_get_time() - start;
    if (len >= 1024) {
        ESP_LOGI(TAG, "Wrote %zu bytes in %lld ms (%.1f KB/s)",
                 len, elapsed_us / 1000,
                 (double)len / (double)elapsed_us * 1000.0);
    }

    return ESP_OK;
}

esp_err_t xmos_jtag_mem_read(xmos_jtag_handle_t h,
                             int tile, uint32_t addr,
                             void *data, size_t len)
{
    uint8_t *dst = (uint8_t *)data;
    size_t words = (len + 3) / 4;
    uint32_t cur_addr = addr;

    for (size_t i = 0; i < words; i++) {
        uint32_t val = 0;
        esp_err_t err = dbg_mem_read_word(h, tile, cur_addr, &val);
        if (err != ESP_OK) return err;

        size_t remaining = len - i * 4;
        memcpy(dst + i * 4, &val, remaining < 4 ? remaining : 4);
        cur_addr += 4;
    }

    return ESP_OK;
}

/* =========================================================================
 * Public API: High-level loading
 * ======================================================================= */

/**
 * Perform the JTAG boot sequence:
 *   1. Assert system reset (if pin available)
 *   2. Set JTAG boot mode via test mode register
 *   3. Release reset
 *   4. Wait for boot ROM to enter JTAG boot wait loop
 */
static esp_err_t jtag_boot_sequence(xmos_jtag_handle_t h)
{
    esp_err_t err;

    /* Close MUX first, we need raw BSCAN access */
    if (h->mux_open) {
        mux_select(h, XMOS_MUX_NC);
    }

    /* Reset TAP */
    err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;
    h->mux_open = false;
    h->mux_state = -1;

    /* Assert system reset if available */
    if (h->pins.srst_n != GPIO_NUM_NC) {
        gpio_set_level(h->pins.srst_n, 0);
        esp_rom_delay_us(1000);
    }

    /* Set test mode: BOOT_FROM_JTAG, don't wait for PLL lock */
    uint32_t test_mode = XMOS_TEST_MODE_BOOT_JTAG
                       | XMOS_TEST_MODE_PLL_LOCK_N;

    err = shift_ir_val(h, XMOS_BSCAN_IR_SET_TEST_MODE, XMOS_BSCAN_IR_LEN);
    if (err != ESP_OK) return err;
    err = shift_dr_val(h, test_mode, NULL, 32);
    if (err != ESP_OK) return err;

    /* Release reset */
    if (h->pins.srst_n != GPIO_NUM_NC) {
        gpio_set_level(h->pins.srst_n, 1);
    } else {
        /* No SRST: write test mode with RESET_N=0 (causes reset), then with RESET_N=1 */
        err = shift_ir_val(h, XMOS_BSCAN_IR_SET_TEST_MODE, XMOS_BSCAN_IR_LEN);
        if (err != ESP_OK) return err;
        err = shift_dr_val(h, test_mode, NULL, 32);  /* RESET_N=0 implicit */
        if (err != ESP_OK) return err;

        err = shift_ir_val(h, XMOS_BSCAN_IR_SET_TEST_MODE, XMOS_BSCAN_IR_LEN);
        if (err != ESP_OK) return err;
        err = shift_dr_val(h, test_mode | XMOS_TEST_MODE_RESET_N, NULL, 32);
        if (err != ESP_OK) return err;
    }

    /* Wait for boot ROM to reach JTAG boot wait loop */
    esp_rom_delay_us(50000);  /* 50ms -- conservative */

    ESP_LOGI(TAG, "JTAG boot mode activated");
    return ESP_OK;
}

esp_err_t xmos_jtag_load_raw(xmos_jtag_handle_t h,
                             int tile,
                             const uint8_t *image, size_t image_len,
                             uint32_t load_addr, uint32_t entry_point)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Loading %zu bytes to tile %d @ 0x%08lx, entry=0x%08lx",
             image_len, tile, (unsigned long)load_addr,
             (unsigned long)entry_point);

    /* JTAG boot sequence */
    err = jtag_boot_sequence(h);
    if (err != ESP_OK) return err;

    /* Open MUX to target tile */
    err = mux_select(h, xmos_tile_to_mux(tile));
    if (err != ESP_OK) return err;

    /* Enter debug mode */
    err = enter_debug(h, tile);
    if (err != ESP_OK) return err;

    /* Write image to RAM */
    err = xmos_jtag_mem_write(h, tile, load_addr, image, image_len);
    if (err != ESP_OK) return err;

    if (entry_point != 0) {
        /* Set PC via processor state register */
        err = dbg_setps(h, tile, XMOS_PS_DBG_SPC, entry_point);
        if (err != ESP_OK) return err;

        /* Clear saved status register (disable interrupts/events) */
        err = dbg_setps(h, tile, XMOS_PS_DBG_SSR, 0);
        if (err != ESP_OK) return err;

        /* Resume execution */
        err = exit_debug(h, tile);
        if (err != ESP_OK) return err;

        ESP_LOGI(TAG, "Execution started at 0x%08lx", (unsigned long)entry_point);
    } else {
        ESP_LOGI(TAG, "Image loaded, core halted in debug mode");
    }

    return ESP_OK;
}

esp_err_t xmos_jtag_load_xe(xmos_jtag_handle_t h,
                            const uint8_t *xe_data, size_t xe_len,
                            bool run)
{
    esp_err_t err;

    /* Parse XE file */
    xe_parsed_t parsed;
    err = xe_parse(xe_data, xe_len, &parsed);
    if (err != ESP_OK) return err;

    if (parsed.num_segments == 0) {
        ESP_LOGE(TAG, "No loadable segments in XE file");
        return ESP_ERR_INVALID_ARG;
    }

    /* JTAG boot sequence */
    err = jtag_boot_sequence(h);
    if (err != ESP_OK) return err;

    /* Load segments, grouped by tile */
    for (uint8_t tile = 0; tile < parsed.num_tiles; tile++) {
        bool has_segments = false;

        for (size_t i = 0; i < parsed.num_segments; i++) {
            if (parsed.segments[i].tile != tile) continue;
            has_segments = true;
            break;
        }
        if (!has_segments) continue;

        /* Open MUX to this tile and enter debug */
        err = mux_select(h, xmos_tile_to_mux(tile));
        if (err != ESP_OK) return err;

        err = enter_debug(h, tile);
        if (err != ESP_OK) return err;

        /* Write all segments for this tile */
        for (size_t i = 0; i < parsed.num_segments; i++) {
            const xe_segment_t *seg = &parsed.segments[i];
            if (seg->tile != tile) continue;

            if (seg->filesz > 0) {
                ESP_LOGI(TAG, "Loading tile %d segment: 0x%08lx (%lu bytes)",
                         tile, (unsigned long)seg->paddr,
                         (unsigned long)seg->filesz);
                err = xmos_jtag_mem_write(h, tile, seg->paddr,
                                          seg->data, seg->filesz);
                if (err != ESP_OK) return err;
            }

            /* Zero-fill BSS (memsz > filesz) */
            if (seg->memsz > seg->filesz) {
                uint32_t bss_addr = seg->paddr + seg->filesz;
                size_t bss_len = seg->memsz - seg->filesz;
                ESP_LOGD(TAG, "Zeroing BSS: 0x%08lx (%zu bytes)",
                         (unsigned long)bss_addr, bss_len);

                /* Zero word-by-word */
                for (size_t off = 0; off < bss_len; off += 4) {
                    err = dbg_mem_write_word(h, tile, bss_addr + off, 0);
                    if (err != ESP_OK) return err;
                }
            }
        }

        if (run && parsed.entry_points[tile] != 0) {
            err = dbg_setps(h, tile, XMOS_PS_DBG_SPC, parsed.entry_points[tile]);
            if (err != ESP_OK) return err;
            err = dbg_setps(h, tile, XMOS_PS_DBG_SSR, 0);
            if (err != ESP_OK) return err;
            err = exit_debug(h, tile);
            if (err != ESP_OK) return err;
            ESP_LOGI(TAG, "Tile %d: started at 0x%08lx",
                     tile, (unsigned long)parsed.entry_points[tile]);
        }
    }

    return ESP_OK;
}

/* =========================================================================
 * Public API: Flash programming via JTAG stub
 *
 * Protocol between ESP32 and the stub running on xCORE:
 *   1. ESP32 loads stub to RAM and runs it
 *   2. Stub initialises SPI flash and signals ready via scratch[0]
 *   3. ESP32 writes flash data in chunks via scratch registers:
 *        scratch[2] = flash offset
 *        scratch[3] = chunk size (bytes, max 256)
 *        scratch[4..7] = data (up to 16 bytes per batch)
 *        scratch[1] = command (1 = write chunk, 2 = erase sector, 0xFF = done)
 *   4. Stub processes command, writes status to scratch[0]
 *   5. Repeat until done
 * ======================================================================= */

/* Stub mailbox commands */
#define STUB_CMD_NONE       0x00
#define STUB_CMD_WRITE      0x01
#define STUB_CMD_ERASE      0x02
#define STUB_CMD_DONE       0xFF

/* Stub status values */
#define STUB_STATUS_READY   0x01
#define STUB_STATUS_BUSY    0x02
#define STUB_STATUS_OK      0x03
#define STUB_STATUS_ERROR   0x80

static esp_err_t wait_stub_status(xmos_jtag_handle_t h, int tile,
                                  uint32_t expected, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint32_t status = 0;
        esp_err_t err = reg_access(h, tile, XMOS_PSWITCH_DBG_STATUS,
                                   0, &status, false);
        if (err != ESP_OK) return err;

        if (status == expected) return ESP_OK;
        if (status >= STUB_STATUS_ERROR) {
            ESP_LOGE(TAG, "Stub error: 0x%08lx", (unsigned long)status);
            return ESP_FAIL;
        }

        h->transport->idle(h->transport, 100);
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t xmos_jtag_program_flash(xmos_jtag_handle_t h,
                                  const uint8_t *flash_image,
                                  size_t flash_image_len,
                                  const uint8_t *stub, size_t stub_len)
{
    if (!stub || stub_len == 0) {
        ESP_LOGE(TAG, "No flash programmer stub provided. "
                 "Build one with the XMOS toolchain or use xmos_spi_flash_program() "
                 "for direct SPI access.");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err;
    int tile = 0;
    uint32_t ram_base = XMOS_XS2_RAM_BASE;

    ESP_LOGI(TAG, "Programming flash: %zu bytes via JTAG stub", flash_image_len);

    /* Load and run the stub */
    err = xmos_jtag_load_raw(h, tile, stub, stub_len, ram_base, ram_base);
    if (err != ESP_OK) return err;

    /* Wait for stub to signal ready */
    err = wait_stub_status(h, tile, STUB_STATUS_READY, 5000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Stub did not become ready");
        return err;
    }

    ESP_LOGI(TAG, "Stub ready, streaming flash data...");

    /* Erase sectors as needed, then write data in chunks.
     * We erase in 4K sectors (most common SPI flash sector size). */
    size_t sector_size = 4096;
    size_t chunk_size = 256;  /* SPI flash page size */

    /* Erase required sectors */
    size_t num_sectors = (flash_image_len + sector_size - 1) / sector_size;
    for (size_t s = 0; s < num_sectors; s++) {
        uint32_t sector_addr = (uint32_t)(s * sector_size);

        /* Enter debug to access mailbox */
        err = enter_debug(h, tile);
        if (err != ESP_OK) return err;

        err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, sector_addr, NULL, true);
        if (err != ESP_OK) return err;
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND, STUB_CMD_ERASE, NULL, true);
        if (err != ESP_OK) return err;

        err = exit_debug(h, tile);
        if (err != ESP_OK) return err;

        err = wait_stub_status(h, tile, STUB_STATUS_OK, 5000);
        if (err != ESP_OK) return err;

        if ((s & 0xF) == 0) {
            ESP_LOGI(TAG, "Erasing: %zu/%zu sectors", s + 1, num_sectors);
        }
    }

    /* Write data in page-sized chunks.
     * For each chunk, we pack up to 16 bytes at a time into scratch registers
     * (4 registers x 4 bytes). The stub reassembles and writes to flash. */
    size_t offset = 0;
    while (offset < flash_image_len) {
        size_t remaining = flash_image_len - offset;
        size_t this_chunk = (remaining < chunk_size) ? remaining : chunk_size;

        err = enter_debug(h, tile);
        if (err != ESP_OK) return err;

        /* Set flash offset */
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0,
                         (uint32_t)offset, NULL, true);
        if (err != ESP_OK) return err;

        /* Set chunk size */
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG1,
                         (uint32_t)this_chunk, NULL, true);
        if (err != ESP_OK) return err;

        /* Write chunk data to xCORE RAM (the stub reads from a fixed buffer) */
        uint32_t buf_addr = ram_base + 0x10000;  /* Stub data buffer area */
        err = xmos_jtag_mem_write(h, tile, buf_addr,
                                  flash_image + offset, this_chunk);
        if (err != ESP_OK) return err;

        /* Issue write command */
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                         STUB_CMD_WRITE, NULL, true);
        if (err != ESP_OK) return err;

        err = exit_debug(h, tile);
        if (err != ESP_OK) return err;

        err = wait_stub_status(h, tile, STUB_STATUS_OK, 5000);
        if (err != ESP_OK) return err;

        offset += this_chunk;

        if ((offset & 0xFFFF) == 0 || offset >= flash_image_len) {
            ESP_LOGI(TAG, "Writing: %zu/%zu bytes", offset, flash_image_len);
        }
    }

    /* Signal done */
    err = enter_debug(h, tile);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                     STUB_CMD_DONE, NULL, true);
    if (err != ESP_OK) return err;
    err = exit_debug(h, tile);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Flash programming complete: %zu bytes", flash_image_len);
    return ESP_OK;
}

/* =========================================================================
 * Public API: Direct SPI flash programming
 *
 * Hold XMOS in reset, bit-bang SPI to the flash chip, release.
 * This is the simplest approach but requires board-level support for
 * shared SPI bus access.
 * ======================================================================= */

/* SPI flash commands */
#define SPI_CMD_WRITE_ENABLE    0x06
#define SPI_CMD_READ_STATUS     0x05
#define SPI_CMD_PAGE_PROGRAM    0x02
#define SPI_CMD_SECTOR_ERASE    0x20
#define SPI_CMD_READ_JEDEC      0x9F
#define SPI_CMD_READ_DATA       0x03

#define SPI_STATUS_WIP          0x01  /* Write in progress */

static void spi_bb_init(const xmos_spi_pins_t *p)
{
    gpio_num_t outputs[] = { p->cs, p->clk, p->mosi };
    for (int i = 0; i < 3; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << outputs[i],
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&cfg);
    }
    gpio_set_level(p->cs, 1);
    gpio_set_level(p->clk, 0);

    gpio_config_t miso_cfg = {
        .pin_bit_mask = 1ULL << p->miso,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&miso_cfg);
}

static inline void spi_bb_cs(const xmos_spi_pins_t *p, int level)
{
    gpio_set_level(p->cs, level);
}

static uint8_t spi_bb_xfer(const xmos_spi_pins_t *p, uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(p->mosi, (out >> i) & 1);
        gpio_set_level(p->clk, 1);
        in |= (gpio_get_level(p->miso) << i);
        gpio_set_level(p->clk, 0);
    }
    return in;
}

static void spi_bb_cmd(const xmos_spi_pins_t *p, uint8_t cmd)
{
    spi_bb_cs(p, 0);
    spi_bb_xfer(p, cmd);
    spi_bb_cs(p, 1);
}

static void spi_bb_wait_ready(const xmos_spi_pins_t *p)
{
    spi_bb_cs(p, 0);
    spi_bb_xfer(p, SPI_CMD_READ_STATUS);
    while (spi_bb_xfer(p, 0) & SPI_STATUS_WIP) {
        /* Busy-wait -- flash erase can take up to 400ms per sector */
    }
    spi_bb_cs(p, 1);
}

esp_err_t xmos_spi_flash_program(xmos_jtag_handle_t h,
                                 const xmos_spi_pins_t *spi_pins,
                                 const uint8_t *image, size_t image_len,
                                 uint32_t offset)
{
    if (h->pins.srst_n == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Direct SPI flash requires srst_n pin to hold XMOS in reset");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Direct SPI flash: %zu bytes at offset 0x%lx",
             image_len, (unsigned long)offset);

    /* Hold XMOS in reset so it releases the SPI bus */
    gpio_set_level(h->pins.srst_n, 0);
    esp_rom_delay_us(10000);

    /* Initialize SPI bit-bang */
    spi_bb_init(spi_pins);

    /* Read JEDEC ID to verify flash is accessible */
    spi_bb_cs(spi_pins, 0);
    spi_bb_xfer(spi_pins, SPI_CMD_READ_JEDEC);
    uint8_t mfg = spi_bb_xfer(spi_pins, 0);
    uint8_t type = spi_bb_xfer(spi_pins, 0);
    uint8_t cap = spi_bb_xfer(spi_pins, 0);
    spi_bb_cs(spi_pins, 1);

    ESP_LOGI(TAG, "SPI flash JEDEC: mfg=0x%02x type=0x%02x cap=0x%02x",
             mfg, type, cap);

    if (mfg == 0xFF || mfg == 0x00) {
        ESP_LOGE(TAG, "No SPI flash detected (JEDEC ID all 0s or 1s)");
        gpio_set_level(h->pins.srst_n, 1);
        return ESP_ERR_NOT_FOUND;
    }

    /* Erase sectors covering the image */
    uint32_t erase_start = offset & ~0xFFF;
    uint32_t erase_end = (offset + image_len + 0xFFF) & ~0xFFF;

    for (uint32_t addr = erase_start; addr < erase_end; addr += 4096) {
        spi_bb_cmd(spi_pins, SPI_CMD_WRITE_ENABLE);

        spi_bb_cs(spi_pins, 0);
        spi_bb_xfer(spi_pins, SPI_CMD_SECTOR_ERASE);
        spi_bb_xfer(spi_pins, (addr >> 16) & 0xFF);
        spi_bb_xfer(spi_pins, (addr >> 8) & 0xFF);
        spi_bb_xfer(spi_pins, addr & 0xFF);
        spi_bb_cs(spi_pins, 1);

        spi_bb_wait_ready(spi_pins);

        if (((addr - erase_start) & 0xFFFF) == 0) {
            ESP_LOGI(TAG, "Erasing: 0x%lx / 0x%lx",
                     (unsigned long)(addr - erase_start),
                     (unsigned long)(erase_end - erase_start));
        }
    }

    /* Write pages (256 bytes each) */
    for (size_t off = 0; off < image_len; off += 256) {
        size_t page_len = image_len - off;
        if (page_len > 256) page_len = 256;

        spi_bb_cmd(spi_pins, SPI_CMD_WRITE_ENABLE);

        uint32_t addr = offset + off;
        spi_bb_cs(spi_pins, 0);
        spi_bb_xfer(spi_pins, SPI_CMD_PAGE_PROGRAM);
        spi_bb_xfer(spi_pins, (addr >> 16) & 0xFF);
        spi_bb_xfer(spi_pins, (addr >> 8) & 0xFF);
        spi_bb_xfer(spi_pins, addr & 0xFF);
        for (size_t i = 0; i < page_len; i++) {
            spi_bb_xfer(spi_pins, image[off + i]);
        }
        spi_bb_cs(spi_pins, 1);

        spi_bb_wait_ready(spi_pins);

        if ((off & 0xFFFF) == 0) {
            ESP_LOGI(TAG, "Writing: %zu / %zu bytes", off, image_len);
        }
    }

    /* Verify first 256 bytes */
    spi_bb_cs(spi_pins, 0);
    spi_bb_xfer(spi_pins, SPI_CMD_READ_DATA);
    spi_bb_xfer(spi_pins, (offset >> 16) & 0xFF);
    spi_bb_xfer(spi_pins, (offset >> 8) & 0xFF);
    spi_bb_xfer(spi_pins, offset & 0xFF);

    bool verify_ok = true;
    size_t verify_len = image_len < 256 ? image_len : 256;
    for (size_t i = 0; i < verify_len; i++) {
        uint8_t rb = spi_bb_xfer(spi_pins, 0);
        if (rb != image[i]) {
            ESP_LOGE(TAG, "Verify failed at offset %zu: wrote 0x%02x read 0x%02x",
                     i, image[i], rb);
            verify_ok = false;
            break;
        }
    }
    spi_bb_cs(spi_pins, 1);

    /* Release XMOS from reset */
    gpio_set_level(h->pins.srst_n, 1);

    if (!verify_ok)
        return ESP_FAIL;

    ESP_LOGI(TAG, "SPI flash programmed and verified OK (%zu bytes)", image_len);
    return ESP_OK;
}
