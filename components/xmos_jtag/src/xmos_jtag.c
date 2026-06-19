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
 * Chip TAP access and MUX management
 *
 * Opening the MUX adds the internal TAPs (OTP, XCORE) to the scan chain.
 * Closing it (MUX_NC) removes them.  SETMUX / SET_TEST_MODE are CHIP TAP
 * instructions; the framing below replicates XMOS sc_jtag
 * jtag_chip_tap_reg_access() exactly.
 * ======================================================================= */

/**
 * Select a MUX target (open the internal chain to an xCORE tile or the
 * system switch).  XS2 single-TAP model, per segher's xena-xtools jtag.c
 * (jtag_reset; jtag_set_mux: do_ir(0x4,4); do_dr_out(8+core,32)):
 * SETMUX is issued on the freshly-reset single top-level TAP.
 */
static esp_err_t mux_select(xmos_jtag_handle_t h, uint32_t target)
{
    /* Reset returns the chain to the single top-level TAP (and, with TRST
     * wired, resets the MUX controller).  segher always resets before
     * SETMUX, so the instruction lands on a clean one-TAP chain. */
    esp_err_t err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;

    err = shift_ir_val(h, XMOS_CHIP_TAP_IR_SETMUX, XMOS_TOP_TAP_IR_LEN);
    if (err != ESP_OK) return err;
    err = shift_dr_val(h, target, NULL, XMOS_SETMUX_DR_LEN);
    if (err != ESP_OK) return err;

    h->mux_open = (target != XMOS_MUX_NC);
    h->mux_state = (int)target;

    /* Dead cycles after a MUX change (segher: "needed when changing MUX"). */
    h->transport->idle(h->transport, 4);

    return ESP_OK;
}

/* =========================================================================
 * Register access through the internal chain (XS2, segher xena-xtools)
 *
 * With a module selected the chain is MODULE(IR10) + top-TAP(IR4 bypass):
 *   IR = (reg<<2)|op  in low 10 bits, top-TAP bypass 0xF in [13:10] (14 bits)
 *   DR = 32-bit data in low bits, top-TAP bypass (1 bit) = 33 bits, NO <<1
 *   readback value = DR[31:0]  (xCORE and SSWITCH identical on XS2)
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

    uint32_t op = is_write ? XMOS_REG_OP_WRITE : XMOS_REG_OP_READ;
    uint32_t ir = (((uint32_t)reg << 2) | op)
                | ((uint32_t)XMOS_BSCAN_IR_BYPASS << XMOS_XCORE_TAP_IR_LEN);

    esp_err_t err = shift_ir_val(h, ir, XMOS_MUX_OPEN_IR_LEN);
    if (err != ESP_OK) return err;

    uint64_t dr_out = 0;
    err = shift_dr_val(h, wr_val, rd_val ? &dr_out : NULL, XMOS_MUX_OPEN_DR_LEN);
    if (err != ESP_OK) return err;

    if (rd_val)
        *rd_val = (uint32_t)dr_out;   /* low 32 bits, no shift */

    return ESP_OK;
}

/* =========================================================================
 * Debug mode management
 * ======================================================================= */

static esp_err_t enter_debug(xmos_jtag_handle_t h, int tile)
{
    /* Request debug interrupt (sc_jtag dbg_enter_debug_mode: DBG_INT=1) */
    esp_err_t err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT,
                               XMOS_DBG_INT_REQ, NULL, true);
    if (err != ESP_OK) return err;

    /* Poll for debug mode entry */
    uint32_t val = 0;
    for (int attempt = 0; attempt < 100; attempt++) {
        err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT, 0, &val, false);
        if (err != ESP_OK) return err;

        /* 0xFFFFFFFF means TDO is floating (cable off / no contact), not a
         * real "in debug" reading -- the IN_DBG bit would falsely look set. */
        if (val != 0xFFFFFFFFu && (val & XMOS_DBG_INT_IN_DBG)) {
            ESP_LOGD(TAG, "Tile %d entered debug mode (attempt %d)", tile, attempt);
            return ESP_OK;
        }

        h->transport->idle(h->transport, 100);
    }

    if (val == 0xFFFFFFFFu) {
        ESP_LOGE(TAG, "Tile %d: DBG_INT reads all-ones -- TDO floating? "
                 "Check JTAG wiring/contact", tile);
    } else {
        ESP_LOGE(TAG, "Tile %d failed to enter debug mode (DBG_INT=0x%08lx). "
                 "Core may not be running; will retry without bus reset.",
                 tile, (unsigned long)val);
    }
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

/** Set a processor state register via SETPS debug command.
 *  ps_res is the ENCODED resource ID (XMOS_PS_*), not a bare number. */
static esp_err_t dbg_setps(xmos_jtag_handle_t h, int tile,
                           uint32_t ps_res, uint32_t value)
{
    esp_err_t err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, ps_res, NULL, true);
    if (err != ESP_OK) return err;
    err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG2, value, NULL, true);
    if (err != ESP_OK) return err;
    return reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND,
                      XMOS_DBG_CMD_SETPS, NULL, true);
}

/** Resume a tile from debug at the given entry point (SPC), interrupts off. */
static esp_err_t start_tile(xmos_jtag_handle_t h, int tile, uint32_t entry)
{
    esp_err_t err = dbg_setps(h, tile, XMOS_PS_DBG_SPC, entry);
    if (err != ESP_OK) return err;
    err = dbg_setps(h, tile, XMOS_PS_DBG_SSR, 0);
    if (err != ESP_OK) return err;
    return exit_debug(h, tile);
}

/**
 * Wait for a resumed tile to re-enter debug mode -- the XE "CALL" return.
 *
 * After a setup (CALL) image is resumed, the XMOS runtime runs its
 * initialisation and, under a debugger, signals completion by trapping
 * back into debug mode (the simulator models this as a "done" syscall;
 * tool_axe BootSequencer runs the cores until that signal).  We poll
 * DBG_INT for IN_DBG.  On timeout we force-halt and continue so a slightly
 * different return mechanism can't wedge the whole sequence.
 */
static esp_err_t wait_debug_reentry(xmos_jtag_handle_t h, int tile, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        uint32_t val = 0;
        esp_err_t err = reg_access(h, tile, XMOS_PSWITCH_DBG_INT, 0, &val, false);
        if (err != ESP_OK) return err;
        if (val != 0xFFFFFFFFu && (val & XMOS_DBG_INT_IN_DBG))
            return ESP_OK;
        h->transport->idle(h->transport, 200);
    }

    ESP_LOGW(TAG, "Tile %d setup did not return to debug in %d ms; force-halting",
             tile, timeout_ms);
    return enter_debug(h, tile);   /* force back into debug, best effort */
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

/* Used by the SVF player to drive raw scans through the same transport. */
jtag_transport_t *xmos_jtag_get_transport(xmos_jtag_handle_t handle)
{
    return handle ? handle->transport : NULL;
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
    /* XMOS -- device codes from sc_jtag jtag.h / tool_axe ProcessorNode */
    { 0x0000FFFF, 0x00005633, "XMOS xCORE-200 (XS2)",   4 },
    { 0x0000FFFF, 0x00006633, "XMOS xCORE.ai (XS3)",    4 },  /* unverified */
    { 0x0000FFFF, 0x00002633, "XMOS XS1-L",             4 },
    { 0x0FFFFFFF, 0x00104731, "XMOS XS1-G4",            4 },
    { 0x0000FFFF, 0x00003633, "XMOS XS1-SU",            4 },
    /* Lattice ECP5 -- IDCODEs from openFPGALoader part.hpp.
     * (iCE40 has no JTAG TAP; it can never appear in a chain.) */
    { 0xFFFFFFFF, 0x41111043, "Lattice ECP5 LFE5U-25",  8 },
    { 0xFFFFFFFF, 0x41112043, "Lattice ECP5 LFE5U-45",  8 },
    { 0xFFFFFFFF, 0x41113043, "Lattice ECP5 LFE5U-85",  8 },
    /* Xilinx / AMD -- openFPGALoader part.hpp */
    { 0x0FFFFFFF, 0x0362D093, "Xilinx XC7A35T",         6 },
    { 0x0FFFFFFF, 0x03631093, "Xilinx XC7A100T",        6 },
    /* Espressif -- OpenOCD tcl/target/esp32*.cfg (ESP32/S3 share the ID) */
    { 0x0FFFFFFF, 0x020034E5, "Espressif ESP32/S3",     5 },
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

/*
 * JEDEC JEP106 manufacturer lookup.  The key is IDCODE bits [11:1]
 * ((bank << 7) | code), e.g. XMOS = bank 6, code 0x19 -> 0x319
 * (openocd jep106.inc).  The others fall out of known IDCODEs:
 * 0x...093 >> 1 -> 0x049 Xilinx, 0x...043 >> 1 -> 0x021 Lattice,
 * 0x120034E5 >> 1 -> 0x272 Espressif, 0x...477 >> 1 -> 0x23B ARM.
 */
static const char *lookup_manufacturer(uint16_t mfg_id)
{
    switch (mfg_id) {
        case 0x319: return "XMOS";
        case 0x049: return "Xilinx/AMD";
        case 0x021: return "Lattice";
        case 0x272: return "Espressif";
        case 0x23B: return "ARM";
        case 0x06E: return "Intel/Altera";
        case 0x01F: return "Atmel/Microchip";
        case 0x020: return "ST";
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
    uint8_t *tdi = malloc(total_bytes);
    uint8_t *tdo = calloc(1, total_bytes);
    if (!tdi || !tdo) { free(tdi); free(tdo); return ESP_ERR_NO_MEM; }

    /* Shift ONES in, capture IDCODEs out.  Once the real chain content has
     * passed through, the captured stream reads back our all-1s fill, which
     * is what the 0xFFFFFFFF end-of-chain check below relies on (shifting
     * zeros would read back as an endless run of fake BYPASS bits). */
    memset(tdi, 0xFF, total_bytes);
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

    /* Reset TAP -- back to the closed chain (BSCAN + CHIP TAPs) */
    h->mux_open = false;
    h->mux_state = -1;
    err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;

    /* Read IDCODE straight from DR.  After Test-Logic-Reset every TAP
     * with an IDCODE register selects it as the DR (IEEE 1149.1), so no
     * IR load is needed -- and loading a wrong opcode here could select
     * something destructive like EXTEST.  The chip TAP sits nearest TDO,
     * so its IDCODE is the first 32 bits out. */
    uint64_t idcode_raw = 0;
    err = shift_dr_val(h, 0xFFFFFFFFu, &idcode_raw, 32);
    if (err != ESP_OK) return err;

    uint32_t idcode = (uint32_t)idcode_raw;
    ESP_LOGI(TAG, "JTAG IDCODE: 0x%08lx", (unsigned long)idcode);

    memset(chip_info, 0, sizeof(*chip_info));

    /* All-1s or all-0s means no device connected (TDO floating) */
    if (idcode == 0xFFFFFFFF || idcode == 0x00000000) {
        ESP_LOGW(TAG, "No device detected (IDCODE=0x%08lx)", (unsigned long)idcode);
        chip_info->family = XMOS_FAMILY_UNKNOWN;
        return ESP_ERR_NOT_FOUND;
    }
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

    /* Self-test the mux-open register path: read the system switch's JTAG
     * device-ID register (XS1_SSWITCH_JTAG_DEVICE_ID_NUM = 0x9).  A value
     * matching the IDCODE confirms SETMUX + 14-bit register access work
     * end-to-end on this board; 0x0 or 0xFFFFFFFF means the mux-open path
     * (not just detection) is the problem -- which isolates a "failed to
     * enter debug mode" to wiring/framing vs. core state. */
    uint32_t sw_id = 0;
    if (xmos_jtag_read_reg(h, -1, XMOS_SSWITCH_JTAG_DEVICE_ID, &sw_id) == ESP_OK) {
        ESP_LOGI(TAG, "Mux-open self-test: SSWITCH device-id (reg 0x9) = 0x%08lx%s",
                 (unsigned long)sw_id,
                 (sw_id == 0 || sw_id == 0xFFFFFFFFu)
                     ? "  <-- register access NOT working" : "");
    }
    /* Leave the chain closed for whatever runs next */
    mux_select(h, XMOS_MUX_NC);
    h->transport->reset(h->transport);
    h->mux_open = false;
    h->mux_state = -1;

    h->chip = *chip_info;
    return ESP_OK;
}

/* =========================================================================
 * Public API: Low-level diagnostics
 * ======================================================================= */

esp_err_t xmos_jtag_diagnose(xmos_jtag_handle_t h, xmos_jtag_diag_t *out)
{
    xmos_jtag_diag_t d = { .tdo_idle = -1, .loopback_hi = -1, .loopback_lo = -1 };

    h->mux_open = false;
    h->mux_state = -1;
    esp_err_t err = h->transport->reset(h->transport);
    if (err != ESP_OK) return err;

    /* Static pin state + TDI->TDO loopback (needs the backend's pin probe) */
    if (h->transport->pin_probe) {
        d.pin_probe_ok = true;
        h->transport->pin_probe(h->transport, &d.tdo_idle,
                                &d.loopback_hi, &d.loopback_lo);
        /* pin_probe left TCK/TDI low; get back to a clean reset */
        h->transport->reset(h->transport);
    }

    /* Raw IDCODE (after reset the IDCODE reg is the default DR) */
    uint64_t idcode = 0;
    err = shift_dr_val(h, 0xFFFFFFFFu, &idcode, 32);
    if (err != ESP_OK) return err;
    d.idcode_raw = (uint32_t)idcode;

    /* TDO activity: clock out 64 more bits and count how many read as 1.
     * 0 ones  -> TDO stuck low (held low / wrong pin / target in reset)
     * 64 ones -> TDO stuck high (floating; TDO not connected, pull-up wins)
     * mixed   -> the target is driving real data back. */
    uint8_t tdi64[8], tdo64[8] = {0};
    memset(tdi64, 0xFF, sizeof(tdi64));
    err = h->transport->shift_dr(h->transport, tdi64, tdo64, 64);
    if (err != ESP_OK) return err;
    int ones = 0;
    for (int i = 0; i < 64; i++)
        ones += (tdo64[i / 8] >> (i % 8)) & 1;
    d.tdo_activity_ones = ones;

    const char *tdo_diag =
        !d.pin_probe_ok        ? "(no pin probe on this backend)" :
        d.tdo_idle < 0         ? "" :
        (d.tdo_idle == 1 && ones == 64) ? "TDO floating HIGH -> TDO wire not connected?" :
        (d.tdo_idle == 0 && ones == 0)  ? "TDO stuck LOW -> wrong pin / shorted / target held in reset?" :
        "TDO toggling -> target is responding";

    ESP_LOGW(TAG, "=== JTAG DIAGNOSTIC ===");
    ESP_LOGW(TAG, "  IDCODE raw        : 0x%08lx", (unsigned long)d.idcode_raw);
    ESP_LOGW(TAG, "  TDO idle level    : %d", d.tdo_idle);
    ESP_LOGW(TAG, "  TDO activity (1s) : %d / 64   %s", ones, tdo_diag);
    if (d.pin_probe_ok)
        ESP_LOGW(TAG, "  TDI->TDO loopback : drive1=%d drive0=%d  %s",
                 d.loopback_hi, d.loopback_lo,
                 (d.loopback_hi == 1 && d.loopback_lo == 0)
                     ? "PINS OK (jumper present)"
                     : "no change (no TDI->TDO jumper, or input pin not reading)");
    ESP_LOGW(TAG, "=======================");

    h->transport->reset(h->transport);
    if (out) *out = d;
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

/** Reset only the TAP state machine (TMS); leaves the target running. */
static esp_err_t jtag_reset_tap(xmos_jtag_handle_t h)
{
    esp_err_t err = h->transport->reset(h->transport);
    h->mux_open = false;
    h->mux_state = -1;
    return err;
}

/** Pulse the system-reset pin and let the boot ROM restart, then reset TAP. */
static esp_err_t jtag_pulse_srst(xmos_jtag_handle_t h)
{
    if (h->pins.srst_n == GPIO_NUM_NC)
        return ESP_ERR_NOT_SUPPORTED;
    gpio_set_level(h->pins.srst_n, 0);
    esp_rom_delay_us(1000);
    gpio_set_level(h->pins.srst_n, 1);
    esp_rom_delay_us(50000);     /* boot ROM start-up */
    return jtag_reset_tap(h);
}

/**
 * Halt a tile in debug mode, ready for RAM access.
 *
 * Strategy: first attach to the RUNNING core and interrupt it WITHOUT a
 * bus reset.  On a board whose flash boot is corrupt, asserting SRST just
 * re-runs the failing boot and can leave the core stopped and un-haltable,
 * whereas an application that is already running (the device enumerates on
 * the host) can be interrupted directly -- which is how XMOS's own xgdb
 * "interrupt" works (sc_jtag dbg_cmd_interrupt: just DBG_INT=1, no reset).
 * Only if the interrupt fails do we fall back to a hard SRST + retry.
 *
 * A previous revision shifted a "SET_TEST_MODE / boot-from-JTAG" word here,
 * but its bit layout had no public source and collided with the 0xFACED00
 * key XMOS itself shifts into that register -- removed.
 */
static esp_err_t attach_tile_debug(xmos_jtag_handle_t h, int tile)
{
    esp_err_t err = jtag_reset_tap(h);
    if (err != ESP_OK) return err;

    err = mux_select(h, xmos_tile_to_mux(tile));
    if (err != ESP_OK) return err;

    err = enter_debug(h, tile);
    if (err == ESP_OK) return ESP_OK;

    if (h->pins.srst_n == GPIO_NUM_NC)
        return err;

    ESP_LOGW(TAG, "Tile %d: interrupt failed, trying SRST reset then halt", tile);
    err = jtag_pulse_srst(h);
    if (err != ESP_OK) return err;
    err = mux_select(h, xmos_tile_to_mux(tile));
    if (err != ESP_OK) return err;
    return enter_debug(h, tile);
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

    /* Halt the tile (interrupt running core, SRST fallback) */
    err = attach_tile_debug(h, tile);
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

/* Write one parsed segment (file data + zero-filled BSS) to a halted tile. */
static esp_err_t load_segment(xmos_jtag_handle_t h, const xe_segment_t *seg)
{
    esp_err_t err;
    if (seg->filesz > 0) {
        ESP_LOGI(TAG, "  tile %d: 0x%08lx (%lu bytes)", seg->tile,
                 (unsigned long)seg->paddr, (unsigned long)seg->filesz);
        err = xmos_jtag_mem_write(h, seg->tile, seg->paddr,
                                  seg->data, seg->filesz);
        if (err != ESP_OK) return err;
    }
    if (seg->memsz > seg->filesz) {
        uint32_t bss = seg->paddr + seg->filesz;
        for (size_t off = 0; off < seg->memsz - seg->filesz; off += 4) {
            err = dbg_mem_write_word(h, seg->tile, bss + off, 0);
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

/* Halt a tile for a LOAD step.  First contact interrupts the running core
 * (SRST fallback); later loads just re-select and re-assert debug, which
 * is a no-op if the tile is already halted from a prior CALL return. */
static esp_err_t halt_tile_for_load(xmos_jtag_handle_t h, int tile,
                                    bool *first_contact)
{
    if (*first_contact) {
        *first_contact = false;
        return attach_tile_debug(h, tile);
    }
    esp_err_t err = mux_select(h, xmos_tile_to_mux(tile));
    if (err != ESP_OK) return err;
    return enter_debug(h, tile);
}

esp_err_t xmos_jtag_load_xe(xmos_jtag_handle_t h,
                            const uint8_t *xe_data, size_t xe_len,
                            bool run)
{
    esp_err_t err;

    xe_parsed_t parsed;
    err = xe_parse(xe_data, xe_len, &parsed);
    if (err != ESP_OK) return err;

    if (parsed.num_steps == 0) {
        ESP_LOGE(TAG, "No boot steps in XE file");
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Execute the boot sequence in order (load -> call -> load -> goto).
     * The setup (CALL) phase configures each tile's clocks/PLL/RAM exactly
     * as the boot ROM would, then returns to debug; only then is the
     * application loaded and started.  This mirrors XMOS xrun / tool_axe.
     */
    uint32_t cur_entry[XE_MAX_TILES] = {0};
    bool first_contact = true;

    for (size_t i = 0; i < parsed.num_steps; i++) {
        const xe_step_t *step = &parsed.steps[i];

        if (step->op == XE_OP_LOAD) {
            err = halt_tile_for_load(h, step->tile, &first_contact);
            if (err != ESP_OK) return err;

            ESP_LOGI(TAG, "Load tile %d (entry 0x%08lx, %u segment(s))",
                     step->tile, (unsigned long)step->entry, step->seg_count);
            for (uint16_t s = 0; s < step->seg_count; s++) {
                err = load_segment(h, &parsed.segments[step->seg_first + s]);
                if (err != ESP_OK) return err;
            }
            if (step->tile < XE_MAX_TILES)
                cur_entry[step->tile] = step->entry;
        } else {  /* XE_OP_RUN */
            if (!run) {
                ESP_LOGI(TAG, "run=false: leaving tiles halted, skipping start");
                continue;
            }
            /* Resume every tile in the batch first... */
            for (int t = 0; t < XE_MAX_TILES; t++) {
                if (!(step->run_mask & (1u << t))) continue;
                ESP_LOGI(TAG, "%s tile %d at 0x%08lx",
                         step->run_wait ? "Call" : "Goto", t,
                         (unsigned long)cur_entry[t]);
                err = mux_select(h, xmos_tile_to_mux(t));
                if (err != ESP_OK) return err;
                err = start_tile(h, t, cur_entry[t]);
                if (err != ESP_OK) return err;
            }
            /* ...then, for CALL, wait for each to return to debug. */
            if (step->run_wait) {
                for (int t = 0; t < XE_MAX_TILES; t++) {
                    if (!(step->run_mask & (1u << t))) continue;
                    err = wait_debug_reentry(h, t, 3000);
                    if (err != ESP_OK) return err;
                }
            }
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
 *   3. ESP32 streams flash data: for each chunk it writes the bytes into a
 *      fixed RAM buffer (ram_base + STUB_DATA_BUF_OFFSET) via the debug
 *      memory interface, then posts a command through the scratch mailbox:
 *        DBG_ARG0 (scratch[2]) = flash address (erase sector / write offset)
 *        DBG_ARG1 (scratch[3]) = byte count (for WRITE)
 *        DBG_COMMAND (scratch[1]) = WRITE / ERASE / DONE
 *   4. Stub processes the command and writes status to DBG_STATUS (scratch[0])
 *   5. Repeat until done.
 *
 * Handshake (race-free): the host halts the core, clears STATUS to BUSY,
 * sets the args + command, then resumes.  The stub, when it sees a command,
 * performs the op, sets STATUS = OK (or >= ERROR), and clears COMMAND.  The
 * host polls STATUS.  Clearing STATUS while the core is halted guarantees a
 * stale OK from the previous command is never mistaken for this one.
 *
 * The exact stub contract and a reference implementation live in
 * flash_stub/ (build it with the XMOS toolchain; not shipped prebuilt).
 * ======================================================================= */

/* Stub mailbox commands (host -> stub, via DBG_COMMAND) */
#define STUB_CMD_NONE       0x00
#define STUB_CMD_WRITE      0x01
#define STUB_CMD_ERASE      0x02
#define STUB_CMD_DONE       0xFF

/* Stub status values (stub -> host, via DBG_STATUS) */
#define STUB_STATUS_READY   0x01
#define STUB_STATUS_BUSY    0x02
#define STUB_STATUS_OK      0x03
#define STUB_STATUS_ERROR   0x80

/* Shared data buffer, relative to the tile RAM base. Must match the stub. */
#define STUB_DATA_BUF_OFFSET  0x10000u

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

/*
 * Post one command to the running stub and wait for completion.
 * Halts the core to write the mailbox atomically (incl. clearing STATUS to
 * BUSY so a previous OK can't be misread), resumes, then polls STATUS.
 */
static esp_err_t stub_command(xmos_jtag_handle_t h, int tile,
                              uint32_t cmd, uint32_t arg0, uint32_t arg1,
                              int timeout_ms)
{
    esp_err_t err = enter_debug(h, tile);
    if (err != ESP_OK) return err;

    if ((err = reg_access(h, tile, XMOS_PSWITCH_DBG_STATUS, STUB_STATUS_BUSY, NULL, true)) ||
        (err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG0, arg0, NULL, true)) ||
        (err = reg_access(h, tile, XMOS_PSWITCH_DBG_ARG1, arg1, NULL, true)) ||
        (err = reg_access(h, tile, XMOS_PSWITCH_DBG_COMMAND, cmd, NULL, true)))
        return err;

    err = exit_debug(h, tile);
    if (err != ESP_OK) return err;

    if (cmd == STUB_CMD_DONE) return ESP_OK;   /* no completion to await */
    return wait_stub_status(h, tile, STUB_STATUS_OK, timeout_ms);
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
    uint32_t buf_addr = XMOS_XS2_RAM_BASE + STUB_DATA_BUF_OFFSET;
    const size_t sector_size = 4096;
    const size_t chunk_size  = 256;   /* SPI flash page size */

    ESP_LOGI(TAG, "Programming flash: %zu bytes via JTAG stub", flash_image_len);

    /* Load and run the stub as a normal XE (two-phase boot sets up the
     * tile clocks, stack and runtime before the mailbox loop starts). */
    err = xmos_jtag_load_xe(h, stub, stub_len, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load flash stub: %s", esp_err_to_name(err));
        return err;
    }

    /* Wait for the stub to initialise the flash and signal ready */
    err = wait_stub_status(h, tile, STUB_STATUS_READY, 5000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Flash stub did not become ready");
        return err;
    }
    ESP_LOGI(TAG, "Stub ready, streaming flash data...");

    /* Erase the sectors the image covers */
    size_t num_sectors = (flash_image_len + sector_size - 1) / sector_size;
    for (size_t s = 0; s < num_sectors; s++) {
        err = stub_command(h, tile, STUB_CMD_ERASE,
                           (uint32_t)(s * sector_size), 0, 5000);
        if (err != ESP_OK) return err;
        if ((s & 0xF) == 0)
            ESP_LOGI(TAG, "Erasing: %zu/%zu sectors", s + 1, num_sectors);
    }

    /* Write the image in page-sized chunks: data into the RAM buffer first,
     * then a WRITE command pointing the stub at it. */
    for (size_t offset = 0; offset < flash_image_len; offset += chunk_size) {
        size_t remaining = flash_image_len - offset;
        size_t this_chunk = remaining < chunk_size ? remaining : chunk_size;

        err = enter_debug(h, tile);
        if (err != ESP_OK) return err;
        err = xmos_jtag_mem_write(h, tile, buf_addr,
                                  flash_image + offset, this_chunk);
        if (err != ESP_OK) return err;
        err = exit_debug(h, tile);
        if (err != ESP_OK) return err;

        err = stub_command(h, tile, STUB_CMD_WRITE,
                           (uint32_t)offset, (uint32_t)this_chunk, 5000);
        if (err != ESP_OK) return err;

        if ((offset & 0xFFFF) == 0 || offset + this_chunk >= flash_image_len)
            ESP_LOGI(TAG, "Writing: %zu/%zu bytes",
                     offset + this_chunk, flash_image_len);
    }

    /* Tell the stub we're done (it sets QE and halts) */
    err = stub_command(h, tile, STUB_CMD_DONE, 0, 0, 1000);
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
#define SPI_CMD_READ_STATUS     0x05  /* RDSR1 */
#define SPI_CMD_READ_STATUS2    0x35  /* RDSR2 (Winbond/GigaDevice/etc.) */
#define SPI_CMD_WRITE_STATUS    0x01  /* WRSR (1 or 2 bytes: SR1[,SR2]) */
#define SPI_CMD_PAGE_PROGRAM    0x02
#define SPI_CMD_SECTOR_ERASE    0x20
#define SPI_CMD_READ_JEDEC      0x9F
#define SPI_CMD_READ_DATA       0x03

#define SPI_STATUS_WIP          0x01  /* Write in progress (SR1 bit 0) */
#define SPI_STATUS2_QE          0x02  /* Quad Enable (SR2 bit 1) */

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

static uint8_t spi_bb_read_reg(const xmos_spi_pins_t *p, uint8_t cmd)
{
    spi_bb_cs(p, 0);
    spi_bb_xfer(p, cmd);
    uint8_t v = spi_bb_xfer(p, 0);
    spi_bb_cs(p, 1);
    return v;
}

/* Poll the flash status register until write-in-progress clears.
 * Bounded: a missing/mis-wired flash would otherwise spin forever. */
static esp_err_t spi_bb_wait_ready(const xmos_spi_pins_t *p, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    spi_bb_cs(p, 0);
    spi_bb_xfer(p, SPI_CMD_READ_STATUS);
    esp_err_t err = ESP_OK;
    while (spi_bb_xfer(p, 0) & SPI_STATUS_WIP) {
        if (esp_timer_get_time() > deadline) { err = ESP_ERR_TIMEOUT; break; }
    }
    spi_bb_cs(p, 1);
    return err;
}

/*
 * Ensure the flash Quad-Enable bit is set so the xCORE-200 boot ROM's
 * quad read (0xEB) works -- xflash does this and the device will not boot
 * from a QE-clear flash.  Uses the common Winbond/GigaDevice layout
 * (QE = bit 1 of Status Register 2, written via WRSR 0x01 with two bytes).
 * Best-effort: parts with a different QE location are logged, not failed,
 * since the image data itself is already written by then.
 */
static void spi_bb_set_quad_enable(const xmos_spi_pins_t *p)
{
    uint8_t sr1 = spi_bb_read_reg(p, SPI_CMD_READ_STATUS);
    uint8_t sr2 = spi_bb_read_reg(p, SPI_CMD_READ_STATUS2);
    if (sr2 & SPI_STATUS2_QE) {
        ESP_LOGI(TAG, "Flash QE already set (SR2=0x%02x)", sr2);
        return;
    }

    spi_bb_cmd(p, SPI_CMD_WRITE_ENABLE);
    spi_bb_cs(p, 0);
    spi_bb_xfer(p, SPI_CMD_WRITE_STATUS);
    spi_bb_xfer(p, sr1);                       /* preserve SR1 */
    spi_bb_xfer(p, (uint8_t)(sr2 | SPI_STATUS2_QE));
    spi_bb_cs(p, 1);
    spi_bb_wait_ready(p, 200);

    uint8_t check = spi_bb_read_reg(p, SPI_CMD_READ_STATUS2);
    if (check & SPI_STATUS2_QE)
        ESP_LOGI(TAG, "Flash QE set (SR2 0x%02x -> 0x%02x)", sr2, check);
    else
        ESP_LOGW(TAG, "Flash QE not set (SR2 still 0x%02x). This flash may "
                 "use a different QE location; device may not boot in quad "
                 "mode.", check);
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

        /* Sector erase can take ~400ms; allow generous margin */
        if (spi_bb_wait_ready(spi_pins, 2000) != ESP_OK) {
            ESP_LOGE(TAG, "Flash erase timed out at 0x%lx", (unsigned long)addr);
            gpio_set_level(h->pins.srst_n, 1);
            return ESP_ERR_TIMEOUT;
        }

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

        if (spi_bb_wait_ready(spi_pins, 500) != ESP_OK) {
            ESP_LOGE(TAG, "Flash page write timed out at 0x%lx", (unsigned long)addr);
            gpio_set_level(h->pins.srst_n, 1);
            return ESP_ERR_TIMEOUT;
        }

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

    if (!verify_ok) {
        gpio_set_level(h->pins.srst_n, 1);
        return ESP_FAIL;
    }

    /* The xCORE-200 boot ROM reads the flash with a quad command (0xEB),
     * so the Quad-Enable bit must be set or the device won't boot from the
     * image we just wrote.  Do this last (after data + verify), like xflash. */
    spi_bb_set_quad_enable(spi_pins);

    /* Release XMOS from reset */
    gpio_set_level(h->pins.srst_n, 1);

    ESP_LOGI(TAG, "SPI flash programmed and verified OK (%zu bytes)", image_len);
    return ESP_OK;
}
