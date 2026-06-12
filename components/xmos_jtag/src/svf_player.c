/*
 * SVF (Serial Vector Format) file player.
 *
 * Reference: ASSET InterTech "Serial Vector Format Specification"
 * Also informed by openFPGALoader svf_jtag.cpp (Apache-2.0).
 *
 * The player parses SVF line-by-line from a memory buffer (typically PSRAM)
 * and executes each command through the JTAG transport layer.
 */

#include "sdkconfig.h"
#include "jtag_svf.h"
#include "jtag_transport.h"
#include "xmos_jtag.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static const char *TAG = "svf";

/* Access the transport from the opaque handle (defined in xmos_jtag.c) */
extern jtag_transport_t *xmos_jtag_get_transport(xmos_jtag_handle_t handle);

/* -------------------------------------------------------------------------
 * Internal types
 * ---------------------------------------------------------------------- */

/* Remembered XYR data (SIR/SDR/HIR/HDR/TIR/TDR) */
typedef struct {
    uint32_t len;
    uint8_t *tdi;
    uint8_t *tdo;
    uint8_t *mask;
    size_t   alloc;          /* allocated byte size */
    bool     tdo_specified;  /* TDO present (comparison requested) */
} svf_xyr_t;

typedef struct {
    jtag_transport_t *transport;
    /* Remembered data */
    svf_xyr_t hdr, hir, tdr, tir, sdr, sir;
    /* Stats */
    size_t commands;
    size_t mismatches;
    /* Config */
    bool stop_on_mismatch;
} svf_ctx_t;

/* -------------------------------------------------------------------------
 * Hex parsing
 * ---------------------------------------------------------------------- */

static void xyr_free(svf_xyr_t *x)
{
    free(x->tdi); free(x->tdo); free(x->mask);
    memset(x, 0, sizeof(*x));
}

/*
 * Resize a scan register.  Per the SVF spec, TDI/TDO/MASK are remembered
 * between commands of the same length and reset when the length changes.
 * Default MASK: all-care for SIR/SDR, all-don't-care for the header and
 * trailer registers (HIR/HDR/TIR/TDR).
 */
static esp_err_t xyr_resize(svf_xyr_t *x, uint32_t bits, uint8_t mask_default)
{
    size_t bytes = (bits + 7) / 8;
    if (bytes > x->alloc) {
        free(x->tdi); free(x->tdo); free(x->mask);
        x->tdi = calloc(1, bytes);
        x->tdo = calloc(1, bytes);
        x->mask = calloc(1, bytes);
        if (!x->tdi || !x->tdo || !x->mask) return ESP_ERR_NO_MEM;
        x->alloc = bytes;
    }
    if (bits != x->len) {
        memset(x->tdi, 0, bytes);
        memset(x->tdo, 0, bytes);
        memset(x->mask, mask_default, bytes);
        x->tdo_specified = false;
    }
    x->len = bits;
    return ESP_OK;
}

/* Copy `nbits` bits from src (LSB-first bit array) into dst at dst_off bits */
static void copy_bits(uint8_t *dst, size_t dst_off,
                      const uint8_t *src, size_t nbits)
{
    for (size_t i = 0; i < nbits; i++) {
        if ((src[i / 8] >> (i % 8)) & 1)
            dst[(dst_off + i) / 8] |= (uint8_t)(1u << ((dst_off + i) % 8));
    }
}

/* Parse hex string like "DEADBEEF" into byte array (LSB at index 0).
 * SVF hex is MSB-first in the string. */
static void parse_hex_to_bytes(const char *hex, size_t hex_len,
                               uint8_t *out, size_t out_bytes)
{
    if (!out || out_bytes == 0)
        return;
    memset(out, 0, out_bytes);
    /* Process from the end of the hex string */
    int byte_idx = 0, nibble = 0;
    for (int i = (int)hex_len - 1; i >= 0 && byte_idx < (int)out_bytes; i--) {
        char c = hex[i];
        uint8_t v;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
        else continue;  /* skip whitespace */

        if (nibble == 0) {
            out[byte_idx] = v;
            nibble = 1;
        } else {
            out[byte_idx] |= (v << 4);
            nibble = 0;
            byte_idx++;
        }
    }
}

/* -------------------------------------------------------------------------
 * Token parser -- extract next whitespace-delimited token
 * ---------------------------------------------------------------------- */
static const char *skip_ws(const char *p, const char *end)
{
    while (p < end && isspace((unsigned char)*p)) p++;
    return p;
}

static const char *next_token(const char *p, const char *end,
                              const char **tok_start, size_t *tok_len)
{
    p = skip_ws(p, end);
    *tok_start = p;
    while (p < end && !isspace((unsigned char)*p) && *p != ';' && *p != '(') p++;
    *tok_len = p - *tok_start;
    return p;
}

/* Extract hex data between parentheses: (DEADBEEF) */
static const char *parse_paren_hex(const char *p, const char *end,
                                   uint8_t *out, size_t out_bytes)
{
    p = skip_ws(p, end);
    if (p < end && *p == '(') p++;

    /* Collect all hex chars until ')' */
    const char *hex_start = p;
    while (p < end && *p != ')') p++;
    size_t hex_len = p - hex_start;
    if (p < end && *p == ')') p++;

    parse_hex_to_bytes(hex_start, hex_len, out, out_bytes);
    return p;
}

/* -------------------------------------------------------------------------
 * SVF command handlers
 * ---------------------------------------------------------------------- */

/*
 * Compare a region of the captured TDO stream (starting at bit_off) against
 * the expected TDO/MASK of one scan register.  Returns true on match.
 */
static bool region_matches(const uint8_t *captured, size_t bit_off,
                           const svf_xyr_t *x)
{
    for (uint32_t i = 0; i < x->len; i++) {
        if (!((x->mask[i / 8] >> (i % 8)) & 1))
            continue;
        int got = (captured[(bit_off + i) / 8] >> ((bit_off + i) % 8)) & 1;
        int exp = (x->tdo[i / 8] >> (i % 8)) & 1;
        if (got != exp)
            return false;
    }
    return true;
}

static esp_err_t handle_xyr(svf_ctx_t *ctx, const char *p, const char *end,
                            svf_xyr_t *xyr, int shift_type)
{
    /* shift_type: 0=SIR, 1=SDR, -1=header/trailer (don't shift) */
    const char *tok; size_t tlen;
    p = next_token(p, end, &tok, &tlen);  /* length field */
    uint32_t len = (uint32_t)strtoul(tok, NULL, 10);

    esp_err_t err = xyr_resize(xyr, len, shift_type >= 0 ? 0xFF : 0x00);
    if (err != ESP_OK) return err;

    size_t bytes = (len + 7) / 8;
    bool saw_tdo = false;

    /* Parse optional TDI, TDO, MASK, SMASK */
    while (p < end && len > 0) {
        p = next_token(p, end, &tok, &tlen);
        if (tlen == 0) break;

        if (tlen == 3 && strncasecmp(tok, "TDI", 3) == 0) {
            p = parse_paren_hex(p, end, xyr->tdi, bytes);
        } else if (tlen == 3 && strncasecmp(tok, "TDO", 3) == 0) {
            p = parse_paren_hex(p, end, xyr->tdo, bytes);
            saw_tdo = true;
        } else if (tlen == 4 && strncasecmp(tok, "MASK", 4) == 0) {
            p = parse_paren_hex(p, end, xyr->mask, bytes);
        } else if (tlen == 5 && strncasecmp(tok, "SMASK", 5) == 0) {
            /* SMASK marks TDI don't-care bits; it must NOT alter the
             * driven TDI values, so parse and discard it. */
            p = parse_paren_hex(p, end, NULL, 0);
        }
    }

    if (shift_type < 0) {
        /* Header/trailer register: TDO presence is sticky until the
         * length changes (compared as part of every following scan). */
        if (saw_tdo)
            xyr->tdo_specified = true;
        return ESP_OK;
    }

    /* SIR/SDR: a comparison happens iff TDO is present in THIS command
     * (an expected value of all-zeros is still a real check). */
    xyr->tdo_specified = saw_tdo;

    /* Build the full scan: header + data + trailer, header shifted first */
    svf_xyr_t *head = (shift_type == 0) ? &ctx->hir : &ctx->hdr;
    svf_xyr_t *tail = (shift_type == 0) ? &ctx->tir : &ctx->tdr;

    size_t total_bits = (size_t)head->len + len + tail->len;
    if (total_bits == 0)
        return ESP_OK;
    size_t total_bytes = (total_bits + 7) / 8;

    uint8_t *tdi_buf = calloc(1, total_bytes);
    uint8_t *tdo_buf = calloc(1, total_bytes);
    if (!tdi_buf || !tdo_buf) {
        free(tdi_buf); free(tdo_buf);
        return ESP_ERR_NO_MEM;
    }

    copy_bits(tdi_buf, 0, head->tdi, head->len);
    copy_bits(tdi_buf, head->len, xyr->tdi, len);
    copy_bits(tdi_buf, head->len + len, tail->tdi, tail->len);

    if (shift_type == 0) {
        err = ctx->transport->shift_ir(ctx->transport,
                                       tdi_buf, tdo_buf, total_bits);
    } else {
        err = ctx->transport->shift_dr(ctx->transport,
                                       tdi_buf, tdo_buf, total_bits);
    }
    free(tdi_buf);
    if (err != ESP_OK) {
        free(tdo_buf);
        return err;
    }

    /* Check captured TDO region by region */
    bool ok = true;
    if (head->tdo_specified && !region_matches(tdo_buf, 0, head))
        ok = false;
    if (ok && xyr->tdo_specified && !region_matches(tdo_buf, head->len, xyr))
        ok = false;
    if (ok && tail->tdo_specified &&
        !region_matches(tdo_buf, head->len + len, tail))
        ok = false;
    free(tdo_buf);

    if (!ok) {
        ctx->mismatches++;
        ESP_LOGW(TAG, "%s TDO mismatch (scan of %zu bits)",
                 shift_type == 0 ? "SIR" : "SDR", total_bits);
        if (ctx->stop_on_mismatch)
            return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t handle_runtest(svf_ctx_t *ctx, const char *p, const char *end)
{
    const char *tok; size_t tlen;
    uint64_t clocks = 0;
    double seconds = 0;

    while (p < end) {
        p = next_token(p, end, &tok, &tlen);
        if (tlen == 0) break;

        /* Check for state name (IDLE, DRPAUSE, etc.) -- skip */
        if (isalpha((unsigned char)tok[0]) && tlen > 2) {
            continue;
        }

        /* Number followed by TCK or SEC */
        if (isdigit((unsigned char)tok[0]) || tok[0] == '.') {
            double val = strtod(tok, NULL);
            const char *unit_tok; size_t unit_len;
            p = next_token(p, end, &unit_tok, &unit_len);
            if (unit_len >= 3 && strncasecmp(unit_tok, "TCK", 3) == 0) {
                clocks = (uint64_t)val;
            } else if (unit_len >= 3 && strncasecmp(unit_tok, "SEC", 3) == 0) {
                seconds = val;
            }
        }
    }

    /* RUNTEST keeps the TAP in the run state with TCK running -- a plain
     * delay would starve devices that clock internal logic from TCK.
     * Convert a SEC requirement into TCK cycles at the configured rate
     * (minimum time, so rounding up / overshooting is fine). */
    if (seconds > 0) {
        uint64_t sec_clocks =
            (uint64_t)(seconds * (double)CONFIG_XMOS_JTAG_TCK_FREQ_KHZ * 1000.0) + 1;
        if (sec_clocks > clocks)
            clocks = sec_clocks;
    }

    while (clocks > 0) {
        unsigned chunk = (clocks > 0x10000000ull) ? 0x10000000u : (unsigned)clocks;
        esp_err_t err = ctx->transport->idle(ctx->transport, chunk);
        if (err != ESP_OK) return err;
        clocks -= chunk;
    }

    return ESP_OK;
}

static esp_err_t handle_state(svf_ctx_t *ctx, const char *p, const char *end)
{
    const char *tok; size_t tlen;
    p = next_token(p, end, &tok, &tlen);

    /* We only handle RESET and IDLE transitions */
    if (tlen >= 5 && strncasecmp(tok, "RESET", 5) == 0) {
        return ctx->transport->reset(ctx->transport);
    }
    /* IDLE: just clock once with TMS=0 (we're likely already in RTI) */
    if (tlen >= 4 && strncasecmp(tok, "IDLE", 4) == 0) {
        return ctx->transport->idle(ctx->transport, 1);
    }
    /* Other states: we'd need full TAP state tracking. For now, ignore. */
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Main SVF parser loop
 * ---------------------------------------------------------------------- */

esp_err_t svf_play(void *jtag_handle,
                   const char *svf_data, size_t svf_len,
                   const svf_config_t *config,
                   svf_result_t *result)
{
    if (!jtag_handle || !svf_data || svf_len == 0)
        return ESP_ERR_INVALID_ARG;

    svf_ctx_t ctx = {
        .transport = xmos_jtag_get_transport((xmos_jtag_handle_t)jtag_handle),
        .stop_on_mismatch = config ? config->stop_on_mismatch : false,
    };

    const char *p = svf_data;
    const char *end = svf_data + svf_len;

    /* Statement buffer (SVF statements end with ;).  Grown on demand:
     * vendor SVFs routinely carry single SDR statements of hundreds of KB
     * (e.g. FPGA CRAM via JTAG), so a fixed buffer would truncate them. */
    size_t stmt_cap = 8192;
    char *stmt = malloc(stmt_cap);
    if (!stmt) return ESP_ERR_NO_MEM;
    size_t stmt_len = 0;
    esp_err_t err = ESP_OK;

    /* Reset TAP before playing */
    ctx.transport->reset(ctx.transport);

    while (p < end && err == ESP_OK) {
        /* Skip to next non-whitespace */
        while (p < end && isspace((unsigned char)*p)) p++;
        if (p >= end) break;

        /* Skip comments (! or //) */
        if (*p == '!' || (p + 1 < end && p[0] == '/' && p[1] == '/')) {
            while (p < end && *p != '\n') p++;
            continue;
        }

        /* Read until semicolon (statement terminator) */
        while (p < end && *p != ';') {
            if (*p == '!' || (p + 1 < end && p[0] == '/' && p[1] == '/')) {
                /* Skip inline comment */
                while (p < end && *p != '\n') p++;
                continue;
            }
            if (stmt_len + 1 >= stmt_cap) {
                size_t new_cap = stmt_cap * 2;
                char *grown = realloc(stmt, new_cap);
                if (!grown) {
                    err = ESP_ERR_NO_MEM;
                    break;
                }
                stmt = grown;
                stmt_cap = new_cap;
            }
            stmt[stmt_len++] = (char)toupper((unsigned char)*p);
            p++;
        }
        if (err != ESP_OK) break;
        if (p < end && *p == ';') p++;  /* consume semicolon */

        if (stmt_len == 0) continue;
        stmt[stmt_len] = '\0';

        /* Parse the first token (command) */
        const char *s = stmt;
        const char *s_end = stmt + stmt_len;
        const char *cmd; size_t cmd_len;
        s = next_token(s, s_end, &cmd, &cmd_len);

        if (cmd_len == 3 && memcmp(cmd, "SIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.sir, 0);
        } else if (cmd_len == 3 && memcmp(cmd, "SDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.sdr, 1);
        } else if (cmd_len == 3 && memcmp(cmd, "HIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.hir, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "HDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.hdr, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "TIR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.tir, -1);
        } else if (cmd_len == 3 && memcmp(cmd, "TDR", 3) == 0) {
            err = handle_xyr(&ctx, s, s_end, &ctx.tdr, -1);
        } else if (cmd_len == 7 && memcmp(cmd, "RUNTEST", 7) == 0) {
            err = handle_runtest(&ctx, s, s_end);
        } else if (cmd_len == 5 && memcmp(cmd, "STATE", 5) == 0) {
            err = handle_state(&ctx, s, s_end);
        } else if (cmd_len == 5 && (memcmp(cmd, "ENDDR", 5) == 0 ||
                                    memcmp(cmd, "ENDIR", 5) == 0)) {
            /* The transport always returns to Run-Test/Idle after a scan
             * and cannot park in a Pause state, so any end state other
             * than IDLE cannot be honoured -- fail loudly rather than
             * silently mis-executing every following scan. */
            s = next_token(s, s_end, &cmd, &cmd_len);
            if (!(cmd_len == 4 && memcmp(cmd, "IDLE", 4) == 0)) {
                ESP_LOGE(TAG, "Unsupported %.5s %.*s (only IDLE supported)",
                         stmt, (int)cmd_len, cmd);
                err = ESP_ERR_NOT_SUPPORTED;
            }
        } else if (cmd_len == 9 && memcmp(cmd, "FREQUENCY", 9) == 0) {
            /* FREQUENCY <hz> HZ; -- informational, we can't change TCK dynamically */
            ESP_LOGD(TAG, "SVF FREQUENCY: %s", s);
        } else if (cmd_len == 4 && memcmp(cmd, "TRST", 4) == 0) {
            s = next_token(s, s_end, &cmd, &cmd_len);
            if (cmd_len >= 2 && memcmp(cmd, "ON", 2) == 0) {
                ctx.transport->reset(ctx.transport);
            }
        } else {
            ESP_LOGD(TAG, "SVF: ignoring '%.*s'", (int)cmd_len, cmd);
        }

        ctx.commands++;
        stmt_len = 0;

        /* Progress callback */
        if (config && config->progress_cb && (ctx.commands % 100) == 0) {
            size_t processed = (size_t)(p - svf_data);
            config->progress_cb(processed, svf_len, ctx.commands, config->user_ctx);
        }
    }

    /* Final progress */
    if (config && config->progress_cb) {
        config->progress_cb(svf_len, svf_len, ctx.commands, config->user_ctx);
    }

    /* Cleanup */
    xyr_free(&ctx.hdr); xyr_free(&ctx.hir);
    xyr_free(&ctx.tdr); xyr_free(&ctx.tir);
    xyr_free(&ctx.sdr); xyr_free(&ctx.sir);
    free(stmt);

    if (result) {
        result->commands_executed = ctx.commands;
        result->tdo_mismatches = ctx.mismatches;
        result->bytes_processed = svf_len;
        result->error = err;
    }

    ESP_LOGI(TAG, "SVF done: %zu commands, %zu mismatches, %s",
             ctx.commands, ctx.mismatches,
             err == ESP_OK ? "OK" : esp_err_to_name(err));

    return err;
}
