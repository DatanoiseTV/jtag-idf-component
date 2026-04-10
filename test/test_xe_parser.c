/*
 * Host-side test for the XE file parser.
 *
 * Compile and run on your development machine (not ESP32):
 *   cc -o test_xe_parser test_xe_parser.c -I../components/xmos_jtag/src -DTEST_HOST
 *   ./test_xe_parser path/to/firmware.xe
 *   ./test_xe_parser path/to/firmware.factory.bin   (should reject gracefully)
 *
 * Also runs built-in tests with synthetic XE data when called without args.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

/* -------------------------------------------------------------------------
 * Stubs for ESP-IDF types/macros when building on host
 * ---------------------------------------------------------------------- */
#ifdef TEST_HOST

typedef int esp_err_t;
#define ESP_OK              0
#define ESP_FAIL            (-1)
#define ESP_ERR_NO_MEM      0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_SIZE 0x103
#define ESP_ERR_NOT_FOUND   0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT     0x107

#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "E [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "W [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "I [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) fprintf(stderr, "D [%s] " fmt "\n", tag, ##__VA_ARGS__)

#endif /* TEST_HOST */

/* Include the XE parser source directly (single-file compilation) */
#include "xmos_regs.h"

/* Pull in xmos_xe.h but define esp_err_t before it */
#include "xmos_xe.h"

/* Pull in the implementation */
#include "../components/xmos_jtag/src/xmos_xe.c"

/* =========================================================================
 * Test helpers
 * ======================================================================= */
static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  TEST: %-50s ", #name); \
    fflush(stdout); \
} while (0)

#define PASS() do { tests_passed++; printf("PASS\n"); } while (0)
#define FAIL(msg) do { printf("FAIL: %s\n", msg); } while (0)

/* Little-endian write helpers for building test data */
static void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void wr32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static void wr64(uint8_t *p, uint64_t v) { wr32(p, (uint32_t)v); wr32(p + 4, (uint32_t)(v >> 32)); }

/* =========================================================================
 * Build a minimal valid ELF32 with one PT_LOAD segment
 * ======================================================================= */
static size_t build_test_elf(uint8_t *buf, size_t bufsize,
                             uint32_t entry, uint32_t paddr,
                             const uint8_t *code, size_t code_len)
{
    /* ELF header = 52 bytes, 1 program header = 32 bytes, then code */
    size_t phoff = 52;
    size_t data_off = 52 + 32;
    size_t total = data_off + code_len;
    assert(total <= bufsize);

    memset(buf, 0, total);

    /* ELF magic */
    buf[0] = 0x7F; buf[1] = 'E'; buf[2] = 'L'; buf[3] = 'F';
    buf[4] = 1;  /* ELFCLASS32 */
    buf[5] = 1;  /* ELFDATA2LSB */
    buf[6] = 1;  /* EV_CURRENT */

    wr16(buf + 16, 2);      /* e_type = ET_EXEC */
    wr16(buf + 18, 0xCB);   /* e_machine = xCORE */
    wr32(buf + 20, 1);      /* e_version */
    wr32(buf + 24, entry);  /* e_entry */
    wr32(buf + 28, phoff);  /* e_phoff */
    wr16(buf + 40, 52);     /* e_ehsize */
    wr16(buf + 42, 32);     /* e_phentsize */
    wr16(buf + 44, 1);      /* e_phnum */

    /* Program header (PT_LOAD) */
    uint8_t *ph = buf + phoff;
    wr32(ph + 0, 1);           /* p_type = PT_LOAD */
    wr32(ph + 4, data_off);    /* p_offset */
    wr32(ph + 8, paddr);       /* p_vaddr */
    wr32(ph + 12, paddr);      /* p_paddr */
    wr32(ph + 16, code_len);   /* p_filesz */
    wr32(ph + 20, code_len);   /* p_memsz */
    wr32(ph + 24, 5);          /* p_flags = PF_R | PF_X */
    wr32(ph + 28, 4);          /* p_align */

    memcpy(buf + data_off, code, code_len);
    return total;
}

/* =========================================================================
 * Build an XE file with sectors
 * ======================================================================= */

/* Append an XE sector. Returns new offset after the sector. */
static size_t xe_add_sector(uint8_t *buf, size_t off,
                            uint16_t type, uint8_t pad_byte,
                            const uint8_t *sub_hdr, size_t sub_hdr_len,
                            const uint8_t *data, size_t data_len)
{
    size_t content_len = sub_hdr_len + data_len + pad_byte;
    uint64_t length = (content_len > 0) ? (content_len + 4) : 0;  /* +4 for pad desc */

    /* Sector header: type(2) + skip(2) + length(8) = 12 bytes */
    wr16(buf + off, type);
    wr16(buf + off + 2, 0);
    wr64(buf + off + 4, length);
    off += 12;

    if (length > 0) {
        /* Padding descriptor: pad_byte(1) + skip(3) */
        buf[off] = pad_byte;
        buf[off + 1] = buf[off + 2] = buf[off + 3] = 0;
        off += 4;

        if (sub_hdr_len > 0) {
            memcpy(buf + off, sub_hdr, sub_hdr_len);
            off += sub_hdr_len;
        }
        if (data_len > 0) {
            memcpy(buf + off, data, data_len);
            off += data_len;
        }
        /* Padding bytes (zeros) */
        memset(buf + off, 0, pad_byte);
        off += pad_byte;
    }

    return off;
}

/* Build a 12-byte sub-header for ELF/GOTO/CALL sectors */
static void build_sub_hdr(uint8_t *buf, uint16_t node, uint16_t core, uint64_t addr)
{
    wr16(buf, node);
    wr16(buf + 2, core);
    wr64(buf + 4, addr);
}

/* =========================================================================
 * Tests
 * ======================================================================= */

static void test_reject_too_small(void)
{
    TEST(reject_too_small);
    xe_parsed_t parsed;
    uint8_t tiny[] = { 'X', 'M' };
    esp_err_t err = xe_parse(tiny, sizeof(tiny), &parsed);
    if (err == ESP_ERR_INVALID_SIZE) PASS(); else FAIL("expected INVALID_SIZE");
}

static void test_reject_bad_magic(void)
{
    TEST(reject_bad_magic);
    xe_parsed_t parsed;
    uint8_t bad[16] = { 'N', 'O', 'P', 'E', 2, 0, 0, 0 };
    esp_err_t err = xe_parse(bad, sizeof(bad), &parsed);
    if (err == ESP_ERR_INVALID_ARG) PASS(); else FAIL("expected INVALID_ARG");
}

static void test_raw_elf_fallback(void)
{
    TEST(raw_elf_fallback);
    uint8_t elf_buf[256];
    uint8_t code[] = { 0xDE, 0xAD, 0xBE, 0xEF };
    size_t elf_len = build_test_elf(elf_buf, sizeof(elf_buf),
                                    0x80000, 0x40000, code, sizeof(code));

    xe_parsed_t parsed;
    esp_err_t err = xe_parse(elf_buf, elf_len, &parsed);
    if (err != ESP_OK) { FAIL("parse failed"); return; }
    if (parsed.num_segments != 1) { FAIL("expected 1 segment"); return; }
    if (parsed.segments[0].tile != 0) { FAIL("expected tile 0"); return; }
    if (parsed.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    if (parsed.segments[0].filesz != 4) { FAIL("wrong filesz"); return; }
    if (parsed.entry_points[0] != 0x80000) { FAIL("wrong entry"); return; }
    PASS();
}

static void test_single_elf_sector(void)
{
    TEST(single_elf_sector);

    /* Build a minimal ELF */
    uint8_t elf_buf[256];
    uint8_t code[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    size_t elf_len = build_test_elf(elf_buf, sizeof(elf_buf),
                                    0x80100, 0x40000, code, sizeof(code));

    /* Build XE with one ELF sector */
    uint8_t xe[512];
    memset(xe, 0, sizeof(xe));

    /* File header */
    xe[0] = 'X'; xe[1] = 'M'; xe[2] = 'O'; xe[3] = 'S';
    wr16(xe + 4, 2);  /* version */

    /* ELF sector for core 0 */
    uint8_t sub_hdr[12];
    build_sub_hdr(sub_hdr, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0,
                               sub_hdr, 12, elf_buf, elf_len);

    /* LAST sector */
    wr16(xe + off, XE_SECTOR_LAST);
    off += 12;

    xe_parsed_t parsed;
    esp_err_t err = xe_parse(xe, off, &parsed);
    if (err != ESP_OK) { FAIL("parse failed"); return; }
    if (parsed.num_segments != 1) {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected 1 segment, got %zu", parsed.num_segments);
        FAIL(msg); return;
    }
    if (parsed.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    if (parsed.segments[0].filesz != 8) { FAIL("wrong filesz"); return; }
    if (parsed.entry_points[0] != 0x80100) { FAIL("wrong entry"); return; }
    if (parsed.num_tiles != 1) { FAIL("wrong num_tiles"); return; }
    PASS();
}

static void test_multi_tile_xe(void)
{
    TEST(multi_tile_xe);

    uint8_t code0[] = { 0xAA, 0xBB, 0xCC, 0xDD };
    uint8_t code1[] = { 0x11, 0x22, 0x33, 0x44 };

    uint8_t elf0[256], elf1[256];
    size_t elf0_len = build_test_elf(elf0, sizeof(elf0), 0x80000, 0x40000, code0, 4);
    size_t elf1_len = build_test_elf(elf1, sizeof(elf1), 0x80200, 0x42000, code1, 4);

    uint8_t xe[1024];
    memset(xe, 0, sizeof(xe));
    xe[0] = 'X'; xe[1] = 'M'; xe[2] = 'O'; xe[3] = 'S';
    wr16(xe + 4, 2);

    /* ELF sector for core 0 */
    uint8_t sh0[12], sh1[12];
    build_sub_hdr(sh0, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh0, 12, elf0, elf0_len);

    /* ELF sector for core 1 */
    build_sub_hdr(sh1, 0, 1, 0);
    off = xe_add_sector(xe, off, XE_SECTOR_ELF, 0, sh1, 12, elf1, elf1_len);

    /* GOTO for core 0 */
    uint8_t goto0[12];
    build_sub_hdr(goto0, 0, 0, 0x80000);
    off = xe_add_sector(xe, off, XE_SECTOR_GOTO, 0, goto0, 12, NULL, 0);

    /* GOTO for core 1 */
    uint8_t goto1[12];
    build_sub_hdr(goto1, 0, 1, 0x80200);
    off = xe_add_sector(xe, off, XE_SECTOR_GOTO, 0, goto1, 12, NULL, 0);

    /* LAST */
    wr16(xe + off, XE_SECTOR_LAST);
    off += 12;

    xe_parsed_t parsed;
    esp_err_t err = xe_parse(xe, off, &parsed);
    if (err != ESP_OK) { FAIL("parse failed"); return; }
    if (parsed.num_tiles != 2) { FAIL("expected 2 tiles"); return; }
    if (parsed.num_segments != 2) { FAIL("expected 2 segments"); return; }
    if (parsed.segments[0].tile != 0) { FAIL("seg0 wrong tile"); return; }
    if (parsed.segments[1].tile != 1) { FAIL("seg1 wrong tile"); return; }
    if (parsed.segments[1].paddr != 0x42000) { FAIL("seg1 wrong paddr"); return; }
    /* GOTO should override ELF entry points */
    if (parsed.entry_points[0] != 0x80000) { FAIL("tile0 entry wrong"); return; }
    if (parsed.entry_points[1] != 0x80200) { FAIL("tile1 entry wrong"); return; }
    PASS();
}

static void test_padding_byte(void)
{
    TEST(padding_byte_handling);

    /* Build an ELF with odd-length code so we need padding */
    uint8_t code[] = { 0xAA, 0xBB, 0xCC };  /* 3 bytes -- odd */
    uint8_t elf_buf[256];
    size_t elf_len = build_test_elf(elf_buf, sizeof(elf_buf),
                                    0x80000, 0x40000, code, 3);

    uint8_t xe[512];
    memset(xe, 0, sizeof(xe));
    xe[0] = 'X'; xe[1] = 'M'; xe[2] = 'O'; xe[3] = 'S';
    wr16(xe + 4, 2);

    uint8_t sub_hdr[12];
    build_sub_hdr(sub_hdr, 0, 0, 0);
    /* 1 byte of padding at end */
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 1,
                               sub_hdr, 12, elf_buf, elf_len);
    wr16(xe + off, XE_SECTOR_LAST);
    off += 12;

    xe_parsed_t parsed;
    esp_err_t err = xe_parse(xe, off, &parsed);
    if (err != ESP_OK) { FAIL("parse failed"); return; }
    if (parsed.num_segments != 1) { FAIL("expected 1 segment"); return; }
    if (parsed.segments[0].filesz != 3) {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected filesz=3, got %u", parsed.segments[0].filesz);
        FAIL(msg); return;
    }
    PASS();
}

/* =========================================================================
 * Test with a real XE file
 * ======================================================================= */
static void test_real_xe_file(const char *path)
{
    printf("\n--- Testing real XE file: %s ---\n", path);

    FILE *f = fopen(path, "rb");
    if (!f) {
        printf("  Cannot open file: %s\n", path);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t *data = malloc(fsize);
    if (!data) {
        printf("  Cannot allocate %ld bytes\n", fsize);
        fclose(f);
        return;
    }

    size_t nread = fread(data, 1, fsize, f);
    fclose(f);

    if ((long)nread != fsize) {
        printf("  Short read: %zu of %ld\n", nread, fsize);
        free(data);
        return;
    }

    TEST(real_xe_parse);
    xe_parsed_t parsed;
    esp_err_t err = xe_parse(data, nread, &parsed);

    if (err != ESP_OK) {
        char msg[64];
        snprintf(msg, sizeof(msg), "parse returned error %d", err);
        FAIL(msg);
    } else {
        if (parsed.num_segments == 0) {
            FAIL("no segments found");
        } else {
            PASS();
        }
    }

    printf("\n  Results:\n");
    printf("    Segments: %zu\n", parsed.num_segments);
    printf("    Tiles:    %d\n", parsed.num_tiles);
    for (int t = 0; t < parsed.num_tiles; t++) {
        printf("    Tile %d entry: 0x%08x\n", t, parsed.entry_points[t]);
    }

    size_t total_code = 0;
    for (size_t i = 0; i < parsed.num_segments; i++) {
        const xe_segment_t *s = &parsed.segments[i];
        printf("    Seg[%zu]: tile=%d addr=0x%08x filesz=%u memsz=%u\n",
               i, s->tile, s->paddr, s->filesz, s->memsz);
        total_code += s->filesz;
    }
    printf("    Total code: %zu bytes\n", total_code);

    free(data);
}

/* =========================================================================
 * Test XMOS register encoding helpers
 * ======================================================================= */
static void test_reg_encoding(void)
{
    TEST(chain_ir_reg_read_encoding);
    /* For register 0x05 (DBG_INT), read operation:
     * xcore_ir = (0x05 << 2) | 0x1 = 0x15
     * chain_ir = OTP_BYPASS(0x3) | (0x15 << 2) | (CHIP_BYPASS << 12) | (BSCAN_BYPASS << 16)
     * = 0x3 | (0x15 << 2) | (0xF << 12) | (0xF << 16)
     * = 0x3 | 0x54 | 0xF000 | 0xF0000
     * = 0xFF057 */
    uint32_t ir = xmos_chain_ir_reg_read(0x05);
    if (ir == 0xFF057) PASS();
    else {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected 0xFF057, got 0x%x", ir);
        FAIL(msg);
    }

    TEST(chain_ir_reg_write_encoding);
    /* Same register, write: xcore_ir = (0x05 << 2) | 0x2 = 0x16
     * = 0x3 | (0x16 << 2) | (0xF << 12) | (0xF << 16) = 0xFF05B */
    ir = xmos_chain_ir_reg_write(0x05);
    if (ir == 0xFF05B) PASS();
    else {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected 0xFF05B, got 0x%x", ir);
        FAIL(msg);
    }

    TEST(tile_to_mux);
    if (xmos_tile_to_mux(0) == 0x8 && xmos_tile_to_mux(1) == 0x9 &&
        xmos_tile_to_mux(2) == 0xA && xmos_tile_to_mux(3) == 0xB) {
        PASS();
    } else {
        FAIL("wrong MUX values");
    }
}

/* =========================================================================
 * Test PARLIO bit packing
 * ======================================================================= */
static void test_parlio_bit_packing(void)
{
    TEST(parlio_tx_packing);

    /* With data_width=2, LSB pack order, each byte = 4 JTAG cycles:
     *   bit0=TMS0, bit1=TDI0, bit2=TMS1, bit3=TDI1, ...
     *
     * Example: 4 cycles of TMS=1,TDI=0 (going to TLR)
     *   cycle 0: TMS=1,TDI=0 -> 0b01 = 1
     *   cycle 1: TMS=1,TDI=0 -> 0b01 = 1
     *   cycle 2: TMS=1,TDI=0 -> 0b01 = 1
     *   cycle 3: TMS=1,TDI=0 -> 0b01 = 1
     *   byte = (1<<0)|(1<<2)|(1<<4)|(1<<6) = 0x55
     */
    uint8_t expected = 0x55;

    /* Simulate the packing algorithm */
    uint8_t byte = 0;
    for (int c = 0; c < 4; c++) {
        int tms = 1, tdi = 0;
        uint8_t bits = (tms & 1) | ((tdi & 1) << 1);
        byte |= bits << (c * 2);
    }

    if (byte == expected) PASS();
    else {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected 0x%02x, got 0x%02x", expected, byte);
        FAIL(msg);
    }

    TEST(parlio_tx_mixed_signals);
    /* cycle 0: TMS=0,TDI=1 -> 0b10 = 2
     * cycle 1: TMS=1,TDI=1 -> 0b11 = 3
     * cycle 2: TMS=0,TDI=0 -> 0b00 = 0
     * cycle 3: TMS=1,TDI=0 -> 0b01 = 1
     * byte = (2<<0)|(3<<2)|(0<<4)|(1<<6) = 0x4E */
    byte = 0;
    int tms_seq[] = {0,1,0,1};
    int tdi_seq[] = {1,1,0,0};
    for (int c = 0; c < 4; c++) {
        uint8_t bits = (tms_seq[c] & 1) | ((tdi_seq[c] & 1) << 1);
        byte |= bits << (c * 2);
    }
    if (byte == 0x4E) PASS();
    else {
        char msg[64];
        snprintf(msg, sizeof(msg), "expected 0x4E, got 0x%02x", byte);
        FAIL(msg);
    }
}

/* =========================================================================
 * Main
 * ======================================================================= */
int main(int argc, char **argv)
{
    printf("=== XMOS JTAG Component Tests ===\n\n");

    printf("--- XE Parser: synthetic tests ---\n");
    test_reject_too_small();
    test_reject_bad_magic();
    test_raw_elf_fallback();
    test_single_elf_sector();
    test_multi_tile_xe();
    test_padding_byte();

    printf("\n--- Register encoding tests ---\n");
    test_reg_encoding();

    printf("\n--- PARLIO bit packing tests ---\n");
    test_parlio_bit_packing();

    /* Test with real XE files if provided */
    for (int i = 1; i < argc; i++) {
        test_real_xe_file(argv[i]);
    }

    printf("\n=== Results: %d/%d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
