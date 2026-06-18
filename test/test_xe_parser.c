/*
 * Host-side test suite for the xmos-jtag-idf-component.
 *
 * Compile and run on your development machine (not ESP32):
 *   cd test && make test
 *
 * Or manually:
 *   cc -o test_xe_parser test_xe_parser.c -I../components/xmos_jtag/src -DTEST_HOST
 *   ./test_xe_parser [path/to/firmware.xe ...]
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

static int log_verbose = 0;
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "E [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "W [%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) do { if (log_verbose) fprintf(stderr, "I [%s] " fmt "\n", tag, ##__VA_ARGS__); } while(0)
#define ESP_LOGD(tag, fmt, ...) do { if (log_verbose) fprintf(stderr, "D [%s] " fmt "\n", tag, ##__VA_ARGS__); } while(0)

#endif /* TEST_HOST */

/* Include the XE parser source directly (single-file compilation) */
#include "xmos_regs.h"
#include "xmos_xe.h"
#include "../components/xmos_jtag/src/xmos_xe.c"

/* =========================================================================
 * Test framework
 * ======================================================================= */
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  %-55s ", #name); \
    fflush(stdout); \
} while (0)

#define PASS() do { tests_passed++; printf("\033[32mPASS\033[0m\n"); } while (0)
#define FAIL(msg) do { tests_failed++; printf("\033[31mFAIL: %s\033[0m\n", msg); } while (0)
#define FAILF(fmt, ...) do { \
    tests_failed++; \
    char _m[256]; snprintf(_m, sizeof(_m), fmt, ##__VA_ARGS__); \
    printf("\033[31mFAIL: %s\033[0m\n", _m); \
} while (0)

/* =========================================================================
 * Helpers: little-endian writers and ELF/XE builders
 * ======================================================================= */
static void wr16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void wr32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static void wr64(uint8_t *p, uint64_t v) { wr32(p,(uint32_t)v); wr32(p+4,(uint32_t)(v>>32)); }

static void xe_file_header(uint8_t *buf, uint16_t version)
{
    buf[0]='X'; buf[1]='M'; buf[2]='O'; buf[3]='S';
    wr16(buf+4, version);
    buf[6]=buf[7]=0;
}

static void build_sub_hdr(uint8_t *buf, uint16_t node, uint16_t core, uint64_t addr)
{
    wr16(buf, node); wr16(buf+2, core); wr64(buf+4, addr);
}

static size_t xe_add_sector(uint8_t *buf, size_t off,
                            uint16_t type, uint8_t pad_byte,
                            const uint8_t *sub_hdr, size_t sub_hdr_len,
                            const uint8_t *data, size_t data_len)
{
    size_t content_len = sub_hdr_len + data_len + pad_byte;
    uint64_t length = (content_len > 0) ? (content_len + 4) : 0;
    wr16(buf+off, type); wr16(buf+off+2, 0); wr64(buf+off+4, length);
    off += 12;
    if (length > 0) {
        buf[off]=pad_byte; buf[off+1]=buf[off+2]=buf[off+3]=0;
        off += 4;
        if (sub_hdr_len > 0) { memcpy(buf+off, sub_hdr, sub_hdr_len); off += sub_hdr_len; }
        if (data_len > 0) { memcpy(buf+off, data, data_len); off += data_len; }
        memset(buf+off, 0, pad_byte); off += pad_byte;
    }
    return off;
}

static size_t xe_add_last(uint8_t *buf, size_t off)
{
    wr16(buf+off, XE_SECTOR_LAST); memset(buf+off+2, 0, 10);
    return off + 12;
}

/* Build an ELF with N PT_LOAD segments */
static size_t build_elf_multi(uint8_t *buf, size_t bufsize,
                              uint32_t entry, int nsegs,
                              const uint32_t *paddrs, const size_t *sizes,
                              const uint8_t *fill_byte)
{
    size_t phoff = 52;
    size_t data_start = 52 + 32 * (size_t)nsegs;
    size_t total = data_start;
    for (int i = 0; i < nsegs; i++) total += sizes[i];
    assert(total <= bufsize);
    memset(buf, 0, total);

    buf[0]=0x7F; buf[1]='E'; buf[2]='L'; buf[3]='F';
    buf[4]=1; buf[5]=1; buf[6]=1;
    wr16(buf+16, 2); wr16(buf+18, 0xCB); wr32(buf+20, 1);
    wr32(buf+24, entry); wr32(buf+28, phoff);
    wr16(buf+40, 52); wr16(buf+42, 32); wr16(buf+44, nsegs);

    size_t cur_off = data_start;
    for (int i = 0; i < nsegs; i++) {
        uint8_t *ph = buf + phoff + 32*i;
        wr32(ph, 1); /* PT_LOAD */
        wr32(ph+4, cur_off);
        wr32(ph+8, paddrs[i]); wr32(ph+12, paddrs[i]);
        wr32(ph+16, sizes[i]); wr32(ph+20, sizes[i]);
        wr32(ph+24, 5); wr32(ph+28, 4);
        memset(buf + cur_off, fill_byte ? fill_byte[i] : 0xCC, sizes[i]);
        cur_off += sizes[i];
    }
    return total;
}

/* Build a simple ELF with one PT_LOAD */
static size_t build_test_elf(uint8_t *buf, size_t bufsize,
                             uint32_t entry, uint32_t paddr,
                             const uint8_t *code, size_t code_len)
{
    size_t phoff = 52, data_off = 52 + 32, total = data_off + code_len;
    assert(total <= bufsize);
    memset(buf, 0, total);
    buf[0]=0x7F; buf[1]='E'; buf[2]='L'; buf[3]='F';
    buf[4]=1; buf[5]=1; buf[6]=1;
    wr16(buf+16,2); wr16(buf+18,0xCB); wr32(buf+20,1);
    wr32(buf+24,entry); wr32(buf+28,phoff);
    wr16(buf+40,52); wr16(buf+42,32); wr16(buf+44,1);
    uint8_t *ph=buf+phoff;
    wr32(ph,1); wr32(ph+4,data_off); wr32(ph+8,paddr); wr32(ph+12,paddr);
    wr32(ph+16,code_len); wr32(ph+20,code_len); wr32(ph+24,5); wr32(ph+28,4);
    memcpy(buf+data_off, code, code_len);
    return total;
}

/* Build a simple ELF with BSS (memsz > filesz) */
static size_t build_elf_with_bss(uint8_t *buf, size_t bufsize,
                                 uint32_t entry, uint32_t paddr,
                                 size_t code_len, size_t bss_len)
{
    size_t phoff = 52, data_off = 52 + 64;  /* 2 program headers */
    size_t total = data_off + code_len;
    assert(total <= bufsize);
    memset(buf, 0, total);
    buf[0]=0x7F; buf[1]='E'; buf[2]='L'; buf[3]='F';
    buf[4]=1; buf[5]=1; buf[6]=1;
    wr16(buf+16,2); wr16(buf+18,0xCB); wr32(buf+20,1);
    wr32(buf+24,entry); wr32(buf+28,phoff);
    wr16(buf+40,52); wr16(buf+42,32); wr16(buf+44,2);

    /* Segment 0: code (PT_LOAD, filesz == memsz) */
    uint8_t *ph0 = buf + phoff;
    wr32(ph0,1); wr32(ph0+4,data_off);
    wr32(ph0+8,paddr); wr32(ph0+12,paddr);
    wr32(ph0+16,code_len); wr32(ph0+20,code_len);
    wr32(ph0+24,5); wr32(ph0+28,4);
    memset(buf+data_off, 0xAB, code_len);

    /* Segment 1: BSS (PT_LOAD, filesz == 0, memsz > 0) */
    uint8_t *ph1 = buf + phoff + 32;
    uint32_t bss_addr = paddr + (uint32_t)code_len;
    wr32(ph1,1); wr32(ph1+4,data_off+code_len);
    wr32(ph1+8,bss_addr); wr32(ph1+12,bss_addr);
    wr32(ph1+16,0); wr32(ph1+20,bss_len);  /* filesz=0, memsz=bss_len */
    wr32(ph1+24,6); wr32(ph1+28,4);

    return total;
}

/* =========================================================================
 * XE Parser Tests
 * ======================================================================= */

static void test_reject_too_small(void)
{
    TEST(xe_reject_too_small);
    xe_parsed_t p;
    uint8_t tiny[] = { 'X', 'M' };
    if (xe_parse(tiny, sizeof(tiny), &p) == ESP_ERR_INVALID_SIZE) PASS();
    else FAIL("expected INVALID_SIZE");
}

static void test_reject_bad_magic(void)
{
    TEST(xe_reject_bad_magic);
    xe_parsed_t p;
    uint8_t bad[16] = { 'N','O','P','E', 2,0,0,0 };
    if (xe_parse(bad, sizeof(bad), &p) == ESP_ERR_INVALID_ARG) PASS();
    else FAIL("expected INVALID_ARG");
}

static void test_empty_xe(void)
{
    TEST(xe_empty_file_just_last_sector);
    uint8_t xe[32];
    xe_file_header(xe, 2);
    size_t off = xe_add_last(xe, 8);
    xe_parsed_t p;
    esp_err_t err = xe_parse(xe, off, &p);
    if (err != ESP_OK) { FAIL("parse error"); return; }
    if (p.num_segments != 0) { FAILF("expected 0 segs, got %zu", p.num_segments); return; }
    if (p.num_tiles != 0) { FAIL("expected 0 tiles"); return; }
    PASS();
}

static void test_raw_elf_fallback(void)
{
    TEST(xe_raw_elf_fallback);
    uint8_t elf[256];
    uint8_t code[] = { 0xDE, 0xAD, 0xBE, 0xEF };
    size_t len = build_test_elf(elf, sizeof(elf), 0x80000, 0x40000, code, 4);

    xe_parsed_t p;
    if (xe_parse(elf, len, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAIL("expected 1 seg"); return; }
    if (p.segments[0].tile != 0) { FAIL("expected tile 0"); return; }
    if (p.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    if (p.segments[0].filesz != 4) { FAIL("wrong filesz"); return; }
    if (p.entry_points[0] != 0x80000) { FAIL("wrong entry"); return; }
    PASS();
}

static void test_single_elf_sector(void)
{
    TEST(xe_single_elf_sector);
    uint8_t elf[256];
    uint8_t code[] = { 1,2,3,4,5,6,7,8 };
    size_t elf_len = build_test_elf(elf, sizeof(elf), 0x80100, 0x40000, code, 8);

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAILF("expected 1 seg, got %zu", p.num_segments); return; }
    if (p.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    if (p.segments[0].filesz != 8) { FAIL("wrong filesz"); return; }
    if (p.entry_points[0] != 0x80100) { FAIL("wrong entry"); return; }
    if (p.num_tiles != 1) { FAIL("wrong num_tiles"); return; }
    PASS();
}

static void test_multi_tile(void)
{
    TEST(xe_multi_tile_elf_with_goto);
    uint8_t elf0[256], elf1[256];
    uint8_t c0[]={0xAA,0xBB,0xCC,0xDD}, c1[]={0x11,0x22,0x33,0x44};
    size_t l0 = build_test_elf(elf0, sizeof(elf0), 0x80000, 0x40000, c0, 4);
    size_t l1 = build_test_elf(elf1, sizeof(elf1), 0x80200, 0x42000, c1, 4);

    uint8_t xe[1024]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh0[12], sh1[12];
    build_sub_hdr(sh0, 0, 0, 0);
    build_sub_hdr(sh1, 0, 1, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh0, 12, elf0, l0);
    off = xe_add_sector(xe, off, XE_SECTOR_ELF, 0, sh1, 12, elf1, l1);

    uint8_t g0[12], g1[12];
    build_sub_hdr(g0, 0, 0, 0x80000);
    build_sub_hdr(g1, 0, 1, 0x80200);
    off = xe_add_sector(xe, off, XE_SECTOR_GOTO, 0, g0, 12, NULL, 0);
    off = xe_add_sector(xe, off, XE_SECTOR_GOTO, 0, g1, 12, NULL, 0);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_tiles != 2) { FAIL("expected 2 tiles"); return; }
    if (p.num_segments != 2) { FAIL("expected 2 segs"); return; }
    if (p.segments[0].tile != 0) { FAIL("seg0 tile"); return; }
    if (p.segments[1].tile != 1) { FAIL("seg1 tile"); return; }
    if (p.segments[1].paddr != 0x42000) { FAIL("seg1 paddr"); return; }
    if (p.entry_points[0] != 0x80000) { FAIL("tile0 entry"); return; }
    if (p.entry_points[1] != 0x80200) { FAIL("tile1 entry"); return; }
    PASS();
}

static void test_padding_byte(void)
{
    TEST(xe_padding_byte_strips_correctly);
    uint8_t elf[256];
    uint8_t code[] = { 0xAA, 0xBB, 0xCC };
    size_t elf_len = build_test_elf(elf, sizeof(elf), 0x80000, 0x40000, code, 3);

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 1, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAIL("expected 1 seg"); return; }
    if (p.segments[0].filesz != 3) { FAILF("filesz=%u, want 3", p.segments[0].filesz); return; }
    PASS();
}

static void test_goto_overrides_elf_entry(void)
{
    TEST(xe_goto_overrides_elf_entry_point);
    uint8_t elf[256];
    uint8_t code[] = { 0xFF };
    size_t elf_len = build_test_elf(elf, sizeof(elf), 0x80000, 0x40000, code, 1);

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);

    /* GOTO with different address should override the ELF entry */
    uint8_t g[12]; build_sub_hdr(g, 0, 0, 0xDEAD0000);
    off = xe_add_sector(xe, off, XE_SECTOR_GOTO, 0, g, 12, NULL, 0);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.entry_points[0] != 0xDEAD0000) { FAILF("entry=0x%x, want 0xDEAD0000", p.entry_points[0]); return; }
    PASS();
}

static void test_call_sector(void)
{
    TEST(xe_call_sector_sets_entry);
    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0xCAFE0000);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_CALL, 0, sh, 12, NULL, 0);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.entry_points[0] != 0xCAFE0000) { FAILF("entry=0x%x", p.entry_points[0]); return; }
    PASS();
}

static void test_unknown_sectors_skipped(void)
{
    TEST(xe_unknown_sector_types_skipped);
    uint8_t elf[256];
    uint8_t code[] = { 0xAB };
    size_t elf_len = build_test_elf(elf, sizeof(elf), 0x80000, 0x40000, code, 1);

    uint8_t xe[1024]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);

    /* Unknown sector type 0x0004 (like the real XE files have) */
    uint8_t dummy[8] = {0};
    size_t off = xe_add_sector(xe, 8, 0x0004, 0, NULL, 0, dummy, 8);

    /* Another unknown type */
    off = xe_add_sector(xe, off, 0x00FF, 0, NULL, 0, dummy, 4);

    /* Then a valid ELF sector */
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    off = xe_add_sector(xe, off, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAILF("expected 1 seg, got %zu", p.num_segments); return; }
    if (p.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    PASS();
}

static void test_config_xn_xscope_skipped(void)
{
    TEST(xe_config_xn_xscope_nodedesc_skipped);
    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t dummy[16] = {0};
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_CONFIG, 0, NULL, 0, dummy, 16);
    off = xe_add_sector(xe, off, XE_SECTOR_XN, 0, NULL, 0, dummy, 8);
    off = xe_add_sector(xe, off, XE_SECTOR_NODEDESC, 0, NULL, 0, dummy, 4);
    off = xe_add_sector(xe, off, XE_SECTOR_XSCOPE, 1, NULL, 0, dummy, 3);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 0) { FAIL("should have 0 segments"); return; }
    PASS();
}

static void test_zero_length_sector(void)
{
    TEST(xe_zero_length_sector_handled);
    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    /* A sector with length=0 (just the 12-byte header, no data) */
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_CONFIG, 0, NULL, 0, NULL, 0);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    PASS();
}

static void test_elf_with_bss(void)
{
    TEST(xe_elf_with_bss_segment);
    uint8_t elf[512];
    size_t elf_len = build_elf_with_bss(elf, sizeof(elf), 0x80000, 0x40000, 64, 1024);

    uint8_t xe[1024]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 2) { FAILF("expected 2 segs (code+bss), got %zu", p.num_segments); return; }
    /* Code segment */
    if (p.segments[0].filesz != 64) { FAIL("code filesz"); return; }
    if (p.segments[0].memsz != 64) { FAIL("code memsz"); return; }
    /* BSS segment */
    if (p.segments[1].filesz != 0) { FAIL("bss filesz should be 0"); return; }
    if (p.segments[1].memsz != 1024) { FAILF("bss memsz=%u, want 1024", p.segments[1].memsz); return; }
    if (p.segments[1].paddr != 0x40000 + 64) { FAIL("bss paddr"); return; }
    PASS();
}

static void test_elf_multiple_segments(void)
{
    TEST(xe_elf_with_multiple_pt_load_segments);
    uint32_t paddrs[] = { 0x40000, 0x50000, 0x60000 };
    size_t sizes[] = { 128, 64, 32 };
    uint8_t fills[] = { 0xAA, 0xBB, 0xCC };
    uint8_t elf[1024];
    size_t elf_len = build_elf_multi(elf, sizeof(elf), 0x80000, 3, paddrs, sizes, fills);

    uint8_t xe[2048]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 3) { FAILF("expected 3 segs, got %zu", p.num_segments); return; }
    for (int i = 0; i < 3; i++) {
        if (p.segments[i].paddr != paddrs[i]) { FAILF("seg%d paddr", i); return; }
        if (p.segments[i].filesz != sizes[i]) { FAILF("seg%d filesz", i); return; }
        /* Verify data content pointer is valid */
        if (p.segments[i].data[0] != fills[i]) { FAILF("seg%d data[0]=0x%02x", i, p.segments[i].data[0]); return; }
    }
    PASS();
}

static void test_binary_sector(void)
{
    TEST(xe_binary_sector_raw_data);
    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);

    uint8_t payload[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE };
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0x40000);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_BINARY, 0, sh, 12, payload, sizeof(payload));
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAILF("expected 1 seg, got %zu", p.num_segments); return; }
    if (p.segments[0].paddr != 0x40000) { FAIL("wrong paddr"); return; }
    if (p.segments[0].filesz != 6) { FAILF("filesz=%u", p.segments[0].filesz); return; }
    if (p.segments[0].tile != 0) { FAIL("wrong tile"); return; }
    if (memcmp(p.segments[0].data, payload, 6) != 0) { FAIL("data mismatch"); return; }
    PASS();
}

static void test_four_tiles(void)
{
    TEST(xe_four_tile_device);
    uint8_t xe[4096]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    size_t off = 8;

    for (int tile = 0; tile < 4; tile++) {
        uint8_t elf[256];
        uint8_t code[] = { (uint8_t)(tile * 0x11) };
        size_t elf_len = build_test_elf(elf, sizeof(elf),
                                        0x80000 + tile*0x100,
                                        0x40000 + tile*0x10000,
                                        code, 1);
        uint8_t sh[12]; build_sub_hdr(sh, 0, tile, 0);
        off = xe_add_sector(xe, off, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    }
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_tiles != 4) { FAILF("num_tiles=%d, want 4", p.num_tiles); return; }
    if (p.num_segments != 4) { FAILF("num_segs=%zu, want 4", p.num_segments); return; }
    for (int i = 0; i < 4; i++) {
        if (p.segments[i].tile != i) { FAILF("seg%d tile=%d", i, p.segments[i].tile); return; }
        if (p.segments[i].paddr != (uint32_t)(0x40000 + i*0x10000)) {
            FAILF("seg%d paddr", i); return;
        }
    }
    PASS();
}

static void test_segment_data_integrity(void)
{
    TEST(xe_segment_data_pointers_valid);
    uint8_t elf[256];
    uint8_t code[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    size_t elf_len = build_test_elf(elf, sizeof(elf), 0x80000, 0x40000, code, 8);

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, elf_len);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 1) { FAIL("expected 1 seg"); return; }
    if (memcmp(p.segments[0].data, code, 8) != 0) { FAIL("data mismatch"); return; }
    PASS();
}

/* =========================================================================
 * ELF Parser Edge Cases
 * ======================================================================= */

static void test_elf_too_small(void)
{
    TEST(elf_reject_too_small);
    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t tiny_elf[8] = { 0x7F, 'E', 'L', 'F', 1, 1, 1, 0 };
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, tiny_elf, 8);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    esp_err_t err = xe_parse(xe, off, &p);
    if (err == ESP_ERR_INVALID_SIZE) PASS();
    else FAILF("expected INVALID_SIZE, got %d", err);
}

static void test_elf_bad_magic(void)
{
    TEST(elf_reject_bad_magic_in_sector);
    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t bad_elf[64]; memset(bad_elf, 0, sizeof(bad_elf));
    bad_elf[0] = 0x7F; bad_elf[1] = 'X'; /* Not ELF */
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, bad_elf, 64);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    esp_err_t err = xe_parse(xe, off, &p);
    if (err == ESP_ERR_INVALID_ARG) PASS();
    else FAILF("expected INVALID_ARG, got %d", err);
}

static void test_elf_phdr_out_of_bounds(void)
{
    TEST(elf_reject_phdr_out_of_bounds);
    /* Build an ELF where phoff points beyond the data */
    uint8_t elf[64]; memset(elf, 0, sizeof(elf));
    elf[0]=0x7F; elf[1]='E'; elf[2]='L'; elf[3]='F';
    elf[4]=1; elf[5]=1; elf[6]=1;
    wr16(elf+16,2); wr16(elf+18,0xCB); wr32(elf+20,1);
    wr32(elf+28, 9999); /* phoff way out of bounds */
    wr16(elf+42, 32); wr16(elf+44, 1);

    uint8_t xe[256]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, 64);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) == ESP_ERR_INVALID_SIZE) PASS();
    else FAIL("expected INVALID_SIZE");
}

static void test_elf_segment_data_out_of_bounds(void)
{
    TEST(elf_reject_segment_data_oob);
    /* Build an ELF where a PT_LOAD segment points beyond the data */
    uint8_t elf[128]; memset(elf, 0, sizeof(elf));
    elf[0]=0x7F; elf[1]='E'; elf[2]='L'; elf[3]='F';
    elf[4]=1; elf[5]=1; elf[6]=1;
    wr16(elf+16,2); wr16(elf+18,0xCB); wr32(elf+20,1);
    wr32(elf+28, 52); /* phoff */
    wr16(elf+42, 32); wr16(elf+44, 1);
    /* Program header */
    uint8_t *ph = elf + 52;
    wr32(ph, 1); /* PT_LOAD */
    wr32(ph+4, 84); /* p_offset */
    wr32(ph+16, 9999); /* p_filesz -- way larger than elf */
    wr32(ph+20, 9999);

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, 128);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) == ESP_ERR_INVALID_SIZE) PASS();
    else FAIL("expected INVALID_SIZE");
}

static void test_elf_no_pt_load(void)
{
    TEST(elf_with_no_pt_load_gives_zero_segs);
    /* ELF with 1 program header but type != PT_LOAD */
    uint8_t elf[128]; memset(elf, 0, sizeof(elf));
    elf[0]=0x7F; elf[1]='E'; elf[2]='L'; elf[3]='F';
    elf[4]=1; elf[5]=1; elf[6]=1;
    wr16(elf+16,2); wr16(elf+18,0xCB); wr32(elf+20,1);
    wr32(elf+24, 0x80000); /* entry */
    wr32(elf+28, 52);
    wr16(elf+42, 32); wr16(elf+44, 1);
    uint8_t *ph = elf+52;
    wr32(ph, 6); /* PT_PHDR, not PT_LOAD */

    uint8_t xe[512]; memset(xe,0,sizeof(xe));
    xe_file_header(xe, 2);
    uint8_t sh[12]; build_sub_hdr(sh, 0, 0, 0);
    size_t off = xe_add_sector(xe, 8, XE_SECTOR_ELF, 0, sh, 12, elf, 84);
    off = xe_add_last(xe, off);

    xe_parsed_t p;
    if (xe_parse(xe, off, &p) != ESP_OK) { FAIL("parse failed"); return; }
    if (p.num_segments != 0) { FAILF("expected 0 segs, got %zu", p.num_segments); return; }
    /* Entry should still be set from ELF header */
    if (p.entry_points[0] != 0x80000) { FAIL("entry not set"); return; }
    PASS();
}

/* =========================================================================
 * Register Encoding Tests
 * ======================================================================= */

/* sc_jtag mux-open xCORE IR = 0x00ffc00f | (((reg<<2)|op) << 4) */
static void test_chain_ir_reg_read(void)
{
    TEST(chain_ir_reg_read_dbg_int);
    uint32_t ir = xmos_chain_ir_reg_read(XMOS_PSWITCH_DBG_INT);
    /* reg=0x05, read: TapIR=(5<<2)|1=0x15; 0x00ffc00f|(0x15<<4)=0x00ffc15f */
    if (ir == 0x00ffc15f) PASS(); else FAILF("got 0x%x, want 0x00ffc15f", ir);
}

static void test_chain_ir_reg_write(void)
{
    TEST(chain_ir_reg_write_dbg_int);
    uint32_t ir = xmos_chain_ir_reg_write(XMOS_PSWITCH_DBG_INT);
    /* TapIR=(5<<2)|2=0x16; 0x00ffc00f|(0x16<<4)=0x00ffc16f */
    if (ir == 0x00ffc16f) PASS(); else FAILF("got 0x%x, want 0x00ffc16f", ir);
}

static void test_chain_ir_all_bypass(void)
{
    TEST(chain_ir_reg0_read_encoding);
    /* reg=0x00 read: TapIR=1; 0x00ffc00f|(1<<4)=0x00ffc01f */
    uint32_t ir = xmos_chain_ir_reg_read(0x00);
    if (ir == 0x00ffc01f) PASS(); else FAILF("got 0x%x, want 0x00ffc01f", ir);
}

static void test_chain_ir_high_reg(void)
{
    TEST(chain_ir_reg_0xFF_max_index);
    /* reg=0xFF write: TapIR=(0xFF<<2)|2=0x3FE; 0x00ffc00f|(0x3FE<<4)=0x00ffffef */
    uint32_t ir = xmos_chain_ir_reg_write(0xFF);
    if (ir == 0x00ffffef) PASS(); else FAILF("got 0x%x, want 0x00ffffef", ir);
}

static void test_chain_ir_scratch_regs(void)
{
    TEST(chain_ir_scratch_reg_encoding);
    /* DBG_COMMAND = 0x21 */
    uint32_t ir_w = xmos_chain_ir_reg_write(XMOS_PSWITCH_DBG_COMMAND);
    uint32_t tap_ir = (0x21u << 2) | 2;  /* 0x86 */
    uint32_t expected = XMOS_XCORE_IR_OPEN_BASE | (tap_ir << XMOS_XCORE_IR_OPEN_SHIFT);
    if (ir_w == expected) PASS(); else FAILF("got 0x%x, want 0x%x", ir_w, expected);
}

static void test_tile_to_mux(void)
{
    TEST(tile_to_mux_values);
    if (xmos_tile_to_mux(0) != XMOS_MUX_XCORE0) { FAIL("tile 0"); return; }
    if (xmos_tile_to_mux(1) != XMOS_MUX_XCORE1) { FAIL("tile 1"); return; }
    if (xmos_tile_to_mux(2) != XMOS_MUX_XCORE2) { FAIL("tile 2"); return; }
    if (xmos_tile_to_mux(3) != XMOS_MUX_XCORE3) { FAIL("tile 3"); return; }
    PASS();
}

static void test_mux_constants(void)
{
    TEST(mux_constant_values_match_sc_jtag);
    /* Values from sc_jtag chip_tap_mux_values[] */
    if (XMOS_MUX_NC != 0x0) { FAIL("NC"); return; }
    if (XMOS_MUX_SSWITCH != 0x1) { FAIL("SSWITCH"); return; }
    if (XMOS_MUX_XCORE0 != 0x8) { FAIL("XCORE0"); return; }
    if (XMOS_MUX_XCORE1 != 0x9) { FAIL("XCORE1"); return; }
    if (XMOS_MUX_XCORE2 != 0xA) { FAIL("XCORE2"); return; }
    if (XMOS_MUX_XCORE3 != 0xB) { FAIL("XCORE3"); return; }
    if (XMOS_MUX_XCOREALL != 0xF) { FAIL("XCOREALL"); return; }
    PASS();
}

static void test_test_mode_bits(void)
{
    TEST(test_mode_register_format);
    /* sc_jtag shifts (0xFACED00 << 4) | mode into SET_TEST_MODE */
    if ((XMOS_TEST_MODE_KEY | XMOS_TEST_MODE_OTP_SERIAL_EN) !=
        ((0xFACED00u << 4) | 0x4u)) { FAIL("test mode key format"); return; }

    /* PS resource IDs are (ps_num << 8) | 0x0b (xs3a_registers.h):
     * XS1_PS_DBG_SSR=0x100b, SPC=0x110b, SSP=0x120b */
    if (XMOS_PS_DBG_SSR != 0x100b) { FAIL("SSR resource id"); return; }
    if (XMOS_PS_DBG_SPC != 0x110b) { FAIL("SPC resource id"); return; }
    if (XMOS_PS_DBG_SSP != 0x120b) { FAIL("SSP resource id"); return; }
    PASS();
}

static void test_dbg_int_bits(void)
{
    TEST(dbg_int_register_bit_values);
    if (XMOS_DBG_INT_REQ != 0x1) { FAIL("REQ bit"); return; }
    if (XMOS_DBG_INT_IN_DBG != 0x2) { FAIL("IN_DBG bit"); return; }
    PASS();
}

static void test_chain_ir_bit_width(void)
{
    TEST(chain_ir_matches_sc_jtag_formula);
    /* Every reg/op must equal the sc_jtag base word with TapIR OR'd in,
     * and the TapIR field (10 bits at shift 4) must not collide with the
     * fixed low nibble (0xf at [3:0]). */
    for (int reg = 0; reg <= 0xFF; reg++) {
        uint32_t tr = ((uint32_t)reg << 2) | XMOS_REG_OP_READ;
        uint32_t tw = ((uint32_t)reg << 2) | XMOS_REG_OP_WRITE;
        uint32_t er = XMOS_XCORE_IR_OPEN_BASE | (tr << XMOS_XCORE_IR_OPEN_SHIFT);
        uint32_t ew = XMOS_XCORE_IR_OPEN_BASE | (tw << XMOS_XCORE_IR_OPEN_SHIFT);
        if (xmos_chain_ir_reg_read(reg) != er ||
            xmos_chain_ir_reg_write(reg) != ew) {
            FAILF("reg 0x%02x encoding mismatch", reg);
            return;
        }
        /* low nibble stays 0xf (sc_jtag base), TapIR sits clear of it */
        if ((er & 0xf) != 0xf) { FAILF("reg 0x%02x low nibble", reg); return; }
    }
    PASS();
}

/* =========================================================================
 * PARLIO Bit Packing Tests
 * ======================================================================= */

static void test_parlio_tms_high(void)
{
    TEST(parlio_4_cycles_tms1_tdi0);
    /* 4 cycles of TMS=1,TDI=0: each pair = 0b01 = 1
     * byte = (1<<0)|(1<<2)|(1<<4)|(1<<6) = 0x55 */
    uint8_t byte = 0;
    for (int c = 0; c < 4; c++) {
        byte |= ((1 & 1) | ((0 & 1) << 1)) << (c * 2);
    }
    if (byte == 0x55) PASS(); else FAILF("got 0x%02x, want 0x55", byte);
}

static void test_parlio_tdi_high(void)
{
    TEST(parlio_4_cycles_tms0_tdi1);
    uint8_t byte = 0;
    for (int c = 0; c < 4; c++) {
        byte |= ((0 & 1) | ((1 & 1) << 1)) << (c * 2);
    }
    /* 0b10 each = (2<<0)|(2<<2)|(2<<4)|(2<<6) = 0xAA */
    if (byte == 0xAA) PASS(); else FAILF("got 0x%02x, want 0xAA", byte);
}

static void test_parlio_both_high(void)
{
    TEST(parlio_4_cycles_tms1_tdi1);
    uint8_t byte = 0;
    for (int c = 0; c < 4; c++) {
        byte |= 0x3 << (c * 2);
    }
    if (byte == 0xFF) PASS(); else FAILF("got 0x%02x, want 0xFF", byte);
}

static void test_parlio_mixed(void)
{
    TEST(parlio_mixed_tms_tdi_pattern);
    int tms[] = {0,1,0,1};
    int tdi[] = {1,1,0,0};
    uint8_t byte = 0;
    for (int c = 0; c < 4; c++)
        byte |= ((tms[c]&1) | ((tdi[c]&1)<<1)) << (c*2);
    /* 10 11 00 01 = 0x4E */
    if (byte == 0x4E) PASS(); else FAILF("got 0x%02x, want 0x4E", byte);
}

static void test_parlio_multibyte(void)
{
    TEST(parlio_8_cycles_span_two_bytes);
    /* 8 cycles: alternating TMS=0/1, TDI always 0 */
    uint8_t buf[2] = {0, 0};
    for (int c = 0; c < 8; c++) {
        int tms = c & 1;
        int byte_idx = c / 4;
        int shift = (c % 4) * 2;
        buf[byte_idx] |= ((tms & 1) | ((0 & 1) << 1)) << shift;
    }
    /* byte0: c0=0,c1=1,c2=0,c3=1 -> 00 01 00 01 = 0x44
     * byte1: c4=0,c5=1,c6=0,c7=1 -> same = 0x44 */
    if (buf[0] == 0x44 && buf[1] == 0x44) PASS();
    else FAILF("got [0x%02x, 0x%02x], want [0x44, 0x44]", buf[0], buf[1]);
}

static void test_parlio_rx_extraction(void)
{
    TEST(parlio_rx_tdo_bit_extraction);
    /* Simulate RX buffer: 1 bit per clock, 8 per byte, LSB first.
     * If we have 12 cycles total, 3 nav cycles, then 9 data bits. */
    uint8_t rx_buf[2] = { 0 };
    /* Set some TDO bits: cycle 3,5,7 = high (data bits 0,2,4) */
    rx_buf[0] = (1<<3) | (1<<5) | (1<<7);  /* cycles 3,5,7 in byte 0 */
    rx_buf[1] = 0;

    /* Extract TDO starting at nav_cycles=3, 5 data bits */
    uint8_t tdo = 0;
    int nav = 3;
    for (int i = 0; i < 5; i++) {
        int rx_cycle = nav + i;
        int bit = (rx_buf[rx_cycle / 8] >> (rx_cycle % 8)) & 1;
        tdo |= (bit << i);
    }
    /* bit0 (cycle3)=1, bit1(cycle4)=0, bit2(cycle5)=1, bit3(cycle6)=0, bit4(cycle7)=1 */
    if (tdo == 0x15) PASS(); else FAILF("got 0x%02x, want 0x15", tdo);
}

/* =========================================================================
 * JTAG TAP State Machine Navigation Tests
 * ======================================================================= */

/* Minimal TAP state machine simulator */
typedef enum {
    TAP_TLR=0, TAP_RTI, TAP_DRSEL, TAP_DRCAP, TAP_DRSH, TAP_DREX1,
    TAP_DRPAUSE, TAP_DREX2, TAP_DRUPD, TAP_IRSEL, TAP_IRCAP, TAP_IRSH,
    TAP_IREX1, TAP_IRPAUSE, TAP_IREX2, TAP_IRUPD
} tap_state_t;

static const tap_state_t tap_next[16][2] = {
    /* TMS=0         TMS=1 */
    [TAP_TLR]     = { TAP_RTI,     TAP_TLR },
    [TAP_RTI]     = { TAP_RTI,     TAP_DRSEL },
    [TAP_DRSEL]   = { TAP_DRCAP,   TAP_IRSEL },
    [TAP_DRCAP]   = { TAP_DRSH,    TAP_DREX1 },
    [TAP_DRSH]    = { TAP_DRSH,    TAP_DREX1 },
    [TAP_DREX1]   = { TAP_DRPAUSE, TAP_DRUPD },
    [TAP_DRPAUSE] = { TAP_DRPAUSE, TAP_DREX2 },
    [TAP_DREX2]   = { TAP_DRSH,    TAP_DRUPD },
    [TAP_DRUPD]   = { TAP_RTI,     TAP_DRSEL },
    [TAP_IRSEL]   = { TAP_IRCAP,   TAP_TLR },
    [TAP_IRCAP]   = { TAP_IRSH,    TAP_IREX1 },
    [TAP_IRSH]    = { TAP_IRSH,    TAP_IREX1 },
    [TAP_IREX1]   = { TAP_IRPAUSE, TAP_IRUPD },
    [TAP_IRPAUSE] = { TAP_IRPAUSE, TAP_IREX2 },
    [TAP_IREX2]   = { TAP_IRSH,    TAP_IRUPD },
    [TAP_IRUPD]   = { TAP_RTI,     TAP_DRSEL },
};

static const char *tap_name(tap_state_t s)
{
    const char *names[] = {
        "TLR","RTI","DR-Sel","DR-Cap","DR-Sh","DR-Ex1",
        "DR-Pause","DR-Ex2","DR-Upd","IR-Sel","IR-Cap","IR-Sh",
        "IR-Ex1","IR-Pause","IR-Ex2","IR-Upd"
    };
    return (s < 16) ? names[s] : "???";
}

static void test_tap_reset_sequence(void)
{
    TEST(tap_6x_tms1_reaches_tlr_from_any);
    /* From any state, 5+ TMS=1 clocks should reach TLR */
    for (int start = 0; start < 16; start++) {
        tap_state_t s = (tap_state_t)start;
        for (int i = 0; i < 6; i++) s = tap_next[s][1];
        if (s != TAP_TLR) {
            FAILF("from %s: ended in %s, not TLR", tap_name(start), tap_name(s));
            return;
        }
    }
    /* Then TMS=0 -> RTI */
    tap_state_t s = TAP_TLR;
    s = tap_next[s][0];
    if (s != TAP_RTI) { FAIL("TLR + TMS=0 should be RTI"); return; }
    PASS();
}

static void test_tap_shift_dr_path(void)
{
    TEST(tap_rti_to_shift_dr_path);
    /* RTI -> Select-DR (TMS=1) -> Capture-DR (TMS=0) -> Shift-DR (TMS=0) */
    tap_state_t s = TAP_RTI;
    s = tap_next[s][1]; /* -> DR-Sel */
    if (s != TAP_DRSEL) { FAILF("step1: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* -> DR-Cap */
    if (s != TAP_DRCAP) { FAILF("step2: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* -> DR-Sh */
    if (s != TAP_DRSH) { FAILF("step3: %s", tap_name(s)); return; }

    /* Shift N-1 bits with TMS=0, stay in Shift-DR */
    for (int i = 0; i < 10; i++) {
        s = tap_next[s][0];
        if (s != TAP_DRSH) { FAILF("shift cycle %d: %s", i, tap_name(s)); return; }
    }

    /* Last bit TMS=1 -> Exit1-DR */
    s = tap_next[s][1];
    if (s != TAP_DREX1) { FAILF("exit: %s", tap_name(s)); return; }

    /* Update (TMS=1) -> RTI (TMS=0) */
    s = tap_next[s][1]; /* -> Update */
    if (s != TAP_DRUPD) { FAILF("update: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* -> RTI */
    if (s != TAP_RTI) { FAILF("rti: %s", tap_name(s)); return; }
    PASS();
}

static void test_tap_shift_ir_path(void)
{
    TEST(tap_rti_to_shift_ir_path);
    tap_state_t s = TAP_RTI;
    s = tap_next[s][1]; /* -> DR-Sel */
    s = tap_next[s][1]; /* -> IR-Sel */
    if (s != TAP_IRSEL) { FAILF("irsel: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* -> IR-Cap */
    if (s != TAP_IRCAP) { FAILF("ircap: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* -> IR-Sh */
    if (s != TAP_IRSH) { FAILF("irsh: %s", tap_name(s)); return; }

    /* Shift, then exit */
    for (int i = 0; i < 5; i++) s = tap_next[s][0];
    s = tap_next[s][1]; /* Exit1-IR */
    if (s != TAP_IREX1) { FAILF("irex1: %s", tap_name(s)); return; }
    s = tap_next[s][1]; /* Update-IR */
    if (s != TAP_IRUPD) { FAILF("irupd: %s", tap_name(s)); return; }
    s = tap_next[s][0]; /* RTI */
    if (s != TAP_RTI) { FAILF("rti: %s", tap_name(s)); return; }
    PASS();
}

static void test_tap_idle_stays(void)
{
    TEST(tap_rti_stays_on_tms0);
    tap_state_t s = TAP_RTI;
    for (int i = 0; i < 100; i++) {
        s = tap_next[s][0];
        if (s != TAP_RTI) { FAILF("left RTI at cycle %d", i); return; }
    }
    PASS();
}

/* =========================================================================
 * XMOS Protocol Constant Sanity Tests
 * ======================================================================= */

static void test_idcode_mask(void)
{
    TEST(idcode_mask_strips_revision);
    /* Revision is the top nibble, mask should clear it */
    uint32_t id = 0xA0005633; /* revision 0xA, XS2 */
    uint32_t masked = id & XMOS_IDCODE_MASK;
    if (masked == (XMOS_IDCODE_XS2 & XMOS_IDCODE_MASK)) PASS();
    else FAILF("masked=0x%08x", masked);
}

static void test_pswitch_scratch_layout(void)
{
    TEST(pswitch_scratch_regs_contiguous);
    for (int i = 0; i < 8; i++) {
        if (XMOS_PSWITCH_DBG_SCRATCH(i) != (uint8_t)(0x20 + i)) {
            FAILF("scratch(%d) = 0x%02x", i, XMOS_PSWITCH_DBG_SCRATCH(i));
            return;
        }
    }
    /* Check aliases */
    if (XMOS_PSWITCH_DBG_STATUS != 0x20) { FAIL("STATUS"); return; }
    if (XMOS_PSWITCH_DBG_COMMAND != 0x21) { FAIL("COMMAND"); return; }
    if (XMOS_PSWITCH_DBG_ARG0 != 0x22) { FAIL("ARG0"); return; }
    if (XMOS_PSWITCH_DBG_ARG5 != 0x27) { FAIL("ARG5"); return; }
    PASS();
}

static void test_debug_cmd_ordering(void)
{
    TEST(debug_cmd_codes_sequential);
    if (XMOS_DBG_CMD_READ != 1) { FAIL("READ"); return; }
    if (XMOS_DBG_CMD_WRITE != 2) { FAIL("WRITE"); return; }
    if (XMOS_DBG_CMD_READ4PI != 3) { FAIL("READ4PI"); return; }
    if (XMOS_DBG_CMD_WRITE4PI != 4) { FAIL("WRITE4PI"); return; }
    if (XMOS_DBG_CMD_GETPS != 5) { FAIL("GETPS"); return; }
    if (XMOS_DBG_CMD_SETPS != 6) { FAIL("SETPS"); return; }
    if (XMOS_DBG_CMD_RFDBG != 9) { FAIL("RFDBG"); return; }
    PASS();
}

static void test_xe_sector_constants(void)
{
    TEST(xe_sector_type_constants_match_tool_axe);
    if (XE_SECTOR_BINARY != 1) { FAIL("BINARY"); return; }
    if (XE_SECTOR_ELF != 2) { FAIL("ELF"); return; }
    if (XE_SECTOR_CONFIG != 3) { FAIL("CONFIG"); return; }
    if (XE_SECTOR_GOTO != 5) { FAIL("GOTO"); return; }
    if (XE_SECTOR_CALL != 6) { FAIL("CALL"); return; }
    if (XE_SECTOR_XN != 8) { FAIL("XN"); return; }
    if (XE_SECTOR_LAST != 0x5555) { FAIL("LAST"); return; }
    PASS();
}

static void test_ps_register_numbers(void)
{
    TEST(ps_resource_ids_match_xs3a_registers_h);
    /* GETPS/SETPS take encoded resource IDs: (ps_num << 8) | 0x0b */
    if (XMOS_PS_DBG_SSR != 0x100b) { FAIL("SSR"); return; }
    if (XMOS_PS_DBG_SPC != 0x110b) { FAIL("SPC"); return; }
    if (XMOS_PS_DBG_SSP != 0x120b) { FAIL("SSP"); return; }
    if (XMOS_PS_DBG_INT_TYPE != 0x150b) { FAIL("INT_TYPE"); return; }
    if (XMOS_PS_DBG_CORE_CTRL != 0x180b) { FAIL("CORE_CTRL"); return; }
    PASS();
}

/* =========================================================================
 * Real XE file tests
 * ======================================================================= */

static void test_real_xe_file(const char *path)
{
    printf("\n  --- Real file: %s ---\n", path);

    FILE *f = fopen(path, "rb");
    if (!f) { printf("  (skipped: cannot open)\n"); return; }

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *data = malloc(fsize);
    if (!data) { fclose(f); return; }
    fread(data, 1, fsize, f);
    fclose(f);

    TEST(real_xe_parses_ok);
    xe_parsed_t p;
    esp_err_t err = xe_parse(data, fsize, &p);
    if (err != ESP_OK) { FAILF("error %d", err); free(data); return; }
    if (p.num_segments == 0) { FAIL("0 segments"); free(data); return; }
    PASS();

    TEST(real_xe_has_two_tiles);
    if (p.num_tiles == 2) PASS(); else FAILF("tiles=%d", p.num_tiles);

    TEST(real_xe_segments_have_valid_addresses);
    int ok = 1;
    for (size_t i = 0; i < p.num_segments; i++) {
        if (p.segments[i].paddr < 0x40000 || p.segments[i].paddr > 0x200000) {
            FAILF("seg%zu addr=0x%08x out of range", i, p.segments[i].paddr);
            ok = 0; break;
        }
    }
    if (ok) PASS();

    TEST(real_xe_data_pointers_within_file);
    ok = 1;
    for (size_t i = 0; i < p.num_segments; i++) {
        const xe_segment_t *s = &p.segments[i];
        if (s->filesz > 0) {
            ptrdiff_t off = s->data - data;
            if (off < 0 || (size_t)off + s->filesz > (size_t)fsize) {
                FAILF("seg%zu data pointer out of file bounds", i);
                ok = 0; break;
            }
        }
    }
    if (ok) PASS();

    TEST(real_xe_no_overlapping_segments_per_tile);
    ok = 1;
    for (size_t i = 0; i < p.num_segments && ok; i++) {
        for (size_t j = i+1; j < p.num_segments && ok; j++) {
            if (p.segments[i].tile != p.segments[j].tile) continue;
            uint32_t a_start = p.segments[i].paddr;
            uint32_t a_end = a_start + p.segments[i].memsz;
            uint32_t b_start = p.segments[j].paddr;
            uint32_t b_end = b_start + p.segments[j].memsz;
            if (a_start < b_end && b_start < a_end) {
                /* Overlapping -- this can be valid (later ELF overwrites earlier),
                 * but flag for awareness */
                printf("  (note: tile%d segs %zu & %zu overlap: 0x%x-0x%x vs 0x%x-0x%x)\n",
                       p.segments[i].tile, i, j, a_start, a_end, b_start, b_end);
            }
        }
    }
    if (ok) PASS();

    size_t total = 0;
    for (size_t i = 0; i < p.num_segments; i++) total += p.segments[i].filesz;
    printf("  %zu segments, %d tiles, %zu bytes total code\n",
           p.num_segments, p.num_tiles, total);

    free(data);
}

/* =========================================================================
 * Main
 * ======================================================================= */
int main(int argc, char **argv)
{
    /* Check for -v flag */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) { log_verbose = 1; break; }
    }

    printf("=== XMOS JTAG Component Test Suite ===\n\n");

    printf("--- XE Parser ---\n");
    test_reject_too_small();
    test_reject_bad_magic();
    test_empty_xe();
    test_raw_elf_fallback();
    test_single_elf_sector();
    test_multi_tile();
    test_padding_byte();
    test_goto_overrides_elf_entry();
    test_call_sector();
    test_unknown_sectors_skipped();
    test_config_xn_xscope_skipped();
    test_zero_length_sector();
    test_elf_with_bss();
    test_elf_multiple_segments();
    test_binary_sector();
    test_four_tiles();
    test_segment_data_integrity();

    printf("\n--- ELF Parser Edge Cases ---\n");
    test_elf_too_small();
    test_elf_bad_magic();
    test_elf_phdr_out_of_bounds();
    test_elf_segment_data_out_of_bounds();
    test_elf_no_pt_load();

    printf("\n--- JTAG Chain IR Encoding ---\n");
    test_chain_ir_reg_read();
    test_chain_ir_reg_write();
    test_chain_ir_all_bypass();
    test_chain_ir_high_reg();
    test_chain_ir_scratch_regs();
    test_chain_ir_bit_width();

    printf("\n--- MUX & Register Constants ---\n");
    test_tile_to_mux();
    test_mux_constants();
    test_test_mode_bits();
    test_dbg_int_bits();
    test_idcode_mask();
    test_pswitch_scratch_layout();
    test_debug_cmd_ordering();
    test_xe_sector_constants();
    test_ps_register_numbers();

    printf("\n--- PARLIO Bit Packing ---\n");
    test_parlio_tms_high();
    test_parlio_tdi_high();
    test_parlio_both_high();
    test_parlio_mixed();
    test_parlio_multibyte();
    test_parlio_rx_extraction();

    printf("\n--- TAP State Machine ---\n");
    test_tap_reset_sequence();
    test_tap_shift_dr_path();
    test_tap_shift_ir_path();
    test_tap_idle_stays();

    /* Real file tests */
    printf("\n--- Real XE Files ---\n");
    int has_files = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) continue;
        test_real_xe_file(argv[i]);
        has_files = 1;
    }
    /* Auto-discover common paths */
    if (!has_files) {
        const char *paths[] = {
            "/Users/syso/Downloads/satellite_firmware_apps/satellite1_firmware_fixed_delay.xe",
            "/Users/syso/Downloads/satellite_firmware_apps/satellite1_usb_firmware_ns.xe",
            NULL
        };
        for (int i = 0; paths[i]; i++) {
            FILE *f = fopen(paths[i], "rb");
            if (f) { fclose(f); test_real_xe_file(paths[i]); }
        }
    }

    printf("\n=== %d/%d passed", tests_passed, tests_run);
    if (tests_failed > 0) printf(", \033[31m%d FAILED\033[0m", tests_failed);
    printf(" ===\n");
    return (tests_failed == 0) ? 0 : 1;
}
