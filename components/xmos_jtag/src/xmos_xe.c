/*
 * XMOS XE file and ELF segment parser.
 *
 * XE format (verified against tool_axe reference implementation):
 *   File header: magic(4,"XMOS") + version(2) + pad(2) = 8 bytes
 *   Sector header: type(2) + skip(2) + length(8) = 12 bytes
 *     If length > 0: padding_byte(1) + skip(3) = 4 extra bytes
 *   For ELF/GOTO/CALL sectors, data has sub-header:
 *     node(2) + core(2) + address(8) = 12 bytes
 *   Next sector at: current_offset + 12 + length
 */

#include "xmos_xe.h"
#include "xmos_regs.h"
#ifndef TEST_HOST
#include "esp_log.h"
#endif
#include <string.h>

static const char *TAG = "xmos_xe";

/* -------------------------------------------------------------------------
 * Minimal ELF32 structures (little-endian, xCORE)
 * ---------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t e_ident_mag;   /* 0x464C457F */
    uint8_t  e_ident_rest[12];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} elf32_ehdr_t;

typedef struct __attribute__((packed)) {
    uint32_t p_type;
    uint32_t p_offset;
    uint32_t p_vaddr;
    uint32_t p_paddr;       /* Physical (load) address */
    uint32_t p_filesz;
    uint32_t p_memsz;
    uint32_t p_flags;
    uint32_t p_align;
} elf32_phdr_t;

/* -------------------------------------------------------------------------
 * Little-endian field readers (avoid alignment issues)
 * ---------------------------------------------------------------------- */
static inline uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t rd64(const uint8_t *p)
{
    return (uint64_t)rd32(p) | ((uint64_t)rd32(p + 4) << 32);
}

/* -------------------------------------------------------------------------
 * Helper: parse ELF segments from an embedded ELF binary
 * ---------------------------------------------------------------------- */
static esp_err_t parse_elf_segments(const uint8_t *elf_data, size_t elf_len,
                                    uint8_t tile, xe_parsed_t *out,
                                    uint32_t *out_entry)
{
    if (elf_len < sizeof(elf32_ehdr_t)) {
        ESP_LOGE(TAG, "ELF too small (%zu bytes)", elf_len);
        return ESP_ERR_INVALID_SIZE;
    }

    if (rd32(elf_data) != ELF_MAGIC) {
        ESP_LOGE(TAG, "Bad ELF magic: 0x%08lx", (unsigned long)rd32(elf_data));
        return ESP_ERR_INVALID_ARG;
    }

    const elf32_ehdr_t *ehdr = (const elf32_ehdr_t *)elf_data;
    uint32_t phoff = ehdr->e_phoff;
    uint16_t phnum = ehdr->e_phnum;
    uint16_t phentsz = ehdr->e_phentsize;

    if (phoff + (uint32_t)phnum * phentsz > elf_len) {
        ESP_LOGE(TAG, "ELF program headers out of bounds");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Entry point: e_entry, or the tile RAM base if zero (tool_axe rule). */
    uint32_t entry = ehdr->e_entry ? ehdr->e_entry : XMOS_XS2_RAM_BASE;
    if (out_entry) *out_entry = entry;
    if (tile < XE_MAX_TILES) {
        out->entry_points[tile] = entry;
        if (tile >= out->num_tiles)
            out->num_tiles = tile + 1;
    }

    for (uint16_t i = 0; i < phnum; i++) {
        const elf32_phdr_t *phdr = (const elf32_phdr_t *)
            (elf_data + phoff + (uint32_t)i * phentsz);

        if (phdr->p_type != PT_LOAD) continue;
        if (phdr->p_filesz == 0 && phdr->p_memsz == 0) continue;

        if (out->num_segments >= XE_MAX_SEGMENTS) {
            ESP_LOGW(TAG, "Too many segments, skipping rest");
            break;
        }

        if (phdr->p_offset + phdr->p_filesz > elf_len) {
            ESP_LOGE(TAG, "ELF segment data out of bounds");
            return ESP_ERR_INVALID_SIZE;
        }

        xe_segment_t *seg = &out->segments[out->num_segments++];
        seg->paddr  = phdr->p_paddr;
        seg->filesz = phdr->p_filesz;
        seg->memsz  = phdr->p_memsz;
        seg->data   = elf_data + phdr->p_offset;
        seg->tile   = tile;

        ESP_LOGD(TAG, "  tile %d segment: addr=0x%08lx filesz=%lu memsz=%lu",
                 tile, (unsigned long)seg->paddr,
                 (unsigned long)seg->filesz, (unsigned long)seg->memsz);
    }

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Boot-step emitters (mirror tool_axe BootSequencer)
 * ---------------------------------------------------------------------- */
static xe_step_t *xe_add_step(xe_parsed_t *out)
{
    if (out->num_steps >= XE_MAX_STEPS) return NULL;
    return &out->steps[out->num_steps++];
}

static void xe_emit_load(xe_parsed_t *out, uint8_t tile, uint32_t entry,
                         uint16_t seg_first, uint16_t seg_count)
{
    xe_step_t *s = xe_add_step(out);
    if (!s) { ESP_LOGW(TAG, "Too many boot steps, truncating"); return; }
    s->op = XE_OP_LOAD;
    s->tile = tile;
    s->entry = entry;
    s->seg_first = seg_first;
    s->seg_count = seg_count;
}

static void xe_emit_run(xe_parsed_t *out, uint8_t mask, bool wait)
{
    if (mask == 0) return;
    xe_step_t *s = xe_add_step(out);
    if (!s) { ESP_LOGW(TAG, "Too many boot steps, truncating"); return; }
    s->op = XE_OP_RUN;
    s->run_mask = mask;
    s->run_wait = wait;
}

/* -------------------------------------------------------------------------
 * Public: parse XE file
 * ---------------------------------------------------------------------- */
esp_err_t xe_parse(const uint8_t *xe_data, size_t xe_len,
                   xe_parsed_t *out)
{
    memset(out, 0, sizeof(*out));

    if (xe_len < XE_FILE_HDR_SIZE) {
        ESP_LOGE(TAG, "XE file too small");
        return ESP_ERR_INVALID_SIZE;
    }

    /* Check magic */
    if (xe_data[0] != XE_MAGIC_0 || xe_data[1] != XE_MAGIC_1 ||
        xe_data[2] != XE_MAGIC_2 || xe_data[3] != XE_MAGIC_3) {

        /* Might be a raw ELF -- try parsing directly as a single load+goto */
        if (xe_len >= 4 && rd32(xe_data) == ELF_MAGIC) {
            ESP_LOGI(TAG, "Input is raw ELF, treating as tile 0");
            uint32_t entry = 0;
            uint16_t seg_first = (uint16_t)out->num_segments;
            esp_err_t err = parse_elf_segments(xe_data, xe_len, 0, out, &entry);
            if (err != ESP_OK) return err;
            xe_emit_load(out, 0, entry, seg_first,
                         (uint16_t)(out->num_segments - seg_first));
            xe_emit_run(out, 0x1, false);   /* run tile 0, final */
            return ESP_OK;
        }

        ESP_LOGE(TAG, "Not an XE file (bad magic)");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t version = rd16(xe_data + 4);
    ESP_LOGD(TAG, "XE version %d, parsing sectors...", version);

    /*
     * Sector -> boot-step state machine, ported from tool_axe BootSequencer:
     *   sched_mask : tiles loaded since the last RUN
     *   call_mask  : cores with a pending CALL (run-and-wait)
     *   goto_mask  : cores with a pending GOTO (final run)
     */
    uint8_t sched_mask = 0, call_mask = 0, goto_mask = 0;
    size_t off = XE_FILE_HDR_SIZE;

    while (off + XE_SECTOR_HDR_SIZE <= xe_len) {
        uint16_t stype = rd16(xe_data + off);

        if (stype == XE_SECTOR_LAST) {
            ESP_LOGD(TAG, "End of sectors at offset 0x%zx", off);
            break;
        }

        uint64_t length = rd64(xe_data + off + 4);
        size_t next_off = off + XE_SECTOR_HDR_SIZE + (size_t)length;

        if (next_off > xe_len) {
            ESP_LOGE(TAG, "Sector at 0x%zx overflows file (length=%llu)",
                     off, (unsigned long long)length);
            return ESP_ERR_INVALID_SIZE;
        }

        const uint8_t *sdata = xe_data + off + XE_SECTOR_HDR_SIZE;
        uint8_t pad_byte = 0;
        size_t content_off = 0;

        if (length > 0) {
            pad_byte = sdata[0];
            content_off = XE_SECTOR_PAD_DESC_SIZE;
        }

        size_t content_len = (length > XE_SECTOR_PAD_DESC_SIZE)
                           ? (size_t)(length - XE_SECTOR_PAD_DESC_SIZE - pad_byte)
                           : 0;

        switch (stype) {
        case XE_SECTOR_ELF:
        case XE_SECTOR_BINARY: {
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;

            uint16_t core = rd16(sdata + content_off + 2);
            uint64_t addr = rd64(sdata + content_off + 4);
            if (core >= XE_MAX_TILES) {
                ESP_LOGW(TAG, "Sector core %u out of range, skipping", core);
                break;
            }

            /* tool_axe: an ELF for a core with a pending CALL flushes the
             * accumulated setup phase as a run-and-wait before loading. */
            if (call_mask & (1u << core)) {
                xe_emit_run(out, sched_mask, true);
                sched_mask = 0;
                call_mask = 0;
            }

            const uint8_t *body = sdata + content_off + XE_SECTOR_SUB_HDR_SIZE;
            size_t body_len = content_len - XE_SECTOR_SUB_HDR_SIZE;
            uint16_t seg_first = (uint16_t)out->num_segments;
            uint32_t entry;

            if (stype == XE_SECTOR_ELF) {
                esp_err_t err = parse_elf_segments(body, body_len,
                                                   (uint8_t)core, out, &entry);
                if (err != ESP_OK) return err;
            } else {
                /* Raw binary: one segment loaded at the given address */
                if (out->num_segments < XE_MAX_SEGMENTS) {
                    xe_segment_t *seg = &out->segments[out->num_segments++];
                    seg->paddr = (uint32_t)addr;
                    seg->filesz = (uint32_t)body_len;
                    seg->memsz = (uint32_t)body_len;
                    seg->data = body;
                    seg->tile = (uint8_t)core;
                }
                entry = (uint32_t)addr;
                out->entry_points[core] = entry;
                if (core >= out->num_tiles) out->num_tiles = (uint8_t)(core + 1);
            }

            xe_emit_load(out, (uint8_t)core, entry, seg_first,
                         (uint16_t)(out->num_segments - seg_first));
            sched_mask |= (uint8_t)(1u << core);
            ESP_LOGD(TAG, "%s sector core=%u entry=0x%08lx",
                     stype == XE_SECTOR_ELF ? "ELF" : "BIN",
                     core, (unsigned long)entry);
            break;
        }

        case XE_SECTOR_CALL: {
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;
            uint16_t core = rd16(sdata + content_off + 2);
            if (core >= XE_MAX_TILES) break;
            /* A repeated core means a new run batch starts (tool_axe). */
            if (call_mask & (1u << core)) {
                xe_emit_run(out, sched_mask, true);
                sched_mask = 0;
                call_mask = 0;
            }
            call_mask |= (uint8_t)(1u << core);
            ESP_LOGD(TAG, "CALL sector core=%u", core);
            break;
        }

        case XE_SECTOR_GOTO: {
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;
            uint16_t core = rd16(sdata + content_off + 2);
            if (core >= XE_MAX_TILES) break;
            /* Any pending CALLs run-and-wait before the final GOTO batch. */
            if (call_mask) {
                xe_emit_run(out, sched_mask, true);
                sched_mask = 0;
                call_mask = 0;
            }
            goto_mask |= (uint8_t)(1u << core);
            ESP_LOGD(TAG, "GOTO sector core=%u", core);
            break;
        }

        default:
            ESP_LOGD(TAG, "Skipping sector type 0x%04x at 0x%zx", stype, off);
            break;
        }

        off = next_off;
    }

    /* Flush the final run batch: GOTO (final, no wait) or trailing CALLs. */
    if (goto_mask)
        xe_emit_run(out, sched_mask, false);
    else if (call_mask)
        xe_emit_run(out, sched_mask, true);

    ESP_LOGI(TAG, "Parsed %zu segments, %zu boot steps across %d tiles",
             out->num_segments, out->num_steps, out->num_tiles);

    return ESP_OK;
}
