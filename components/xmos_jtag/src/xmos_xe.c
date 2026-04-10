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
                                    uint8_t tile, xe_parsed_t *out)
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

    /* Record entry point for this tile */
    if (tile < 4) {
        out->entry_points[tile] = ehdr->e_entry;
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

        /* Might be a raw ELF -- try parsing directly */
        if (xe_len >= 4 && rd32(xe_data) == ELF_MAGIC) {
            ESP_LOGI(TAG, "Input is raw ELF, treating as tile 0");
            return parse_elf_segments(xe_data, xe_len, 0, out);
        }

        ESP_LOGE(TAG, "Not an XE file (bad magic)");
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t version = rd16(xe_data + 4);
    ESP_LOGD(TAG, "XE version %d, parsing sectors...", version);

    size_t off = XE_FILE_HDR_SIZE;

    while (off + XE_SECTOR_HDR_SIZE <= xe_len) {
        uint16_t stype = rd16(xe_data + off);

        if (stype == XE_SECTOR_LAST) {
            ESP_LOGD(TAG, "End of sectors at offset 0x%zx", off);
            break;
        }

        /* Read sector length (uint64) */
        uint64_t length = rd64(xe_data + off + 4);
        size_t next_off = off + XE_SECTOR_HDR_SIZE + (size_t)length;

        if (next_off > xe_len) {
            ESP_LOGE(TAG, "Sector at 0x%zx overflows file (length=%llu)",
                     off, (unsigned long long)length);
            return ESP_ERR_INVALID_SIZE;
        }

        /* Pointer to sector data (after 12-byte header) */
        const uint8_t *sdata = xe_data + off + XE_SECTOR_HDR_SIZE;
        uint8_t pad_byte = 0;
        size_t content_off = 0;  /* offset into sdata where content starts */

        if (length > 0) {
            pad_byte = sdata[0];
            content_off = XE_SECTOR_PAD_DESC_SIZE;  /* skip padding descriptor */
        }

        size_t content_len = (length > XE_SECTOR_PAD_DESC_SIZE)
                           ? (size_t)(length - XE_SECTOR_PAD_DESC_SIZE - pad_byte)
                           : 0;

        switch (stype) {
        case XE_SECTOR_ELF: {
            /* Sub-header: node(2) + core(2) + address(8) */
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;

            uint16_t core = rd16(sdata + content_off + 2);
            /* address at content_off + 4 (8 bytes, unused for ELF) */

            const uint8_t *elf_start = sdata + content_off + XE_SECTOR_SUB_HDR_SIZE;
            size_t elf_len_actual = content_len - XE_SECTOR_SUB_HDR_SIZE;

            ESP_LOGD(TAG, "ELF sector at 0x%zx: core=%d elf_size=%zu",
                     off, core, elf_len_actual);

            esp_err_t err = parse_elf_segments(elf_start, elf_len_actual,
                                               (uint8_t)core, out);
            if (err != ESP_OK) return err;
            break;
        }

        case XE_SECTOR_BINARY: {
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;

            uint16_t core = rd16(sdata + content_off + 2);
            uint64_t addr = rd64(sdata + content_off + 4);

            const uint8_t *bin_data = sdata + content_off + XE_SECTOR_SUB_HDR_SIZE;
            size_t bin_len = content_len - XE_SECTOR_SUB_HDR_SIZE;

            if (out->num_segments < XE_MAX_SEGMENTS) {
                xe_segment_t *seg = &out->segments[out->num_segments++];
                seg->paddr  = (uint32_t)addr;
                seg->filesz = (uint32_t)bin_len;
                seg->memsz  = (uint32_t)bin_len;
                seg->data   = bin_data;
                seg->tile   = (uint8_t)core;
                if (core < 4 && core >= out->num_tiles)
                    out->num_tiles = (uint8_t)(core + 1);

                ESP_LOGD(TAG, "BIN sector: core=%d addr=0x%08lx size=%zu",
                         core, (unsigned long)(uint32_t)addr, bin_len);
            }
            break;
        }

        case XE_SECTOR_GOTO:
        case XE_SECTOR_CALL: {
            if (content_len < XE_SECTOR_SUB_HDR_SIZE) break;

            uint16_t core = rd16(sdata + content_off + 2);
            uint64_t addr = rd64(sdata + content_off + 4);

            if (core < 4) {
                out->entry_points[core] = (uint32_t)addr;
                if (core >= out->num_tiles)
                    out->num_tiles = (uint8_t)(core + 1);

                ESP_LOGD(TAG, "%s sector: core=%d addr=0x%08lx",
                         stype == XE_SECTOR_GOTO ? "GOTO" : "CALL",
                         core, (unsigned long)(uint32_t)addr);
            }
            break;
        }

        case XE_SECTOR_CONFIG:
        case XE_SECTOR_XN:
        case XE_SECTOR_NODEDESC:
        case XE_SECTOR_XSCOPE:
            ESP_LOGD(TAG, "Skipping sector type 0x%04x at 0x%zx", stype, off);
            break;

        default:
            ESP_LOGD(TAG, "Unknown sector type 0x%04x at 0x%zx", stype, off);
            break;
        }

        off = next_off;
    }

    ESP_LOGI(TAG, "Parsed %zu segments across %d tiles",
             out->num_segments, out->num_tiles);

    return ESP_OK;
}
