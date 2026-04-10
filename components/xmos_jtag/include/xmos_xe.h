/*
 * XMOS XE file and ELF segment parser.
 *
 * XE files contain one or more ELF binaries (one per tile) wrapped in
 * a sector-based container format.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#ifndef TEST_HOST
#include "esp_err.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of loadable segments we track */
#define XE_MAX_SEGMENTS  32

typedef struct {
    uint32_t  paddr;        /* Physical (load) address in xCORE RAM */
    uint32_t  filesz;       /* Size in the file */
    uint32_t  memsz;        /* Size in memory (may be > filesz for BSS) */
    const uint8_t *data;    /* Pointer into the XE data buffer */
    uint8_t   tile;         /* Target tile index */
} xe_segment_t;

typedef struct {
    xe_segment_t segments[XE_MAX_SEGMENTS];
    size_t       num_segments;
    uint32_t     entry_points[4];   /* Per-tile entry point, 0 if none */
    uint8_t      num_tiles;         /* Number of tiles referenced */
} xe_parsed_t;

/**
 * Parse an XE file in memory.
 *
 * Extracts all loadable ELF segments and entry points.
 * The returned xe_parsed_t contains pointers into `xe_data` --
 * the caller must keep xe_data alive while using the result.
 */
esp_err_t xe_parse(const uint8_t *xe_data, size_t xe_len,
                   xe_parsed_t *out);

#ifdef __cplusplus
}
#endif
