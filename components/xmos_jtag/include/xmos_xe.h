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

/* Maximum number of loadable segments / boot steps we track */
#define XE_MAX_SEGMENTS  64
#define XE_MAX_STEPS     24
#define XE_MAX_TILES     4

typedef struct {
    uint32_t  paddr;        /* Physical (load) address in xCORE RAM */
    uint32_t  filesz;       /* Size in the file */
    uint32_t  memsz;        /* Size in memory (may be > filesz for BSS) */
    const uint8_t *data;    /* Pointer into the XE data buffer */
    uint8_t   tile;         /* Target tile index */
} xe_segment_t;

/*
 * Ordered boot step, mirroring the XMOS simulator's BootSequencer
 * (tool_axe lib/BootSequencer.cpp).  A real .xe is a SEQUENCE:
 *   load setup ELF -> CALL (run setup, wait for return) ->
 *   load app ELF   -> GOTO (run app).
 * The CALL/GOTO sector address field is ignored; each core runs at the
 * entry point of the ELF most recently loaded to it.
 */
typedef enum {
    XE_OP_LOAD = 0,   /* write segments[seg_first .. seg_first+seg_count) */
    XE_OP_RUN,        /* resume the tiles in run_mask, optionally wait */
} xe_op_t;

typedef struct {
    xe_op_t  op;
    /* XE_OP_LOAD */
    uint8_t  tile;            /* target tile */
    uint32_t entry;           /* entry point (ELF e_entry, or RAM base) */
    uint16_t seg_first;       /* first segment index */
    uint16_t seg_count;       /* number of segments */
    /* XE_OP_RUN */
    uint8_t  run_mask;        /* bitmask of tiles to resume together */
    bool     run_wait;        /* true = CALL (wait for debug re-entry),
                                 false = GOTO (final, leave running) */
} xe_step_t;

typedef struct {
    xe_segment_t segments[XE_MAX_SEGMENTS];
    size_t       num_segments;
    xe_step_t    steps[XE_MAX_STEPS];
    size_t       num_steps;
    uint32_t     entry_points[XE_MAX_TILES];  /* per-tile final entry, info */
    uint8_t      num_tiles;                   /* number of tiles referenced */
} xe_parsed_t;

/**
 * Parse an XE file in memory.
 *
 * Extracts all loadable ELF segments plus the ordered boot-step sequence
 * (load / call / goto).  The returned xe_parsed_t contains pointers into
 * `xe_data` -- the caller must keep xe_data alive while using the result.
 */
esp_err_t xe_parse(const uint8_t *xe_data, size_t xe_len,
                   xe_parsed_t *out);

#ifdef __cplusplus
}
#endif
