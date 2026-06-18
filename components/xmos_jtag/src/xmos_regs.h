/*
 * XMOS register definitions for JTAG debug access.
 *
 * Sources:
 *   - XU208-128-TQ64 datasheet (X009606) appendices B/C/D
 *   - xcore/sc_jtag open-source JTAG master (GitHub)
 *   - xcore.com forum discussions on JTAG protocol
 */

#pragma once

#include <stdint.h>

/* =========================================================================
 * Top-level TAPs.
 *
 * Even with the MUX closed, an XMOS device exposes TWO TAPs in the chain
 * (sc_jtag jtag_chip_tap_reg_access scans an 8-bit IR / 33-bit DR):
 *
 *   TDI -> BSCAN TAP (4-bit IR) -> CHIP TAP (4-bit IR) -> TDO
 *
 * SETMUX / SET_TEST_MODE are CHIP TAP instructions (sc_jtag jtag.xc).
 * The boundary-scan instructions (EXTEST/SAMPLE) live on the BSCAN TAP;
 * their opcodes below are NOT confirmed by any public source (no XMOS
 * BSDL is published) -- treat as best-effort.
 * ======================================================================= */
#define XMOS_BSCAN_IR_LEN            4
#define XMOS_BSCAN_IR_EXTEST         0x0   /* UNVERIFIED (no public BSDL) */
#define XMOS_BSCAN_IR_SAMPLE         0x2   /* UNVERIFIED (no public BSDL) */
#define XMOS_BSCAN_IR_BYPASS         0xF

#define XMOS_CHIP_TAP_IR_DEVICE_ID   0x3   /* sc_jtag jtag_read_idcode (0xfff3) */
#define XMOS_CHIP_TAP_IR_SETMUX      0x4   /* sc_jtag SETMUX_IR */
#define XMOS_CHIP_TAP_IR_GETMUX      0x5   /* sc_jtag GETMUX_IR */
#define XMOS_CHIP_TAP_IR_SET_TEST_MODE 0x8 /* sc_jtag SET_TEST_MODE_IR */

/* Closed-chain (MUX_NC) scan geometry, per sc_jtag:
 *   IR = CHIP(4, bits[3:0], nearest TDO) + BSCAN(4, bits[7:4]) = 8 bits
 *   DR = CHIP DR(32) + BSCAN bypass(1) = 33 bits, data in low 32 */
#define XMOS_CLOSED_IR_LEN           8
#define XMOS_CLOSED_DR_LEN           33

/* =========================================================================
 * Chip TAP MUX target values (shifted into DR after SETMUX IR)
 *
 * Source: sc_jtag jtag.xc chip_tap_mux_values[]
 * ======================================================================= */
#define XMOS_MUX_NC        0x0     /* not connected */
#define XMOS_MUX_SSWITCH   0x1     /* system switch */
#define XMOS_MUX_XCORE0    0x8     /* xCORE tile 0 */
#define XMOS_MUX_XCORE1    0x9     /* xCORE tile 1 */
#define XMOS_MUX_XCORE2    0xA     /* xCORE tile 2 */
#define XMOS_MUX_XCORE3    0xB     /* xCORE tile 3 */
#define XMOS_MUX_XCOREALL  0xF     /* all xCORE tiles simultaneously */

static inline uint32_t xmos_tile_to_mux(int tile)
{
    return XMOS_MUX_XCORE0 + (uint32_t)tile;
}

/* =========================================================================
 * SET_TEST_MODE register (32-bit chip TAP DR)
 *
 * Per sc_jtag, the upper 28 bits carry a magic key (0xFACED00 << 4) and
 * the low nibble is the mode (e.g. 0x4 = OTP serial enable).  The
 * formerly-defined "BOOT_JTAG"/"RESET_N" bit positions had no public
 * source and collided with the key, so they were removed.
 * ======================================================================= */
#define XMOS_TEST_MODE_KEY           (0xFACED00u << 4)
#define XMOS_TEST_MODE_OTP_SERIAL_EN 0x4

/* =========================================================================
 * Mux-open scan geometry (when MUX is open to an xCORE / SSWITCH)
 *
 * Chain when open: BSCAN(4) + CHIP(4) + XCORE(10) + OTP(2) = 20 IR bits.
 * sc_jtag, however, ALWAYS scans 22 IR bits / 35 DR bits for mux-open
 * access -- the two extra MSB IR bits are don't-cares that flush through.
 * It does NOT build the IR from named fields; it OR's the instruction
 * into a fixed base word that already carries every bypass.  Matching
 * those literal words is what makes register access actually land on the
 * XS2 -- a hand-rolled 20-bit field layout reads IDCODE fine but fails
 * to enter debug.  Verified bug 2026-06: 20-bit IR -> "failed to enter
 * debug mode"; 22-bit sc_jtag framing fixes it.
 *
 *   chip-TAP IR  = 0x00fc3fff | (chip_cmd << 14)   (SETMUX etc.)
 *   xCORE-reg IR = 0x00ffc00f | (tap_ir   << 4)
 *   DR (both)    = 35 bits, shift in (data << N)
 * ======================================================================= */
#define XMOS_CHIP_TAP_IR_LEN         4
#define XMOS_XCORE_TAP_IR_LEN        10
#define XMOS_OTP_TAP_IR_LEN          2

#define XMOS_MUX_OPEN_IR_LEN         22          /* sc_jtag mux-open IR scan */
#define XMOS_MUX_CHAIN_IR_LEN        20          /* actual open chain IR len */
#define XMOS_MUX_OPEN_DR_LEN         35
#define XMOS_MUX_OPEN_BYP_LEN        4           /* sc_jtag MUX_XCORE_BYP_LEN */

#define XMOS_CHIP_IR_OPEN_BASE       0x00fc3fffu /* | (chip_cmd << 14) */
#define XMOS_CHIP_IR_OPEN_SHIFT      14
#define XMOS_XCORE_IR_OPEN_BASE      0x00ffc00fu /* | (tap_ir   << 4)  */
#define XMOS_XCORE_IR_OPEN_SHIFT     4

/* DR scan for register access (sc_jtag jtag_module_reg_access):
 *   - 35 DR bits, shift in (data << 1)
 *   - readback: SSWITCH value = dr_out[31:0]
 *               xCORE   value = (dr_out >> 1)[31:0] */
#define XMOS_REG_DR_LEN              35

/* xCORE TAP IR encoding (sc_jtag): bits [9:2] = register, [1:0] = op */
#define XMOS_REG_OP_BYPASS  0
#define XMOS_REG_OP_READ    1
#define XMOS_REG_OP_WRITE   2

/* =========================================================================
 * Construct the mux-open IR word for an xCORE register access.
 * Mirrors sc_jtag: TapIR = (reg << 2) | op, OR'd into the base at bit 4.
 * ======================================================================= */
static inline uint32_t xmos_chain_ir_reg_read(uint8_t reg)
{
    uint32_t tap_ir = ((uint32_t)reg << 2) | XMOS_REG_OP_READ;
    return XMOS_XCORE_IR_OPEN_BASE | (tap_ir << XMOS_XCORE_IR_OPEN_SHIFT);
}

static inline uint32_t xmos_chain_ir_reg_write(uint8_t reg)
{
    uint32_t tap_ir = ((uint32_t)reg << 2) | XMOS_REG_OP_WRITE;
    return XMOS_XCORE_IR_OPEN_BASE | (tap_ir << XMOS_XCORE_IR_OPEN_SHIFT);
}

/* =========================================================================
 * PSWITCH registers (per-tile, accessed via xCORE TAP)
 *
 * Source: XU208 datasheet Appendix C
 * ======================================================================= */
#define XMOS_PSWITCH_DEVICE_ID0      0x00
#define XMOS_PSWITCH_DEVICE_ID1      0x01
#define XMOS_PSWITCH_DEVICE_ID2      0x02
#define XMOS_PSWITCH_DBG_CTRL        0x04
#define XMOS_PSWITCH_DBG_INT         0x05
#define XMOS_PSWITCH_CLK_DIVIDER     0x06
#define XMOS_PSWITCH_SECU_CONFIG     0x07

/* Debug scratch registers -- used as debug command mailbox */
#define XMOS_PSWITCH_DBG_SCRATCH(n)  (0x20 + (n))  /* n = 0..7 */
#define XMOS_PSWITCH_DBG_STATUS      0x20  /* scratch 0: status/flags */
#define XMOS_PSWITCH_DBG_COMMAND     0x21  /* scratch 1: command */
#define XMOS_PSWITCH_DBG_ARG0        0x22  /* scratch 2: address */
#define XMOS_PSWITCH_DBG_ARG1        0x23  /* scratch 3 */
#define XMOS_PSWITCH_DBG_ARG2        0x24  /* scratch 4: data in/out */
#define XMOS_PSWITCH_DBG_ARG3        0x25  /* scratch 5 */
#define XMOS_PSWITCH_DBG_ARG4        0x26  /* scratch 6 */
#define XMOS_PSWITCH_DBG_ARG5        0x27  /* scratch 7 */

/* Per-core PC/SR (read-only via PSWITCH) */
#define XMOS_PSWITCH_PC_CORE(n)      (0x40 + (n))
#define XMOS_PSWITCH_SR_CORE(n)      (0x60 + (n))

/* =========================================================================
 * DBG_INT register bits
 * ======================================================================= */
#define XMOS_DBG_INT_REQ             0x1   /* Write 1 to request debug */
#define XMOS_DBG_INT_IN_DBG          0x2   /* Read: 1 = core in debug */

/* =========================================================================
 * SSWITCH registers (node level)
 * ======================================================================= */
#define XMOS_SSWITCH_DEVICE_ID0      0x00
#define XMOS_SSWITCH_DEVICE_ID1      0x01
#define XMOS_SSWITCH_NODE_CONFIG     0x04
#define XMOS_SSWITCH_NODE_ID         0x05
#define XMOS_SSWITCH_PLL_CTL         0x06
#define XMOS_SSWITCH_CLK_DIVIDER     0x07
#define XMOS_SSWITCH_REF_CLK_DIV     0x08

/* =========================================================================
 * Processor State (PS) resource identifiers
 *
 * GETPS/SETPS take a fully-encoded resource ID in ARG0, not a bare
 * register number: (ps_number << 8) | 0x0b (resource type PS).
 * Values match xs3a_registers.h (XS1_PS_DBG_SSR=0x100b, SPC=0x110b,
 * SSP=0x120b); sc_jtag dbg_access.xc composes the ID the same way.
 * ======================================================================= */
#define XMOS_PS_RES_ID(n)            (((uint32_t)(n) << 8) | 0x0b)
#define XMOS_PS_RAM_BASE             XMOS_PS_RES_ID(0x00)
#define XMOS_PS_VECTOR_BASE          XMOS_PS_RES_ID(0x01)
#define XMOS_PS_BOOT_STATUS          XMOS_PS_RES_ID(0x03)
#define XMOS_PS_RAM_SIZE             XMOS_PS_RES_ID(0x0C)
#define XMOS_PS_DBG_SSR              XMOS_PS_RES_ID(0x10)  /* Saved status register */
#define XMOS_PS_DBG_SPC              XMOS_PS_RES_ID(0x11)  /* Saved program counter */
#define XMOS_PS_DBG_SSP              XMOS_PS_RES_ID(0x12)  /* Saved stack pointer */
#define XMOS_PS_DBG_INT_TYPE         XMOS_PS_RES_ID(0x15)
#define XMOS_PS_DBG_CORE_CTRL        XMOS_PS_RES_ID(0x18)

/* =========================================================================
 * Debug commands (written to PSWITCH_DBG_COMMAND)
 * ======================================================================= */
#define XMOS_DBG_CMD_READ            1   /* Read memory word */
#define XMOS_DBG_CMD_WRITE           2   /* Write memory word */
#define XMOS_DBG_CMD_READ4PI         3   /* Read 4 words, post-increment */
#define XMOS_DBG_CMD_WRITE4PI        4   /* Write 4 words, post-increment */
#define XMOS_DBG_CMD_GETPS           5   /* Get processor state register */
#define XMOS_DBG_CMD_SETPS           6   /* Set processor state register */
#define XMOS_DBG_CMD_GETSTATE        7   /* Get thread state */
#define XMOS_DBG_CMD_SETSTATE        8   /* Set thread state */
#define XMOS_DBG_CMD_RFDBG           9   /* Return from debug (resume) */

/* =========================================================================
 * Default RAM base addresses
 * ======================================================================= */
#define XMOS_XS1_RAM_BASE            0x00040000u
#define XMOS_XS2_RAM_BASE            0x00040000u
#define XMOS_XS3_RAM_BASE            0x00040000u

/* =========================================================================
 * Known JTAG IDCODE values
 * ======================================================================= */
/* XMOS tools (tool_axe getTypeFromJtagID) compare the low 16-bit device
 * code only; upper IDCODE bits carry revision/variant information. */
#define XMOS_IDCODE_MASK             0x0000FFFF
#define XMOS_IDCODE_XS1_G4           0x00104731u
#define XMOS_IDCODE_XS1_G1           0x00002633u
#define XMOS_IDCODE_XS1_SU           0x00003633u
#define XMOS_IDCODE_XS2              0x00005633u
#define XMOS_IDCODE_XS3              0x00006633u  /* Verify against XU316 DS */

/* =========================================================================
 * Soft reset machine code (XS1 ISA, 48 bytes)
 * Loaded to RAM_BASE and executed for software reset.
 * Source: sc_jtag dbg_soft_reset_code.h
 * ======================================================================= */
static const uint32_t xmos_soft_reset_code[] = {
    0xf30c86ce, 0xac8d6acc, 0xf1f06846, 0xa4816882,
    0x16d34483, 0x6ac0f003, 0xa6d04f2f, 0xad34a480,
    0xaec34483, 0xaecb0ed7, 0x17e34edd, 0x07ec07ed,
};

/* =========================================================================
 * XE file format constants
 *
 * Source: xcore/tool_axe XE.h / XE.cpp (reference XE parser)
 *
 * File layout:
 *   File header: magic(4) + version(2) + pad(2) = 8 bytes
 *   Sector header: type(2) + skip(2) + length(8) = 12 bytes
 *     if length > 0: padding_byte(1) + skip(3) = 4 bytes
 *   For ELF/GOTO/CALL sectors, data starts with sub-header:
 *     node(2) + core(2) + address(8) = 12 bytes
 *   Next sector at: sector_start + 12 + length
 * ======================================================================= */
#define XE_MAGIC_0                   'X'
#define XE_MAGIC_1                   'M'
#define XE_MAGIC_2                   'O'
#define XE_MAGIC_3                   'S'

#define XE_SECTOR_BINARY             0x0001
#define XE_SECTOR_ELF                0x0002
#define XE_SECTOR_CONFIG             0x0003
#define XE_SECTOR_GOTO               0x0005
#define XE_SECTOR_CALL               0x0006
#define XE_SECTOR_XN                 0x0008
#define XE_SECTOR_NODEDESC           0x0009
#define XE_SECTOR_XSCOPE             0x000C
#define XE_SECTOR_LAST               0x5555

/* Sector header sizes */
#define XE_FILE_HDR_SIZE             8
#define XE_SECTOR_HDR_SIZE           12   /* type + skip + length */
#define XE_SECTOR_PAD_DESC_SIZE      4    /* padding_byte + 3 skip */
#define XE_SECTOR_SUB_HDR_SIZE       12   /* node + core + address */

/* ELF constants we need for parsing */
#define ELF_MAGIC                    0x464C457F  /* "\x7FELF" as LE uint32 */
#define PT_LOAD                      1
