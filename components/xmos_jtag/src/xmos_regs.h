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
 * Top-level TAP (XS2 / xCORE-200).
 *
 * IMPORTANT XS1 -> XS2 difference (XMOS engineer "XMatt", xcore.com t=3277,
 * and segher's xena-xtools jtag.c): XS1 exposed TWO top TAPs after reset
 * (boundary-scan + chip).  XS2/XU208 MERGED them into ONE top-level TAP
 * (4-bit IR) and REMOVED the OTP TAP from the internal chain.  An earlier
 * revision here used the XS1 two-TAP / OTP framing (8-bit closed IR, 22-bit
 * open IR with OTP, data<<1) and on real XU208 silicon every mux-open
 * register read returned 0x00000000 (the identify() self-test caught it).
 * The framing below is the XS2 single-TAP model.
 *
 * The same top-TAP instructions as XS1 are kept (SETMUX etc.).  EXTEST/
 * SAMPLE opcodes are still unconfirmed (no public XMOS BSDL).
 * ======================================================================= */
#define XMOS_BSCAN_IR_LEN            4     /* top-level TAP IR width */
#define XMOS_BSCAN_IR_EXTEST         0x0   /* UNVERIFIED (no public BSDL) */
#define XMOS_BSCAN_IR_SAMPLE         0x2   /* UNVERIFIED (no public BSDL) */
#define XMOS_BSCAN_IR_BYPASS         0xF

#define XMOS_TOP_TAP_IR_LEN          4     /* single merged top-level TAP */
#define XMOS_CHIP_TAP_IR_DEVICE_ID   0x3   /* segher jtag_read_idcode */
#define XMOS_CHIP_TAP_IR_SETMUX      0x4   /* SETMUX */
#define XMOS_CHIP_TAP_IR_GETMUX      0x5   /* GETMUX */
#define XMOS_CHIP_TAP_IR_SET_TEST_MODE 0x8 /* SET_TEST_MODE */

/* SETMUX DR width.  segher writes the 4-bit mux value as a wide DR
 * (do_dr_out(mux, 32)); on the XS2 single TAP we shift the value over this
 * width.  Flagged for hardware confirmation (see flash_stub/README notes). */
#define XMOS_SETMUX_DR_LEN           32

/* =========================================================================
 * Chip TAP MUX target values (shifted into DR after SETMUX IR)
 *
 * Source: segher xena-xtools jtag.c (jtag_set_mux: 8 + core), sc_jtag.
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
 * Mux-open register-access geometry (XS2 single-tile: XU208/XUF208).
 *
 * From segher's xena-xtools jtag.c (the authoritative reverse-engineered
 * reference) with XMatt's XS2 deltas applied (merged top TAP, OTP removed):
 *
 *   debug_read_pswitch(reg):  do_ir((reg<<2)|1, 10); do_dr_in (val, 32)
 *   debug_write_pswitch(reg): do_ir((reg<<2)|2, 10); do_dr_out(val, 32)
 *
 * The module (xCORE / SSWITCH) TAP IR is 10 bits = (reg<<2)|op (op 1=read,
 * 2=write).  The data DR is PLAIN 32 bits -- NOT data<<1; the bypass bits
 * are pre/post-chain, never folded into the data.
 *
 * XS2 chain when a module is selected (no OTP, one merged top TAP):
 *   IR = MODULE(10, low bits) + TOP bypass(4) = 14 bits
 *   DR = MODULE data(32, low bits) + TOP bypass(1) = 33 bits
 *   readback value = DR[31:0]  (same for xCORE and SSWITCH on XS2)
 * ======================================================================= */
#define XMOS_XCORE_TAP_IR_LEN        10          /* module (xCORE/SSWITCH) IR */

#define XMOS_MUX_OPEN_IR_LEN         14          /* MODULE(10) + TOP bypass(4) */
#define XMOS_MUX_OPEN_DR_LEN         33          /* data(32) + TOP bypass(1) */

/* module TAP IR encoding: bits[9:2] = register, [1:0] = op */
#define XMOS_REG_OP_BYPASS  0
#define XMOS_REG_OP_READ    1
#define XMOS_REG_OP_WRITE   2

/* Build the 14-bit mux-open IR: module IR in low 10 bits, top-TAP bypass
 * (0xF) in bits [13:10]. */
static inline uint32_t xmos_chain_ir_reg_read(uint8_t reg)
{
    uint32_t mod = (((uint32_t)reg << 2) | XMOS_REG_OP_READ) & 0x3FF;
    return mod | ((uint32_t)XMOS_BSCAN_IR_BYPASS << XMOS_XCORE_TAP_IR_LEN);
}

static inline uint32_t xmos_chain_ir_reg_write(uint8_t reg)
{
    uint32_t mod = (((uint32_t)reg << 2) | XMOS_REG_OP_WRITE) & 0x3FF;
    return mod | ((uint32_t)XMOS_BSCAN_IR_BYPASS << XMOS_XCORE_TAP_IR_LEN);
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
#define XMOS_SSWITCH_JTAG_DEVICE_ID  0x09   /* xs3a_registers.h JTAG_DEVICE_ID */

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
