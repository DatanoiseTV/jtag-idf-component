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
 * Top-level JTAG TAP (boundary scan TAP) -- 4-bit IR
 * ======================================================================= */
#define XMOS_BSCAN_IR_LEN            4
#define XMOS_BSCAN_IR_EXTEST         0x0   /* Drive pins from BSR */
#define XMOS_BSCAN_IR_IDCODE         0x1
#define XMOS_BSCAN_IR_SAMPLE         0x2   /* Capture pin states (non-destructive) */
#define XMOS_BSCAN_IR_SETMUX         0x4
#define XMOS_BSCAN_IR_SET_TEST_MODE  0x8
#define XMOS_BSCAN_IR_BYPASS         0xF

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
 * SET_TEST_MODE register bits (32-bit DR after SET_TEST_MODE IR)
 * ======================================================================= */
#define XMOS_TEST_MODE_RESET_N       (1u << 31)  /* 1 = do NOT reset */
#define XMOS_TEST_MODE_PLL_LOCK_N    (1u << 30)  /* 1 = don't wait PLL */
#define XMOS_TEST_MODE_BOOT_JTAG     (1u << 29)  /* 1 = boot from JTAG */
#define XMOS_TEST_MODE_PLL_BYPASS    (1u << 28)  /* 1 = bypass PLL */

/* =========================================================================
 * Internal TAP chain lengths (when MUX is open to an xCORE)
 *
 * Chain order (TDI first): OTP -> XCORE -> CHIP -> BSCAN
 * Total IR when mux open: 2 + 10 + 4 + 4 = 20 bits
 *
 * When scanning IR, bits go:
 *   [OTP 2b][XCORE 10b][CHIP 4b][BSCAN 4b]
 * LSB-first means we shift OTP bits first, BSCAN bits last.
 * ======================================================================= */
#define XMOS_CHIP_TAP_IR_LEN         4
#define XMOS_XCORE_TAP_IR_LEN        10
#define XMOS_OTP_TAP_IR_LEN          2

#define XMOS_MUX_TOTAL_IR_LEN        (XMOS_OTP_TAP_IR_LEN + \
                                       XMOS_XCORE_TAP_IR_LEN + \
                                       XMOS_CHIP_TAP_IR_LEN + \
                                       XMOS_BSCAN_IR_LEN)
/* = 2 + 10 + 4 + 4 = 20 bits */

/* Bypass values */
#define XMOS_OTP_TAP_BYPASS          0x3    /* 2 bits all-1 */
#define XMOS_CHIP_TAP_BYPASS         0xF    /* 4 bits all-1 */

/* xCORE TAP IR encoding:
 *   bits [9:2] = register index (8 bits)
 *   bits [1:0] = operation: 0=bypass, 1=read, 2=write */
#define XMOS_REG_OP_BYPASS  0
#define XMOS_REG_OP_READ    1
#define XMOS_REG_OP_WRITE   2

/* DR length for register access:
 *   SSWITCH: 32 bits
 *   xCORE:   33 bits (data << 1, with status bit at LSB on read) */
#define XMOS_SSWITCH_DR_LEN          32
#define XMOS_XCORE_DR_LEN            33

/* Total DR through the chain when mux is open:
 *   OTP bypass(1) + xCORE DR + CHIP bypass(1) + BSCAN bypass(1)
 *   = 1 + 33 + 1 + 1 = 36 bits for xCORE access
 *   = 1 + 32 + 1 + 1 = 35 bits for SSWITCH access */

/* =========================================================================
 * Construct the full-chain IR word for a register access
 *
 * Bit layout (LSB-first shifting, so OTP goes first):
 *   bits [1:0]   = OTP IR (bypass = 0x3)
 *   bits [11:2]  = xCORE IR (reg_op)
 *   bits [15:12] = CHIP IR (bypass = 0xF)
 *   bits [19:16] = BSCAN IR (bypass = 0xF)
 * ======================================================================= */
static inline uint32_t xmos_chain_ir_reg_read(uint8_t reg)
{
    uint16_t xcore_ir = (uint16_t)((reg << 2) | XMOS_REG_OP_READ);
    return (uint32_t)XMOS_OTP_TAP_BYPASS
         | ((uint32_t)xcore_ir << XMOS_OTP_TAP_IR_LEN)
         | ((uint32_t)XMOS_CHIP_TAP_BYPASS << (XMOS_OTP_TAP_IR_LEN + XMOS_XCORE_TAP_IR_LEN))
         | ((uint32_t)XMOS_BSCAN_IR_BYPASS << (XMOS_OTP_TAP_IR_LEN + XMOS_XCORE_TAP_IR_LEN + XMOS_CHIP_TAP_IR_LEN));
}

static inline uint32_t xmos_chain_ir_reg_write(uint8_t reg)
{
    uint16_t xcore_ir = (uint16_t)((reg << 2) | XMOS_REG_OP_WRITE);
    return (uint32_t)XMOS_OTP_TAP_BYPASS
         | ((uint32_t)xcore_ir << XMOS_OTP_TAP_IR_LEN)
         | ((uint32_t)XMOS_CHIP_TAP_BYPASS << (XMOS_OTP_TAP_IR_LEN + XMOS_XCORE_TAP_IR_LEN))
         | ((uint32_t)XMOS_BSCAN_IR_BYPASS << (XMOS_OTP_TAP_IR_LEN + XMOS_XCORE_TAP_IR_LEN + XMOS_CHIP_TAP_IR_LEN));
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
 * Processor State (PS) register numbers
 * Accessed via debug GETPS/SETPS commands
 * ======================================================================= */
#define XMOS_PS_RAM_BASE             0x00
#define XMOS_PS_VECTOR_BASE          0x01
#define XMOS_PS_BOOT_STATUS          0x03
#define XMOS_PS_RAM_SIZE             0x0C
#define XMOS_PS_DBG_SSR              0x10   /* Saved status register */
#define XMOS_PS_DBG_SPC              0x11   /* Saved program counter */
#define XMOS_PS_DBG_SSP              0x12   /* Saved stack pointer */
#define XMOS_PS_DBG_INT_TYPE         0x15
#define XMOS_PS_DBG_CORE_CTRL        0x18

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
#define XMOS_IDCODE_MASK             0x0FFFFFFF  /* Ignore revision nibble */
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
