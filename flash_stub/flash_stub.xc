/*
 * XMOS JTAG flash programmer stub  (REFERENCE -- compile with xcc, validate
 * on hardware; see README.md).
 *
 * Runs on the xCORE, driven over JTAG by xmos_jtag_program_flash() on the
 * ESP32 side. It polls the PSWITCH debug scratch mailbox, programs the QSPI
 * boot flash with the streamed boot image, and reports status.
 *
 * ONE stub for the whole family -- the QSPI boot ports are the same on
 * every part, so this builds for all of them by just selecting the arch:
 *
 *   Part            Arch    Build target / -march   Flash
 *   --------------  ------  ----------------------  -------------------
 *   XU208           xs2a    xcore-200               external QSPI
 *   XUF208 / XUF216 xs2a    xcore-200               internal (same ports)
 *   XU316           xs3a    xcore.ai                external QSPI
 *
 * (There is no XUF316 -- xcore.ai is external-flash only; the "UF" internal-
 * flash suffix exists only on xCORE-200/XS2. Internal flash on XUF parts is
 * transparent QSPI on the SAME ports, so nothing changes here.)
 *
 * Flash access uses the QSPI driver from XMOS's permissively-licensed
 * xcore/sc_flash (module_quad_spi_flash). Drop that module's
 * quad_spi_flash.{xc,h} next to this file (or add it to USED_MODULES).
 *
 * THINGS TO CONFIRM FOR YOUR TARGET before this runs:
 *   1. The mailbox access. The host writes the PSWITCH debug scratch
 *      registers (0x20..0x23) over JTAG; this stub must read/write the SAME
 *      registers from the running tile. getps/setps is used below; verify
 *      the register space against your tools (xs1.h / the XS1 architecture
 *      manual) -- this is the one part that cannot be checked without the
 *      toolchain.
 *   2. On xcore.ai (XS3) ONLY: the sc_flash low-level driver's read sample
 *      point was hand-tuned for XS2's port/ref clock. It compiles for xs3a
 *      but the timing must be re-validated on silicon. Keep SCLK
 *      conservative (<= ~25 MHz, libquadflash's default class) -- erase/
 *      page-program timing is forgiving; only fast reads need the
 *      data-eye calibration that lib_qspi_fast_read does. Slow the divider
 *      in quad_spi_flash_init() if reads come back wrong on XS3.
 *      (Ports are identical to XS2: CS=1B, SCLK=1C, SIO=4B -- verified
 *      against XMOS's xcore.ai audio-board XN files.)
 */

#include <xs1.h>
#include <platform.h>
#include "quad_spi_flash.h"

/* --- Mailbox: PSWITCH debug scratch registers (match xmos_regs.h host side) */
#define DBG_STATUS    0x20   /* scratch[0]  stub -> host */
#define DBG_COMMAND   0x21   /* scratch[1]  host -> stub */
#define DBG_ARG0      0x22   /* scratch[2]  address */
#define DBG_ARG1      0x23   /* scratch[3]  byte count */

#define CMD_NONE   0x00
#define CMD_WRITE  0x01
#define CMD_ERASE  0x02
#define CMD_DONE   0xFF

#define ST_READY   0x01
#define ST_BUSY    0x02
#define ST_OK      0x03
#define ST_ERROR   0x80

/* Shared data buffer in tile RAM. Must equal RAM_BASE + STUB_DATA_BUF_OFFSET
 * on the host (0x00040000 + 0x10000 = 0x00050000) and be reserved by the
 * linker so it does not overlap this program. */
#define DATA_BUF_ADDR  0x00050000

/* QSPI boot ports -- IDENTICAL on xCORE-200 (XU208/XUF208/XUF216) and
 * xcore.ai (XU316): AN00185 §2.3 / XU208 §8.1 / XU316 boot section, and
 * confirmed against XMOS's xk-audio-316-mc XN. Flash is always on tile[0]. */
on tile[0]: quad_spi_ports qspi = {
    XS1_PORT_1B,   /* CS   = X0D01 */
    XS1_PORT_1C,   /* SCLK = X0D10 */
    XS1_PORT_4B,   /* SIO  = X0D04..07 */
    XS1_CLKBLK_1
};

/* Mailbox helpers. getps/setps reach the local tile's debug scratch regs;
 * see the header note -- confirm the register space for your tools. */
static inline unsigned mbox_get(unsigned reg)        { return getps(reg); }
static inline void     mbox_set(unsigned reg, unsigned v) { setps(reg, v); }

int main(void)
{
    /* Bring up the flash (init also issues write-enable + sets Quad-Enable
     * and waits for idle -- see sc_flash quad_spi_flash_init). */
    quad_spi_flash_init(qspi);
    mbox_set(DBG_COMMAND, CMD_NONE);
    mbox_set(DBG_STATUS,  ST_READY);

    for (;;) {
        unsigned cmd = mbox_get(DBG_COMMAND);
        if (cmd == CMD_NONE)
            continue;

        unsigned addr = mbox_get(DBG_ARG0);

        if (cmd == CMD_ERASE) {
            quad_spi_flash_sector_erase(qspi, addr);
            quad_spi_wait_until_idle(qspi);
            mbox_set(DBG_COMMAND, CMD_NONE);
            mbox_set(DBG_STATUS,  ST_OK);

        } else if (cmd == CMD_WRITE) {
            unsigned nbytes = mbox_get(DBG_ARG1);
            unsigned nwords = (nbytes + 3) >> 2;
            /* quad_spi_flash_write_page() (the only write primitive this
             * version of sc_flash actually implements -- there is no
             * write_sub_page in quad_spi_flash.xc) always writes a full
             * 64-word/256-byte page. Pad any short final chunk with the
             * flash's erased-state value (0xFFFFFFFF) so a sub-page write
             * doesn't clobber already-written bytes beyond nbytes -- the
             * host's chunking must still align each WRITE to a page
             * boundary (offset a multiple of 256 within the image). */
            unsigned buf[QUAD_SPI_FLASH_BYTES_PER_PAGE / 4];
            for (unsigned i = 0; i < QUAD_SPI_FLASH_BYTES_PER_PAGE / 4; i++)
                buf[i] = QUAD_SPI_FLASH_ERASED;
            unsafe {
                unsigned * unsafe src = (unsigned * unsafe)DATA_BUF_ADDR;
                for (unsigned i = 0; i < nwords; i++)
                    buf[i] = src[i];
            }
            quad_spi_flash_write_page(qspi, addr, buf);
            quad_spi_wait_until_idle(qspi);
            mbox_set(DBG_COMMAND, CMD_NONE);
            mbox_set(DBG_STATUS,  ST_OK);

        } else if (cmd == CMD_DONE) {
            /* init() already set QE; nothing more to do. Halt. */
            mbox_set(DBG_COMMAND, CMD_NONE);
            mbox_set(DBG_STATUS,  ST_OK);
            break;

        } else {
            mbox_set(DBG_STATUS, ST_ERROR);
            break;
        }
    }
    return 0;
}
