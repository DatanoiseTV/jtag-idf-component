/*
 * XMOS JTAG flash programmer stub  (REFERENCE -- compile with xcc, validate
 * on hardware; see README.md).
 *
 * Runs on the xCORE, driven over JTAG by xmos_jtag_program_flash() on the
 * ESP32 side. It polls the PSWITCH debug scratch mailbox, programs the QSPI
 * boot flash with the streamed boot image, and reports status.
 *
 * Flash access uses the QSPI driver from XMOS's permissively-licensed
 * xcore/sc_flash (module_quad_spi_flash). Drop that module's
 * quad_spi_flash.{xc,h} next to this file (or add it to USED_MODULES).
 *
 * TWO THINGS MUST BE CONFIRMED FOR YOUR TARGET before this runs:
 *   1. The QSPI port map (quad_spi_ports below) -- the values here are the
 *      xCORE-200 hardware boot ports (CS=X0D01, SCLK=X0D10, SIO=X0D04..07).
 *   2. The mailbox access. The host writes the PSWITCH debug scratch
 *      registers (0x20..0x23) over JTAG; this stub must read/write the SAME
 *      registers from the running tile. getps/setps is used below; verify
 *      the register space against your tools (xs1.h / the XS1 architecture
 *      manual) -- this is the one part that cannot be checked without the
 *      toolchain.
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

/* QSPI boot ports for xCORE-200 (AN00185 §2.3 / XU208 datasheet §8.1). */
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
            /* Copy the chunk the host placed in the shared buffer. */
            unsigned buf[QUAD_SPI_FLASH_BYTES_PER_PAGE / 4];
            unsigned * unsafe src = (unsigned * unsafe)DATA_BUF_ADDR;
            for (unsigned i = 0; i < nwords; i++)
                buf[i] = src[i];
            quad_spi_flash_write_sub_page(qspi, addr, buf, nwords);
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
