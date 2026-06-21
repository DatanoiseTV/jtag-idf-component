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

/* --- RAM mailbox -----------------------------------------------------------
 * A normally-running core can't reach the JTAG-written PSWITCH debug-scratch
 * registers (getps/setps don't alias them), so the mailbox lives in plain tile
 * RAM: the host reads/writes it over the JTAG debug *memory* interface while
 * the core is briefly halted, and this stub reads/writes the same words with
 * ordinary volatile loads/stores.
 *
 * Base = RAM_BASE + 0x10000 (must equal STUB_MBOX_OFFSET on the host).
 * RAM_BASE is arch-specific: xCORE-200 (XS2) maps SRAM at 0x40000, xcore.ai
 * (XS3) at 0x80000.
 *   words: [0]=STATUS  [1]=COMMAND  [2]=ARG0(addr)  [3]=ARG1(nbytes)  [4..]=DATA */
#if defined(__XS3A__)
#define MBOX_ADDR  0x00090000
#else
#define MBOX_ADDR  0x00050000
#endif
#define M_STATUS   0
#define M_COMMAND  1
#define M_ARG0     2
#define M_ARG1     3
#define M_DATA     4   /* page buffer starts here (256 bytes) */

#define CMD_NONE   0x00
#define CMD_WRITE  0x01
#define CMD_ERASE  0x02
#define CMD_DONE   0xFF

#define ST_ALIVE   0x55   /* set before flash init -- liveness marker */
#define ST_READY   0x01
#define ST_BUSY    0x02
#define ST_OK      0x03
#define ST_ERROR   0x80

/* QSPI boot ports -- IDENTICAL on xCORE-200 (XU208/XUF208/XUF216) and
 * xcore.ai (XU316): AN00185 §2.3 / XU208 §8.1 / XU316 boot section, and
 * confirmed against XMOS's xk-audio-316-mc XN. Flash is always on tile[0]. */
on tile[0]: quad_spi_ports qspi = {
    XS1_PORT_1B,   /* CS   = X0D01 */
    XS1_PORT_1C,   /* SCLK = X0D10 */
    XS1_PORT_4B,   /* SIO  = X0D04..07 */
    XS1_CLKBLK_1
};

int main(void)
{
    unsafe {
        /* The mailbox is plain RAM; volatile so the poll loop re-reads it. */
        volatile unsigned * unsafe mb = (volatile unsigned * unsafe)MBOX_ADDR;

        /* Liveness marker BEFORE touching the flash: if the host times out
         * seeing 0x55 the stub runs and the RAM mailbox works but flash init
         * stalled; 0x00 means the stub never ran / the mailbox isn't reaching
         * the host. */
        mb[M_STATUS] = ST_ALIVE;

        /* Bring up the flash (init issues write-enable + sets Quad-Enable and
         * waits for idle -- see sc_flash quad_spi_flash_init). */
        quad_spi_flash_init(qspi);
        mb[M_COMMAND] = CMD_NONE;
        mb[M_STATUS]  = ST_READY;

        for (;;) {
            unsigned cmd = mb[M_COMMAND];
            if (cmd == CMD_NONE)
                continue;

            unsigned addr = mb[M_ARG0];

            if (cmd == CMD_ERASE) {
                quad_spi_flash_sector_erase(qspi, addr);
                quad_spi_wait_until_idle(qspi);
                mb[M_COMMAND] = CMD_NONE;
                mb[M_STATUS]  = ST_OK;

            } else if (cmd == CMD_WRITE) {
                unsigned nbytes = mb[M_ARG1];
                unsigned nwords = (nbytes + 3) >> 2;
                /* quad_spi_flash_write_page() always writes a full 256-byte
                 * page, so pad a short final chunk with the erased value
                 * (0xFFFFFFFF). The host keeps every WRITE page-aligned. */
                unsigned buf[QUAD_SPI_FLASH_BYTES_PER_PAGE / 4];
                for (unsigned i = 0; i < QUAD_SPI_FLASH_BYTES_PER_PAGE / 4; i++)
                    buf[i] = QUAD_SPI_FLASH_ERASED;
                for (unsigned i = 0; i < nwords; i++)
                    buf[i] = mb[M_DATA + i];
                quad_spi_flash_write_page(qspi, addr, buf);
                quad_spi_wait_until_idle(qspi);
                mb[M_COMMAND] = CMD_NONE;
                mb[M_STATUS]  = ST_OK;

            } else if (cmd == CMD_DONE) {
                /* init() already set QE; nothing more to do. Halt. */
                mb[M_COMMAND] = CMD_NONE;
                mb[M_STATUS]  = ST_OK;
                break;

            } else {
                mb[M_STATUS] = ST_ERROR;
                break;
            }
        }
    }
    return 0;
}
