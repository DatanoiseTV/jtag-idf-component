/*
 * SWD (Serial Wire Debug) -- ARM ADIv5 two-wire debug, bit-banged on an ESP32.
 *
 * Implements the wire protocol (line reset, JTAG-to-SWD switch, packet
 * request/ACK/data with parity and turnaround + WAIT retry), DP/AP register
 * access, MEM-AP memory read/write, Cortex-M core halt/run/reset, and device
 * identification (DPIDR, AP IDR, SCB CPUID, vendor DBGMCU IDCODE).
 *
 * STATUS: spec-correct against ADIv5 / ARMv7-M, but NOT yet hardware-verified.
 * Per-target flashing (STM32/GD32 register FPEC, RP2040 bootrom stub) is a
 * separate phase and not implemented here.
 *
 * References: ARM IHI0031 (ADIv5), ARM DDI0403 (ARMv7-M, SCS/DCB registers).
 */

#include "swd.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "swd";

/* Half-clock delay -- bit-bang SWD, conservative on jumper wiring (~250 kHz). */
#define SWD_HALF_US        2
#define SWD_WAIT_RETRIES   64    /* ACK=WAIT retries per transfer */

/* ARMv7-M debug registers (System Control Space), addressed via the MEM-AP. */
#define SCB_CPUID          0xE000ED00u
#define DCB_DHCSR          0xE000EDF0u
#define DCB_DEMCR          0xE000EDFCu
#define SCB_AIRCR          0xE000ED0Cu
#define STM32_DBGMCU_IDCODE 0xE0042000u   /* STM32/GD32 device ID */

#define DHCSR_DBGKEY       0xA05F0000u
#define DHCSR_C_DEBUGEN    (1u << 0)
#define DHCSR_C_HALT       (1u << 1)
#define DHCSR_S_HALT       (1u << 17)
#define DEMCR_VC_CORERESET (1u << 0)
#define AIRCR_VECTKEY      0x05FA0000u
#define AIRCR_SYSRESETREQ  (1u << 2)

struct swd_ctx {
    swd_pins_t pins;
    uint8_t    ap;            /* current MEM-AP index */
    uint32_t   select;        /* shadow of DP SELECT */
    bool       select_valid;
    int        dir_out;       /* 1 = SWDIO driven by us, 0 = input */
};

/* -------------------------------------------------------------------------
 * Bit-bang primitives
 * ---------------------------------------------------------------------- */
static inline void clk_pulse(swd_handle_t h)
{
    gpio_set_level(h->pins.swclk, 0);
    esp_rom_delay_us(SWD_HALF_US);
    gpio_set_level(h->pins.swclk, 1);
    esp_rom_delay_us(SWD_HALF_US);
}

static void swdio_dir(swd_handle_t h, int out)
{
    if (h->dir_out == out) return;
    gpio_set_direction(h->pins.swdio, out ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT);
    h->dir_out = out;
}

/* Write one bit (host drives SWDIO; target samples on the rising edge). */
static inline void wr_bit(swd_handle_t h, int b)
{
    gpio_set_level(h->pins.swdio, b & 1);
    clk_pulse(h);
}

/* Read one bit (target drives SWDIO; sample while clock is low, then pulse). */
static inline int rd_bit(swd_handle_t h)
{
    gpio_set_level(h->pins.swclk, 0);
    esp_rom_delay_us(SWD_HALF_US);
    int b = gpio_get_level(h->pins.swdio);
    gpio_set_level(h->pins.swclk, 1);
    esp_rom_delay_us(SWD_HALF_US);
    return b;
}

static void wr_bits(swd_handle_t h, uint32_t val, int n)
{
    swdio_dir(h, 1);
    for (int i = 0; i < n; i++)
        wr_bit(h, (val >> i) & 1);
}

static uint32_t rd_bits(swd_handle_t h, int n)
{
    uint32_t v = 0;
    swdio_dir(h, 0);
    for (int i = 0; i < n; i++)
        v |= (uint32_t)rd_bit(h) << i;
    return v;
}

/* Turnaround: one clock with SWDIO released (neither side drives it). */
static void trn(swd_handle_t h)
{
    swdio_dir(h, 0);
    clk_pulse(h);
}

static inline int parity32(uint32_t v)
{
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return v & 1;
}

/* -------------------------------------------------------------------------
 * Line reset + JTAG-to-SWD switch
 * ---------------------------------------------------------------------- */
static void line_reset(swd_handle_t h)
{
    /* >= 50 clocks with SWDIO high, then >= 2 idle clocks low. */
    swdio_dir(h, 1);
    gpio_set_level(h->pins.swdio, 1);
    for (int i = 0; i < 56; i++) clk_pulse(h);
    gpio_set_level(h->pins.swdio, 0);
    for (int i = 0; i < 4; i++) clk_pulse(h);
}

/* -------------------------------------------------------------------------
 * Core transfer: one DP/AP read or write with ACK + parity + WAIT retry.
 * ---------------------------------------------------------------------- */
static esp_err_t swd_xfer(swd_handle_t h, int apndp, int rnw, uint8_t addr,
                          uint32_t *data)
{
    int a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
    int par = (apndp ^ rnw ^ a2 ^ a3) & 1;
    uint32_t req = 0x81                      /* start(1) ... park(1) */
                 | (apndp << 1) | (rnw << 2)
                 | (a2 << 3) | (a3 << 4) | (par << 5);

    for (int attempt = 0; attempt < SWD_WAIT_RETRIES; attempt++) {
        wr_bits(h, req, 8);
        trn(h);                              /* host -> target */
        uint32_t ack = rd_bits(h, 3);

        if (ack == SWD_ACK_WAIT) {
            trn(h);                          /* target -> host, retry */
            continue;
        }
        if (ack != SWD_ACK_OK) {
            trn(h);
            ESP_LOGD(TAG, "xfer ack=0x%lx (addr 0x%x %s%s)", (unsigned long)ack,
                     addr, apndp ? "AP" : "DP", rnw ? " rd" : " wr");
            return (ack == SWD_ACK_FAULT) ? ESP_FAIL : ESP_ERR_INVALID_RESPONSE;
        }

        if (rnw) {                           /* READ: 32 data + parity, then trn */
            uint32_t v = rd_bits(h, 32);
            int p = rd_bit(h);
            trn(h);                          /* target -> host */
            if (p != parity32(v)) {
                ESP_LOGW(TAG, "read parity error @0x%x", addr);
                return ESP_ERR_INVALID_CRC;
            }
            if (data) *data = v;
        } else {                             /* WRITE: trn, then 32 data + parity */
            trn(h);                          /* target -> host */
            uint32_t v = data ? *data : 0;
            wr_bits(h, v, 32);
            swdio_dir(h, 1);
            wr_bit(h, parity32(v));
        }
        return ESP_OK;
    }
    ESP_LOGW(TAG, "xfer timed out (WAIT) @0x%x", addr);
    return ESP_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------
 * DP / AP access
 * ---------------------------------------------------------------------- */
esp_err_t swd_dp_read(swd_handle_t h, uint8_t addr, uint32_t *val)
{
    return swd_xfer(h, 0, 1, addr, val);
}
esp_err_t swd_dp_write(swd_handle_t h, uint8_t addr, uint32_t val)
{
    return swd_xfer(h, 0, 0, addr, &val);
}

/* Point DP SELECT at the right AP + register bank before an AP access. */
static esp_err_t ap_select(swd_handle_t h, uint8_t ap, uint8_t addr)
{
    uint32_t sel = ((uint32_t)ap << 24) | (addr & 0xF0);
    if (h->select_valid && h->select == sel) return ESP_OK;
    esp_err_t err = swd_dp_write(h, SWD_DP_SELECT, sel);
    if (err == ESP_OK) { h->select = sel; h->select_valid = true; }
    return err;
}

esp_err_t swd_ap_read(swd_handle_t h, uint8_t ap, uint8_t addr, uint32_t *val)
{
    esp_err_t err = ap_select(h, ap, addr);
    if (err != ESP_OK) return err;
    /* AP reads are posted: the result of this read comes back on the next
     * read, so issue the AP read (discard) then read RDBUFF for the value. */
    err = swd_xfer(h, 1, 1, addr, NULL);
    if (err != ESP_OK) return err;
    return swd_dp_read(h, SWD_DP_RDBUFF, val);
}

esp_err_t swd_ap_write(swd_handle_t h, uint8_t ap, uint8_t addr, uint32_t val)
{
    esp_err_t err = ap_select(h, ap, addr);
    if (err != ESP_OK) return err;
    return swd_xfer(h, 1, 0, addr, &val);
}

/* -------------------------------------------------------------------------
 * MEM-AP memory access
 * ---------------------------------------------------------------------- */
void swd_set_ap(swd_handle_t h, uint8_t ap) { h->ap = ap; }

/* Set CSW size=word + the given auto-increment, preserving the upper
 * (Prot/Debug) bits the AP powered up with rather than guessing them. */
static esp_err_t csw_word(swd_handle_t h, uint32_t addrinc)
{
    uint32_t csw;
    esp_err_t err = swd_ap_read(h, h->ap, SWD_AP_CSW, &csw);
    if (err != ESP_OK) return err;
    csw = (csw & ~0x37u) | 0x02u | addrinc;   /* size[2:0]=word, addrinc[5:4] */
    return swd_ap_write(h, h->ap, SWD_AP_CSW, csw);
}

esp_err_t swd_mem_read32(swd_handle_t h, uint32_t addr, uint32_t *val)
{
    esp_err_t err = csw_word(h, 0x00);        /* no auto-increment */
    if (err != ESP_OK) return err;
    if ((err = swd_ap_write(h, h->ap, SWD_AP_TAR, addr)) != ESP_OK) return err;
    return swd_ap_read(h, h->ap, SWD_AP_DRW, val);
}

esp_err_t swd_mem_write32(swd_handle_t h, uint32_t addr, uint32_t val)
{
    esp_err_t err = csw_word(h, 0x00);
    if (err != ESP_OK) return err;
    if ((err = swd_ap_write(h, h->ap, SWD_AP_TAR, addr)) != ESP_OK) return err;
    return swd_ap_write(h, h->ap, SWD_AP_DRW, val);
}

/* Block access uses TAR auto-increment, which the AP only guarantees within a
 * 1 KB page, so reset TAR at every page boundary. */
esp_err_t swd_mem_read(swd_handle_t h, uint32_t addr, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len >= 4) {
        esp_err_t err = csw_word(h, 0x10);    /* single auto-increment */
        if (err != ESP_OK) return err;
        if ((err = swd_ap_write(h, h->ap, SWD_AP_TAR, addr)) != ESP_OK) return err;
        uint32_t end_page = (addr | 0x3FF) + 1;
        while (len >= 4 && addr < end_page) {
            uint32_t v;
            if ((err = swd_ap_read(h, h->ap, SWD_AP_DRW, &v)) != ESP_OK) return err;
            memcpy(p, &v, 4);
            p += 4; addr += 4; len -= 4;
        }
    }
    return ESP_OK;
}

esp_err_t swd_mem_write(swd_handle_t h, uint32_t addr, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len >= 4) {
        esp_err_t err = csw_word(h, 0x10);
        if (err != ESP_OK) return err;
        if ((err = swd_ap_write(h, h->ap, SWD_AP_TAR, addr)) != ESP_OK) return err;
        uint32_t end_page = (addr | 0x3FF) + 1;
        while (len >= 4 && addr < end_page) {
            uint32_t v; memcpy(&v, p, 4);
            if ((err = swd_ap_write(h, h->ap, SWD_AP_DRW, v)) != ESP_OK) return err;
            p += 4; addr += 4; len -= 4;
        }
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Connect / power up
 * ---------------------------------------------------------------------- */
esp_err_t swd_connect(swd_handle_t h, uint32_t *dpidr)
{
    h->select_valid = false;

    /* Line reset, JTAG-to-SWD select (0xE79E), line reset, then read DPIDR. */
    line_reset(h);
    wr_bits(h, 0xE79E, 16);
    line_reset(h);
    /* >= 2 idle cycles with SWDIO low before the first packet. */
    swdio_dir(h, 1); gpio_set_level(h->pins.swdio, 0);
    for (int i = 0; i < 4; i++) clk_pulse(h);

    uint32_t id = 0;
    esp_err_t err = swd_dp_read(h, SWD_DP_DPIDR, &id);
    if (err != ESP_OK) { ESP_LOGW(TAG, "no DPIDR (ACK fail) -- check wiring"); return err; }
    if (id == 0 || id == 0xFFFFFFFFu) {
        ESP_LOGW(TAG, "implausible DPIDR 0x%08lx", (unsigned long)id);
        return ESP_ERR_NOT_FOUND;
    }
    if (dpidr) *dpidr = id;

    /* Clear sticky errors, then power up debug + system domains. */
    swd_dp_write(h, SWD_DP_ABORT, 0x1E);      /* clear STKCMP/STKERR/WDERR/ORUN */
    err = swd_dp_write(h, SWD_DP_SELECT, 0);
    h->select = 0; h->select_valid = true;
    err |= swd_dp_write(h, SWD_DP_CTRLSTAT, (1u << 28) | (1u << 30)); /* CDBGPWRUPREQ|CSYSPWRUPREQ */
    if (err != ESP_OK) return ESP_FAIL;

    int64_t deadline = esp_timer_get_time() + 200000;
    for (;;) {
        uint32_t cs = 0;
        if (swd_dp_read(h, SWD_DP_CTRLSTAT, &cs) == ESP_OK &&
            (cs & (1u << 29)) && (cs & (1u << 31)))   /* CDBGPWRUPACK|CSYSPWRUPACK */
            break;
        if (esp_timer_get_time() > deadline) {
            ESP_LOGW(TAG, "debug power-up not acked");
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Cortex-M core control
 * ---------------------------------------------------------------------- */
esp_err_t swd_halt(swd_handle_t h)
{
    esp_err_t err = swd_mem_write32(h, DCB_DHCSR,
                                    DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT);
    if (err != ESP_OK) return err;
    int64_t deadline = esp_timer_get_time() + 200000;
    for (;;) {
        uint32_t v = 0;
        if (swd_mem_read32(h, DCB_DHCSR, &v) == ESP_OK && (v & DHCSR_S_HALT))
            return ESP_OK;
        if (esp_timer_get_time() > deadline) return ESP_ERR_TIMEOUT;
    }
}

esp_err_t swd_run(swd_handle_t h)
{
    return swd_mem_write32(h, DCB_DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN);
}

bool swd_is_halted(swd_handle_t h)
{
    uint32_t v = 0;
    return swd_mem_read32(h, DCB_DHCSR, &v) == ESP_OK && (v & DHCSR_S_HALT);
}

esp_err_t swd_reset_halt(swd_handle_t h)
{
    esp_err_t err = swd_mem_write32(h, DCB_DHCSR,
                                    DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT);
    err |= swd_mem_write32(h, DCB_DEMCR, DEMCR_VC_CORERESET);   /* halt at reset vector */
    err |= swd_mem_write32(h, SCB_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);
    if (err != ESP_OK) return ESP_FAIL;
    esp_rom_delay_us(20000);
    /* The reset may drop the debug connection; re-establish it. */
    if (swd_connect(h, NULL) != ESP_OK) return ESP_FAIL;
    int64_t deadline = esp_timer_get_time() + 300000;
    for (;;) {
        uint32_t v = 0;
        if (swd_mem_read32(h, DCB_DHCSR, &v) == ESP_OK && (v & DHCSR_S_HALT)) break;
        if (esp_timer_get_time() > deadline) return ESP_ERR_TIMEOUT;
    }
    swd_mem_write32(h, DCB_DEMCR, 0);   /* clear the reset vector catch */
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Identify
 * ---------------------------------------------------------------------- */
static const char *core_name(uint16_t partno)
{
    switch (partno) {
        case 0xC20: return "Cortex-M0";
        case 0xC60: return "Cortex-M0+";
        case 0xC21: return "Cortex-M1";
        case 0xC23: return "Cortex-M3";
        case 0xC24: return "Cortex-M4";
        case 0xC27: return "Cortex-M7";
        case 0xD20: return "Cortex-M23";
        case 0xD21: return "Cortex-M33";
        default:    return "unknown Cortex-M";
    }
}

/* Best-effort vendor/family from the STM32-style DBGMCU IDCODE DEV_ID. */
static const char *vendor_name(uint32_t dbg_idcode)
{
    uint16_t dev = dbg_idcode & 0xFFF;
    switch (dev) {
        case 0x410: return "STM32F1 / GD32F1 (medium-density)";
        case 0x411: return "STM32F2";
        case 0x413: return "STM32F405/407/415/417";
        case 0x419: return "STM32F42x/43x";
        case 0x431: return "STM32F411";
        case 0x440: return "STM32F030/F05x";
        case 0x444: return "STM32F03x";
        case 0x447: return "STM32L0x";
        case 0x450: return "STM32H74x/75x";
        case 0x451: return "STM32F76x/77x";
        case 0x460: return "STM32G07x/08x";
        case 0x470: return "STM32L4Rx/4Sx";
        default:    return NULL;
    }
}

esp_err_t swd_identify(swd_handle_t h, swd_info_t *info)
{
    memset(info, 0, sizeof(*info));
    info->core = "unknown";

    esp_err_t err = swd_connect(h, &info->dpidr);
    if (err != ESP_OK) return err;

    info->dp_version = (info->dpidr >> 12) & 0xF;
    info->designer   = (info->dpidr >> 1) & 0x7FF;
    info->dp_partno  = (info->dpidr >> 20) & 0xFF;

    /* MEM-AP IDR (best-effort; AP 0 is the MEM-AP on virtually all Cortex-M). */
    swd_ap_read(h, h->ap, SWD_AP_IDR, &info->ap_idr);

    if (swd_mem_read32(h, SCB_CPUID, &info->cpuid) == ESP_OK) {
        info->core_partno = (info->cpuid >> 4) & 0xFFF;
        info->core = core_name(info->core_partno);
    }
    if (swd_mem_read32(h, STM32_DBGMCU_IDCODE, &info->dbg_idcode) == ESP_OK)
        info->vendor = vendor_name(info->dbg_idcode);

    ESP_LOGI(TAG, "SWD: DPIDR=0x%08lx (DPv%d, designer 0x%x) core=%s CPUID=0x%08lx",
             (unsigned long)info->dpidr, info->dp_version, info->designer,
             info->core, (unsigned long)info->cpuid);
    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------- */
esp_err_t swd_init(const swd_pins_t *pins, swd_handle_t *out)
{
    if (!pins || !out) return ESP_ERR_INVALID_ARG;
    swd_handle_t h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->pins = *pins;
    h->ap = 0;
    h->dir_out = -1;

    gpio_config_t clk = {
        .pin_bit_mask = 1ULL << pins->swclk, .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&clk);
    gpio_set_level(pins->swclk, 1);

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pins->swdio, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    h->dir_out = 1;
    gpio_set_level(pins->swdio, 1);

    if (pins->srst_n != GPIO_NUM_NC) {
        gpio_config_t rst = {
            .pin_bit_mask = 1ULL << pins->srst_n, .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&rst);
        gpio_set_level(pins->srst_n, 1);
    }

    ESP_LOGI(TAG, "SWD init: SWCLK=%d SWDIO=%d", pins->swclk, pins->swdio);
    *out = h;
    return ESP_OK;
}

void swd_deinit(swd_handle_t h)
{
    if (!h) return;
    gpio_reset_pin(h->pins.swclk);
    gpio_reset_pin(h->pins.swdio);
    if (h->pins.srst_n != GPIO_NUM_NC) gpio_reset_pin(h->pins.srst_n);
    free(h);
}
