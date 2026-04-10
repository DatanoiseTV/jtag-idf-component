/*
 * Abstract JTAG transport interface.
 *
 * Both the GPIO bit-bang and PARLIO DMA backends implement this interface.
 * The XMOS protocol layer calls only these functions.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "xmos_jtag.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jtag_transport jtag_transport_t;

/**
 * JTAG transport vtable.
 *
 * All shift functions operate from the Run-Test/Idle (RTI) state and
 * return to RTI when done.
 *
 * Bit ordering: LSB-first (bit 0 of data[0] is shifted first).
 */
struct jtag_transport {
    /**
     * Reset the TAP state machine to Test-Logic-Reset, then move to RTI.
     * If TRST is available, pulse it; otherwise clock TMS=1 for 5+ cycles.
     */
    esp_err_t (*reset)(jtag_transport_t *self);

    /**
     * Shift data into IR.  Navigates RTI -> Shift-IR, shifts `bits` bits,
     * then returns to RTI via Exit1-IR -> Update-IR -> RTI.
     *
     * @param tdi    Data to shift in (LSB-first), at least ceil(bits/8) bytes
     * @param tdo    Buffer for captured output, or NULL to discard
     * @param bits   Number of bits to shift
     */
    esp_err_t (*shift_ir)(jtag_transport_t *self,
                          const uint8_t *tdi, uint8_t *tdo, size_t bits);

    /**
     * Shift data through DR.  RTI -> Shift-DR -> ... -> RTI.
     *
     * @param tdi    Data to shift in (LSB-first)
     * @param tdo    Buffer for captured output, or NULL
     * @param bits   Number of bits to shift
     */
    esp_err_t (*shift_dr)(jtag_transport_t *self,
                          const uint8_t *tdi, uint8_t *tdo, size_t bits);

    /**
     * Stay in RTI for the given number of TCK cycles.
     */
    esp_err_t (*idle)(jtag_transport_t *self, unsigned cycles);

    /**
     * Free backend resources.
     */
    void (*free)(jtag_transport_t *self);
};

/* -------------------------------------------------------------------------
 * Backend constructors (defined in jtag_gpio.c / jtag_parlio.c)
 * ---------------------------------------------------------------------- */

/** Create GPIO bit-bang backend. */
esp_err_t jtag_transport_gpio_create(const xmos_jtag_pins_t *pins,
                                     jtag_transport_t **out);

#if CONFIG_XMOS_JTAG_BACKEND_PARLIO
/** Create PARLIO DMA backend (ESP32-P4). */
esp_err_t jtag_transport_parlio_create(const xmos_jtag_pins_t *pins,
                                       uint32_t tck_freq_hz,
                                       jtag_transport_t **out);
#endif

#ifdef __cplusplus
}
#endif
