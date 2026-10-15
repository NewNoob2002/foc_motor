/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: ADC2 readings of the phase currents, bus voltage and NTC, plus the
 * board's count-to-unit conversions (docs/hardware/hardware_reference.md
 * section 4 and schematic R36/R38).
 *
 * Stage 3 uses the Zephyr ADC API from a thread, one blocking conversion at
 * a time. It must not be called from the FOC ISR; Stage 4 moves IA/IC to a
 * TIM1-triggered sequence.
 *
 * All conversions are integer and take VDDA from the devicetree (vref-mv).
 */

#ifndef FOC_MOTOR_ANALOG_H
#define FOC_MOTOR_ANALOG_H

#include <stdint.h>

/* 12-bit full-scale code (RM0440: V = code * VREF+ / 4095). */
#define ANALOG_FULL_SCALE 4095

typedef enum {
    ANALOG_CH_IA,
    ANALOG_CH_IC,
    ANALOG_CH_VBUS,
    ANALOG_CH_TEMP,
    ANALOG_CH_COUNT,
} analog_channel_t;

/* Sets up the four ADC2 channels. Returns 0 or -errno. */
int analog_init(void);

/* One blocking conversion. Returns 0 or -errno; *code is 0..ANALOG_FULL_SCALE. */
int analog_read(analog_channel_t ch, uint16_t* code);

const char* analog_name(analog_channel_t ch);

/* Pin voltage in mV for a code given in hundredths of a count. */
int32_t analog_code_x100_to_mv(int32_t code_x100);

/*
 * Phase current in mA: I = (V - V_offset) / (5 mOhm * 10) = 20 A/V
 * (HW-ADC-001/002). The offset is the measured zero-current code, both in
 * hundredths of a count. The sign convention is not confirmed (HW-ADC-010).
 */
int32_t analog_phase_current_ma(int32_t code_x100, int32_t offset_x100);

/* DC bus in mV: V_BUS pin * (100k + 4.7k) / 4.7k (HW-ADC-006). */
int32_t analog_vbus_mv(int32_t code_x100);

/*
 * NTC resistance in ohms: 3.3 V -> R38 10k -> TEMP -> NTC -> GND, so
 * R_ntc = 10k * code / (4095 - code), independent of VDDA. Returns -1 at
 * full scale (NTC open). The NTC nominal value and B constant are unknown.
 */
int32_t analog_ntc_ohm(int32_t code_x100);

#endif /* FOC_MOTOR_ANALOG_H */
