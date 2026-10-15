/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: the board's ADC count-to-unit conversions for the phase currents,
 * bus voltage and NTC (docs/hardware/hardware_reference.md section 4 and
 * schematic R36/R38). The samples come from src/pwm_adc.c.
 *
 * All conversions are integer and take VDDA from the ADC2 devicetree node
 * (vref-mv, default 3300).
 */

#ifndef FOC_MOTOR_ANALOG_H
#define FOC_MOTOR_ANALOG_H

#include <stdint.h>

/* 12-bit full-scale code (RM0440: V = code * VREF+ / 4095). */
#define ANALOG_FULL_SCALE 4095

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
