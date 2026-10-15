/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Control: the Stage 6 current-loop step run by the FOC ISR, with no motor
 * power (no DCBUS, SD1-SD4 low, TIM1 outputs only on unconnected pins).
 *
 * After pwm_adc_init(foc_ctrl_step) the ISR calls foc_ctrl_step() once per
 * PWM period. The first FOC_CTRL_CAL_SAMPLES periods measure the IA/IC
 * zero-current offsets (duties stay at 50 %); then the selected mode runs:
 *
 * - FOC_MODE_VOLTAGE: a fixed v_d at the encoder's electrical angle,
 *   through inverse Park and SVPWM. The duty pattern turns with the shaft,
 *   7 electrical turns per mechanical turn.
 * - FOC_MODE_CURRENT: the full current loop (Clarke, Park, 2 PI, inverse
 *   Park, SVPWM) with a small q reference. No current can flow, so the PIs
 *   run to their limits; that is expected.
 *
 * The mode is read from foc_ctrl_mode (a debugger can write it, since the
 * console UART has no RX wire); an unknown value runs FOC_MODE_VOLTAGE.
 * Changing the mode resets both PI integrals.
 */

#ifndef FOC_MOTOR_FOC_CTRL_H
#define FOC_MOTOR_FOC_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/atomic.h>

#include "pwm_adc.h"

/* 4096 periods at 20 kHz = 205 ms of zero-current samples. */
#define FOC_CTRL_CAL_SAMPLES 4096U

typedef enum {
    FOC_MODE_VOLTAGE = 0,
    FOC_MODE_CURRENT = 1,
} foc_mode_t;

/* Requested mode (foc_mode_t). Written by a thread or the debugger, read by the ISR. */
extern atomic_t foc_ctrl_mode;

/* Latest values of the step, written only by the ISR. */
typedef struct {
    uint32_t steps; /* control steps after calibration */
    uint32_t mode;  /* foc_mode_t that ran */
    bool calibrated;
    float ia_offset; /* ADC counts */
    float ic_offset;
    uint32_t enc_count;     /* raw TIM3 count of this step */
    uint32_t theta_e_count; /* electrical angle, 0..4095 = 0..2 pi */
    float i_d;              /* A */
    float i_q;
    float v_d; /* V */
    float v_q;
    float duty_a; /* 0..1 */
    float duty_b;
    float duty_c;
    bool limited; /* SVPWM shortened the vector */
} foc_ctrl_snapshot_t;

/* Resets the state and starts the offset calibration. Call before pwm_adc_init(). */
void foc_ctrl_init(void);

/* The pwm_adc_step_fn: FOC ISR only. */
void foc_ctrl_step(const pwm_adc_sample_t* sample, pwm_adc_duty_t* duty);

/* Consistent copy of the latest step; retries if the ISR ran during it. Thread context. */
void foc_ctrl_snapshot(foc_ctrl_snapshot_t* out);

#endif /* FOC_MOTOR_FOC_CTRL_H */
