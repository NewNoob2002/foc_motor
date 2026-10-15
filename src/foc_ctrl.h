/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Control: the step run by the FOC ISR once per PWM period
 * (pwm_adc_init(foc_ctrl_step)).
 *
 * The first FOC_CTRL_CAL_SAMPLES periods measure the IA/IC zero-current
 * offsets (zero vector). Then the mode in foc_ctrl_cmd runs:
 *
 * - FOC_MODE_IDLE: zero vector (all duties 50 %). The default.
 * - FOC_MODE_DC: a vector of cmd.v_mv at the fixed electrical angle
 *   cmd.theta_e, ramped at 1 V/s. With the power stage on this drives a DC
 *   current (phase resistance) and pulls the rotor to that angle
 *   (alignment).
 * - FOC_MODE_STEP: each change of cmd.step_seq starts one capture of
 *   FOC_CTRL_CAPTURE_LEN periods: FOC_CTRL_CAPTURE_PRE periods of zero
 *   vector, FOC_CTRL_CAPTURE_ON periods of cmd.v_mv at cmd.theta_e (no
 *   ramp), then zero again, recording IA and IC every period (inductance
 *   from the current rise).
 * - FOC_MODE_VOLTAGE, FOC_MODE_CURRENT: the Stage 6 modes (1 V at the
 *   encoder angle; the current loop). Only while the power stage is off;
 *   with it on they run as FOC_MODE_IDLE (snapshot.mode_refused).
 *
 * The power stage (Stage 7) switches on only through src/power_guard.c:
 * cmd.arm = FOC_CTRL_ARM_KEY, offsets calibrated, VBUS 14.0-16.5 V for
 * 100 ms, mode FOC_MODE_IDLE. cmd.arm = 0 switches it off. While it is on,
 * the vector is at most POWER_GUARD_V_MAX, |I| above 2 A or VBUS outside
 * 12-17 V trips (latched until reset).
 *
 * foc_ctrl_cmd is the command block for the debugger (the console UART
 * has no RX wire); the ISR reads it every period.
 */

#ifndef FOC_MOTOR_FOC_CTRL_H
#define FOC_MOTOR_FOC_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "pwm_adc.h"

/* 4096 periods at 20 kHz = 205 ms of zero-current samples. */
#define FOC_CTRL_CAL_SAMPLES 4096U

/* "ARM!" in ASCII. Any other value of cmd.arm keeps or switches the power stage off. */
#define FOC_CTRL_ARM_KEY     0x41524D21U

/* Largest cmd.v_mv used; larger values are clamped. */
#define FOC_CTRL_V_MAX_MV    1000U

#define FOC_CTRL_CAPTURE_LEN 256U /* 12.8 ms */
#define FOC_CTRL_CAPTURE_PRE 16U
#define FOC_CTRL_CAPTURE_ON  128U /* 6.4 ms */

typedef enum {
    FOC_MODE_IDLE = 0,
    FOC_MODE_DC = 1,
    FOC_MODE_STEP = 2,
    FOC_MODE_VOLTAGE = 3,
    FOC_MODE_CURRENT = 4,
} foc_mode_t;

typedef struct {
    uint32_t arm;      /* FOC_CTRL_ARM_KEY to switch the power stage on */
    uint32_t mode;     /* foc_mode_t; unknown values run FOC_MODE_IDLE */
    uint32_t v_mv;     /* DC and step vector length, mV */
    uint32_t theta_e;  /* DC and step vector angle, electrical counts, 0..4095 = 0..2 pi */
    uint32_t step_seq; /* change it to start one capture in FOC_MODE_STEP */
} foc_ctrl_cmd_t;

extern volatile foc_ctrl_cmd_t foc_ctrl_cmd;

/* Latest values of the step, written only by the ISR. */
typedef struct {
    uint32_t steps; /* control steps after calibration */
    uint32_t mode;  /* foc_mode_t that ran */
    bool calibrated;
    bool power_on;     /* power stage on during this step (BSP state) */
    bool power_req;    /* power_guard request */
    bool mode_refused; /* the commanded mode is not allowed now; FOC_MODE_IDLE ran */
    float ia_offset;   /* ADC counts */
    float ic_offset;
    uint32_t enc_count;     /* raw TIM3 count of this step */
    uint32_t theta_e_count; /* electrical angle used, 0..4095 */
    float vbus;             /* V */
    float vbus_f;           /* V, low-pass (tau 51 ms) */
    float ia_f;             /* A, low-pass (tau 51 ms) */
    float ib_f;
    float ic_f;
    float i_d; /* A, this step */
    float i_q;
    float v_d; /* V, applied from the next counter top */
    float v_q;
    float duty_a; /* 0..1 */
    float duty_b;
    float duty_c;
    bool limited; /* SVPWM shortened the vector */
} foc_ctrl_snapshot_t;

/* One step capture; sample k is the current measured at the counter top of period k. */
typedef struct {
    uint32_t seq;     /* cmd.step_seq that started it */
    uint32_t v_mv;    /* step size */
    uint32_t theta_e; /* step angle, electrical counts */
    float vbus;       /* V, at the start */
    bool power_on;    /* power stage on during the whole capture */
    int16_t ia_ma[FOC_CTRL_CAPTURE_LEN];
    int16_t ic_ma[FOC_CTRL_CAPTURE_LEN];
} foc_ctrl_capture_t;

/* Resets the state and starts the offset calibration. Call before pwm_adc_init(). */
void foc_ctrl_init(void);

/* The pwm_adc_step_fn: FOC ISR only. */
void foc_ctrl_step(const pwm_adc_sample_t* sample, pwm_adc_cmd_t* cmd);

/* Consistent copy of the latest step; retries if the ISR ran during it. Thread context. */
void foc_ctrl_snapshot(foc_ctrl_snapshot_t* out);

/*
 * The finished capture, or NULL if none is waiting. The ISR does not touch
 * it, and starts no new capture, until foc_ctrl_capture_release(). Thread
 * context.
 */
const foc_ctrl_capture_t* foc_ctrl_capture_get(void);

void foc_ctrl_capture_release(void);

#endif /* FOC_MOTOR_FOC_CTRL_H */
