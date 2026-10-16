/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Control: the step run by the FOC ISR once per PWM period
 * (pwm_adc_init(foc_ctrl_step)).
 *
 * The first FOC_CTRL_CAL_SAMPLES periods measure the IA/IC zero-current
 * offsets (zero vector). Both means must be within FOC_CTRL_CAL_MAX_DEV_CODES
 * of midscale and both spreads at least FOC_CTRL_CAL_MIN_SPREAD_CODES.
 * Rejection latches cal_failed until reset: no arming or current trips,
 * and the zero vector keeps running. Then, if accepted, foc_ctrl_cmd runs:
 *
 * - FOC_MODE_IDLE: zero vector (all duties 50 %). The default.
 * - FOC_MODE_DC: a vector of cmd.v_mv at the fixed electrical angle
 *   cmd.theta_e, ramped at 1 V/s. With the power stage on this drives a DC
 *   current (phase resistance) and pulls the rotor to that angle
 *   (alignment).
 * - FOC_MODE_OPENLOOP: a d-axis vector ramped as in DC, starting at cmd.theta_e
 *   on entry and rotating at signed cmd.rate_e electrical counts/s (clamped
 *   to +/-FOC_CTRL_OL_RATE_MAX). Allowed with the power stage on or off.
 *   The per-boot measured rotor angle is diagnostic only; no angle feedback.
 * - FOC_MODE_STEP: each change of cmd.step_seq starts one capture of
 *   FOC_CTRL_CAPTURE_LEN periods: FOC_CTRL_CAPTURE_PRE periods of zero
 *   vector, FOC_CTRL_CAPTURE_ON periods of cmd.v_mv at cmd.theta_e (no
 *   ramp), then zero again, recording IA and IC every period (inductance
 *   from the current rise). Amplitude and angle are latched at capture
 *   start; later command edits do not change the running capture.
 * - FOC_MODE_IDENT: automatic encoder alignment, R and L measurement; requires
 *   power already on. Ignores v_mv/theta_e, aborts on disarm, and publishes
 *   one terminal result through foc_ctrl_ident_get()/release().
 * - FOC_MODE_VOLTAGE: Stage 6 1 V at the encoder angle, refused while powered.
 * - FOC_MODE_CURRENT: powered dq loop at the calibrated rotor angle, permitted
 *   only after successful per-boot identification with 0.02<R<2 ohm and
 *   10 uH<L<5 mH. Gains are L*wc and R*wc, wc=2*pi*1000 Hz, with a 1 V
 *   applied-vector cap. Optional dtc=1 reserves PI headroom for the identified
 *   per-phase voltage loss; the same 1 V cap covers PI plus compensation.
 *   DTC requires 0.05<V0<0.6 V and is off by default. Unpowered checks use raw
 *   encoder angle and placeholder gains. Entry resets both references and integrals
 *   and takes step_seq as its baseline. Write id_ma/iq_ma/dtc then change step_seq
 *   to latch clamped references and the accepted DTC state;
 *   a free capture buffer records dq currents, old/new refs and PRE old-reference
 *   samples before the new command applies at top PRE. Busy captures defer the
 *   next sequence until released. Speed above 205 counts/10 ms trips when powered.
 *   Integrals reset while power is off.
 * - FOC_MODE_SPEED: same permission, calibrated angle, DTC and 1 V current loop
 *   as CURRENT, with id=0 and iq from a 1 kHz speed PI limited to +/-0.5 A.
 *   Entry clears PI, reference and estimator. Write spd_rpm/dtc/spd_step before
 *   changing step_seq. Ordinary commands ramp at 400 rpm/s; spd_step=1 requests
 *   a 256 ms kind-2 capture and applies the latched reference as a step at
 *   sample PRE. Busy captures defer sequences until release. Speed and iq_ref
 *   are recorded at the same 1 kHz update; iq reaches PWM one top (50 us) later.
 *   Speed feedback follows the calibrated mechanical direction while powered;
 *   unpowered checks use raw encoder direction. The rolling 10 ms window has
 *   about 5 ms delay and 1.46484375 rpm resolution. Overspeed is checked at
 *   the existing 10 ms cadence in both current-loop modes.
 *
 * The power stage (Stage 7) switches on only through src/power_guard.c:
 * cmd.arm = FOC_CTRL_ARM_KEY, offsets calibrated, VBUS 14.0-16.5 V for
 * 100 ms, explicitly allowed FOC_MODE_IDLE for 20 consecutive periods
 * (1 ms, so the previous duty clears CCR preload). Unknown or refused
 * modes run the zero vector but cannot arm. cmd.arm = 0 switches it off. While it is on,
 * the vector is at most POWER_GUARD_V_MAX, |I| above 2 A or VBUS outside
 * 12-17 V trips (latched until reset).
 *
 * foc_ctrl_cmd is the command block for the debugger (the console UART
 * has no RX wire); the ISR reads its control fields every period. The test-only
 * crash field is consumed only by the report thread.
 */

#ifndef FOC_MOTOR_FOC_CTRL_H
#define FOC_MOTOR_FOC_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "cog_map.h"
#include "foc_speed.h"
#include "motor_ident.h"
#include "pwm_adc.h"

/* 4096 periods at 20 kHz = 205 ms of zero-current samples. */
#define FOC_CTRL_CAL_SAMPLES          4096U
#define FOC_CTRL_CAL_MAX_DEV_CODES    200
#define FOC_CTRL_CAL_MIN_SPREAD_CODES 2

/* "ARM!" in ASCII. Any other value of cmd.arm keeps or switches the power stage off. */
#define FOC_CTRL_ARM_KEY              0x41524D21U

/* Largest cmd.v_mv used; larger values are clamped. */
#define FOC_CTRL_V_MAX_MV             1000U
/* One electrical revolution/s; 1/7 mechanical revolution/s with this motor. */
#define FOC_CTRL_OL_RATE_MAX          4096

/* Current-loop tuning and learning-bench reference/speed limits. */
#define FOC_CTRL_DTC_BAND_A           0.05f
#define FOC_CTRL_CC_BW_HZ             1000U
#define FOC_CTRL_ID_MAX_A             1.0f
#define FOC_CTRL_IQ_MAX_A             0.5f
#define FOC_CTRL_OVERSPEED_RPM        300U
#define FOC_CTRL_OVERSPEED_COUNTS     205 /* counts per 10 ms; trip strictly above */

#define FOC_CTRL_CAPTURE_LEN          256U /* 12.8 ms; kind 2: 256 ms */
#define FOC_CTRL_CAPTURE_PRE          16U
#define FOC_CTRL_CAPTURE_ON           128U /* 6.4 ms */

typedef enum {
    FOC_MODE_IDLE = 0,
    FOC_MODE_DC = 1,
    FOC_MODE_STEP = 2,
    FOC_MODE_VOLTAGE = 3,
    FOC_MODE_CURRENT = 4,
    FOC_MODE_IDENT = 5,
    FOC_MODE_OPENLOOP = 6,
    FOC_MODE_SPEED = 7,
    FOC_MODE_MAP = 8, /* Powered gain sweep, then bidirectional 10 rpm torque map. */
} foc_mode_t;

typedef struct {
    uint32_t arm;      /* FOC_CTRL_ARM_KEY to switch the power stage on */
    uint32_t mode;     /* foc_mode_t; unknown values run FOC_MODE_IDLE */
    uint32_t v_mv;     /* DC, open-loop and step vector length, mV */
    uint32_t theta_e;  /* fixed/start vector angle, electrical counts, 0..4095 = 0..2 pi */
    uint32_t step_seq; /* change it to start one capture in STEP or latch refs/capture in CURRENT */
    int32_t rate_e;    /* +20: open-loop electrical counts/s, clamped to +/-FOC_CTRL_OL_RATE_MAX */
    int32_t id_ma;     /* +24: write both references, then change step_seq to latch */
    int32_t iq_ma;     /* +28: mA, clamped to +/-FOC_CTRL_IQ_MAX_A */
    uint32_t dtc;      /* +32: 0 off, 1 on; latched with refs on step_seq change */
    int32_t spd_rpm;   /* +36: signed calibrated-frame rpm, clamped to +/-200 */
    uint32_t spd_step; /* +40: 1 = kind-2 step capture, 0 = ordinary ramp; latched with sequence */
    uint32_t cff;      /* +44: 0 off, 1 on; latched on step_seq, refused without a valid boot map */
    uint32_t crash;    /* +48: test-only; 0x43524153 ("CRAS") is cleared by the report thread, then k_oops() */
} foc_ctrl_cmd_t;

extern volatile foc_ctrl_cmd_t foc_ctrl_cmd;

/* Latest values of the step, written only by the ISR. */
typedef struct {
    uint32_t steps;                  /* control steps after calibration */
    uint32_t mode;                   /* foc_mode_t that ran */
    motor_ident_phase_t ident_phase; /* automatic identification phase */
    bool calibrated;
    bool cal_failed;   /* plausibility rejection, sticky until reset */
    bool power_on;     /* power stage on during this step (BSP state) */
    bool power_req;    /* power_guard request */
    bool mode_refused; /* the commanded mode is not allowed now; FOC_MODE_IDLE ran */
    float ia_offset;   /* ADC counts */
    float ic_offset;
    uint16_t ia_spread; /* max - min ADC codes during calibration */
    uint16_t ic_spread;
    bool cff;
    bool cff_refused;
    bool ff_valid;
    cog_phase_t map_phase;
    cog_fail_t map_reason;
    bool dtc;                  /* latched and accepted compensation request */
    bool dtc_refused;          /* latched request rejected by V0 plausibility */
    bool calib_valid;          /* successful angle identification in this boot; cleared on new IDENT */
    int32_t calib_direction;   /* +1 or -1, valid only when calib_valid */
    uint32_t calib_offset_e;   /* electrical counts, valid only when calib_valid */
    uint32_t theta_meas_count; /* calibrated rotor electrical angle, valid only when calib_valid */
    uint32_t enc_count;        /* raw TIM3 count of this step */
    uint32_t theta_e_count;    /* electrical angle used, 0..4095 */
    float vbus;                /* V */
    float vbus_f;              /* V, low-pass (tau 51 ms) */
    float ia_f;                /* A, low-pass (tau 51 ms) */
    float ib_f;
    float ic_f;
    float i_d_ref; /* A, latched reference */
    float i_q_ref;
    float cc_kp;            /* V/A */
    float cc_ki;            /* V/(A s) */
    int32_t speed_counts;   /* signed mechanical encoder counts per rolling 10 ms */
    float speed_rpm;        /* calibrated-frame rpm in powered operation */
    float spd_ref_rpm;      /* ramped reference */
    int32_t spd_target_rpm; /* sequence-latched reference */
    float i_d;              /* A, this step */
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
    uint32_t seq;  /* cmd.step_seq that started it */
    uint32_t kind; /* 0: IA/IC voltage step, 1: id/iq current step, 2: speed step at 1 kHz */
    int32_t old_spd_rpm;
    int32_t new_spd_rpm;
    int32_t old_id_ma;
    int32_t old_iq_ma;
    int32_t new_id_ma;
    int32_t new_iq_ma;
    uint32_t v_mv;    /* step size */
    uint32_t theta_e; /* step angle, electrical counts */
    float vbus;       /* V, at the start */
    bool power_on;    /* power stage on during the whole capture */
    bool cff;
    bool cff_refused;
    bool dtc;         /* accepted state for the new references at top PRE */
    bool dtc_refused; /* V0 was outside the strict (0.05, 0.6) V window */
    /* Kind 1: id/iq in mA; kind 2: speed in deci-rpm / iq reference in mA. */
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

/** Terminal identification measurements, immutable until released by the thread. */
typedef motor_ident_result_t foc_ctrl_ident_t;

/** Return a finished identification, or NULL; thread context, no copying required. */
const foc_ctrl_ident_t* foc_ctrl_ident_get(void);

/** Return the finished identification buffer to the ISR after reporting it. */
void foc_ctrl_ident_release(void);

/** Publish any active map as failed; called by the BSP fault callback in ISR context. */
void foc_ctrl_fault(void);

/** Acquire the immutable terminal map; NULL while the ISR owns the raw buffer. */
const cog_raw_t* foc_ctrl_map_get(void);
/** Publish the processed table while holding the terminal raw buffer, then release it.
 * The thread copies only while ff_valid=0. Starting a new map is deferred while
 * ready=1; publication precedes ready release, so an ISR restart cannot race a
 * table write. A new map or identification invalidates the table in the ISR.
 */
void foc_ctrl_map_release(const int16_t* table);

#endif /* FOC_MOTOR_FOC_CTRL_H */
