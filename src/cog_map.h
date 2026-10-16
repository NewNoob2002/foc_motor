/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_COG_MAP_H
#define FOC_MOTOR_COG_MAP_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#define FOC_CTRL_MAP_BINS   256U
#define COG_GAIN_POINTS     12U
#define COG_SETTLE_N        6000U
#define COG_AVERAGE_N       2000U
#define COG_TIMEOUT_N       1400000U /* 70 s: gain check about 16 s, sweep about 42 s. */
#define COG_MIN_SAMPLES     32U

/*
 * Phase B is a position-loop sweep, not a speed-loop run: on the bench the
 * 10 rpm speed loop went stick-slip (stage 8 run 1), because its iq builds
 * up too slowly against stiction and cogging. A stiff position loop drags a
 * target angle at COG_POS_RATE_CPS; the iq it needs at each measured
 * position is cogging + friction (+ a small inertial term at this speed).
 * Positions are calibrated mechanical counts (direction applied).
 */
#define COG_POS_RATE_CPS    410    /* counts/s, about 6.0 rpm */
#define COG_POS_REVS        2U     /* turns each way */
#define COG_POS_HOLD_N      10000U /* 0.5 s hold before each sweep */
#define COG_POS_SLIP_COUNTS 400.0f /* |target - position| that aborts (35 deg mechanical) */
#define COG_POS_I_MAX       0.5f   /* A, as the speed loop */
#define COG_POS_INT_MAX     0.3f   /* A, integral clamp */

/*
 * PID gains for an inertia J and torque constant Kt at bandwidth w with
 * damping zeta: kp = J*w^2/Kt, kd = 2*zeta*J*w/Kt, ki = kp*w/5. J is the
 * stage 7 assumption (not measured: a pulse test is dominated by the
 * 3-4 mNm cogging at any current that stays below the overspeed trip for
 * an unknown J). For J three times smaller the loop is faster and more
 * damped; three times larger gives w/sqrt(3) and zeta/sqrt(3) = 0.46 (host
 * simulation covers both). Units: A per count, A per rpm, A per count per
 * period.
 */
#define COG_J_EST           1.5e-5f /* kg m^2 */
#define COG_KT              0.0217f /* Nm/A */
#define COG_POS_W           (6.28318530718f * 10.0f)
#define COG_POS_ZETA        0.8f
#define COG_POS_KP          (COG_J_EST * COG_POS_W * COG_POS_W / COG_KT * (6.28318530718f / 4096.0f))
#define COG_POS_KD          (2.0f * COG_POS_ZETA * COG_J_EST * COG_POS_W / COG_KT * (6.28318530718f / 60.0f))
#define COG_POS_KI_DT       (COG_POS_KP * COG_POS_W * 0.2f * 50e-6f)

typedef enum {
    COG_IDLE,
    COG_CLEAR,
    COG_GAIN_RAMP,
    COG_GAIN_SETTLE,
    COG_GAIN_AVERAGE,
    COG_GAIN_DECAY,
    COG_FORWARD_SETTLE,
    COG_FORWARD,
    COG_REVERSE_SETTLE,
    COG_REVERSE,
    COG_DONE,
    COG_FAILED
} cog_phase_t;
typedef enum {
    COG_OK,
    COG_NOT_POWERED,
    COG_CALIBRATION,
    COG_CANCELLED,
    COG_FAULT,
    COG_NUMERIC,
    COG_SLIP,
    COG_TIMEOUT,
    COG_OVERFLOW
} cog_fail_t;
typedef struct {
    float alpha_ma;
    float beta_ma;
    uint32_t enc;
} cog_gain_t;
typedef struct {
    int32_t sum[2][FOC_CTRL_MAP_BINS]; /* mA; forward then reverse, calibrated torque sign. */
    uint16_t count[2][FOC_CTRL_MAP_BINS];
    cog_gain_t gain[COG_GAIN_POINTS];
    cog_phase_t phase;
    cog_fail_t reason;
} cog_raw_t;

typedef struct {
    uint32_t enc;
    float alpha;
    float beta;
    float rpm; /* calibrated frame, from the 10 ms estimator */
    bool power;
    bool armed;
    bool fault;
} cog_map_in_t;

/** ISR-owned sequencer; ready transfers the entire raw buffer to the thread.
 * A start is refused while ready=1. The thread must finish processing, table
 * publication and reporting before release; no raw writes occur in that time.
 */
typedef struct {
    cog_raw_t raw;
    atomic_uint ready;
    uint32_t ticks;
    uint32_t clear_bin;
    uint32_t dwell;
    uint32_t slip;
    uint32_t point;
    uint32_t previous;
    int32_t travel;
    int32_t direction;
    float voltage;
    float target_v;
    float alpha_sum;
    float beta_sum;
    int32_t pos;     /* measured, calibrated counts since the sweep start */
    int32_t tgt_q16; /* target, Q16 calibrated counts */
    int32_t tgt_rem; /* fractional carry of the target, denominator 20000 */
    float integ;     /* A, PID integral */
    float iq_cmd;    /* A, the sweep's iq reference for the next period */
} cog_map_t;

/** Begin a run only if the thread has released the preceding result. */
bool cog_map_start(cog_map_t* m, bool power, bool calibration, float voltage, int32_t direction, uint32_t enc);
/** Advance one 50 us period and publish a terminal buffer with release ordering. */
void cog_map_step(cog_map_t* m, const cog_map_in_t* in);
/** Abort an active run and publish its immutable partial result. */
void cog_map_abort(cog_map_t* m, cog_fail_t reason);
/** Acquire a terminal raw buffer, or NULL while the ISR owns it. */
const cog_raw_t* cog_map_get(const cog_map_t* m);
/** Return raw-buffer ownership to the ISR after processing and reporting. */
void cog_map_release(cog_map_t* m);
/** Return the calibrated electrical command angle for the gain check. */
uint32_t cog_map_angle(const cog_map_t* m);
/** True while phase B runs the current loop at the calibrated angle with iq = iq_cmd. */
bool cog_map_current(const cog_map_t* m);
/** Interpolate mA samples at mechanical bin centres, including the wrap. */
float cog_ff_lookup(const int16_t table[FOC_CTRL_MAP_BINS], uint32_t enc);
/** Add an accepted feed-forward request and clamp to the current reference limit. */
float cog_ff_apply(float iq, float limit, bool enabled, bool valid, const int16_t* table, uint32_t enc);

#endif
