/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_ctrl.h"

#include <sincos.h>
#include "analog.h"
#include "encoder.h"
#include "foc_ctrl_motion.h"
#include "power_guard.h"

#include <math.h>
#include <stddef.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

/* 14-pole motor (docs/hardware/hardware_reference.md, motor table). */
#define POLE_PAIRS         7U

/*
 * Bus used for the duty scale. The measured VBUS when it is a real bus
 * (at least VBUS_REAL_MIN_V); otherwise (no DCBUS, Stage 6 style) this
 * nominal value, so the duties still show the vector on PA8-PA10/PB9.
 */
#define VBUS_NOMINAL_V     12.0f
#define VBUS_REAL_MIN_V    10.0f

/* FOC_MODE_VOLTAGE: 1 V on d at the encoder angle. */
#define VOLTAGE_MODE_VD_V  1.0f

/* FOC_MODE_CURRENT placeholders until R and L are measured. */
#define CURRENT_WC         (6.28318530718f * (float)FOC_CTRL_CC_BW_HZ)
#define CURRENT_KP         1.0f    /* V/A */
#define CURRENT_KI         1000.0f /* V/(A*s) */

#define DT_S               50e-6f

/* FOC_MODE_DC ramp: 1 V/s = 50 uV per period. */
#define DC_RAMP_V_PER_STEP 50e-6f

/* Q16 electrical counts with exact signed sub-Q16 carry at 20 kHz. */
#define OL_PHASE_ONE       65536
#define OL_PERIODS_PER_S   20000
#define OL_PHASE_MASK      0x0FFFFFFFU

/* Low-pass for the telemetry averages: x += (y - x) / 1024, tau = 1024 periods = 51 ms. */
#define LPF_K              (1.0f / 1024.0f)

BUILD_ASSERT(ENCODER_COUNTS_PER_REV == 4096, "RAD_PER_E_COUNT and the angle math assume 4096 counts per turn");
BUILD_ASSERT(FOC_CTRL_V_MAX_MV <= (uint32_t)(POWER_GUARD_V_MAX * 1000.0f), "commanded vector above the guard cap");
BUILD_ASSERT(FOC_CTRL_CAPTURE_PRE + FOC_CTRL_CAPTURE_ON < FOC_CTRL_CAPTURE_LEN, "capture needs a decay part");

BUILD_ASSERT(offsetof(foc_ctrl_cmd_t, arm) == 0U && offsetof(foc_ctrl_cmd_t, mode) == 4U
                 && offsetof(foc_ctrl_cmd_t, v_mv) == 8U && offsetof(foc_ctrl_cmd_t, theta_e) == 12U
                 && offsetof(foc_ctrl_cmd_t, step_seq) == 16U && offsetof(foc_ctrl_cmd_t, rate_e) == 20U
                 && offsetof(foc_ctrl_cmd_t, id_ma) == 24U && offsetof(foc_ctrl_cmd_t, iq_ma) == 28U
                 && offsetof(foc_ctrl_cmd_t, dtc) == 32U && offsetof(foc_ctrl_cmd_t, spd_rpm) == 36U
                 && offsetof(foc_ctrl_cmd_t, spd_step) == 40U && offsetof(foc_ctrl_cmd_t, cff) == 44U,
             "debugger command layout changed");

volatile foc_ctrl_cmd_t foc_ctrl_cmd;

typedef enum {
    CAPTURE_IDLE = 0,
    CAPTURE_RUNNING = 1,
    CAPTURE_DONE = 2, /* owned by the thread until foc_ctrl_capture_release() */
} capture_state_t;

/* ISR-only state. */
typedef struct {
    uint32_t cal_n;
    uint32_t ia_sum;
    uint32_t ic_sum;
    uint16_t ia_min;
    uint16_t ia_max;
    uint16_t ic_min;
    uint16_t ic_max;
    bool cal_failed;
    float ia_offset;
    float ic_offset;
    foc_mode_t mode;
    foc_current_t cc;
    power_guard_t guard;
    motor_ident_t ident;
    bool ident_published;
    bool calib_valid;
    int32_t calib_direction;
    uint32_t calib_offset_e;
    float calib_r;
    float calib_l;
    float calib_v0;
    float calib_v_gc;
    bool cff;
    bool cff_refused;
    bool dtc;
    bool dtc_refused;
    float calib_kp;
    float calib_ki;
    int32_t id_ref_ma;
    int32_t iq_ref_ma;
    bool current_powered;
    foc_ctrl_speed_t speed;
    foc_speed_loop_t speed_loop;
    int32_t spd_target;
    float rpm;
    uint32_t ol_phase;    /* Q16 counts, modulo one electrical turn */
    int32_t ol_remainder; /* signed fractional Q16 carry, denominator OL_PERIODS_PER_S */
    float dc_v;           /* DC/open-loop ramp state, V */
    uint32_t cap_idx;     /* next capture sample */
    uint32_t step_seen;   /* last cmd.step_seq acted on */
    float step_v;         /* latched capture amplitude, V */
    uint32_t step_theta;  /* latched capture angle, electrical counts */
    float vbus_f;
    float ia_f;
    float ib_f;
    float ic_f;
    uint32_t steps;
} ctrl_state_t;

static ctrl_state_t st;

/* Single writer (the ISR); seq is odd while it writes. */
static foc_ctrl_snapshot_t snap;
static atomic_t snap_seq;

static foc_ctrl_capture_t cap;
static atomic_t cap_state;

/* The ISR writes only while ready is zero; the report thread releases ownership. */
static foc_ctrl_ident_t ident_result;
static atomic_t ident_ready;
static cog_map_t map;
static int16_t ff_table[FOC_CTRL_MAP_BINS];
/* 0: writable by terminal-map owner, 1: immutable/valid, 2: invalidated.
 * CAS 0->1 prevents a thread interrupted by a new IDENT from publishing a
 * stale map. New MAP starts only after raw-buffer release, then sets 0.
 */
static atomic_t ff_valid;

/** Publish a terminal result once, waiting for release of any older result. */
static void ident_publish(void) {
    if (!st.ident_published && (st.ident.result.state == IDENT_DONE || st.ident.result.state == IDENT_FAILED)
        && atomic_get(&ident_ready) == 0) {
        /* Only the ISR owns this buffer while ready is zero. The atomic_set
         * publishes every scalar store before the reader acquires ownership.
         */
        motor_ident_result_copy(&ident_result, &st.ident.result);
        st.ident_published = true;
        (void)atomic_set(&ident_ready, 1);
    }
}

/** Reset PI state and choose precomputed identified gains only for the powered loop. */
/* Mode entry and power-state changes only: flash (.text.ctrl_once). */
static void __attribute__((noinline, section(".text.ctrl_once"))) pi_reset(bool powered) {
    float kp = powered && st.calib_valid ? st.calib_kp : CURRENT_KP;
    float ki = powered && st.calib_valid ? st.calib_ki : CURRENT_KI;

    st.cc.pi_d.kp = kp;
    st.cc.pi_d.ki = ki;
    st.cc.pi_d.out_min = 0.0f;
    st.cc.pi_d.out_max = 0.0f;
    st.cc.pi_q.kp = kp;
    st.cc.pi_q.ki = ki;
    st.cc.pi_q.out_min = 0.0f;
    st.cc.pi_q.out_max = 0.0f;
    foc_pi_reset(&st.cc.pi_d);
    foc_pi_reset(&st.cc.pi_q);
}

void foc_ctrl_init(void) {
    st = (ctrl_state_t){
        .ia_min = UINT16_MAX, .ic_min = UINT16_MAX, .mode = FOC_MODE_IDLE, .step_seen = foc_ctrl_cmd.step_seq};
    pi_reset(false);
    power_guard_reset(&st.guard);
    snap = (foc_ctrl_snapshot_t){.duty_a = 0.5f, .duty_b = 0.5f, .duty_c = 0.5f};
    (void)atomic_set(&cap_state, CAPTURE_IDLE);
    (void)atomic_set(&ident_ready, 0);
    map = (cog_map_t){0};
    (void)atomic_set(&ff_valid, 2);
}

/* Electrical angle in counts: mechanical position times the pole pairs, modulo one turn. */
static uint32_t theta_e_count(uint32_t enc_count) {
    return ((enc_count % ENCODER_COUNTS_PER_REV) * POLE_PAIRS) % ENCODER_COUNTS_PER_REV;
}

/** Return the per-boot calibrated rotor angle modulo 4096, using unsigned wrap for either direction. */
static uint32_t theta_meas_count(uint32_t enc_count) {
    return foc_ctrl_rotor_angle(enc_count, st.calib_direction, st.calib_offset_e);
}

/** Return this period's angle and integrate the clamped rate exactly, carrying fractional Q16 counts. */
static uint32_t openloop_angle(void) {
    int32_t rate = foc_ctrl_cmd.rate_e;

    rate = MIN(MAX(rate, -FOC_CTRL_OL_RATE_MAX), FOC_CTRL_OL_RATE_MAX);
    int32_t numerator = rate * OL_PHASE_ONE + st.ol_remainder;
    int32_t increment = numerator / OL_PERIODS_PER_S;
    uint32_t angle = st.ol_phase >> 16U;

    st.ol_remainder = numerator % OL_PERIODS_PER_S;
    st.ol_phase = (st.ol_phase + (uint32_t)increment) & OL_PHASE_MASK;
    return angle;
}

static int16_t to_ma(float amps) {
    float ma = amps * 1000.0f;

    if (!(ma > -32767.0f)) {
        return -32767; /* also NaN */
    }
    if (ma > 32767.0f) {
        return 32767;
    }
    return (int16_t)ma;
}

/* Commanded mode, or FOC_MODE_IDLE if it is unknown or not allowed with the power stage on. */
static foc_mode_t mode_allowed(uint32_t m, bool power, bool* refused) {
    bool known = m <= (uint32_t)FOC_MODE_MAP;
    bool current_ok =
        st.calib_valid && st.calib_r > 0.02f && st.calib_r < 2.0f && st.calib_l > 10e-6f && st.calib_l < 5e-3f;
    bool forbidden = m == (uint32_t)FOC_MODE_VOLTAGE
                     || ((m == (uint32_t)FOC_MODE_CURRENT || m == (uint32_t)FOC_MODE_SPEED) && !current_ok);

    *refused = !known || (power && forbidden);
    return *refused ? FOC_MODE_IDLE : (foc_mode_t)m;
}

/* Clamps the (v_d, v_q) vector to max_v, keeping its angle. */
static void cap_vector(foc_dq_t* v, float max_v) {
    float len = sqrtf(v->d * v->d + v->q * v->q);

    if (len > max_v) {
        v->d *= max_v / len;
        v->q *= max_v / len;
    }
}

/* FOC_MODE_STEP: the vector length for this period, and the capture bookkeeping. */
static float step_capture(float ia, float ic, float vbus, bool power, float v_cmd, uint32_t theta_cmd) {
    uint32_t seq = foc_ctrl_cmd.step_seq;

    if (atomic_get(&cap_state) == CAPTURE_IDLE && seq != st.step_seen) {
        st.step_seen = seq;
        st.cap_idx = 0U;
        st.step_v = v_cmd;
        st.step_theta = theta_cmd;
        cap.seq = seq;
        cap.kind = 0U;
        cap.dtc = false;
        cap.dtc_refused = false;
        cap.cff = false;
        cap.cff_refused = false;
        cap.v_mv = (uint32_t)(st.step_v * 1000.0f + 0.5f);
        cap.theta_e = st.step_theta;
        cap.vbus = vbus;
        cap.power_on = power;
        (void)atomic_set(&cap_state, CAPTURE_RUNNING);
    }
    if (atomic_get(&cap_state) != CAPTURE_RUNNING) {
        return 0.0f;
    }

    cap.ia_ma[st.cap_idx] = to_ma(ia);
    cap.ic_ma[st.cap_idx] = to_ma(ic);
    cap.power_on = cap.power_on && power;
    st.cap_idx++;
    if (st.cap_idx == FOC_CTRL_CAPTURE_LEN) {
        (void)atomic_set(&cap_state, CAPTURE_DONE);
        return 0.0f;
    }
    /* The vector computed now applies from the next counter top, i.e. to sample cap_idx onwards. */
    return (st.cap_idx >= FOC_CTRL_CAPTURE_PRE && st.cap_idx < FOC_CTRL_CAPTURE_PRE + FOC_CTRL_CAPTURE_ON) ? st.step_v
                                                                                                           : 0.0f;
}

/** Latch a current-step request and schedule its command for counter top PRE. */
static void current_capture_prepare(float vbus, bool power) {
    uint32_t seq = foc_ctrl_cmd.step_seq;

    if (atomic_get(&cap_state) == CAPTURE_IDLE && seq != st.step_seen) {
        st.step_seen = seq;
        st.cap_idx = 0U;
        cap.seq = seq;
        cap.kind = 1U;
        cap.old_id_ma = st.id_ref_ma;
        cap.old_iq_ma = st.iq_ref_ma;
        int32_t id = foc_ctrl_cmd.id_ma;
        int32_t iq = foc_ctrl_cmd.iq_ma;

        cap.new_id_ma = MIN(MAX(id, -(int32_t)(FOC_CTRL_ID_MAX_A * 1000.0f)), (int32_t)(FOC_CTRL_ID_MAX_A * 1000.0f));
        cap.new_iq_ma = MIN(MAX(iq, -(int32_t)(FOC_CTRL_IQ_MAX_A * 1000.0f)), (int32_t)(FOC_CTRL_IQ_MAX_A * 1000.0f));
        bool requested = foc_ctrl_cmd.dtc == 1U;
        bool plausible = st.calib_valid && st.calib_v0 > 0.05f && st.calib_v0 < 0.6f;
        cap.dtc = requested && plausible;
        cap.dtc_refused = requested && !plausible;
        cap.cff = foc_ctrl_cmd.cff == 1U && atomic_get(&ff_valid) == 1;
        cap.cff_refused = foc_ctrl_cmd.cff == 1U && !cap.cff;
        cap.vbus = vbus;
        cap.power_on = power;
        (void)atomic_set(&cap_state, CAPTURE_RUNNING);
    }
    /* i[k] is already sampled. u[k] reaches the PWM at top k+1, so compute
     * the new reference at PRE-1, preserving PRE samples of the old command.
     */
    if (atomic_get(&cap_state) == CAPTURE_RUNNING && st.cap_idx + 1U == FOC_CTRL_CAPTURE_PRE) {
        st.id_ref_ma = cap.new_id_ma;
        st.iq_ref_ma = cap.new_iq_ma;
        st.dtc = cap.dtc;
        st.dtc_refused = cap.dtc_refused;
        st.cff = cap.cff;
        st.cff_refused = cap.cff_refused;
    }
}

/** Record this top's measured dq currents without copying or clearing the shared buffer. */
static void current_capture_sample(foc_dq_t measured, bool power) {
    if (atomic_get(&cap_state) != CAPTURE_RUNNING) {
        return;
    }
    cap.ia_ma[st.cap_idx] = to_ma(measured.d);
    cap.ic_ma[st.cap_idx] = to_ma(measured.q);
    cap.power_on = cap.power_on && power;
    st.cap_idx++;
    if (st.cap_idx == FOC_CTRL_CAPTURE_LEN) {
        (void)atomic_set(&cap_state, CAPTURE_DONE);
    }
}

/** Latch one command in flash only when its sequence changes and the buffer is free. */
static void __attribute__((noinline, section(".text.ctrl_once"))) speed_latch(float vbus, bool power, uint32_t seq) {
    bool step = foc_ctrl_cmd.spd_step == 1U;
    st.step_seen = seq;
    int32_t target = foc_ctrl_cmd.spd_rpm;
    target = MIN(MAX(target, -FOC_CTRL_SPD_MAX_RPM), FOC_CTRL_SPD_MAX_RPM);
    bool requested = foc_ctrl_cmd.dtc == 1U;
    bool plausible = st.calib_valid && st.calib_v0 > 0.05f && st.calib_v0 < 0.6f;
    if (step) {
        st.cap_idx = 0U;
        cap.seq = seq;
        cap.kind = 2U;
        cap.old_spd_rpm = st.spd_target;
        cap.new_spd_rpm = target;
        cap.dtc = requested && plausible;
        cap.dtc_refused = requested && !plausible;
        cap.cff = foc_ctrl_cmd.cff == 1U && atomic_get(&ff_valid) == 1;
        cap.cff_refused = foc_ctrl_cmd.cff == 1U && !cap.cff;
        cap.vbus = vbus;
        cap.power_on = power;
        (void)atomic_set(&cap_state, CAPTURE_RUNNING);
    } else {
        st.dtc = requested && plausible;
        st.dtc_refused = requested && !plausible;
        st.cff = foc_ctrl_cmd.cff == 1U && atomic_get(&ff_valid) == 1;
        st.cff_refused = foc_ctrl_cmd.cff == 1U && !st.cff;
    }
    st.spd_target = target;
}

/** Check command readiness in CCM without entering flash on ordinary periods. */
static void speed_prepare(float vbus, bool power) {
    uint32_t seq = foc_ctrl_cmd.step_seq;
    if (seq != st.step_seen && atomic_get(&cap_state) == CAPTURE_IDLE) {
        speed_latch(vbus, power, seq);
    }
}

/** Update PI and capture at 1 kHz, applying the captured step at sample PRE. */
static void speed_update(void) {
    bool capturing = atomic_get(&cap_state) == CAPTURE_RUNNING;
    bool step = capturing && st.cap_idx >= FOC_CTRL_CAPTURE_PRE;
    float target = (float)st.spd_target;

    if (capturing && !step) {
        target = st.speed_loop.ref; /* Preserve the pre-trigger reference, including an unfinished ramp. */
    }
    if (step) {
        st.dtc = cap.dtc;
        st.dtc_refused = cap.dtc_refused;
        st.cff = cap.cff;
        st.cff_refused = cap.cff_refused;
    }
    foc_speed_loop_update(&st.speed_loop, target, st.rpm, step);
    if (capturing) {
        cap.ia_ma[st.cap_idx] = to_ma(st.rpm * 0.01f); /* to_ma scales by 1000: deci-rpm. */
        cap.ic_ma[st.cap_idx] = to_ma(st.speed_loop.iq);
        st.cap_idx++;
        if (st.cap_idx == FOC_CTRL_CAPTURE_LEN) {
            (void)atomic_set(&cap_state, CAPTURE_DONE);
        }
    }
}

/* First 4096 periods after boot only, power stage off: flash (.text.ctrl_once). */
static void __attribute__((noinline, section(".text.ctrl_once"))) calibrate(const pwm_adc_sample_t* sample) {
    st.ia_sum += sample->ia_code;
    st.ic_sum += sample->ic_code;
    st.ia_min = MIN(st.ia_min, sample->ia_code);
    st.ia_max = MAX(st.ia_max, sample->ia_code);
    st.ic_min = MIN(st.ic_min, sample->ic_code);
    st.ic_max = MAX(st.ic_max, sample->ic_code);
    st.cal_n++;
    if (st.cal_n == FOC_CTRL_CAL_SAMPLES) {
        st.ia_offset = (float)st.ia_sum / (float)FOC_CTRL_CAL_SAMPLES;
        st.ic_offset = (float)st.ic_sum / (float)FOC_CTRL_CAL_SAMPLES;
        /* A stuck or railed sensor would read 0 A after offset removal and blind the overcurrent trip. */
        st.cal_failed = fabsf(st.ia_offset - (float)ANALOG_FULL_SCALE / 2.0f) > FOC_CTRL_CAL_MAX_DEV_CODES
                        || fabsf(st.ic_offset - (float)ANALOG_FULL_SCALE / 2.0f) > FOC_CTRL_CAL_MAX_DEV_CODES
                        || st.ia_max - st.ia_min < FOC_CTRL_CAL_MIN_SPREAD_CODES
                        || st.ic_max - st.ic_min < FOC_CTRL_CAL_MIN_SPREAD_CODES;
    }
}

/*
 * Mode-change bookkeeping, once per mode change. It lives in flash
 * (.text.ctrl_once, excluded from the CCM relocation in CMakeLists.txt) to keep
 * CCM for the per-period path; the one-off flash fetch costs a few hundred
 * cycles in the period of the change only.
 */
static void __attribute__((noinline, section(".text.ctrl_once")))
mode_enter(foc_mode_t mode, const pwm_adc_sample_t* sample, bool powered, bool calibrated) {
    if (st.mode == FOC_MODE_MAP) {
        cog_map_abort(&map, COG_CANCELLED);
    }
    if (st.mode == FOC_MODE_IDENT) {
        motor_ident_abort(&st.ident, IDENT_FAIL_CANCELLED);
        ident_publish();
    }
    if (mode == FOC_MODE_IDENT) {
        motor_ident_start(&st.ident);
        st.ident_published = false;
        st.calib_valid = false;
        (void)atomic_set(&ff_valid, 2);
    }
    if (mode == FOC_MODE_MAP && cog_map_get(&map) == NULL) {
        (void)atomic_set(&ff_valid, 0);
        bool plausible = calibrated && st.calib_valid && st.calib_r > 0.02f && st.calib_r < 2.0f && st.calib_l > 10e-6f
                         && st.calib_l < 5e-3f && st.calib_v0 > 0.05f && st.calib_v0 < 0.6f;
        (void)cog_map_start(&map, sample->power_on, plausible, st.calib_v_gc, st.calib_direction, sample->enc_count);
    }
    st.mode = mode;
    st.dc_v = 0.0f;
    if (mode == FOC_MODE_OPENLOOP) {
        st.ol_phase = (foc_ctrl_cmd.theta_e % ENCODER_COUNTS_PER_REV) << 16U;
        st.ol_remainder = 0;
    }
    if (mode == FOC_MODE_CURRENT || mode == FOC_MODE_SPEED || mode == FOC_MODE_MAP) {
        st.cff = false;
        st.cff_refused = false;
        st.id_ref_ma = 0;
        st.iq_ref_ma = 0;
        st.dtc = false;
        st.dtc_refused = false;
        st.current_powered = powered;
        st.step_seen = foc_ctrl_cmd.step_seq;
        foc_ctrl_speed_reset(&st.speed, sample->enc_count);
        foc_speed_loop_reset(&st.speed_loop);
        st.spd_target = 0;
        st.rpm = 0.0f;
    }
    pi_reset(powered);
    if (atomic_get(&cap_state) == CAPTURE_RUNNING) {
        (void)atomic_set(&cap_state, CAPTURE_IDLE);
    }
}

void foc_ctrl_step(const pwm_adc_sample_t* sample, pwm_adc_cmd_t* cmd) {
    bool calibrated = st.cal_n >= FOC_CTRL_CAL_SAMPLES && !st.cal_failed;

    if (st.cal_n < FOC_CTRL_CAL_SAMPLES) {
        calibrate(sample);
    }

    /* Sign convention of the current sense is not confirmed yet (HW-ADC-010). */
    float ia = ((float)sample->ia_code - st.ia_offset) * ANALOG_PHASE_A_PER_CODE;
    float ic = ((float)sample->ic_code - st.ic_offset) * ANALOG_PHASE_A_PER_CODE;
    float vbus = (float)sample->vbus_code * ANALOG_VBUS_V_PER_CODE;
    bool refused = false;
    /* Powered: on now, or switched on by the guard in the previous step (the BSP acts on it this period). */
    bool powered = sample->power_on || st.guard.request;
    foc_mode_t mode = foc_ctrl_cmd.mode == FOC_MODE_MAP ? FOC_MODE_MAP
                      : calibrated                      ? mode_allowed(foc_ctrl_cmd.mode, powered, &refused)
                                                        : FOC_MODE_IDLE;

    st.vbus_f += (vbus - st.vbus_f) * LPF_K;
    if (calibrated) {
        st.ia_f += (ia - st.ia_f) * LPF_K;
        st.ic_f += (ic - st.ic_f) * LPF_K;
        st.ib_f += (-(ia + ic) - st.ib_f) * LPF_K;
    }

    bool entered = mode != st.mode;

    if (entered) {
        mode_enter(mode, sample, powered, calibrated);
    }

    power_guard_in_t gin;

    gin.ia = ia;
    gin.ic = ic;
    gin.vbus = vbus;
    gin.calibrated = calibrated;
    gin.arm_key = foc_ctrl_cmd.arm == FOC_CTRL_ARM_KEY;
    gin.idle = mode == FOC_MODE_IDLE && !refused;
    gin.power_on = sample->power_on;
    power_guard_out_t gout = power_guard_step(&st.guard, &gin);

    uint32_t v_mv = MIN(foc_ctrl_cmd.v_mv, FOC_CTRL_V_MAX_MV);
    float v_cmd = (float)v_mv * 1e-3f;
    uint32_t theta_cmd = foc_ctrl_cmd.theta_e % ENCODER_COUNTS_PER_REV;
    uint32_t e_count = theta_cmd;
    /* The 51 ms telemetry filter lags supply changes and could make the applied vector exceed the 1.0 V cap. */
    float vbus_duty = (vbus >= VBUS_REAL_MIN_V) ? vbus : VBUS_NOMINAL_V;
    foc_abc_t i_abc;

    i_abc.a = ia;
    i_abc.b = -(ia + ic);
    i_abc.c = ic;
    /*
     * Fields set one by one: an aggregate initializer here becomes a memset
     * call, and picolibc's memset in this build is a byte loop (about 130
     * cycles for this struct, every period). Each branch below sets i_dq,
     * duty and limited.
     */
    foc_current_out_t out;

    out.v_dq.d = 0.0f;
    out.v_dq.q = 0.0f;

    bool speed_tick = false;
    switch (mode) {
        case FOC_MODE_OPENLOOP:
            e_count = openloop_angle();
            /* Fall through: both voltage modes share the 1 V/s ramp. */
            __attribute__((fallthrough));
        case FOC_MODE_DC:
            if (st.dc_v < v_cmd) {
                st.dc_v = MIN(st.dc_v + DC_RAMP_V_PER_STEP, v_cmd);
            } else {
                st.dc_v = MAX(st.dc_v - DC_RAMP_V_PER_STEP, v_cmd);
            }
            out.v_dq.d = st.dc_v;
            break;
        case FOC_MODE_STEP:
            out.v_dq.d = step_capture(ia, ic, vbus, sample->power_on, v_cmd, theta_cmd);
            if (atomic_get(&cap_state) == CAPTURE_RUNNING) {
                e_count = st.step_theta;
            }
            break;
        case FOC_MODE_IDENT:
            /* Park at the angle requested last period, before advancing the module. */
            e_count = st.ident.theta_e_count;
            break;
        case FOC_MODE_VOLTAGE:
            e_count = theta_e_count(sample->enc_count);
            out.v_dq.d = VOLTAGE_MODE_VD_V;
            break;
        case FOC_MODE_MAP:
            e_count = cog_map_current(&map) ? theta_meas_count(sample->enc_count) : cog_map_angle(&map);
            __attribute__((fallthrough));
        case FOC_MODE_SPEED:
        case FOC_MODE_CURRENT:
            if (mode != FOC_MODE_MAP) {
                e_count = powered ? theta_meas_count(sample->enc_count) : theta_e_count(sample->enc_count);
            }
            if (!entered) {
                speed_tick = foc_ctrl_speed_step(&st.speed, sample->enc_count);
                if (speed_tick) {
                    st.rpm = (float)st.speed.counts * FOC_CTRL_SPD_COUNTS_TO_RPM;
                    if (powered && st.calib_direction < 0) {
                        st.rpm = -st.rpm;
                    }
                }
            }
            break;
        default:
            break;
    }

    foc_sincos_t sc;

    bool mapping_current = false;
    if (mode == FOC_MODE_MAP && cog_map_get(&map) == NULL) {
        foc_ab_t ab = foc_clarke(i_abc);
        cog_map_in_t in;
        in.enc = sample->enc_count;
        in.alpha = ab.alpha;
        in.beta = ab.beta;
        in.rpm = st.rpm;
        in.power = sample->power_on && gout.power_on;
        in.armed = gin.arm_key;
        in.fault = gout.trip != 0U;
        cog_map_step(&map, &in);
        mapping_current = cog_map_current(&map);
        if (mapping_current) {
            /* Phase B: the map's position loop sets iq; current loop at the calibrated angle, DTC on. */
            e_count = theta_meas_count(sample->enc_count);
            st.dtc = true;
        } else {
            e_count = cog_map_angle(&map);
            out.v_dq.d = map.voltage;
        }
    }
    sincos_e_count(e_count, &sc.sin_theta, &sc.cos_theta);
    float iq_total = mode == FOC_MODE_SPEED ? st.speed_loop.iq
                     : mapping_current      ? map.iq_cmd
                                            : (float)st.iq_ref_ma * 0.001f;
    if (mode == FOC_MODE_CURRENT || mode == FOC_MODE_SPEED || mapping_current) {
        if (st.current_powered != powered) {
            pi_reset(powered);
            st.current_powered = powered;
        }
        if (mode == FOC_MODE_SPEED) {
            speed_prepare(vbus, sample->power_on);
            if (atomic_get(&cap_state) == CAPTURE_RUNNING) {
                cap.power_on = cap.power_on && sample->power_on && gout.power_on;
            }
            if (speed_tick) {
                speed_update();
            }
        } else if (mode == FOC_MODE_CURRENT) {
            current_capture_prepare(vbus, sample->power_on);
        }
        foc_dq_t i_ref;

        i_ref.d = (float)st.id_ref_ma * 0.001f;
        /* Evaluated after speed_update()/current_capture_prepare(), which update these references. */
        i_ref.q = mode == FOC_MODE_SPEED ? st.speed_loop.iq
                  : mapping_current      ? map.iq_cmd
                                         : (float)st.iq_ref_ma * 0.001f;
        /* Phase B uses the map's own iq limit (above the mode 4/7 limit, see cog_map.h). */
        i_ref.q = cog_ff_apply(i_ref.q, mapping_current ? COG_POS_I_MAX : FOC_CTRL_IQ_MAX_A, st.cff && !mapping_current,
                               atomic_get(&ff_valid) == 1, ff_table, sample->enc_count);
        iq_total = i_ref.q;
        if (!sample->power_on || !gout.power_on) {
            foc_pi_reset(&st.cc.pi_d);
            foc_pi_reset(&st.cc.pi_q);
        }
        if (powered && st.calib_valid && st.dtc) {
            /* Reference signs avoid the measured-current noise (SD about 11 mA).
             * The 50 mA linear band is above that noise and below the bench's
             * 0.3/0.5 A steps. At A=+I, B=C=-I/2, Clarke of the phase loss is
             * (2*Vdt + Vdt + Vdt)/3 = 4/3*Vdt, so Vdt = 0.75*identified V0.
             */
            foc_abc_t ref_abc = foc_inv_clarke(foc_inv_park(i_ref, sc));
            foc_ab_t comp = foc_deadtime_comp(ref_abc, 0.75f * st.calib_v0, FOC_CTRL_DTC_BAND_A);
            foc_current_step_ff(&st.cc, i_abc, sc, i_ref, vbus_duty, DT_S, comp, POWER_GUARD_V_MAX, &out);
        } else if (powered) {
            foc_current_step_lim(&st.cc, i_abc, sc, i_ref, vbus_duty, DT_S, POWER_GUARD_V_MAX, &out);
        } else {
            foc_current_step(&st.cc, i_abc, sc, i_ref, vbus_duty, DT_S, &out);
        }
        if (mode == FOC_MODE_CURRENT) {
            current_capture_sample(out.i_dq, sample->power_on);
        }
        if (!sample->power_on || !gout.power_on) {
            foc_pi_reset(&st.cc.pi_d);
            foc_pi_reset(&st.cc.pi_q);
        }
    } else {
        out.i_dq = foc_park(foc_clarke(i_abc), sc);
        if (mode == FOC_MODE_IDENT) {
            motor_ident_in_t ident_in;
            motor_ident_out_t ident_out;

            ident_in.i_d = out.i_dq.d;
            ident_in.enc_count = sample->enc_count;
            ident_in.power_on = sample->power_on && gout.power_on;

            motor_ident_step(&st.ident, &ident_in, &ident_out);
            if (st.ident.result.state == IDENT_DONE && !st.calib_valid) {
                st.calib_direction = st.ident.result.direction;
                st.calib_offset_e = st.ident.result.offset_e_new;
                st.calib_r = st.ident.result.r;
                st.calib_l = st.ident.result.l;
                st.calib_v0 = st.ident.result.v0;
                st.calib_v_gc = st.ident.result.points[IDENT_R_BIAS_POINT].v;
                st.calib_kp = st.calib_l * CURRENT_WC;
                st.calib_ki = st.calib_r * CURRENT_WC;
                st.calib_valid = true;
            }
            out.v_dq.d = ident_out.v_d;
            if (e_count != ident_out.theta_e_count) {
                e_count = ident_out.theta_e_count;
                sincos_e_count(e_count, &sc.sin_theta, &sc.cos_theta);
            }
        }
        if (powered || gout.power_on) {
            cap_vector(&out.v_dq, POWER_GUARD_V_MAX);
        }
        out.limited = foc_svpwm(foc_inv_park(out.v_dq, sc), vbus_duty, &out.duty);
    }

    cmd->a = out.duty.a;
    cmd->b = out.duty.b;
    cmd->c = out.duty.c;
    cmd->power_on = gout.power_on;
    cmd->trip = ((gout.trip & POWER_GUARD_TRIP_OVERCURRENT) != 0U ? PWM_ADC_FAULT_OVERCURRENT : 0U)
                | ((gout.trip & POWER_GUARD_TRIP_VBUS) != 0U ? PWM_ADC_FAULT_VBUS : 0U);
    if (powered && (mode == FOC_MODE_CURRENT || mode == FOC_MODE_SPEED || mapping_current)
        && (st.speed.trip_counts > FOC_CTRL_OVERSPEED_COUNTS || st.speed.trip_counts < -FOC_CTRL_OVERSPEED_COUNTS)) {
        cmd->trip |= PWM_ADC_FAULT_OVERSPEED;
    }
    if (calibrated) {
        st.steps++;
    }

    ident_publish();
    (void)atomic_inc(&snap_seq);
    snap.steps = st.steps;
    snap.ident_phase = st.ident.result.state;
    snap.mode = (uint32_t)mode;
    snap.calibrated = calibrated;
    snap.cal_failed = st.cal_failed;
    snap.power_on = sample->power_on;
    snap.power_req = gout.power_on;
    snap.mode_refused = refused;
    snap.ia_offset = st.ia_offset;
    snap.ic_offset = st.ic_offset;
    snap.ia_spread = st.ia_max - st.ia_min;
    snap.ic_spread = st.ic_max - st.ic_min;
    snap.cff = st.cff && atomic_get(&ff_valid) == 1;
    snap.cff_refused = st.cff_refused;
    snap.ff_valid = atomic_get(&ff_valid) == 1;
    snap.map_phase = map.raw.phase;
    snap.map_reason = map.raw.reason;
    snap.dtc = st.dtc;
    snap.dtc_refused = st.dtc_refused;
    snap.calib_valid = st.calib_valid;
    snap.calib_direction = st.calib_direction;
    snap.calib_offset_e = st.calib_offset_e;
    snap.theta_meas_count = st.calib_valid ? theta_meas_count(sample->enc_count) : 0U;
    snap.enc_count = sample->enc_count;
    snap.theta_e_count = e_count;
    snap.vbus = vbus;
    snap.vbus_f = st.vbus_f;
    snap.ia_f = st.ia_f;
    snap.ib_f = st.ib_f;
    snap.ic_f = st.ic_f;
    snap.i_d_ref = (float)st.id_ref_ma * 0.001f;
    snap.i_q_ref = iq_total;
    snap.cc_kp = st.cc.pi_d.kp;
    snap.cc_ki = st.cc.pi_d.ki;
    snap.speed_counts = (mode == FOC_MODE_CURRENT || mode == FOC_MODE_SPEED || mapping_current) ? st.speed.counts : 0;
    snap.speed_rpm = st.rpm;
    snap.spd_ref_rpm = st.speed_loop.ref;
    snap.spd_target_rpm = st.spd_target;
    snap.i_d = out.i_dq.d;
    snap.i_q = out.i_dq.q;
    snap.v_d = out.v_dq.d;
    snap.v_q = out.v_dq.q;
    snap.duty_a = out.duty.a;
    snap.duty_b = out.duty.b;
    snap.duty_c = out.duty.c;
    snap.limited = out.limited;
    (void)atomic_inc(&snap_seq);
}

void foc_ctrl_snapshot(foc_ctrl_snapshot_t* out) {
    atomic_val_t before;

    do {
        before = atomic_get(&snap_seq);
        *out = snap;
    } while (((uint32_t)before & 1U) != 0U || atomic_get(&snap_seq) != before);
}

const foc_ctrl_capture_t* foc_ctrl_capture_get(void) {
    return (atomic_get(&cap_state) == CAPTURE_DONE) ? &cap : NULL;
}

void foc_ctrl_capture_release(void) {
    (void)atomic_cas(&cap_state, CAPTURE_DONE, CAPTURE_IDLE);
}

/** Acquire the immutable ISR result after its atomic publication. */
const foc_ctrl_ident_t* foc_ctrl_ident_get(void) {
    return atomic_get(&ident_ready) != 0 ? &ident_result : NULL;
}

/** Release result ownership so a later run can publish its measurements. */
void foc_ctrl_ident_release(void) {
    (void)atomic_cas(&ident_ready, 1, 0);
}

/** Acquire the sequencer's immutable terminal buffer in report-thread context. */
const cog_raw_t* foc_ctrl_map_get(void) {
    return cog_map_get(&map);
}

/** Copy before atomic publication; invalidation wins over an interrupted publisher. */
void foc_ctrl_map_release(const int16_t* table) {
    if (cog_map_get(&map) == NULL) {
        return;
    }
    if (table != NULL && atomic_get(&ff_valid) == 0 && map.raw.phase == COG_DONE) {
        for (uint32_t b = 0U; b < FOC_CTRL_MAP_BINS; b++) {
            ff_table[b] = table[b];
        }
        (void)atomic_cas(&ff_valid, 0, 1);
    }
    cog_map_release(&map);
}

/** Publish a map abort when the BSP stops calling the control step after a fault. */
void foc_ctrl_fault(void) {
    cog_map_abort(&map, COG_FAULT);
}
