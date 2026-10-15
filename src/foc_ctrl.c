/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_ctrl.h"

#include "analog.h"
#include "encoder.h"
#include "power_guard.h"

#include <math.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

/* 14-pole motor (docs/hardware/hardware_reference.md, motor table). */
#define POLE_PAIRS         7U

/* 2 pi / 4096: one electrical count in radians. */
#define RAD_PER_E_COUNT    0.0015339808f

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
#define CURRENT_IQ_REF_A   1.0f
#define CURRENT_KP         1.0f    /* V/A */
#define CURRENT_KI         1000.0f /* V/(A*s) */

#define DT_S               50e-6f

/* FOC_MODE_DC ramp: 1 V/s = 50 uV per period. */
#define DC_RAMP_V_PER_STEP 50e-6f

/* Low-pass for the telemetry averages: x += (y - x) / 1024, tau = 1024 periods = 51 ms. */
#define LPF_K              (1.0f / 1024.0f)

BUILD_ASSERT(ENCODER_COUNTS_PER_REV == 4096, "RAD_PER_E_COUNT and the angle math assume 4096 counts per turn");
BUILD_ASSERT(FOC_CTRL_V_MAX_MV <= (uint32_t)(POWER_GUARD_V_MAX * 1000.0f), "commanded vector above the guard cap");
BUILD_ASSERT(FOC_CTRL_CAPTURE_PRE + FOC_CTRL_CAPTURE_ON < FOC_CTRL_CAPTURE_LEN, "capture needs a decay part");

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
    float dc_v;          /* FOC_MODE_DC ramp state, V */
    uint32_t cap_idx;    /* next capture sample */
    uint32_t step_seen;  /* last cmd.step_seq acted on */
    float step_v;        /* latched capture amplitude, V */
    uint32_t step_theta; /* latched capture angle, electrical counts */
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

static void pi_reset(void) {
    st.cc.pi_d.kp = CURRENT_KP;
    st.cc.pi_d.ki = CURRENT_KI;
    st.cc.pi_d.out_min = 0.0f;
    st.cc.pi_d.out_max = 0.0f;
    st.cc.pi_q.kp = CURRENT_KP;
    st.cc.pi_q.ki = CURRENT_KI;
    st.cc.pi_q.out_min = 0.0f;
    st.cc.pi_q.out_max = 0.0f;
    foc_pi_reset(&st.cc.pi_d);
    foc_pi_reset(&st.cc.pi_q);
}

void foc_ctrl_init(void) {
    st = (ctrl_state_t){
        .ia_min = UINT16_MAX, .ic_min = UINT16_MAX, .mode = FOC_MODE_IDLE, .step_seen = foc_ctrl_cmd.step_seq};
    pi_reset();
    power_guard_reset(&st.guard);
    snap = (foc_ctrl_snapshot_t){.duty_a = 0.5f, .duty_b = 0.5f, .duty_c = 0.5f};
    (void)atomic_set(&cap_state, CAPTURE_IDLE);
}

/* Electrical angle in counts: mechanical position times the pole pairs, modulo one turn. */
static uint32_t theta_e_count(uint32_t enc_count) {
    return ((enc_count % ENCODER_COUNTS_PER_REV) * POLE_PAIRS) % ENCODER_COUNTS_PER_REV;
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
    bool known = m <= (uint32_t)FOC_MODE_CURRENT;
    bool stage6 = m == (uint32_t)FOC_MODE_VOLTAGE || m == (uint32_t)FOC_MODE_CURRENT;

    *refused = !known || (power && stage6);
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

static void calibrate(const pwm_adc_sample_t* sample) {
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
    foc_mode_t mode = calibrated ? mode_allowed(foc_ctrl_cmd.mode, powered, &refused) : FOC_MODE_IDLE;

    st.vbus_f += (vbus - st.vbus_f) * LPF_K;
    if (calibrated) {
        st.ia_f += (ia - st.ia_f) * LPF_K;
        st.ic_f += (ic - st.ic_f) * LPF_K;
        st.ib_f += (-(ia + ic) - st.ib_f) * LPF_K;
    }

    if (mode != st.mode) {
        st.mode = mode;
        st.dc_v = 0.0f;
        pi_reset();
        if (atomic_get(&cap_state) == CAPTURE_RUNNING) {
            (void)atomic_set(&cap_state, CAPTURE_IDLE);
        }
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

    switch (mode) {
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
        case FOC_MODE_VOLTAGE:
            e_count = theta_e_count(sample->enc_count);
            out.v_dq.d = VOLTAGE_MODE_VD_V;
            break;
        case FOC_MODE_CURRENT:
            e_count = theta_e_count(sample->enc_count);
            break;
        default:
            break;
    }

    float theta = (float)e_count * RAD_PER_E_COUNT;
    foc_sincos_t sc;

    sc.sin_theta = sinf(theta);
    sc.cos_theta = cosf(theta);

    if (mode == FOC_MODE_CURRENT) {
        /* Only while the power stage is off (mode_allowed), so its PI voltages need no cap. */
        foc_dq_t i_ref;

        i_ref.d = 0.0f;
        i_ref.q = CURRENT_IQ_REF_A;

        foc_current_step(&st.cc, i_abc, sc, i_ref, vbus_duty, DT_S, &out);
    } else {
        out.i_dq = foc_park(foc_clarke(i_abc), sc);
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
    if (calibrated) {
        st.steps++;
    }

    (void)atomic_inc(&snap_seq);
    snap.steps = st.steps;
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
    snap.enc_count = sample->enc_count;
    snap.theta_e_count = e_count;
    snap.vbus = vbus;
    snap.vbus_f = st.vbus_f;
    snap.ia_f = st.ia_f;
    snap.ib_f = st.ib_f;
    snap.ic_f = st.ic_f;
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
