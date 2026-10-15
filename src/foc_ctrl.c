/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_ctrl.h"

#include "analog.h"
#include "encoder.h"

#include <math.h>

#include <zephyr/sys/util.h>

#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

/* 14-pole motor (docs/hardware/hardware_reference.md, motor table). */
#define POLE_PAIRS        7U

/* 2 pi / 4096: one electrical count in radians. */
#define RAD_PER_E_COUNT   0.0015339808f

/*
 * No DCBUS in Stage 6 and ADC2 does not sample VBUS yet, so the step uses
 * this nominal bus. It sets the voltage scale of the duties only.
 */
#define VBUS_NOMINAL_V    12.0f

/* FOC_MODE_VOLTAGE: a small fixed vector on d, 1 V of the 12 V bus. */
#define VOLTAGE_MODE_VD_V 1.0f

/*
 * FOC_MODE_CURRENT placeholders: the real gains come from R and L measured
 * in Stage 7. With no current the q PI winds up to vbus / sqrt(3) in about
 * 140 steps (7 ms).
 */
#define CURRENT_IQ_REF_A  1.0f
#define CURRENT_KP        1.0f    /* V/A */
#define CURRENT_KI        1000.0f /* V/(A*s) */

#define DT_S              50e-6f

BUILD_ASSERT(ENCODER_COUNTS_PER_REV == 4096, "RAD_PER_E_COUNT and the angle math assume 4096 counts per turn");

atomic_t foc_ctrl_mode = FOC_MODE_VOLTAGE;

/* ISR-only state. */
typedef struct {
    uint32_t cal_n;
    uint32_t ia_sum;
    uint32_t ic_sum;
    float ia_offset;
    float ic_offset;
    foc_mode_t mode;
    foc_current_t cc;
    uint32_t steps;
} ctrl_state_t;

static ctrl_state_t st;

/* Single writer (the ISR); seq is odd while it writes. */
static foc_ctrl_snapshot_t snap;
static atomic_t snap_seq;

static void pi_reset(void) {
    st.cc = (foc_current_t){
        .pi_d = {.kp = CURRENT_KP, .ki = CURRENT_KI},
        .pi_q = {.kp = CURRENT_KP, .ki = CURRENT_KI},
    };
    foc_pi_reset(&st.cc.pi_d);
    foc_pi_reset(&st.cc.pi_q);
}

void foc_ctrl_init(void) {
    st = (ctrl_state_t){.mode = FOC_MODE_VOLTAGE};
    pi_reset();
    snap = (foc_ctrl_snapshot_t){.duty_a = 0.5f, .duty_b = 0.5f, .duty_c = 0.5f};
}

/* Electrical angle in counts: mechanical position times the pole pairs, modulo one turn. */
static uint32_t theta_e_count(uint32_t enc_count) {
    return ((enc_count % ENCODER_COUNTS_PER_REV) * POLE_PAIRS) % ENCODER_COUNTS_PER_REV;
}

static void calibrate(const pwm_adc_sample_t* sample, pwm_adc_duty_t* duty) {
    st.ia_sum += sample->ia_code;
    st.ic_sum += sample->ic_code;
    st.cal_n++;
    if (st.cal_n == FOC_CTRL_CAL_SAMPLES) {
        st.ia_offset = (float)st.ia_sum / (float)FOC_CTRL_CAL_SAMPLES;
        st.ic_offset = (float)st.ic_sum / (float)FOC_CTRL_CAL_SAMPLES;
    }
    *duty = (pwm_adc_duty_t){.a = 0.5f, .b = 0.5f, .c = 0.5f};
}

void foc_ctrl_step(const pwm_adc_sample_t* sample, pwm_adc_duty_t* duty) {
    if (st.cal_n < FOC_CTRL_CAL_SAMPLES) {
        calibrate(sample, duty);
        return;
    }

    foc_mode_t mode = (atomic_get(&foc_ctrl_mode) == FOC_MODE_CURRENT) ? FOC_MODE_CURRENT : FOC_MODE_VOLTAGE;

    if (mode != st.mode) {
        st.mode = mode;
        pi_reset();
    }

    /* Sign convention of the current sense is not confirmed yet (HW-ADC-010). */
    float ia = ((float)sample->ia_code - st.ia_offset) * ANALOG_PHASE_A_PER_CODE;
    float ic = ((float)sample->ic_code - st.ic_offset) * ANALOG_PHASE_A_PER_CODE;
    foc_abc_t i_abc = {.a = ia, .b = -(ia + ic), .c = ic};
    uint32_t e_count = theta_e_count(sample->enc_count);
    float theta = (float)e_count * RAD_PER_E_COUNT;
    foc_sincos_t sc = {.sin_theta = sinf(theta), .cos_theta = cosf(theta)};
    foc_current_out_t out;

    if (mode == FOC_MODE_CURRENT) {
        foc_dq_t i_ref = {.d = 0.0f, .q = CURRENT_IQ_REF_A};

        foc_current_step(&st.cc, i_abc, sc, i_ref, VBUS_NOMINAL_V, DT_S, &out);
    } else {
        out.i_dq = foc_park(foc_clarke(i_abc), sc);
        out.v_dq = (foc_dq_t){.d = VOLTAGE_MODE_VD_V, .q = 0.0f};
        out.limited = foc_svpwm(foc_inv_park(out.v_dq, sc), VBUS_NOMINAL_V, &out.duty);
    }
    *duty = (pwm_adc_duty_t){.a = out.duty.a, .b = out.duty.b, .c = out.duty.c};
    st.steps++;

    (void)atomic_inc(&snap_seq);
    snap = (foc_ctrl_snapshot_t){
        .steps = st.steps,
        .mode = (uint32_t)mode,
        .calibrated = true,
        .ia_offset = st.ia_offset,
        .ic_offset = st.ic_offset,
        .enc_count = sample->enc_count,
        .theta_e_count = e_count,
        .i_d = out.i_dq.d,
        .i_q = out.i_dq.q,
        .v_d = out.v_dq.d,
        .v_q = out.v_dq.q,
        .duty_a = out.duty.a,
        .duty_b = out.duty.b,
        .duty_c = out.duty.c,
        .limited = out.limited,
    };
    (void)atomic_inc(&snap_seq);
}

void foc_ctrl_snapshot(foc_ctrl_snapshot_t* out) {
    atomic_val_t before;

    do {
        before = atomic_get(&snap_seq);
        *out = snap;
    } while (((uint32_t)before & 1U) != 0U || atomic_get(&snap_seq) != before);
}
