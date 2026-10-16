/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>

#include "analog.h"
#include "foc/foc_transform.h"
#include "foc_ctrl.h"
#include "foc_test.h"
#include "power_guard.h"

/* Supply a valid zero-current sample at the requested bus voltage. */
static pwm_adc_sample_t sample_at(float vbus) {
    pwm_adc_sample_t sample = {
        .ia_code = 2048U,
        .ic_code = 2048U,
        .vbus_code = (uint16_t)(vbus / ANALOG_VBUS_V_PER_CODE + 0.5f),
    };

    return sample;
}

/* Calibrate the production controller with alternating low and high codes. */
static foc_ctrl_snapshot_t calibrate_codes(uint16_t ia_lo, uint16_t ia_hi, uint16_t ic_lo, uint16_t ic_hi) {
    foc_ctrl_cmd = (foc_ctrl_cmd_t){0};
    foc_ctrl_init();
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    for (uint32_t n = 0U; n < FOC_CTRL_CAL_SAMPLES; n++) {
        sample.ia_code = (n & 1U) != 0U ? ia_hi : ia_lo;
        sample.ic_code = (n & 1U) != 0U ? ic_hi : ic_lo;
        foc_ctrl_step(&sample, &cmd);
        CHECK(!cmd.power_on && cmd.trip == 0U);
        CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
    }
    /* Acceptance becomes usable on the first step after all calibration samples. */
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    return snap;
}

/* Accept both offset-deviation boundaries and the minimum nonzero spread. */
static void test_calibration_acceptance_boundaries(void) {
    foc_ctrl_snapshot_t snap = calibrate_codes(2246U, 2249U, 1846U, 1849U);

    CHECK(snap.calibrated && !snap.cal_failed);
    CHECK(snap.ia_offset == 2247.5f && snap.ic_offset == 1847.5f);
    CHECK(snap.ia_spread == 3U && snap.ic_spread == 3U);
    snap = calibrate_codes(2047U, 2049U, 2047U, 2049U);
    CHECK(snap.calibrated && !snap.cal_failed);
    CHECK(snap.ia_spread == FOC_CTRL_CAL_MIN_SPREAD_CODES);
    CHECK(snap.ic_spread == FOC_CTRL_CAL_MIN_SPREAD_CODES);
}

/* Reject either stuck, railed or implausible sensor permanently, without false current trips. */
static void test_calibration_rejection_is_sticky(void) {
    const uint16_t cases[][4] = {
        {2048U, 2048U, 2047U, 2049U}, /* IA stuck at midscale */
        {2047U, 2049U, 2048U, 2048U}, /* IC stuck at midscale */
        {2047U, 2048U, 2047U, 2049U}, /* IA spread below minimum */
        {2047U, 2049U, 2047U, 2048U}, /* IC spread below minimum */
        {0U, 2U, 2047U, 2049U},       /* IA near the low rail */
        {2047U, 2049U, 4093U, 4095U}, /* IC near the high rail */
        {2247U, 2250U, 2047U, 2049U}, /* IA just above maximum offset */
        {2047U, 2049U, 1845U, 1848U}, /* IC just below minimum offset */
    };

    for (size_t k = 0U; k < sizeof(cases) / sizeof(cases[0]); k++) {
        foc_ctrl_snapshot_t snap = calibrate_codes(cases[k][0], cases[k][1], cases[k][2], cases[k][3]);
        pwm_adc_sample_t sample = sample_at(15.0f);
        pwm_adc_cmd_t cmd;

        CHECK(snap.cal_failed && !snap.calibrated);
        foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
        foc_ctrl_cmd.mode = FOC_MODE_DC;
        foc_ctrl_cmd.v_mv = 1000U;
        for (uint32_t n = 0U; n < FOC_CTRL_CAL_SAMPLES + POWER_GUARD_VBUS_ARM_PERIODS; n++) {
            foc_ctrl_step(&sample, &cmd);
            CHECK(!cmd.power_on && cmd.trip == 0U);
            CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
        }
        foc_ctrl_snapshot(&snap);
        CHECK(snap.cal_failed && !snap.calibrated && snap.steps == 0U);
    }
}

/* Refused modes cannot arm, and returning to explicit idle still needs 20 steps. */
static void test_unknown_mode_cannot_arm(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
    foc_ctrl_cmd.mode = UINT32_MAX;
    for (uint32_t n = 0U; n < POWER_GUARD_VBUS_ARM_PERIODS; n++) {
        foc_ctrl_step(&sample, &cmd);
        CHECK(!cmd.power_on && cmd.trip == 0U);
        CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
    }
    foc_ctrl_snapshot(&snap);
    CHECK(snap.mode_refused && snap.mode == FOC_MODE_IDLE);
    foc_ctrl_cmd.mode = FOC_MODE_IDLE;
    for (uint32_t n = 0U; n < POWER_GUARD_IDLE_PERIODS - 1U; n++) {
        foc_ctrl_step(&sample, &cmd);
        CHECK(!cmd.power_on);
    }
    foc_ctrl_step(&sample, &cmd);
    CHECK(cmd.power_on);
}

/* Recover the applied alpha/beta vector from the returned duties and actual supply. */
static foc_ab_t applied_vector(const pwm_adc_cmd_t* cmd, float vbus) {
    foc_abc_t volts = {.a = cmd->a * vbus, .b = cmd->b * vbus, .c = cmd->c * vbus};

    return foc_clarke(volts);
}

/* A sudden supply rise must preserve the one-volt vector despite lagging telemetry. */
static void test_duty_scale_uses_raw_vbus(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(16.5f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    foc_ctrl_cmd.mode = FOC_MODE_VOLTAGE;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK(snap.vbus - snap.vbus_f > 1.0f);
    foc_ab_t vector = applied_vector(&cmd, snap.vbus);
    CHECK(fabsf(vector.alpha - 1.0f) < 1e-5f);
    CHECK(fabsf(vector.beta) < 1e-5f);

    /* The no-DCBUS learning mode retains its nominal 12 V scale. */
    sample = sample_at(0.0f);
    foc_ctrl_step(&sample, &cmd);
    vector = applied_vector(&cmd, 12.0f);
    CHECK(fabsf(vector.alpha - 1.0f) < 1e-5f);
}

/* Command edits during a capture cannot alter its amplitude, direction or reported metadata. */
static void test_step_capture_latches_vector(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;
    uint32_t on_steps = 0U;

    foc_ctrl_cmd.mode = FOC_MODE_STEP;
    foc_ctrl_cmd.v_mv = 800U;
    foc_ctrl_cmd.theta_e = 1024U;
    foc_ctrl_cmd.step_seq = 1U;
    for (uint32_t n = 0U; n < FOC_CTRL_CAPTURE_LEN; n++) {
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        if (n < FOC_CTRL_CAPTURE_LEN - 1U) {
            CHECK(snap.theta_e_count == 1024U);
        }
        if (snap.v_d > 0.0f) {
            foc_ab_t vector = applied_vector(&cmd, snap.vbus);

            on_steps++;
            CHECK(fabsf(snap.v_d - 0.8f) < 1e-6f);
            CHECK(fabsf(vector.alpha) < 1e-5f);
            CHECK(fabsf(vector.beta - 0.8f) < 1e-5f);
        }
        foc_ctrl_cmd.v_mv = 200U;
        foc_ctrl_cmd.theta_e = 0U;
    }
    CHECK(on_steps == FOC_CTRL_CAPTURE_ON);
    const foc_ctrl_capture_t* capture = foc_ctrl_capture_get();

    CHECK(capture != NULL);
    if (capture != NULL) {
        CHECK(capture->v_mv == 800U && capture->theta_e == 1024U && capture->seq == 1U);
    }
    foc_ctrl_capture_release();
}

/** Mode 5 cannot run without power and publishes its failure exactly once until restarted. */
static void test_ident_unpowered(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    foc_ctrl_cmd.mode = FOC_MODE_IDENT;
    foc_ctrl_cmd.v_mv = UINT32_MAX;
    foc_ctrl_cmd.theta_e = 123U;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    const foc_ctrl_ident_t* result = foc_ctrl_ident_get();

    CHECK(!snap.mode_refused && snap.mode == FOC_MODE_IDENT);
    CHECK(snap.ident_phase == IDENT_FAILED);
    CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
    CHECK(result != NULL);
    if (result != NULL) {
        CHECK(result->fail_reason == IDENT_FAIL_NOT_POWERED);
    }
    foc_ctrl_ident_release();
    foc_ctrl_step(&sample, &cmd);
    CHECK(foc_ctrl_ident_get() == NULL);
    foc_ctrl_cmd.mode = FOC_MODE_IDLE;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_cmd.mode = FOC_MODE_IDENT;
    foc_ctrl_step(&sample, &cmd);
    CHECK(foc_ctrl_ident_get() != NULL);
    foc_ctrl_ident_release();
}

/** Arm through explicit idle, then verify powered permission, disarm, and immutable result ownership. */
static void test_ident_powered(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
    for (uint32_t k = 0U; k < POWER_GUARD_VBUS_ARM_PERIODS; k++) {
        foc_ctrl_step(&sample, &cmd);
    }
    CHECK(cmd.power_on);
    sample.power_on = true;
    /* Stage 6 modes remain refused while powered; unknown values are still refused. */
    const uint32_t refused[] = {FOC_MODE_VOLTAGE, FOC_MODE_CURRENT, 7U, UINT32_MAX};

    for (uint32_t k = 0U; k < sizeof(refused) / sizeof(refused[0]); k++) {
        foc_ctrl_cmd.mode = refused[k];
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(snap.mode_refused && snap.mode == FOC_MODE_IDLE);
    }
    foc_ctrl_cmd.mode = FOC_MODE_IDENT;
    foc_ctrl_cmd.v_mv = 1000U;
    foc_ctrl_cmd.theta_e = 3072U;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK(!snap.mode_refused && snap.mode == FOC_MODE_IDENT);
    CHECK(snap.ident_phase == IDENT_ALIGN_FIRST);
    CHECK(snap.theta_e_count == 0U && snap.v_d > 0.0f && snap.v_d < 0.001f);
    CHECK(cmd.power_on && foc_ctrl_ident_get() == NULL);
    foc_ab_t vector = applied_vector(&cmd, snap.vbus);

    CHECK(vector.alpha > 0.0f && fabsf(vector.beta) < 1e-5f);

    /* Drive each hold/decay and compare all three fixed-angle Park tables
     * with an independent library-trig reference, including final zero.
     */
    static const uint32_t angles[] = {0U, IDENT_ALIGN_120, IDENT_ALIGN_240, 0U};
    static const motor_ident_phase_t phases[] = {IDENT_ALIGN_FIRST, IDENT_ALIGN_SECOND, IDENT_ALIGN_THIRD,
                                                 IDENT_ALIGN_ZERO};

    for (uint32_t angle = 0U; angle < sizeof(angles) / sizeof(angles[0]); angle++) {
        if (angle != 0U) {
            for (uint32_t k = 0U; k < IDENT_ALIGN_HOLD_N + 2U * IDENT_REACHED_N + 2000U; k++) {
                float theta = (float)angles[angle - 1U] * 0.0015339808f;
                float ia = snap.ident_phase == IDENT_ALIGN_DECAY ? 0.0f : IDENT_ALIGN_A * cosf(theta);
                float ic = snap.ident_phase == IDENT_ALIGN_DECAY
                               ? 0.0f
                               : IDENT_ALIGN_A * (-0.5f * cosf(theta) - 0.8660254f * sinf(theta));

                sample.ia_code = (uint16_t)lroundf(2048.0f + ia / ANALOG_PHASE_A_PER_CODE);
                sample.ic_code = (uint16_t)lroundf(2048.0f + ic / ANALOG_PHASE_A_PER_CODE);
                foc_ctrl_step(&sample, &cmd);
                foc_ctrl_snapshot(&snap);
                if (snap.ident_phase == phases[angle]) {
                    CHECK(snap.v_d == 0.0f);
                    break;
                }
            }
            CHECK(snap.ident_phase == phases[angle]);
        }
        sample.ia_code = 2079U;
        sample.ic_code = 2004U;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(snap.theta_e_count == angles[angle]);
        float theta = (float)snap.theta_e_count * 0.0015339808f;
        foc_sincos_t sc = {.sin_theta = sinf(theta), .cos_theta = cosf(theta)};
        foc_abc_t current = {.a = 31.0f * ANALOG_PHASE_A_PER_CODE,
                             .b = 13.0f * ANALOG_PHASE_A_PER_CODE,
                             .c = -44.0f * ANALOG_PHASE_A_PER_CODE};
        foc_dq_t expected = foc_park(foc_clarke(current), sc);

        CHECK_NEAR(snap.i_d, expected.d, 1e-7);
        CHECK_NEAR(snap.i_q, expected.q, 1e-7);
        vector = applied_vector(&cmd, snap.vbus);
        CHECK_NEAR(vector.alpha, snap.v_d * sc.cos_theta, 1e-6);
        CHECK_NEAR(vector.beta, snap.v_d * sc.sin_theta, 1e-6);
    }
    foc_ctrl_cmd.arm = 0U;
    foc_ctrl_step(&sample, &cmd);
    const foc_ctrl_ident_t* result = foc_ctrl_ident_get();

    CHECK(result != NULL && !cmd.power_on);
    if (result != NULL) {
        CHECK(result->fail_reason == IDENT_FAIL_NOT_POWERED);
        foc_ctrl_ident_t saved = *result;

        /* A new run must not modify the buffer still owned by the report thread. */
        foc_ctrl_cmd.mode = FOC_MODE_IDLE;
        sample.power_on = false;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_cmd.mode = FOC_MODE_IDENT;
        foc_ctrl_step(&sample, &cmd);
        CHECK(result->state == saved.state && result->fail_reason == saved.fail_reason);
        foc_ctrl_ident_release();
        foc_ctrl_step(&sample, &cmd);
        CHECK(foc_ctrl_ident_get() != NULL);
        foc_ctrl_ident_release();
    }
}

/** Leaving identification aborts excitation and publishes a cancellation result. */
static void test_ident_mode_exit(void) {
    (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
    pwm_adc_sample_t sample = sample_at(15.0f);
    pwm_adc_cmd_t cmd;

    foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
    for (uint32_t k = 0U; k < POWER_GUARD_VBUS_ARM_PERIODS; k++) {
        foc_ctrl_step(&sample, &cmd);
    }
    sample.power_on = true;
    foc_ctrl_cmd.mode = FOC_MODE_IDENT;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_cmd.mode = FOC_MODE_IDLE;
    foc_ctrl_step(&sample, &cmd);
    const foc_ctrl_ident_t* result = foc_ctrl_ident_get();

    CHECK(result != NULL);
    if (result != NULL) {
        CHECK(result->fail_reason == IDENT_FAIL_CANCELLED);
    }
    CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
    foc_ctrl_ident_release();
}

/** Check signed rate integration against elapsed time at every period for ten seconds. */
static void test_openloop_rates(void) {
    const int32_t rates[] = {4096, -4096, 100, 100000, -100000, INT32_MIN, INT32_MAX, 0, -100};
    const uint32_t start = 4093U;

    for (size_t k = 0U; k < sizeof(rates) / sizeof(rates[0]); k++) {
        (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
        pwm_adc_sample_t sample = sample_at(0.0f);
        pwm_adc_cmd_t cmd;
        foc_ctrl_snapshot_t snap;
        int32_t rate = rates[k];

        if (rate > FOC_CTRL_OL_RATE_MAX) {
            rate = FOC_CTRL_OL_RATE_MAX;
        } else if (rate < -FOC_CTRL_OL_RATE_MAX) {
            rate = -FOC_CTRL_OL_RATE_MAX;
        }
        foc_ctrl_cmd.mode = FOC_MODE_OPENLOOP;
        foc_ctrl_cmd.theta_e = start + 4096U;
        foc_ctrl_cmd.rate_e = rates[k];
        for (uint32_t n = 0U; n <= 200000U; n++) {
            foc_ctrl_step(&sample, &cmd);
            foc_ctrl_snapshot(&snap);
            double expected = (double)start + (double)rate * (double)n / 20000.0;
            double error = remainder((double)snap.theta_e_count - expected, 4096.0);

            CHECK(fabs(error) <= 1.0);
            CHECK(snap.theta_e_count < 4096U);
            CHECK(!snap.mode_refused && snap.mode == FOC_MODE_OPENLOOP);
            CHECK(!snap.calib_valid);
            /* Start angle is latched on entry; live theta_e edits must not reset the phase. */
            foc_ctrl_cmd.theta_e = 123U;
        }
        CHECK(fabs(remainder((double)snap.theta_e_count - ((double)start + rate * 10.0), 4096.0)) <= 1.0);
    }
}

/** Verify powered permission, shared ramp/caps, live rate changes and a fresh phase/ramp on reentry. */
static void test_openloop_voltage_and_reentry(void) {
    for (uint32_t powered = 0U; powered <= 1U; powered++) {
        (void)calibrate_codes(2047U, 2049U, 2047U, 2049U);
        pwm_adc_sample_t sample = sample_at(15.0f);
        pwm_adc_cmd_t cmd;
        foc_ctrl_snapshot_t snap;

        if (powered != 0U) {
            foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
            for (uint32_t n = 0U; n < POWER_GUARD_VBUS_ARM_PERIODS; n++) {
                foc_ctrl_step(&sample, &cmd);
            }
            CHECK(cmd.power_on);
            sample.power_on = true;
        }
        foc_ctrl_cmd.mode = FOC_MODE_OPENLOOP;
        foc_ctrl_cmd.theta_e = 1024U;
        foc_ctrl_cmd.v_mv = UINT32_MAX;
        for (uint32_t n = 1U; n <= 30000U; n++) {
            foc_ctrl_step(&sample, &cmd);
            foc_ctrl_snapshot(&snap);
            float expected = fminf((float)n * 50e-6f, 1.0f);
            foc_ab_t vector = applied_vector(&cmd, snap.vbus);

            CHECK(!snap.mode_refused && snap.mode == FOC_MODE_OPENLOOP);
            CHECK(cmd.power_on == (powered != 0U));
            CHECK(fabsf(snap.v_d - expected) < 0.0002f);
            CHECK(snap.v_d <= (float)FOC_CTRL_V_MAX_MV * 0.001f);
            CHECK(snap.v_d <= POWER_GUARD_V_MAX && snap.v_q == 0.0f);
            CHECK(fabsf(vector.alpha) < 1e-5f && fabsf(vector.beta - snap.v_d) < 1e-5f);
        }
        foc_ctrl_cmd.v_mv = 200U;
        for (uint32_t n = 0U; n < 20000U; n++) {
            foc_ctrl_step(&sample, &cmd);
        }
        foc_ctrl_snapshot(&snap);
        CHECK_NEAR(snap.v_d, 0.2f, 1e-6);
        foc_ctrl_cmd.rate_e = 100;
        for (uint32_t n = 0U; n < 20001U; n++) {
            foc_ctrl_step(&sample, &cmd);
        }
        foc_ctrl_snapshot(&snap);
        CHECK(snap.theta_e_count == 1124U);
        foc_ctrl_cmd.rate_e = -100;
        for (uint32_t n = 0U; n < 20000U; n++) {
            foc_ctrl_step(&sample, &cmd);
        }
        foc_ctrl_snapshot(&snap);
        CHECK(snap.theta_e_count == 1024U);
        foc_ctrl_cmd.mode = FOC_MODE_IDLE;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_cmd.theta_e = 4095U;
        foc_ctrl_cmd.mode = FOC_MODE_OPENLOOP;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(snap.theta_e_count == 4095U);
        CHECK_NEAR(snap.v_d, 50e-6f, 1e-8);
    }
}

/* Run production-control regressions with board constants and C11 atomics supplied by host stubs. */
int main(void) {
    test_calibration_acceptance_boundaries();
    test_calibration_rejection_is_sticky();
    test_unknown_mode_cannot_arm();
    test_duty_scale_uses_raw_vbus();
    test_step_capture_latches_vector();
    test_ident_unpowered();
    test_ident_powered();
    test_ident_mode_exit();
    test_openloop_rates();
    test_openloop_voltage_and_reentry();
    return foc_test_result("test_foc_ctrl");
}
