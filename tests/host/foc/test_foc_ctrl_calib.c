/* SPDX-License-Identifier: Apache-2.0 */

#include <math.h>

#include "analog.h"
#include "foc/foc_transform.h"
#include "foc_ctrl.h"
#include "foc_test.h"
#include "power_guard.h"

/* Stub only the terminal result at the module boundary. All other calls use
 * the real identification state machine. Its accuracy is tested separately by
 * test_motor_ident; this fixture tests production control's calibration handoff.
 */
static bool result_pending;
static int32_t result_direction;
static uint32_t result_offset;
static float result_r;
static float result_l;

/** Linker alias for the real identification step used outside the terminal fixture. */
void __real_motor_ident_step(motor_ident_t* ident, const motor_ident_in_t* in, motor_ident_out_t* out);

/** Inject one successful module result on request, otherwise execute real identification. */
void __wrap_motor_ident_step(motor_ident_t* ident, const motor_ident_in_t* in, motor_ident_out_t* out) {
    if (result_pending) {
        result_pending = false;
        ident->result.state = IDENT_DONE;
        ident->result.direction = result_direction;
        ident->result.offset_e = result_offset;
        ident->result.r = result_r;
        ident->result.l = result_l;
        out->v_d = 0.0f;
        out->theta_e_count = 0U;
        out->done = true;
        out->failed = false;
    } else {
        __real_motor_ident_step(ident, in, out);
    }
}

/** Initialize control with plausible current offsets and arm through the real guard. */
static pwm_adc_sample_t prepare_control(void) {
    result_pending = false;
    result_r = 0.2f;
    result_l = 70e-6f;
    foc_ctrl_cmd = (foc_ctrl_cmd_t){0};
    foc_ctrl_init();
    pwm_adc_sample_t sample = {
        .vbus_code = (uint16_t)(15.0f / ANALOG_VBUS_V_PER_CODE + 0.5f),
    };
    pwm_adc_cmd_t cmd;

    for (uint32_t n = 0U; n < FOC_CTRL_CAL_SAMPLES; n++) {
        sample.ia_code = (n & 1U) != 0U ? 2049U : 2047U;
        sample.ic_code = sample.ia_code;
        foc_ctrl_step(&sample, &cmd);
    }
    sample.ia_code = 2048U;
    sample.ic_code = 2048U;
    foc_ctrl_cmd.arm = FOC_CTRL_ARM_KEY;
    for (uint32_t n = 0U; n < POWER_GUARD_VBUS_ARM_PERIODS; n++) {
        foc_ctrl_step(&sample, &cmd);
    }
    CHECK(cmd.power_on);
    sample.power_on = true;
    return sample;
}

/** Supply a minimal successful module result, then let production control consume it. */
static void finish_ident(pwm_adc_sample_t* sample, int32_t direction, uint32_t offset) {
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snapshot;

    foc_ctrl_cmd.mode = FOC_MODE_IDENT;
    foc_ctrl_step(sample, &cmd);
    foc_ctrl_snapshot(&snapshot);
    CHECK(!snapshot.calib_valid);
    CHECK(snapshot.ident_phase == IDENT_ALIGN_FIRST);
    result_pending = true;
    result_direction = direction;
    result_offset = offset;
    foc_ctrl_step(sample, &cmd);
    foc_ctrl_snapshot(&snapshot);
    CHECK(snapshot.calib_valid);
    CHECK(snapshot.calib_direction == direction && snapshot.calib_offset_e == offset);
    const foc_ctrl_ident_t* result = foc_ctrl_ident_get();

    CHECK(result != NULL && result->state == IDENT_DONE);
    /* Keep the result owned by the reader to test that calibration does not
     * depend on releasing the separate publication buffer.
     */
}

/** Check both directions, subtraction underflow, encoder wrap and large raw counts independently. */
static void test_measured_angle(void) {
    const uint32_t offsets[] = {0U, 1U, 2048U, 4095U};
    const uint32_t encoders[] = {0U, 1U, 585U, 4095U, 4096U, 4097U, 65535U, UINT32_MAX};

    for (int32_t direction = -1; direction <= 1; direction += 2) {
        for (size_t o = 0U; o < sizeof(offsets) / sizeof(offsets[0]); o++) {
            pwm_adc_sample_t sample = prepare_control();
            pwm_adc_cmd_t cmd;
            foc_ctrl_snapshot_t snapshot;

            foc_ctrl_snapshot(&snapshot);
            CHECK(!snapshot.calib_valid);
            finish_ident(&sample, direction, offsets[o]);
            foc_ctrl_cmd.mode = FOC_MODE_OPENLOOP;
            for (size_t k = 0U; k < sizeof(encoders) / sizeof(encoders[0]); k++) {
                sample.enc_count = encoders[k];
                int64_t raw = (int64_t)direction * encoders[k] * 7 - offsets[o];
                uint32_t expected = (uint32_t)((raw % 4096 + 4096) % 4096);

                foc_ctrl_step(&sample, &cmd);
                foc_ctrl_snapshot(&snapshot);
                CHECK(snapshot.calib_valid && snapshot.theta_meas_count == expected);
                CHECK(snapshot.calib_direction == direction && snapshot.calib_offset_e == offsets[o]);
            }
            /* A new run clears validity even while an older result is held. */
            foc_ctrl_cmd.mode = FOC_MODE_IDENT;
            foc_ctrl_step(&sample, &cmd);
            foc_ctrl_snapshot(&snapshot);
            CHECK(!snapshot.calib_valid && snapshot.ident_phase == IDENT_ALIGN_FIRST);
            foc_ctrl_ident_release();
            foc_ctrl_cmd.arm = 0U;
            foc_ctrl_step(&sample, &cmd);
            foc_ctrl_snapshot(&snapshot);
            CHECK(snapshot.ident_phase == IDENT_FAILED && !snapshot.calib_valid);
            foc_ctrl_init();
            foc_ctrl_snapshot(&snapshot);
            CHECK(!snapshot.calib_valid);
        }
    }
}

/** Check calibration commits independently of unread telemetry and is lost on a simulated boot. */
static void test_result_ownership_and_boot_reset(void) {
    pwm_adc_sample_t sample = prepare_control();
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snapshot;

    finish_ident(&sample, -1, 4095U);
    const foc_ctrl_ident_t* held = foc_ctrl_ident_get();

    foc_ctrl_cmd.mode = FOC_MODE_IDLE;
    foc_ctrl_step(&sample, &cmd);
    finish_ident(&sample, 1, 123U);
    CHECK(foc_ctrl_ident_get() == held);
    if (held != NULL) {
        CHECK(held->direction == -1 && held->offset_e == 4095U);
    }
    foc_ctrl_snapshot(&snapshot);
    CHECK(snapshot.calib_valid && snapshot.calib_direction == 1 && snapshot.calib_offset_e == 123U);
    foc_ctrl_init();
    foc_ctrl_snapshot(&snapshot);
    CHECK(!snapshot.calib_valid);
}

/** Refuse each strict R/L boundary and invalid numeric result while preserving unpowered checks. */
static void test_current_permission(void) {
    const float motors[][2] = {{0.02f, 70e-6f}, {2.0f, 70e-6f}, {0.2f, 10e-6f},     {0.2f, 5e-3f},
                               {NAN, 70e-6f},   {0.2f, NAN},    {INFINITY, 70e-6f}, {0.2f, -1.0f}};

    for (size_t k = 0U; k < sizeof(motors) / sizeof(motors[0]); k++) {
        pwm_adc_sample_t sample = prepare_control();
        pwm_adc_cmd_t cmd;
        foc_ctrl_snapshot_t snap;

        result_r = motors[k][0];
        result_l = motors[k][1];
        finish_ident(&sample, 1, 0U);
        foc_ctrl_cmd.mode = FOC_MODE_CURRENT;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(snap.mode_refused && snap.mode == FOC_MODE_IDLE);
        CHECK(cmd.a == 0.5f && cmd.b == 0.5f && cmd.c == 0.5f);
        foc_ctrl_cmd.arm = 0U;
        sample.power_on = false;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(!snap.mode_refused && snap.mode == FOC_MODE_CURRENT);
        CHECK(snap.cc_kp == 1.0f && snap.cc_ki == 1000.0f);
    }
}

/** Check entry, calibrated Park/inverse Park, latched clamped references, capture ownership and disarm. */
static void test_current_references(void) {
    pwm_adc_sample_t sample = prepare_control();
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;

    sample.enc_count = 4000U;
    finish_ident(&sample, -1, 123U);
    foc_ctrl_cmd.id_ma = 800;
    foc_ctrl_cmd.iq_ma = 300;
    foc_ctrl_cmd.step_seq = 42U;
    foc_ctrl_cmd.mode = FOC_MODE_CURRENT;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK(snap.mode == FOC_MODE_CURRENT && !snap.mode_refused && cmd.power_on);
    CHECK(snap.i_d_ref == 0.0f && snap.i_q_ref == 0.0f && foc_ctrl_capture_get() == NULL);
    CHECK_NEAR(snap.cc_kp, 70e-6 * 6283.18530718, 1e-6);
    CHECK_NEAR(snap.cc_ki, 0.2 * 6283.18530718, 0.001);
    CHECK(snap.theta_e_count == ((0U - 4000U * 7U - 123U) & 4095U));
    foc_ctrl_cmd.id_ma = INT32_MAX;
    foc_ctrl_cmd.iq_ma = INT32_MIN;
    for (uint32_t k = 0U; k < 20U; k++) {
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        CHECK(snap.i_d_ref == 0.0f && snap.i_q_ref == 0.0f);
    }
    foc_ctrl_cmd.step_seq++;
    for (uint32_t k = 0U; k < FOC_CTRL_CAPTURE_LEN; k++) {
        /* Independent samples show capture uses dq, not the phase buffer meaning. */
        sample.ia_code = 2058U;
        sample.ic_code = 2043U;
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        if (k + 1U < FOC_CTRL_CAPTURE_PRE) {
            CHECK(snap.i_d_ref == 0.0f && snap.i_q_ref == 0.0f);
        } else {
            CHECK_NEAR(snap.i_d_ref, 1.0, 1e-6);
            CHECK_NEAR(snap.i_q_ref, -0.5, 1e-6);
        }
        CHECK(hypotf(snap.v_d, snap.v_q) <= 1.000001f);
        foc_sincos_t sc = {.sin_theta = sinf((float)snap.theta_e_count * 0.0015339808f),
                           .cos_theta = cosf((float)snap.theta_e_count * 0.0015339808f)};
        foc_abc_t volts = {.a = cmd.a * snap.vbus, .b = cmd.b * snap.vbus, .c = cmd.c * snap.vbus};
        foc_dq_t applied = foc_park(foc_clarke(volts), sc);

        CHECK_NEAR(applied.d, snap.v_d, 2e-6);
        CHECK_NEAR(applied.q, snap.v_q, 2e-6);
        foc_ctrl_cmd.id_ma = -250;
        foc_ctrl_cmd.iq_ma = 200;
    }
    const foc_ctrl_capture_t* cap = foc_ctrl_capture_get();

    CHECK(cap != NULL);
    if (cap != NULL) {
        CHECK(cap->kind == 1U && cap->seq == 43U && cap->power_on);
        CHECK(cap->old_id_ma == 0 && cap->old_iq_ma == 0);
        CHECK(cap->new_id_ma == 1000 && cap->new_iq_ma == -500);
        CHECK(cap->ia_ma[0] == (int16_t)(snap.i_d * 1000.0f));
        CHECK(cap->ic_ma[0] == (int16_t)(snap.i_q * 1000.0f));
    }
    foc_ctrl_capture_release();
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK_NEAR(snap.i_d_ref, 1.0, 1e-6);
    foc_ctrl_cmd.step_seq++;
    for (uint32_t k = 0U; k < FOC_CTRL_CAPTURE_LEN; k++) {
        foc_ctrl_step(&sample, &cmd);
    }
    cap = foc_ctrl_capture_get();
    CHECK(cap != NULL && cap->old_id_ma == 1000 && cap->old_iq_ma == -500);
    CHECK(cap != NULL && cap->new_id_ma == -250 && cap->new_iq_ma == 200);
    foc_ctrl_capture_release();
    foc_ctrl_cmd.id_ma = INT32_MIN;
    foc_ctrl_cmd.iq_ma = INT32_MAX;
    foc_ctrl_cmd.step_seq++;
    for (uint32_t k = 0U; k < FOC_CTRL_CAPTURE_LEN; k++) {
        foc_ctrl_step(&sample, &cmd);
    }
    cap = foc_ctrl_capture_get();
    CHECK(cap != NULL && cap->new_id_ma == -1000 && cap->new_iq_ma == 500);
    foc_ctrl_capture_release();
    foc_ctrl_cmd.arm = 0U;
    foc_ctrl_step(&sample, &cmd);
    CHECK(!cmd.power_on);
    sample.power_on = false;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK(snap.cc_kp == 1.0f && snap.cc_ki == 1000.0f);
    float voltage = snap.v_d;

    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK_NEAR(snap.v_d, voltage, 1e-6); /* Off-state PI history cannot accumulate. */
    foc_ctrl_cmd.mode = FOC_MODE_IDLE;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_cmd.mode = FOC_MODE_CURRENT;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_snapshot(&snap);
    CHECK(snap.i_d_ref == 0.0f && snap.i_q_ref == 0.0f && foc_ctrl_capture_get() == NULL);
}

/** Exercise 200-period signed speed windows, strict thresholds, wrap and powered-only tripping. */
static void test_current_overspeed(void) {
    const int32_t speeds[] = {-400, -206, -205, -204, 0, 204, 205, 206, 400};

    for (uint32_t power = 0U; power <= 1U; power++) {
        for (size_t k = 0U; k < sizeof(speeds) / sizeof(speeds[0]); k++) {
            pwm_adc_sample_t sample = prepare_control();
            pwm_adc_cmd_t cmd;
            foc_ctrl_snapshot_t snap;

            finish_ident(&sample, -1, 0U);
            if (power == 0U) {
                foc_ctrl_cmd.arm = 0U;
                sample.power_on = false;
            }
            sample.enc_count = speeds[k] < 0 ? 3U : 65530U;
            uint32_t start = sample.enc_count;

            foc_ctrl_cmd.mode = FOC_MODE_CURRENT;
            foc_ctrl_step(&sample, &cmd);
            for (uint32_t n = 1U; n <= 3U * FOC_CTRL_SPEED_PERIODS; n++) {
                sample.enc_count = start + (uint32_t)(speeds[k] * (int32_t)n / (int32_t)FOC_CTRL_SPEED_PERIODS);
                foc_ctrl_step(&sample, &cmd);
                bool trip = power != 0U && (speeds[k] > 205 || speeds[k] < -205) && n >= FOC_CTRL_SPEED_PERIODS;

                CHECK(cmd.trip == (trip ? PWM_ADC_FAULT_OVERSPEED : 0U));
                if (n % FOC_CTRL_SPEED_PERIODS == 0U) {
                    foc_ctrl_snapshot(&snap);
                    CHECK(snap.speed_counts == speeds[k]);
                }
            }
        }
    }
}

/** Drive the production ISR with a delayed locked-rotor dq plant and inspect the captured step. */
static void test_current_plant(bool d_axis) {
    pwm_adc_sample_t sample = prepare_control();
    pwm_adc_cmd_t cmd;
    foc_ctrl_snapshot_t snap;
    const double decay = exp(-0.2 * 50e-6 / 70e-6);
    double current = 0.0;
    double pending = 0.0;
    double peak = 0.0;
    uint32_t rise = 0U;

    finish_ident(&sample, 1, 0U);
    foc_ctrl_cmd.mode = FOC_MODE_CURRENT;
    foc_ctrl_step(&sample, &cmd);
    foc_ctrl_cmd.id_ma = d_axis ? 300 : 0;
    foc_ctrl_cmd.iq_ma = d_axis ? 0 : 300;
    foc_ctrl_cmd.step_seq++;
    for (uint32_t k = 0U; k < FOC_CTRL_CAPTURE_LEN; k++) {
        double ia = d_axis ? current : 0.0;
        double ic = d_axis ? -0.5 * current : -0.866025403784 * current;

        sample.ia_code = (uint16_t)lround(2048.0 + ia / (double)ANALOG_PHASE_A_PER_CODE);
        sample.ic_code = (uint16_t)lround(2048.0 + ic / (double)ANALOG_PHASE_A_PER_CODE);
        foc_ctrl_step(&sample, &cmd);
        foc_ctrl_snapshot(&snap);
        current = current * decay + pending / 0.2 * (1.0 - decay);
        pending = d_axis ? snap.v_d : snap.v_q;
        if (k >= FOC_CTRL_CAPTURE_PRE && current >= 0.27 && rise == 0U) {
            /* Include the preload delay from command computation at PRE-1. */
            rise = k - FOC_CTRL_CAPTURE_PRE + 2U;
        }
        peak = fmax(peak, current);
        CHECK(cmd.trip == 0U && cmd.power_on);
    }
    printf("production %s rise90 %.3f ms, overshoot %.3f %%\n", d_axis ? "id" : "iq", (double)rise * 0.05,
           (peak / 0.3 - 1.0) * 100.0);
    CHECK(rise > 0U && (double)rise * 0.05 <= 0.6);
    CHECK(peak < 0.36);
    const foc_ctrl_capture_t* cap = foc_ctrl_capture_get();

    CHECK(cap != NULL && cap->kind == 1U);
    if (cap != NULL) {
        for (uint32_t k = 0U; k <= FOC_CTRL_CAPTURE_PRE; k++) {
            CHECK(cap->ia_ma[k] == 0 && cap->ic_ma[k] == 0);
        }
        CHECK((d_axis ? cap->ia_ma[FOC_CTRL_CAPTURE_PRE + 1U] : cap->ic_ma[FOC_CTRL_CAPTURE_PRE + 1U]) > 0);
    }
}

/** Run calibration and powered-current fixtures against the production ISR implementation. */
int main(void) {
    test_current_permission();
    test_current_references();
    test_current_overspeed();
    test_current_plant(false);
    test_current_plant(true);
    test_measured_angle();
    test_result_ownership_and_boot_reset();
    return foc_test_result("test_foc_ctrl_calib");
}
