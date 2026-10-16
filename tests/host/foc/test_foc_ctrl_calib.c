/* SPDX-License-Identifier: Apache-2.0 */

#include "analog.h"
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

/** Linker alias for the real identification step used outside the terminal fixture. */
void __real_motor_ident_step(motor_ident_t* ident, const motor_ident_in_t* in, motor_ident_out_t* out);

/** Inject one successful module result on request, otherwise execute real identification. */
void __wrap_motor_ident_step(motor_ident_t* ident, const motor_ident_in_t* in, motor_ident_out_t* out) {
    if (result_pending) {
        result_pending = false;
        ident->result.state = IDENT_DONE;
        ident->result.direction = result_direction;
        ident->result.offset_e = result_offset;
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

/** Run calibration handoff fixtures against the production ISR implementation. */
int main(void) {
    test_measured_angle();
    test_result_ownership_and_boot_reset();
    return foc_test_result("test_foc_ctrl_calib");
}
