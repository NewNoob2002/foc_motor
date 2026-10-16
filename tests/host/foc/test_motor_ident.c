/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>
#include <stdio.h>

#include "foc_test.h"
#include "motor_ident.h"

typedef struct {
    double r;
    double l;
    double i;
    double pending_v;
    double vdt;
    unsigned int rng;
    int direction;
    uint32_t enc0;
    uint32_t raw_base;
    double i0;
    bool stuck;
    bool noisy;
} plant_t;

/** Quantize a noisy ADC observation; bounded uniform noise has SD exactly 0.1 A before quantization. */
static float current_sample(plant_t* p) {
    double noise = p->noisy ? (double)foc_test_rand(&p->rng, -0.17320508f, 0.17320508f) : 0.0;

    return p->noisy ? (float)(round((p->i + noise) / 0.016) * 0.016) : (float)p->i;
}

/** Snap the unloaded rotor to the requested angle, including encoder wrap and reverse wiring. */
static uint32_t encoder_sample(const plant_t* p, uint32_t theta) {
    int32_t move = p->stuck ? 0 : (int32_t)lround((double)theta / IDENT_POLE_PAIRS) * p->direction;

    return p->raw_base + (uint32_t)((int32_t)p->enc0 + move + (int32_t)IDENT_COUNTS) % IDENT_COUNTS;
}

/** Integrate an exact per-period RL solution with inverter loss frozen at the initial current. */
static void plant_step(plant_t* p, float voltage) {
    double effective_v = p->pending_v - p->vdt * tanh(p->i / (p->i0 > 0.0 ? p->i0 : 0.3));
    double decay = exp(-p->r * (double)IDENT_PERIOD_S / p->l);

    p->i = p->i * decay + effective_v / p->r * (1.0 - decay);
    /* u[k] must wait: it affects i[k+2]-i[k+1], not i[k+1]-i[k]. */
    p->pending_v = (double)voltage;
}

/** Run the real state machine against an independent delayed plant within the absolute bound. */
static motor_ident_t run_plant(plant_t* p) {
    motor_ident_t m;
    motor_ident_out_t out;

    motor_ident_start(&m);
    for (uint32_t k = 0U; k < 200002U; k++) {
        motor_ident_in_t in = {
            .i_d = current_sample(p),
            .enc_count = encoder_sample(p, m.theta_e_count),
            .power_on = true,
        };

        motor_ident_step(&m, &in, &out);
        CHECK(isfinite(out.v_d) && fabsf(out.v_d) <= 1.0f);
        if (out.done || out.failed) {
            CHECK(out.v_d == 0.0f);
            return m;
        }
        plant_step(p, out.v_d);
    }
    CHECK(false);
    return m;
}

/** Check accuracy for reachable motors with both encoder directions and realistic ADC noise. */
static void test_accuracy(double r, double l, int direction, uint32_t enc0) {
    plant_t p = {.r = r,
                 .l = l,
                 .vdt = 0.15,
                 .rng = 12345U,
                 .direction = direction,
                 .enc0 = enc0,
                 .raw_base = 4096U,
                 .noisy = true};
    motor_ident_t m = run_plant(&p);
    motor_ident_result_t* result = &m.result;
    uint32_t offset = enc0 * IDENT_POLE_PAIRS % IDENT_COUNTS;

    if (direction < 0) {
        offset = (IDENT_COUNTS - offset) % IDENT_COUNTS;
    }
    printf("R %.3f L %.0f uH dir %d: state %u reason %u, R %.6f L %.3f uH, ipp %.3f A, %.3f s\n", r, l * 1e6, direction,
           (unsigned int)result->state, (unsigned int)result->fail_reason, (double)result->r, (double)result->l * 1e6,
           (double)result->ipp, (double)m.run_n * (double)IDENT_PERIOD_S);
    CHECK(result->state == IDENT_DONE && result->fail_reason == IDENT_FAIL_NONE);
    CHECK_NEAR(result->r, r, r * 0.05);
    CHECK_NEAR(result->l, l, l * 0.10);
    CHECK(result->direction == direction);
    CHECK(result->enc_0 == p.raw_base + enc0 && result->offset_e == offset);
    CHECK(result->enc_90 == encoder_sample(&p, IDENT_ALIGN_90));
    CHECK(result->delta_mech == -direction * 146);
    CHECK(!result->r_nonlinear);
    CHECK_NEAR(result->v0, 0.15, 0.015);
    for (uint32_t k = 1U; k < IDENT_R_POINTS; k++) {
        CHECK(result->points[k].i > result->points[k - 1U].i);
    }
}

/** Unreachable currents must time out rather than violating the 0.9 V ceiling. */
static void test_unreachable(double r, double l, int direction) {
    plant_t p = {.r = r, .l = l, .vdt = 0.15, .rng = 1U, .direction = direction, .enc0 = 3745U, .noisy = true};
    motor_ident_t m = run_plant(&p);

    CHECK(m.result.state == IDENT_FAILED && m.result.fail_reason == IDENT_FAIL_SERVO_TIMEOUT);
    CHECK(m.run_n == 20000U);
}

/** Prove the two-call sign association with an exact integrator and detect either adjacent wrong lag. */
static void test_delay_association(void) {
    motor_ident_t m;
    motor_ident_out_t out;
    double current = 1.0;
    double pending = 0.0;
    double previous = 1.0;
    double wrong_one = 0.0;
    double wrong_three = 0.0;
    const double inductance = 200e-6;
    const uint32_t total = IDENT_L_CYCLES * 2U * IDENT_L_HALF;

    motor_ident_start(&m);
    m.result.state = IDENT_INDUCTANCE;
    m.result.points[IDENT_R_BIAS_POINT].v = 0.3f;
    for (uint32_t k = 0U; k <= total + 1U; k++) {
        motor_ident_in_t in = {.i_d = (float)current, .power_on = true};

        if (k >= IDENT_L_DISCARD * 2U * IDENT_L_HALF + 2U) {
            double delta = current - previous;
            double sign_one = (((k - 1U) / IDENT_L_HALF) & 1U) == 0U ? 1.0 : -1.0;
            double sign_three = (((k - 3U) / IDENT_L_HALF) & 1U) == 0U ? 1.0 : -1.0;

            wrong_one += sign_one * delta;
            wrong_three += sign_three * delta;
        }
        previous = current;
        motor_ident_step(&m, &in, &out);
        current += pending * (double)IDENT_PERIOD_S / inductance;
        pending = (double)out.v_d - 0.3;
    }
    CHECK(out.done && !out.failed && out.v_d == 0.0f);
    CHECK(m.increments == (IDENT_L_CYCLES - IDENT_L_DISCARD) * 2U * IDENT_L_HALF);
    CHECK_NEAR(m.result.l, inductance, inductance * 1e-3);
    CHECK_NEAR(wrong_one / (double)m.sum_delta, 0.5, 1e-4);
    CHECK_NEAR(wrong_three / (double)m.sum_delta, 0.5, 1e-4);
}

/** Check immediate zero output, latched failures, restart, and the absolute run bound. */
static void test_safety(void) {
    const float invalid[] = {NAN, INFINITY, -INFINITY, 1.801f, -1.801f};
    motor_ident_t m;
    motor_ident_out_t out;
    motor_ident_in_t in = {.power_on = true};

    for (uint32_t k = 0U; k < sizeof(invalid) / sizeof(invalid[0]); k++) {
        motor_ident_start(&m);
        in.i_d = invalid[k];
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed && out.v_d == 0.0f);
        CHECK(m.result.fail_reason == (k < 3U ? IDENT_FAIL_NUMERIC : IDENT_FAIL_OVERCURRENT));
        in.i_d = 0.0f;
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed && out.v_d == 0.0f);
    }
    motor_ident_start(&m);
    for (uint32_t k = 0U; k < 100U; k++) {
        motor_ident_step(&m, &in, &out);
    }
    CHECK(out.v_d > 0.0f);
    in.power_on = false;
    motor_ident_step(&m, &in, &out);
    CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_NOT_POWERED);
    in.power_on = true;
    motor_ident_step(&m, &in, &out);
    CHECK(out.failed && out.v_d == 0.0f);
    motor_ident_start(&m);
    m.run_n = 200000U;
    motor_ident_step(&m, &in, &out);
    CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_RUN_TIMEOUT);
}

/** Reject a stationary encoder even when the electrical current servo succeeds. */
static void test_stuck_encoder(void) {
    plant_t p = {.r = 0.1, .l = 100e-6, .vdt = 0.15, .rng = 1U, .direction = 1, .enc0 = 3745U, .stuck = true};
    motor_ident_t m = run_plant(&p);

    CHECK(m.result.state == IDENT_FAILED && m.result.fail_reason == IDENT_FAIL_ALIGNMENT);
}

/** Reject nonpositive L response and excessive inferred ripple at the end of injection. */
static void test_l_failures(void) {
    motor_ident_t m;
    motor_ident_out_t out;
    motor_ident_in_t in = {.i_d = 1.0f, .power_on = true};

    for (uint32_t k = 0U; k < 2U; k++) {
        motor_ident_start(&m);
        m.result.state = IDENT_INDUCTANCE;
        m.l_n = IDENT_L_CYCLES * 2U * IDENT_L_HALF + 1U;
        m.prev_i = in.i_d;
        m.increments = 100U;
        m.sum_delta = k == 0U ? -1.0f : 100.0f;
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed && out.v_d == 0.0f);
        CHECK(m.result.fail_reason == (k == 0U ? IDENT_FAIL_INDUCTANCE : IDENT_FAIL_RIPPLE));
    }
}

/** Flag curvature when inverter loss has not saturated over the fitted upper points. */
static void test_nonlinear_resistance(void) {
    plant_t p = {.r = 0.1, .l = 300e-6, .vdt = 0.5, .i0 = 0.5, .rng = 1U, .direction = 1, .enc0 = 3745U};
    motor_ident_t m = run_plant(&p);

    CHECK(m.result.state == IDENT_DONE);
    CHECK(m.result.r_nonlinear);
    CHECK(m.result.points[0].i > 0.4f && m.result.points[4].i > 1.4f);
}

/** Exercise production identification with reachable plants and explicit physical-limit failures. */
int main(void) {
    for (int direction = -1; direction <= 1; direction += 2) {
        test_accuracy(0.1, 100e-6, direction, 3745U);
        test_accuracy(0.2, 300e-6, direction, direction > 0 ? 4000U : 50U);
        /* 1 ohm needs >1.0 V even for ALIGN, before the unreachable 1.5 A R point. */
        test_unreachable(1.0, 1e-3, direction);
    }
    test_unreachable(5.0, 1e-3, 1);
    test_delay_association();
    test_safety();
    test_stuck_encoder();
    test_l_failures();
    test_nonlinear_resistance();
    return foc_test_result("test_motor_ident");
}
