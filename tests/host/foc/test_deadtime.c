/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc_test.h"

/** Check each phase, the linear band, saturation and the identified alpha-axis loss. */
static void test_compensation(void) {
    const float samples[] = {-2.0f, -0.05f, -0.025f, 0.0f, 0.025f, 0.05f, 2.0f};
    for (size_t phase = 0U; phase < 3U; phase++) {
        for (size_t k = 0U; k < sizeof(samples) / sizeof(samples[0]); k++) {
            foc_abc_t i = {0};
            float x = samples[k];
            if (phase == 0U) {
                i.a = x;
            }
            if (phase == 1U) {
                i.b = x;
            }
            if (phase == 2U) {
                i.c = x;
            }
            foc_ab_t actual = foc_deadtime_comp(i, 0.175f, 0.05f);
            float v = 0.175f * fminf(1.0f, fmaxf(-1.0f, x / 0.05f));
            foc_abc_t expected = {0};
            if (phase == 0U) {
                expected.a = v;
            }
            if (phase == 1U) {
                expected.b = v;
            }
            if (phase == 2U) {
                expected.c = v;
            }
            foc_ab_t ab = foc_clarke(expected);
            CHECK_NEAR(actual.alpha, ab.alpha, 1e-7);
            CHECK_NEAR(actual.beta, ab.beta, 1e-7);
        }
    }
    foc_abc_t i = {1.0f, -0.5f, -0.5f};
    foc_ab_t ab = foc_deadtime_comp(i, 0.175f, 0.05f);
    CHECK_NEAR(ab.alpha, 4.0 / 3.0 * 0.175, 1e-7);
    CHECK_NEAR(ab.beta, 0.0, 1e-7);
}

/** Compare feed-forward with an independent manual chain, including reduced PI bounds and total cap. */
static void test_feedforward(void) {
    unsigned int rng = 123U;
    for (uint32_t k = 0U; k < 2000U; k++) {
        float theta = foc_test_rand(&rng, -3.14f, 3.14f);
        foc_sincos_t sc = {sinf(theta), cosf(theta)};
        foc_current_t cc = {.pi_d = {.kp = 0.44f, .ki = 1258.0f}, .pi_q = {.kp = 0.44f, .ki = 1258.0f}};
        foc_current_t manual = cc;
        foc_dq_t ref = {foc_test_rand(&rng, -10.0f, 10.0f), foc_test_rand(&rng, -10.0f, 10.0f)};
        foc_abc_t i = {0};
        foc_ab_t ff = {foc_test_rand(&rng, -1.0f, 1.0f), foc_test_rand(&rng, -1.0f, 1.0f)};
        float lim = fmaxf(0.2f, 1.0f - hypotf(ff.alpha, ff.beta));
        manual.pi_d.out_min = manual.pi_q.out_min = -lim;
        manual.pi_d.out_max = manual.pi_q.out_max = lim;
        foc_dq_t v = {foc_pi_step(&manual.pi_d, ref.d, 50e-6f), foc_pi_step(&manual.pi_q, ref.q, 50e-6f)};
        float length = hypotf(v.d, v.q);
        if (length > lim) {
            v.d *= lim / length;
            v.q *= lim / length;
        }
        foc_ab_t total = foc_inv_park(v, sc);
        total.alpha += ff.alpha;
        total.beta += ff.beta;
        length = hypotf(total.alpha, total.beta);
        if (length > 1.0f) {
            total.alpha /= length;
            total.beta /= length;
        }
        foc_abc_t duty;
        (void)foc_svpwm(total, 15.0f, &duty);
        foc_current_out_t out;
        foc_current_step_ff(&cc, i, sc, ref, 15.0f, 50e-6f, ff, 1.0f, &out);
        CHECK_NEAR(out.duty.a, duty.a, 2e-7);
        CHECK_NEAR(out.duty.b, duty.b, 2e-7);
        CHECK_NEAR(out.duty.c, duty.c, 2e-7);
        CHECK(hypotf(out.v_dq.d, out.v_dq.q) <= 1.000001f);
        CHECK_NEAR(cc.pi_d.integral, manual.pi_d.integral, 1e-7);
        CHECK_NEAR(cc.pi_q.integral, manual.pi_q.integral, 1e-7);
        CHECK(fabsf(cc.pi_d.integral) <= lim + 1e-7f);
    }
    foc_current_t cc = {.pi_d = {.ki = 1258.0f}, .pi_q = {.ki = 1258.0f}};
    foc_current_out_t out;
    for (uint32_t k = 0U; k < 1000U; k++) {
        foc_current_step_ff(&cc, (foc_abc_t){0}, (foc_sincos_t){0.0f, 1.0f}, (foc_dq_t){10.0f, -10.0f}, 15.0f, 50e-6f,
                            (foc_ab_t){0.3f, 0.0f}, 1.0f, &out);
    }
    CHECK_NEAR(cc.pi_d.integral, 0.7, 1e-6);
    CHECK_NEAR(cc.pi_q.integral, -0.7, 1e-6);
}

/** Integrate a delayed two-axis RL plant with per-phase smooth inverter voltage loss. */
static void test_inverter(bool enabled) {
    const double r = 0.2, l = 70e-6, dt = 50e-6;
    double decay = exp(-r * dt / l);
    foc_sincos_t sc = {0.0f, 1.0f};
    foc_current_t cc = {.pi_d = {.kp = (float)(l * 6283.18530718), .ki = (float)(r * 6283.18530718)},
                        .pi_q = {.kp = (float)(l * 6283.18530718), .ki = (float)(r * 6283.18530718)}};
    foc_ab_t i = {0}, pending = {0};
    foc_dq_t ref = {0.5f, 0.0f};
    double peak = 0.0;
    uint32_t rise = 0U;
    for (uint32_t k = 0U; k < 200U; k++) {
        foc_abc_t abc = foc_inv_clarke(i);
        foc_abc_t volts = foc_inv_clarke(pending);
        volts.a -= (float)(0.175 * tanh((double)abc.a / 0.03));
        volts.b -= (float)(0.175 * tanh((double)abc.b / 0.03));
        volts.c -= (float)(0.175 * tanh((double)abc.c / 0.03));
        foc_ab_t effective = foc_clarke(volts);
        foc_current_out_t out;
        if (enabled) {
            foc_ab_t comp = foc_deadtime_comp(foc_inv_clarke(foc_inv_park(ref, sc)), 0.175f, 0.05f);
            foc_current_step_ff(&cc, abc, sc, ref, 15.0f, (float)dt, comp, 1.0f, &out);
        } else {
            foc_current_step_lim(&cc, abc, sc, ref, 15.0f, (float)dt, 1.0f, &out);
        }
        i.alpha = (float)((double)i.alpha * decay + (double)effective.alpha / r * (1.0 - decay));
        i.beta = (float)((double)i.beta * decay + (double)effective.beta / r * (1.0 - decay));
        pending = foc_inv_park(out.v_dq, sc);
        if (i.alpha >= 0.45f && rise == 0U) {
            rise = k + 1U;
        }
        peak = fmax(peak, i.alpha);
    }
    double rise_ms = rise * dt * 1000.0;
    double overshoot = (peak / 0.5 - 1.0) * 100.0;
    printf("inverter DTC %s: rise90 %.3f ms, overshoot %.3f %%\n", enabled ? "on" : "off", rise_ms, overshoot);
    CHECK(rise > 0U);
    if (enabled) {
        CHECK(rise_ms <= 0.5);
        CHECK(overshoot <= 20.0);
    } else {
        CHECK(rise_ms >= 0.7 && rise_ms <= 1.5);
    }
}

/** Run dead-time unit tests and the independent inverter simulation. */
int main(void) {
    test_compensation();
    test_feedforward();
    test_inverter(false);
    test_inverter(true);
    return foc_test_result("test_deadtime");
}
