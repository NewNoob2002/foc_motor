/* SPDX-License-Identifier: Apache-2.0 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    double cogging;            /* torque amplitude, 84 mechanical detents = 12 electrical */
    double cogging_modulation; /* zero-mean second spatial harmonic, nonuniform cogging torque */
    double cogging_phase;      /* optional 84-detent phase; default 2.7 radians */
    double modulation_phase;   /* phase of the lower spatial harmonic */
    double stiction;           /* optional friction threshold, default 0.1 */
    double rotor;              /* unwrapped electrical counts in the physical command frame */
    double stop_rotor;
    uint32_t last_theta;
    bool half_move;
    bool stuck;
    bool noisy;
    uint32_t spike_burst;
    uint32_t spikes;
} plant_t;

/** Quantize an ADC observation with approximately Gaussian noise of SD 0.1 A and tails beyond 3 sigma. */
static float current_sample(plant_t* p) {
    double noise = 0.0;

    if (p->noisy) {
        for (uint32_t k = 0U; k < 12U; k++) {
            noise += (double)foc_test_rand(&p->rng, -0.5f, 0.5f);
        }
        noise *= 0.1;
    }
    return p->noisy ? (float)(round((p->i + noise) / 0.016) * 0.016) : (float)p->i;
}

/** Quantize the physical rotor position, preserving raw counter wrap and reverse wiring. */
static uint32_t encoder_sample(const plant_t* p) {
    int32_t move = (int32_t)lround(p->rotor / IDENT_POLE_PAIRS) * p->direction;

    return p->raw_base + (uint32_t)((int32_t)p->enc0 + move + 4 * (int32_t)IDENT_COUNTS) % IDENT_COUNTS;
}

/** Integrate overdamped sinusoidal torque, with static friction and an optional half-travel stop. */
static void rotor_step(plant_t* p, uint32_t theta) {
    const double radians_per_count = 6.283185307179586 / (double)IDENT_COUNTS;
    double error = remainder((double)theta - p->rotor, (double)IDENT_COUNTS);
    double torque = sin(error * radians_per_count) * p->i / (double)IDENT_ALIGN_A;
    double phase = p->cogging_phase != 0.0 ? p->cogging_phase : 2.7;
    torque -= p->cogging * sin(12.0 * p->rotor * radians_per_count + phase)
              + p->cogging_modulation * sin(2.0 * p->rotor * radians_per_count + p->modulation_phase);

    if (theta != p->last_theta) {
        p->stop_rotor = p->rotor + 0.5 * error;
        p->last_theta = theta;
    }
    if (p->stuck || fabs(torque) < (p->stiction > 0.0 ? p->stiction : 0.1)) {
        return;
    }
    double move = 8.0 * torque;

    if (p->half_move && fabs(move) > fabs(p->stop_rotor - p->rotor)) {
        move = p->stop_rotor - p->rotor;
    }
    p->rotor += move;
}

/** Integrate an exact per-period RL solution with inverter loss frozen at the initial current. */
static void plant_step(plant_t* p, float voltage, uint32_t theta) {
    rotor_step(p, theta);
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
    uint32_t r_n = 0U;
    uint32_t angle_step = 0U;
    double initial_rotor = p->rotor;
    static const uint32_t angles[] = {0U, IDENT_ALIGN_120, IDENT_ALIGN_240, 0U, IDENT_ALIGN_120, 0U};

    motor_ident_start(&m);
    for (uint32_t k = 0U; k < IDENT_RUN_TIMEOUT_N + 2U; k++) {
        motor_ident_in_t in = {
            .i_d = current_sample(p),
            .enc_count = encoder_sample(p),
            .power_on = true,
        };

        if (m.result.state == IDENT_RESISTANCE && p->spike_burst > 0U) {
            if (r_n % 2000U < p->spike_burst) {
                in.i_d = 1.9f;
                p->spikes++;
            }
            r_n++;
        }
        uint32_t previous_angle = m.theta_e_count;

        motor_ident_step(&m, &in, &out);
        if (previous_angle != out.theta_e_count && m.result.state != IDENT_OFFSET_ROT && !out.done) {
            angle_step++;
            CHECK(angle_step < sizeof(angles) / sizeof(angles[0]));
            CHECK(out.theta_e_count == angles[angle_step]);
            CHECK(out.v_d == 0.0f); /* Every switch follows a verified zero-current decay. */
            if (angle_step == 1U && !p->stuck && !p->half_move) {
                /* 300 degrees is unstable for the 120 pull, but has |sin|=0.866
                 * during the zero park, so cannot remain there with stiction 0.1.
                 */
                CHECK(fabs(remainder(p->rotor - ((double)IDENT_ALIGN_120 + 0.5 * IDENT_COUNTS), (double)IDENT_COUNTS))
                      > 100.0);
                if (initial_rotor == 0.5 * IDENT_COUNTS) {
                    CHECK(p->rotor == initial_rotor); /* Reproduce the friction-held unstable first park. */
                }
            }
            if (angle_step == 2U && !p->stuck && !p->half_move && p->cogging == 0.0) {
                CHECK(fabs(remainder(p->rotor - IDENT_ALIGN_120, (double)IDENT_COUNTS)) < 70.0);
            }
        }
        CHECK(isfinite(out.v_d) && fabsf(out.v_d) <= 1.0f);
        if (out.done || out.failed) {
            CHECK(out.v_d == 0.0f);
            return m;
        }
        plant_step(p, out.v_d, out.theta_e_count);
    }
    CHECK(false);
    return m;
}

/** Check accuracy for reachable motors with both encoder directions and realistic ADC noise. */
static void test_accuracy(double r, double l, int direction, uint32_t enc0, double start_degrees) {
    plant_t p = {.r = r,
                 .l = l,
                 .vdt = 0.15,
                 .rng = 12345U,
                 .direction = direction,
                 .enc0 = enc0,
                 .raw_base = 4096U,
                 .rotor = start_degrees * IDENT_COUNTS / 360.0,
                 .noisy = true};
    motor_ident_t m = run_plant(&p);
    motor_ident_result_t* result = &m.result;
    uint32_t offset = enc0 * IDENT_POLE_PAIRS % IDENT_COUNTS;

    if (direction < 0) {
        offset = (IDENT_COUNTS - offset) % IDENT_COUNTS;
    }
    printf("R %.3f L %.0f uH dir %d start %.0f deg: state %u reason %u, R %.6f L %.3f uH, ipp %.3f A, %.3f s\n", r,
           l * 1e6, direction, start_degrees, (unsigned int)result->state, (unsigned int)result->fail_reason,
           (double)result->r, (double)result->l * 1e6, (double)result->ipp, (double)m.run_n * (double)IDENT_PERIOD_S);
    CHECK(result->state == IDENT_DONE && result->fail_reason == IDENT_FAIL_NONE);
    CHECK_NEAR(result->r, r, r * 0.05);
    CHECK_NEAR(result->l, l, l * 0.10);
    CHECK(result->direction == direction);
    uint32_t forward_offset = result->enc_360 * IDENT_POLE_PAIRS % IDENT_COUNTS;

    if (direction < 0) {
        forward_offset = (IDENT_COUNTS - forward_offset) % IDENT_COUNTS;
    }
    double old_error = fabs(remainder((double)forward_offset - offset, (double)IDENT_COUNTS));
    double new_error = fabs(remainder((double)result->offset_e - offset, (double)IDENT_COUNTS));

    printf("two-sided dir %d: one-sided %.1f -> mean %.1f electrical counts, band %d\n", direction, old_error,
           new_error, result->friction_half_e);
    /* Stiction 0.1 produces about 65 electrical counts of one-sided error.
     * Symmetric arrivals cancel it to within 8 counts (encoder quantum is 7).
     */
    CHECK(old_error >= 50.0);
    CHECK(new_error <= 8.0 && new_error < old_error / 5.0);
    CHECK(abs(result->friction_half_e) >= 50 && abs(result->friction_half_e) <= 75);
    /* The physical zero is only known to the stiction band asin(0.1), plus
     * encoder quantization; snapping exactly to the command would hide this.
     */
    CHECK(fabs(remainder((double)result->offset_e - offset, (double)IDENT_COUNTS)) <= 70.0);
    CHECK(result->enc_120 >= p.raw_base && result->enc_120 < p.raw_base + IDENT_COUNTS);
    CHECK(result->enc_240 >= p.raw_base && result->enc_240 < p.raw_base + IDENT_COUNTS);
    CHECK(result->enc_360 >= p.raw_base && result->enc_360 < p.raw_base + IDENT_COUNTS);
    CHECK(result->delta_a * direction > 0 && result->delta_b * direction > 0);
    CHECK_NEAR(fabs((double)result->delta_a) * IDENT_POLE_PAIRS, IDENT_ALIGN_120, 200);
    CHECK_NEAR(fabs((double)result->delta_b) * IDENT_POLE_PAIRS, IDENT_ALIGN_120, 200);
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
    CHECK(!out.done && !out.failed && m.result.state == IDENT_OFFSET_ROT);
    CHECK_NEAR(out.v_d, 0.3, 1e-6);
    CHECK(m.increments == (IDENT_L_CYCLES - IDENT_L_DISCARD) * 2U * IDENT_L_HALF);
    CHECK_NEAR(m.result.l, inductance, inductance * 1e-3);
    CHECK_NEAR(wrong_one / (double)m.sum_delta, 0.5, 1e-4);
    CHECK_NEAR(wrong_three / (double)m.sum_delta, 0.5, 1e-4);
}

/** Tolerate repeated short R-phase spike bursts, but stop on the fourth consecutive spike. */
static void test_r_spikes(void) {
    const uint32_t bursts[] = {1U, IDENT_I_ABORT_N - 1U, IDENT_I_ABORT_N};

    for (uint32_t k = 0U; k < sizeof(bursts) / sizeof(bursts[0]); k++) {
        plant_t p = {.r = 0.1, .l = 100e-6, .vdt = 0.15, .direction = 1, .enc0 = 3745U, .spike_burst = bursts[k]};
        motor_ident_t m = run_plant(&p);

        if (bursts[k] < IDENT_I_ABORT_N) {
            CHECK(m.result.state == IDENT_DONE && m.result.fail_reason == IDENT_FAIL_NONE);
            CHECK(p.spikes > bursts[k]);
        } else {
            CHECK(m.result.state == IDENT_FAILED && m.result.fail_reason == IDENT_FAIL_OVERCURRENT);
            CHECK(p.spikes == IDENT_I_ABORT_N);
        }
    }
}

/** Check both current signs, threshold resets, restart, and immediate numeric failure during a burst. */
static void test_overcurrent(void) {
    motor_ident_t m;
    motor_ident_out_t out;
    motor_ident_in_t in = {.power_on = true};

    for (int sign = -1; sign <= 1; sign += 2) {
        motor_ident_start(&m);
        CHECK(m.overcurrent_n == 0U);
        for (uint32_t burst = 0U; burst < 3U; burst++) {
            in.i_d = (float)sign * 1.9f;
            for (uint32_t k = 1U; k < IDENT_I_ABORT_N; k++) {
                motor_ident_step(&m, &in, &out);
                CHECK(!out.failed && m.overcurrent_n == k);
            }
            if (burst == 0U) {
                in.i_d = (float)sign * IDENT_I_ABORT_A;
                motor_ident_step(&m, &in, &out);
                CHECK(!out.failed && m.overcurrent_n == 0U);
            } else if (burst == 1U) {
                in.i_d = 0.0f;
                motor_ident_step(&m, &in, &out);
                CHECK(!out.failed && m.overcurrent_n == 0U);
            } else {
                motor_ident_step(&m, &in, &out);
                CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_OVERCURRENT);
            }
        }
        motor_ident_start(&m);
        CHECK(m.overcurrent_n == 0U);
        motor_ident_step(&m, &in, &out);
        CHECK(!out.failed);
        in.i_d = NAN;
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_NUMERIC);
    }
}

/** Check immediate zero output, latched failures, restart, and the absolute run bound. */
static void test_safety(void) {
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    motor_ident_t m;
    motor_ident_out_t out;
    motor_ident_in_t in = {.power_on = true};

    for (uint32_t k = 0U; k < sizeof(invalid) / sizeof(invalid[0]); k++) {
        motor_ident_start(&m);
        in.i_d = invalid[k];
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed && out.v_d == 0.0f);
        CHECK(m.result.fail_reason == IDENT_FAIL_NUMERIC);
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
    /* Six alignment holds, five decays, five R targets and one bias servo.
     * Include all arrival budgets, R settling/averages, bias dwell and L flush.
     */
    const uint32_t phase_bound = (6U + 5U + IDENT_R_POINTS + 1U) * IDENT_SERVO_TIMEOUT_N + 6U * IDENT_ALIGN_HOLD_N
                                 + IDENT_R_POINTS * (IDENT_R_SETTLE_N + IDENT_R_AVG_N) + IDENT_R_SETTLE_N
                                 + IDENT_L_CYCLES * 2U * IDENT_L_HALF + 2U + 2U * IDENT_ROT_DIRECTION_N;

    CHECK(IDENT_RUN_TIMEOUT_N > phase_bound);
    motor_ident_start(&m);
    m.run_n = IDENT_RUN_TIMEOUT_N;
    motor_ident_step(&m, &in, &out);
    CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_RUN_TIMEOUT);
}

/** Reject an immobile rotor and a rotor stopped halfway through each commanded move, in both directions. */
static void test_alignment_failures(void) {
    for (int direction = -1; direction <= 1; direction += 2) {
        for (uint32_t fault = 0U; fault < 2U; fault++) {
            plant_t p = {.r = 0.1,
                         .l = 100e-6,
                         .vdt = 0.15,
                         .rng = 1U,
                         .direction = direction,
                         .enc0 = 3745U,
                         .stuck = fault == 0U,
                         .half_move = fault == 1U};
            motor_ident_t m = run_plant(&p);

            CHECK(m.result.state == IDENT_FAILED && m.result.fail_reason == IDENT_FAIL_ALIGNMENT);
        }
    }
}

/** Check each delta independently, sign agreement, tolerance edges, raw counts, and wrap in both directions. */
static void test_alignment_validation(void) {
    static const struct {
        int32_t a;
        int32_t b;
        bool valid;
    } cases[] = {
        {195, 195, true},  {110, 280, true},   {280, 110, true},   {109, 281, false}, {281, 109, false},
        {241, 124, true},  {138, 195, true},   {137, 195, false},  {252, 195, true},  {253, 195, false},
        {195, 109, false}, {195, 281, false},  {110, 110, false},  {280, 280, false}, {195, 0, false},
        {0, 195, false},   {195, -195, false}, {-195, 195, false}, {98, 97, false},   {213, 190, true},
        {204, 179, true},  {219, 186, true},   {193, 190, true},
    };

    for (int32_t direction = -1; direction <= 1; direction += 2) {
        for (uint32_t k = 0U; k < sizeof(cases) / sizeof(cases[0]); k++) {
            motor_ident_t m;
            motor_ident_out_t out;
            int32_t start = direction > 0 ? 4070 : 25;
            int32_t middle = (start + direction * cases[k].a + (int32_t)IDENT_COUNTS) % (int32_t)IDENT_COUNTS;
            int32_t end = (middle + direction * cases[k].b + (int32_t)IDENT_COUNTS) % (int32_t)IDENT_COUNTS;
            motor_ident_in_t in = {.i_d = 1.0f, .enc_count = (uint32_t)end + 8192U, .power_on = true};

            motor_ident_start(&m);
            m.result.state = IDENT_ALIGN_ZERO;
            m.result.enc_120 = (uint32_t)start + 4096U;
            m.result.enc_240 = (uint32_t)middle;
            m.reached = true;
            m.hold_n = IDENT_ALIGN_HOLD_N - 1U;
            m.filtered_i = 1.0f;
            m.servo_v = 0.25f;
            motor_ident_step(&m, &in, &out);
            CHECK(m.result.enc_360 == in.enc_count);
            CHECK(m.result.delta_a == direction * cases[k].a);
            CHECK(m.result.delta_b == direction * cases[k].b);
            if (cases[k].valid) {
                CHECK(m.result.state == IDENT_ALIGN_DECAY && !out.failed);
                CHECK(m.result.direction == direction);
                CHECK(out.theta_e_count == 0U && out.v_d == 0.0f);
            } else {
                CHECK(out.failed && out.v_d == 0.0f && m.result.fail_reason == IDENT_FAIL_ALIGNMENT);
                CHECK(m.result.direction == 0 && m.result.offset_e == 0U);
            }
        }
    }
}

/** Validate both extra moves independently, including sign, magnitude and circular-mean wrap. */
static void test_two_sided_validation(void) {
    const int32_t moves[][2] = {{195, -195}, {110, -280},  {280, -110}, {109, -195}, {281, -195}, {195, -109},
                                {195, -281}, {-195, -195}, {195, 195},  {0, -195},   {195, 0}};

    for (int32_t dir = -1; dir <= 1; dir += 2) {
        for (size_t k = 0U; k < sizeof(moves) / sizeof(moves[0]); k++) {
            motor_ident_t m;
            motor_ident_out_t out;
            uint32_t a = dir > 0 ? 4090U : 6U;
            uint32_t mid = (a + (uint32_t)(dir * moves[k][0])) & 4095U;
            uint32_t b = (mid + (uint32_t)(dir * moves[k][1])) & 4095U;
            motor_ident_in_t in = {.i_d = IDENT_ALIGN_A, .enc_count = b + 8192U, .power_on = true};

            motor_ident_start(&m);
            m.result.state = IDENT_ALIGN_BACK_ZERO;
            m.result.direction = dir;
            m.result.enc_360 = a + 4096U;
            m.result.enc_return_120 = mid;
            m.reached = true;
            m.hold_n = IDENT_ALIGN_HOLD_N - 1U;
            m.servo_v = 0.25f;
            motor_ident_step(&m, &in, &out);
            if (k < 3U) {
                CHECK(!out.failed && m.result.state == IDENT_RESISTANCE);
                double raw_a = (double)((a * 7U) & 4095U);
                double raw_b = (double)((b * 7U) & 4095U);
                double mean = raw_a + remainder(raw_b - raw_a, 4096.0) / 2.0;

                CHECK(fabs(remainder((double)m.result.offset_e - dir * mean, 4096.0)) <= 1.0);
                CHECK(out.theta_e_count == 0U && out.v_d == 0.25f);
            } else {
                CHECK(out.failed && m.result.fail_reason == IDENT_FAIL_ALIGNMENT && out.v_d == 0.0f);
            }
        }
        motor_ident_t m;
        motor_ident_out_t out;
        motor_ident_in_t in = {.i_d = 1.0f, .enc_count = dir > 0 ? 6U : 4090U, .power_on = true};

        motor_ident_start(&m);
        m.result.state = IDENT_ALIGN_BACK_ZERO;
        m.result.direction = dir;
        m.result.enc_360 = dir > 0 ? 4090U : 6U;
        m.result.enc_return_120 = (m.result.enc_360 + (uint32_t)(dir * 195)) & 4095U;
        m.reached = true;
        m.hold_n = IDENT_ALIGN_HOLD_N - 1U;
        motor_ident_step(&m, &in, &out);
        CHECK(!out.failed && m.result.offset_e == 0U);
        CHECK(m.result.friction_half_e == dir * 42);
    }
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

/** Verify restart resets every semantic field even when all storage previously held dirty bytes. */
static void test_dirty_start(void) {
    motor_ident_t clean = {0};
    motor_ident_t dirty;

    (void)memset(&dirty, 0xa5, sizeof(dirty));
    motor_ident_start(&clean);
    motor_ident_start(&dirty);
    /* Compare values rather than padding, whose representation is not part of the contract. */
    CHECK(dirty.result.state == clean.result.state);
    CHECK(dirty.result.fail_reason == clean.result.fail_reason);
    CHECK(dirty.result.enc_120 == clean.result.enc_120);
    CHECK(dirty.result.enc_240 == clean.result.enc_240);
    CHECK(dirty.result.enc_360 == clean.result.enc_360);
    CHECK(dirty.result.enc_return_120 == clean.result.enc_return_120);
    CHECK(dirty.result.enc_zero_back == clean.result.enc_zero_back);
    CHECK(dirty.result.friction_half_e == clean.result.friction_half_e);
    CHECK(dirty.result.delta_a == clean.result.delta_a);
    CHECK(dirty.result.delta_b == clean.result.delta_b);
    CHECK(dirty.result.direction == clean.result.direction);
    CHECK(dirty.result.offset_e == clean.result.offset_e);
    for (uint32_t k = 0U; k < IDENT_R_POINTS; k++) {
        CHECK(dirty.result.points[k].v == clean.result.points[k].v);
        CHECK(dirty.result.points[k].i == clean.result.points[k].i);
    }
    CHECK(dirty.result.r == clean.result.r);
    CHECK(dirty.result.v0 == clean.result.v0);
    CHECK(dirty.result.max_residual == clean.result.max_residual);
    CHECK(dirty.result.r_nonlinear == clean.result.r_nonlinear);
    CHECK(dirty.result.l == clean.result.l);
    CHECK(dirty.result.ipp == clean.result.ipp);
    CHECK(dirty.theta_e_count == clean.theta_e_count);
    CHECK(dirty.align_step == clean.align_step);
    CHECK(dirty.run_n == clean.run_n);
    CHECK(dirty.overcurrent_n == clean.overcurrent_n);
    CHECK(dirty.target_n == clean.target_n);
    CHECK(dirty.reached_n == clean.reached_n);
    CHECK(dirty.hold_n == clean.hold_n);
    CHECK(dirty.point == clean.point);
    CHECK(dirty.avg_n == clean.avg_n);
    CHECK(dirty.l_n == clean.l_n);
    CHECK(dirty.increments == clean.increments);
    CHECK(dirty.reached == clean.reached);
    CHECK(dirty.filtered_i == clean.filtered_i);
    CHECK(dirty.servo_v == clean.servo_v);
    CHECK(dirty.sum_v == clean.sum_v);
    CHECK(dirty.sum_i == clean.sum_i);
    CHECK(dirty.prev_i == clean.prev_i);
    CHECK(dirty.rot_phase == clean.rot_phase && dirty.rot_remainder == clean.rot_remainder);
    CHECK(dirty.rot_n == clean.rot_n && dirty.rot_backward == clean.rot_backward);
    CHECK(dirty.rot_sum_f == clean.rot_sum_f && dirty.rot_sum_b == clean.rot_sum_b);
    CHECK(dirty.result.offset_c == clean.result.offset_c && dirty.result.offset_load == clean.result.offset_load);
    CHECK(dirty.result.offset_e_new == clean.result.offset_e_new);
    CHECK(dirty.sum_delta == clean.sum_delta);
}

/** Check that publication copies every result value, including partial and terminal diagnostics. */
static void test_result_copy(void) {
    motor_ident_result_t src = {
        .state = IDENT_FAILED,
        .fail_reason = IDENT_FAIL_RIPPLE,
        .enc_120 = 1234U,
        .enc_240 = 1429U,
        .enc_360 = 1624U,
        .enc_return_120 = 1819U,
        .enc_zero_back = 1640U,
        .friction_half_e = 56,
        .delta_a = 195,
        .delta_b = 195,
        .direction = 1,
        .offset_e = 3520U,
        .offset_c = -157,
        .offset_load = 102,
        .offset_e_new = 3677U,
        .r = 0.1f,
        .v0 = 0.15f,
        .max_residual = 0.02f,
        .r_nonlinear = true,
        .l = 100e-6f,
        .ipp = 0.4f,
    };
    motor_ident_result_t dst;

    for (uint32_t k = 0U; k < IDENT_R_POINTS; k++) {
        src.points[k].i = 0.5f + (float)k * 0.25f;
        src.points[k].v = 0.15f + src.points[k].i * 0.1f;
    }
    (void)memset(&dst, 0xa5, sizeof(dst));
    motor_ident_result_copy(&dst, &src);
    CHECK(dst.state == src.state && dst.fail_reason == src.fail_reason);
    CHECK(dst.enc_120 == src.enc_120 && dst.enc_240 == src.enc_240 && dst.enc_360 == src.enc_360);
    CHECK(dst.enc_return_120 == src.enc_return_120 && dst.enc_zero_back == src.enc_zero_back);
    CHECK(dst.friction_half_e == src.friction_half_e);
    CHECK(dst.delta_a == src.delta_a && dst.delta_b == src.delta_b);
    CHECK(dst.offset_c == src.offset_c && dst.offset_load == src.offset_load && dst.offset_e_new == src.offset_e_new);
    CHECK(dst.direction == src.direction && dst.offset_e == src.offset_e);
    for (uint32_t k = 0U; k < IDENT_R_POINTS; k++) {
        CHECK(dst.points[k].i == src.points[k].i && dst.points[k].v == src.points[k].v);
    }
    CHECK(dst.r == src.r && dst.v0 == src.v0 && dst.max_residual == src.max_residual);
    CHECK(dst.r_nonlinear == src.r_nonlinear && dst.l == src.l && dst.ipp == src.ipp);
    src.state = IDENT_DONE;
    src.fail_reason = IDENT_FAIL_NONE;
    src.r_nonlinear = false;
    motor_ident_result_copy(&dst, &src);
    CHECK(dst.state == src.state && dst.fail_reason == src.fail_reason && !dst.r_nonlinear);
}

/** Demonstrate removal of an 84-detent cogging park bias with either encoder polarity. */
static void test_cogging_offset(void) {
    for (int direction = -1; direction <= 1; direction += 2) {
        /* Isolate the post-L refinement from the preceding direction/R/L
         * tests. Park in a physical cogging detent first; do not inject an
         * artificial encoder bias or snap the rotor to the command.
         * This waveform has 84 mechanical detents; its local equilibrium
         * displaces the zero park by about 10 electrical degrees.
         */
        plant_t p = {.r = 0.1,
                     .l = 100e-6,
                     .vdt = 0.15,
                     .direction = direction,
                     .enc0 = 3745U,
                     .cogging = 0.25,
                     .stiction = 0.01,
                     .rotor = -150.0,
                     .i = 1.0,
                     .pending_v = 0.25};
        for (uint32_t n = 0U; n < IDENT_ALIGN_HOLD_N; n++) {
            rotor_step(&p, 0U);
        }
        motor_ident_t m;
        motor_ident_out_t out;
        motor_ident_start(&m);
        m.result.state = IDENT_OFFSET_ROT;
        m.result.direction = direction;
        m.result.offset_e = ((uint32_t)direction * encoder_sample(&p) * IDENT_POLE_PAIRS) & 4095U;
        m.result.points[IDENT_R_BIAS_POINT].v = 0.25f;
        for (uint32_t n = 0U; n < 2U * IDENT_ROT_DIRECTION_N; n++) {
            motor_ident_in_t in = {.i_d = (float)p.i, .enc_count = encoder_sample(&p), .power_on = true};
            motor_ident_step(&m, &in, &out);
            if (out.done || out.failed) {
                break;
            }
            plant_step(&p, out.v_d, out.theta_e_count);
        }
        uint32_t expected = ((uint32_t)direction * p.enc0 * IDENT_POLE_PAIRS) & 4095U;
        double old_error = remainder((double)m.result.offset_e - expected, 4096.0);
        double new_error = remainder((double)m.result.offset_e_new - expected, 4096.0);
        printf("cogging dir %d: park error %.1f -> rotation error %.1f counts; c %d load %d, state %u fail %u\n",
               direction, old_error, new_error, m.result.offset_c, m.result.offset_load, (unsigned int)m.result.state,
               (unsigned int)m.result.fail_reason);
        CHECK(m.result.state == IDENT_DONE);
        CHECK(fabs(old_error) > 100.0);
        CHECK(fabs(new_error) < 30.0);
    }
}

/** Reproduce unequal bench-like moves with physical cogging torque, then refine both encoder polarities. */
static void test_strong_cogging_alignment(void) {
    for (int direction = -1; direction <= 1; direction += 2) {
        /* The 84-detent term alone repeats every 120 degrees. A zero-mean
         * lower spatial harmonic makes successive parks unequal without
         * injecting encoder errors or prescribing the rotor's position.
         * This is a regression plant, not a fitted model of the bench motor.
         */
        plant_t p = {.r = 0.1,
                     .l = 100e-6,
                     .vdt = 0.15,
                     .direction = direction,
                     .enc0 = 3745U,
                     .raw_base = 57344U,
                     .cogging = 0.093,
                     .cogging_modulation = 0.396,
                     .cogging_phase = 1.912,
                     .modulation_phase = 1.920,
                     .stiction = 0.055,
                     .rotor = -394.0};
        motor_ident_t m = run_plant(&p);
        uint32_t expected = ((uint32_t)direction * p.enc0 * IDENT_POLE_PAIRS) & 4095U;
        double error = remainder((double)m.result.offset_e_new - expected, (double)IDENT_COUNTS);
        double move_a = (double)(direction * m.result.delta_a) * IDENT_POLE_PAIRS;
        double move_b = (double)(direction * m.result.delta_b) * IDENT_POLE_PAIRS;

        printf("strong cogging dir %d: deltas %d %d (%.1f, %.1f deg), offset error %.1f counts, %.3f s\n", direction,
               m.result.delta_a, m.result.delta_b, move_a * 360.0 / IDENT_COUNTS, move_b * 360.0 / IDENT_COUNTS, error,
               (double)m.run_n * (double)IDENT_PERIOD_S);
        CHECK(m.result.state == IDENT_DONE && m.result.fail_reason == IDENT_FAIL_NONE);
        CHECK(m.result.direction == direction);
        CHECK_NEAR(move_a * 360.0 / IDENT_COUNTS, 148.0, 5.0);
        CHECK_NEAR(move_b * 360.0 / IDENT_COUNTS, 76.0, 5.0);
        CHECK(fabs(move_a - IDENT_ALIGN_120) > 200.0 && fabs(move_b - IDENT_ALIGN_120) > 200.0);
        CHECK(fabs(error) < 30.0);
        CHECK(m.run_n < IDENT_RUN_TIMEOUT_N);
    }
}

/** Verify correction sign, wrap, exact rotation timing, settling exclusion and every plausibility boundary. */
static void test_rotation_validation(void) {
    const int32_t cases[][3] = {{-157, 102, 1}, {157, 102, 1}, {600, 400, 1}, {-600, 0, 1}, {601, 0, 0},
                                {-601, 0, 0},   {0, -1, 0},    {0, 601, 0},   {0, 600, 1}};
    for (int32_t dir = -1; dir <= 1; dir += 2) {
        for (size_t k = 0U; k < sizeof(cases) / sizeof(cases[0]); k++) {
            motor_ident_t m;
            motor_ident_out_t out;
            motor_ident_start(&m);
            m.result.state = IDENT_OFFSET_ROT;
            m.result.direction = dir;
            m.result.offset_e = 4090U;
            m.result.points[IDENT_R_BIAS_POINT].v = 0.4f;
            for (uint32_t n = 0U; n < 2U * IDENT_ROT_DIRECTION_N; n++) {
                int32_t err = cases[k][0] + (m.rot_backward ? -cases[k][1] : cases[k][1]);
                if (m.rot_n < IDENT_ROT_SETTLE_N) {
                    err = 999;
                }
                uint32_t measured = (m.theta_e_count - (uint32_t)err) & 4095U;
                /* 7 inverse modulo 4096 = 3511: exact synthetic encoder count. */
                uint32_t enc = ((uint32_t)dir * (measured + m.result.offset_e) * 3511U) & 4095U;
                motor_ident_in_t in = {.i_d = 1.0f, .enc_count = enc, .power_on = true};
                motor_ident_step(&m, &in, &out);
                if (n + 1U < 2U * IDENT_ROT_DIRECTION_N) {
                    CHECK(!out.done && !out.failed);
                    CHECK_NEAR(out.v_d, 0.4, 1e-7);
                }
            }
            CHECK(out.done == (cases[k][2] != 0));
            CHECK(out.failed == (cases[k][2] == 0));
            CHECK(out.v_d == 0.0f && m.rot_phase == 0U && m.rot_remainder == 0);
            CHECK(m.result.offset_c == cases[k][0] && m.result.offset_load == cases[k][1]);
            if (out.done) {
                CHECK(m.result.offset_e_new == ((4090U - (uint32_t)cases[k][0]) & 4095U));
            } else {
                CHECK(m.result.fail_reason == IDENT_FAIL_ALIGNMENT);
            }
        }
    }
    const int32_t errors[] = {-1025, -1024, 1024, 1025};
    for (size_t k = 0U; k < sizeof(errors) / sizeof(errors[0]); k++) {
        motor_ident_t m;
        motor_ident_out_t out;
        motor_ident_start(&m);
        m.result.state = IDENT_OFFSET_ROT;
        m.result.direction = 1;
        motor_ident_in_t in = {.power_on = true, .enc_count = ((0U - (uint32_t)errors[k]) * 3511U) & 4095U};
        motor_ident_step(&m, &in, &out);
        CHECK(out.failed == (abs(errors[k]) > 1024));
        if (out.failed) {
            CHECK(m.result.fail_reason == IDENT_FAIL_ALIGNMENT && out.v_d == 0.0f);
        }
    }
}

/** Exercise production identification with reachable plants and explicit physical-limit failures. */
int main(void) {
    test_strong_cogging_alignment();
    test_cogging_offset();
    test_rotation_validation();
    test_dirty_start();
    test_result_copy();
    for (int direction = -1; direction <= 1; direction += 2) {
        /* 180 is unstable for the first park. The last start is exactly
         * 2048 counts from the quantized 120 command (299.970703125 degrees),
         * but the first park moves it away (see run_plant).
         */
        const double starts[] = {0.0, 90.0, 180.0, 270.0,
                                 ((double)IDENT_ALIGN_120 + 0.5 * (double)IDENT_COUNTS) * 360.0 / IDENT_COUNTS};

        for (uint32_t k = 0U; k < sizeof(starts) / sizeof(starts[0]); k++) {
            test_accuracy(0.1, 100e-6, direction, 3745U, starts[k]);
            test_accuracy(0.2, 300e-6, direction, direction > 0 ? 4000U : 50U, starts[k]);
        }
        /* 1 ohm needs >1.0 V even for ALIGN, before the unreachable 1.5 A R point. */
        test_unreachable(1.0, 1e-3, direction);
    }
    test_unreachable(5.0, 1e-3, 1);
    test_delay_association();
    test_r_spikes();
    test_overcurrent();
    test_safety();
    test_alignment_failures();
    test_alignment_validation();
    test_two_sided_validation();
    test_l_failures();
    test_nonlinear_resistance();
    return foc_test_result("test_motor_ident");
}
