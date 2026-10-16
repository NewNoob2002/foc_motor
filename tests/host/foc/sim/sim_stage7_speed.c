/* SPDX-License-Identifier: Apache-2.0 */
/* Independent double-precision PMSM plant; production float speed/current kernels. */
#include <math.h>
#include <stdio.h>

#include "foc/foc_current.h"
#include "foc/foc_transform.h"
#include "foc_speed.h"
#include "foc_test.h"

#define PI               3.14159265358979323846
#define DT               0.00005
#define KT               0.0217
#define PSI              (KT / 10.5)
#define RIPPLE_BOUND_RPM 8.0

typedef struct {
    double id;
    double iq;
    double omega;
    double theta;
} plant_t;

/** Convert dq currents to physical phase currents using the unquantized rotor. */
static foc_abc_t phase_currents(const plant_t* p) {
    double a = p->id * cos(7.0 * p->theta) - p->iq * sin(7.0 * p->theta);
    double b = p->id * sin(7.0 * p->theta) + p->iq * cos(7.0 * p->theta);
    foc_abc_t phase = {
        .a = (float)a, .b = (float)(-0.5 * a + sqrt(3.0) * 0.5 * b), .c = (float)(-0.5 * a - sqrt(3.0) * 0.5 * b)};
    return phase;
}

/** Model the inverter voltage loss with a narrow continuous zero-current transition. */
static double phase_loss(double current) {
    return 0.175 * fmax(-1.0, fmin(1.0, current / 0.001));
}

/** Integrate electrical cross coupling, back EMF and loaded mechanics at 1 us resolution. */
static void plant_step(plant_t* p, foc_abc_t duty, double inertia) {
    const double dt = DT / 50.0;
    for (unsigned int n = 0U; n < 50U; n++) {
        foc_abc_t phase = phase_currents(p);
        double va = 15.0 * duty.a - phase_loss(phase.a);
        double vb = 15.0 * duty.b - phase_loss(phase.b);
        double vc = 15.0 * duty.c - phase_loss(phase.c);
        double alpha = (2.0 * va - vb - vc) / 3.0;
        double beta = (vb - vc) / sqrt(3.0);
        double s = sin(7.0 * p->theta);
        double c = cos(7.0 * p->theta);
        double vd = alpha * c + beta * s;
        double vq = -alpha * s + beta * c;
        double did = (vd - 0.2 * p->id + 7.0 * p->omega * 70e-6 * p->iq) / 70e-6;
        double diq = (vq - 0.2 * p->iq - 7.0 * p->omega * (70e-6 * p->id + PSI)) / 70e-6;
        double torque = KT * p->iq - 1e-6 * p->omega - 0.002 * sin(84.0 * p->theta);
        /* Coulomb friction opposes motion; static friction balances torque at rest. */
        if (fabs(p->omega) < 1e-5 && fabs(torque) <= 0.0005) {
            p->omega = 0.0;
            torque = 0.0;
        } else {
            torque -= copysign(0.0005, fabs(p->omega) < 1e-5 ? torque : p->omega);
        }
        p->id += did * dt;
        p->iq += diq * dt;
        p->omega += torque / inertia * dt;
        p->theta += p->omega * dt;
    }
}

/** Run sequential 0->100 and 100->150 rpm steps and assert independent response metrics. */
static void run_case(double inertia) {
    plant_t plant = {0};
    foc_ctrl_speed_t estimate;
    foc_speed_loop_t speed;
    foc_current_t cc = {.pi_d = {.kp = 70e-6f * 6283.18530718f, .ki = 0.2f * 6283.18530718f},
                        .pi_q = {.kp = 70e-6f * 6283.18530718f, .ki = 0.2f * 6283.18530718f}};
    foc_abc_t applied = {.a = 0.5f, .b = 0.5f, .c = 0.5f};
    foc_ctrl_speed_reset(&estimate, 0U);
    foc_speed_loop_reset(&speed);
    for (unsigned int stage = 0U; stage < 2U; stage++) {
        double initial = stage == 0U ? 0.0 : 100.0;
        double target = stage == 0U ? 100.0 : 150.0;
        double t10 = -1.0;
        double t90 = -1.0;
        double peak = 0.0;
        double iq_ref_peak = 0.0;
        double iq_peak = 0.0;
        double sum = 0.0;
        double square = 0.0;
        double last_mean_error_time = 0.0;
        double block_sum = 0.0;
        double last_block_mean = 0.0;
        unsigned int count = 0U;
        for (unsigned int n = 0U; n < 20000U; n++) {
            double t = (double)n * DT;
            int64_t count_raw = (int64_t)floor(plant.theta * 4096.0 / (2.0 * PI));
            uint32_t enc = (uint32_t)count_raw & 4095U;
            if (foc_ctrl_speed_step(&estimate, enc)) {
                foc_speed_loop_update(&speed, (float)target, (float)estimate.counts * FOC_CTRL_SPD_COUNTS_TO_RPM, true);
            }
            double angle = (double)((enc * 7U) & 4095U) * (2.0 * PI / 4096.0);
            foc_sincos_t sc = {.sin_theta = (float)sin(angle), .cos_theta = (float)cos(angle)};
            foc_abc_t current = phase_currents(&plant);
            foc_dq_t ref = {.d = 0.0f, .q = speed.iq};
            foc_ab_t comp = foc_deadtime_comp(foc_inv_clarke(foc_inv_park(ref, sc)), 0.175f, 0.05f);
            foc_current_out_t out;
            foc_current_step_ff(&cc, current, sc, ref, 15.0f, (float)DT, comp, 1.0f, &out);
            plant_step(&plant, applied, inertia); /* Exactly one PWM-period delay. */
            applied = out.duty;
            double rpm = plant.omega * 60.0 / (2.0 * PI);
            CHECK(isfinite(rpm) && isfinite(plant.id) && isfinite(plant.iq));
            CHECK(fabsf(speed.iq) <= FOC_CTRL_SPD_IQ_MAX_A);
            CHECK(fabs(plant.iq) <= FOC_CTRL_SPD_IQ_MAX_A);
            CHECK(estimate.trip_counts >= -205 && estimate.trip_counts <= 205);
            CHECK(hypotf(out.v_dq.d, out.v_dq.q) <= 1.00001f);
            if (t10 < 0.0 && rpm >= initial + 0.1 * (target - initial)) {
                t10 = t;
            }
            if (t90 < 0.0 && rpm >= initial + 0.9 * (target - initial)) {
                t90 = t;
            }
            peak = fmax(peak, rpm);
            iq_ref_peak = fmax(iq_ref_peak, (double)fabsf(speed.iq));
            iq_peak = fmax(iq_peak, fabs(plant.iq));
            /* Settling is defined on non-overlapping 20 ms means to separate cogging ripple. */
            block_sum += rpm;
            if ((n + 1U) % 400U == 0U) {
                last_block_mean = block_sum / 400.0;
                if (fabs(last_block_mean - target) > 5.0) {
                    last_mean_error_time = t + DT;
                }
                block_sum = 0.0;
            }
            if (t >= 0.3) {
                sum += rpm;
                square += rpm * rpm;
                count++;
            }
        }
        double mean = sum / count;
        double ripple = sqrt(fmax(0.0, square / count - mean * mean));
        double overshoot = fmax(0.0, (peak - target) / (target - initial) * 100.0);
        printf("J %.2e, %.0f->%.0f rpm: rise10-90 %.2f ms, overshoot %.2f %%, settle(mean20ms) %.1f ms, "
               "mean %.2f rpm, ripple RMS %.2f rpm (bound %.1f), peak |iq_ref| %.4f A, |iq| %.4f A\n",
               inertia, initial, target, (t90 - t10) * 1000.0, overshoot, last_mean_error_time * 1000.0, mean, ripple,
               RIPPLE_BOUND_RPM, iq_ref_peak, iq_peak);
        CHECK(t90 > t10 && t10 >= 0.0);
        CHECK(fabs(mean - target) <= 5.0 && fabs(last_block_mean - target) <= 5.0);
        CHECK(ripple < RIPPLE_BOUND_RPM);
        if (inertia == 1.5e-5) {
            CHECK(t90 - t10 < 0.1);
            CHECK(overshoot < 25.0);
            CHECK(last_mean_error_time <= 0.3);
        }
    }
}

/** Compare nominal tuning with one-third and three-times true inertia. */
int main(void) {
    run_case(1.5e-5);
    run_case(0.5e-5);
    run_case(4.5e-5);
    return foc_test_result("sim_stage7_speed");
}
