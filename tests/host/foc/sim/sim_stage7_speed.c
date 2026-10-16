/* SPDX-License-Identifier: Apache-2.0 */
/* Independent double-precision PMSM plant; production float speed/current kernels. */
#include <math.h>
#include <stdio.h>

#include "cog_map.h"
#include "cog_proc.h"
#include "foc/foc_current.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"
#include "foc_speed.h"
#include "foc_test.h"

#define PI               3.14159265358979323846
#define TAU              (2.0 * PI)
#define DT               0.00005
#define KT               0.0217
#define PSI              (KT / 10.5)
#define RIPPLE_BOUND_RPM 8.0

static double cog14_torque;
/* 84/turn slot cogging amplitude. Stage 8 run 2 stuck at one position with
 * iq at the 0.5 A map cap (about 11 mNm), so the bench case uses more.
 */
static double cog84_torque = 0.002;
/* Breakaway torque at rest; 0.5 mNm (= Coulomb) for the stage 7 cases. Stage 8
 * uses 2 mNm: on the bench 50 mA (about 1.1 mNm) did not move the rotor, and the
 * 10 rpm speed-loop map went stick-slip.
 */
static double stiction_torque = 0.0005;

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
        double torque =
            KT * p->iq - 1e-6 * p->omega - cog84_torque * sin(84.0 * p->theta) - cog14_torque * sin(14.0 * p->theta);
        /* Coulomb friction opposes motion; static friction balances torque at rest. */
        if (fabs(p->omega) < 1e-5 && fabs(torque) <= stiction_torque) {
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

/** Production-kernel controller and independent plant for the Stage 8 experiment. */
typedef struct {
    plant_t plant;
    foc_ctrl_speed_t estimator;
    foc_speed_loop_t speed;
    foc_current_t cc;
    foc_abc_t applied;
    float rpm;
    double inertia;
} experiment_t;

/** Reset all control history and PWM preload for an independent comparison. */
static void experiment_init(experiment_t* e) {
    *e = (experiment_t){0};
    e->applied = (foc_abc_t){.a = 0.5f, .b = 0.5f, .c = 0.5f};
    e->inertia = 1.5e-5;
    e->cc.pi_d.kp = 70e-6f * 6283.18530718f;
    e->cc.pi_d.ki = 0.2f * 6283.18530718f;
    e->cc.pi_q = e->cc.pi_d;
    foc_ctrl_speed_reset(&e->estimator, 0U);
    foc_speed_loop_reset(&e->speed);
}

/** Return the same 12-bit position presented to the firmware. */
static uint32_t experiment_encoder(const experiment_t* e) {
    return (uint32_t)(int64_t)floor(e->plant.theta * 4096.0 / TAU) & 4095U;
}

/** Execute one period using the production map, speed, current and interpolation functions. */
static void experiment_step(experiment_t* e, cog_map_t* map, float target, const int16_t* table) {
    uint32_t enc = experiment_encoder(e);
    bool tick = foc_ctrl_speed_step(&e->estimator, enc);
    if (tick) {
        e->rpm = (float)e->estimator.counts * FOC_CTRL_SPD_COUNTS_TO_RPM;
    }
    foc_abc_t currents = phase_currents(&e->plant);
    if (map != NULL) {
        foc_ab_t ab = foc_clarke(currents);
        cog_map_in_t in = {.enc = enc, .alpha = ab.alpha, .beta = ab.beta, .rpm = e->rpm, .power = true, .armed = true};
        cog_map_step(map, &in);
    }
    bool speed_mode = map == NULL;
    bool current_mode = speed_mode || cog_map_current(map);
    if (tick && speed_mode) {
        foc_speed_loop_update(&e->speed, target, e->rpm, false);
    }
    uint32_t angle_count = current_mode ? (enc * 7U) & 4095U : cog_map_angle(map);
    double angle = angle_count * TAU / 4096.0;
    foc_sincos_t sc = {.sin_theta = (float)sin(angle), .cos_theta = (float)cos(angle)};
    foc_current_out_t out;
    if (current_mode) {
        /* Same clamp as foc_ctrl.c: the map's own limit in phase B, feed-forward only in speed mode. */
        float iq = cog_ff_apply(speed_mode ? e->speed.iq : map->iq_cmd, speed_mode ? 0.5f : COG_POS_I_MAX,
                                speed_mode && table != NULL, table != NULL, table, enc);
        foc_dq_t ref = {.q = iq};
        foc_ab_t comp = foc_deadtime_comp(foc_inv_clarke(foc_inv_park(ref, sc)), 0.175f, 0.05f);
        foc_current_step_ff(&e->cc, currents, sc, ref, 15.0f, (float)DT, comp, 1.0f, &out);
    } else {
        foc_dq_t voltage = {.d = map->voltage};
        (void)foc_svpwm(foc_inv_park(voltage, sc), 15.0f, &out.duty);
    }
    plant_step(&e->plant, e->applied, e->inertia);
    e->applied = out.duty;
    CHECK(isfinite(e->plant.omega) && fabs(e->plant.iq) < 2.0);
}

/** Speed ripple RMS and its position-locked amplitudes at mechanical orders 14 and 84. */
typedef struct {
    double rms;
    double order14;
    double order84;
} ripple_t;

/** Measure steady speed ripple over four seconds after a two-second settle. */
static ripple_t experiment_ripple(float target, const int16_t* table) {
    experiment_t e;
    experiment_init(&e);
    double sum = 0.0;
    double square = 0.0;
    /* Sums of rpm*cos, rpm*sin, cos, sin for orders 14 and 84. Since d(theta)/dt = omega,
     * the time average of omega*exp(-jk*theta) cancels over whole turns: remove the mean.
     */
    double h[2][4] = {{0.0}};
    unsigned int count = 0U;
    for (uint32_t n = 0U; n < 120000U; n++) {
        experiment_step(&e, NULL, target, table);
        if (n >= 40000U) {
            double rpm = e.plant.omega * 60.0 / TAU;
            sum += rpm;
            square += rpm * rpm;
            for (unsigned int k = 0U; k < 2U; k++) {
                double angle = (k == 0U ? 14.0 : 84.0) * e.plant.theta;
                h[k][0] += rpm * cos(angle);
                h[k][1] += rpm * sin(angle);
                h[k][2] += cos(angle);
                h[k][3] += sin(angle);
            }
            count++;
        }
    }
    CHECK(fabs(sum / count - target) < 1.0);
    double mean = sum / count;
    ripple_t r = {.rms = sqrt(fmax(0.0, square / count - mean * mean)),
                  .order14 = 2.0 * hypot(h[0][0] - mean * h[0][2], h[0][1] - mean * h[0][3]) / count,
                  .order84 = 2.0 * hypot(h[1][0] - mean * h[1][2], h[1][1] - mean * h[1][3]) / count};
    return r;
}

/** Print off/on ripple; return the order-14 reduction, the component the stage 7 bench saw. */
static double ripple_compare(const char* label, uint32_t rpm, const int16_t* table) {
    ripple_t off = experiment_ripple((float)rpm, NULL);
    ripple_t on = experiment_ripple((float)rpm, table);
    printf("%s %u rpm ripple RMS: off %.3f rpm, on %.3f rpm, reduction %.2fx; order 14 %.3f -> %.3f rpm, "
           "order 84 %.3f -> %.3f rpm\n",
           label, rpm, off.rms, on.rms, off.rms / on.rms, off.order14, on.order14, off.order84, on.order84);
    return off.order14 / on.order14;
}

/** Run the full mode-8 sequencer on a plant with the given inertia; return the processed map. */
static bool run_map(double inertia, cog_processed_t* processed) {
    experiment_t e;
    static cog_map_t map;
    experiment_init(&e);
    e.inertia = inertia;
    map = (cog_map_t){0};
    CHECK(cog_map_start(&map, true, true, 0.434f, 1, 0U));
    double err_max = 0.0;
    for (uint32_t n = 0U; n <= COG_TIMEOUT_N && cog_map_get(&map) == NULL; n++) {
        experiment_step(&e, &map, 0.0f, NULL);
        if (cog_map_current(&map)) {
            err_max = fmax(err_max, fabs(map.tgt_q16 / 65536.0 - map.pos));
        }
    }
    printf("sim map J %.1e: phase %u reason %u duration %.3f s, max position error %.1f counts\n", inertia,
           map.raw.phase, map.raw.reason, map.ticks * DT, err_max);
    return cog_map_get(&map) != NULL && map.raw.phase == COG_DONE && cog_process(&map.raw, processed);
}

/** Map with stiction at three inertias, then compare identical off/on plants at the nominal one. */
static void run_cogging_experiment(void) {
    static cog_processed_t processed;
    static cog_processed_t other;
    cog14_torque = 0.003;
    stiction_torque = 0.002;
    CHECK(run_map(0.5e-5, &other));
    CHECK(run_map(4.5e-5, &other));
    /* Bench-like: 84/turn cogging 8 mNm + 4 mNm breakaway (12 mNm static, 0.55 A at Kt). */
    cog84_torque = 0.008;
    stiction_torque = 0.004;
    CHECK(run_map(1.5e-5, &other));
    printf("sim map (cog84 8 mNm, stiction 4 mNm) harmonic 14: %.1f mA (expected %.1f mA)\n",
           (double)other.harmonic[3].amplitude_ma, 0.003 / KT * 1000.0);
    for (uint32_t rpm = 60U; rpm <= 100U; rpm += 40U) {
        /* The 84/turn term is barely resolved by 256 bins (3 per detent); order 14 is the bench ripple. */
        CHECK(ripple_compare("bench-like", rpm, other.ff) >= 3.0);
    }
    cog84_torque = 0.002;
    stiction_torque = 0.002;
    CHECK(run_map(1.5e-5, &processed));
    printf("sim map harmonic 14: %.1f mA (expected %.1f mA)\n", (double)processed.harmonic[3].amplitude_ma,
           0.003 / KT * 1000.0);
    for (uint32_t rpm = 60U; rpm <= 100U; rpm += 40U) {
        CHECK(ripple_compare("Stage 8", rpm, processed.ff) >= 3.0);
    }
}

/** Compare nominal tuning with inertia variation, then map and compensate cogging. */
int main(void) {
    run_case(1.5e-5);
    run_case(0.5e-5);
    run_case(4.5e-5);
    run_cogging_experiment();
    return foc_test_result("sim_stage7_speed");
}
