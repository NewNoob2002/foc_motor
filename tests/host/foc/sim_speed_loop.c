/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Closed current-loop simulation of a PMSM on the host. No hardware.
 *
 * The controller part (current_loop_step) has the shape of the future FOC
 * ISR: sampled IA/IC + electrical angle in, PWM duties out, with a one-period
 * PWM update delay like TIM1 shadow registers. The plant uses its own double
 * precision math, independent of lib/foc.
 *
 * Usage: sim_current_loop            run and check, print a summary
 *        sim_current_loop --csv      also print CSV (see plot_sim.py)
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "foc/foc_pi.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

#define SIM_PI           3.14159265358979323846

/*
 * Illustrative 4006 KV380-class values, NOT measurements of the real motor
 * (see docs/hardware_reference.md section 10). psi follows from KV:
 * psi = 60 / (sqrt(3) * 2 * pi * KV * pole_pairs).
 */
#define MOTOR_POLE_PAIRS 7
#define MOTOR_R_OHM      0.17
#define MOTOR_L_H        100e-6
#define MOTOR_PSI_WB     (60.0 / (sqrt(3.0) * 2.0 * SIM_PI * 380.0 * MOTOR_POLE_PAIRS))
#define MOTOR_J_KGM2     1.5e-5
#define MOTOR_B_NMS      1e-6

#define VBUS_V           12.0f
#define PWM_HZ           20000.0
#define TS_S             (1.0 / PWM_HZ)
#define PLANT_SUBSTEPS   25
#define ENCODER_COUNTS   4096
#define SIM_END_S        0.200
#define CSV_DECIMATION   1

/*
 * R and L the controller believes in. Equal to the simulated motor by
 * default; change them alone to see what a wrong measurement does.
 */
#define TUNE_R_OHM       MOTOR_R_OHM
#define TUNE_L_H         MOTOR_L_H

/* Current-loop bandwidth; pole-zero cancellation tuning. */
#define LOOP_BW_HZ       1000.0
#define LOOP_KP          ((float)(TUNE_L_H * 2.0 * SIM_PI * LOOP_BW_HZ))
#define LOOP_KI          ((float)(TUNE_R_OHM * 2.0 * SIM_PI * LOOP_BW_HZ))

typedef struct {
    double id;
    double iq;
    double omega_m; /* rad/s mechanical */
    double theta_m; /* rad mechanical, unwrapped */
} motor_t;

typedef struct {
    foc_pi_t pi_d;
    foc_pi_t pi_q;
} current_loop_t;

static void motor_step(motor_t* m, foc_abc_t duty, double vbus, double dt) {
    double theta_e = MOTOR_POLE_PAIRS * m->theta_m;
    double omega_e = MOTOR_POLE_PAIRS * m->omega_m;
    double va = duty.a * vbus;
    double vb = duty.b * vbus;
    double vc = duty.c * vbus;
    double v_alpha = (2.0 * va - vb - vc) / 3.0;
    double v_beta = (vb - vc) / sqrt(3.0);
    double vd = v_alpha * cos(theta_e) + v_beta * sin(theta_e);
    double vq = -v_alpha * sin(theta_e) + v_beta * cos(theta_e);

    double did = (vd - MOTOR_R_OHM * m->id + omega_e * MOTOR_L_H * m->iq) / MOTOR_L_H;
    double diq = (vq - MOTOR_R_OHM * m->iq - omega_e * MOTOR_L_H * m->id - omega_e * MOTOR_PSI_WB) / MOTOR_L_H;
    double torque = 1.5 * MOTOR_POLE_PAIRS * MOTOR_PSI_WB * m->iq;
    double domega = (torque - MOTOR_B_NMS * m->omega_m) / MOTOR_J_KGM2;

    m->id += did * dt;
    m->iq += diq * dt;
    m->omega_m += domega * dt;
    m->theta_m += m->omega_m * dt;
}

static void motor_phase_currents(const motor_t* m, double* ia, double* ic) {
    double theta_e = MOTOR_POLE_PAIRS * m->theta_m;
    double i_alpha = m->id * cos(theta_e) - m->iq * sin(theta_e);
    double i_beta = m->id * sin(theta_e) + m->iq * cos(theta_e);

    *ia = i_alpha;
    *ic = -0.5 * i_alpha - 0.5 * sqrt(3.0) * i_beta;
}

/* Electrical angle as the TIM3 encoder would report it (quantized). */
static float encoder_theta_e(const motor_t* m) {
    double counts = floor(m->theta_m / (2.0 * SIM_PI) * ENCODER_COUNTS);
    double theta_m = counts * 2.0 * SIM_PI / ENCODER_COUNTS;

    return (float)fmod(MOTOR_POLE_PAIRS * theta_m, 2.0 * SIM_PI);
}

static foc_abc_t current_loop_step(current_loop_t* c, float ia, float ic, float theta_e, float vbus, foc_dq_t i_ref,
                                   foc_dq_t* i_meas, foc_dq_t* v_cmd) {
    foc_abc_t i_abc = {.a = ia, .b = -(ia + ic), .c = ic};
    foc_sincos_t sc = {.sin_theta = sinf(theta_e), .cos_theta = cosf(theta_e)};
    foc_dq_t i_dq = foc_park(foc_clarke(i_abc), sc);
    float vmax = vbus * 0.57735027f;

    /* d has priority; q gets what is left of the linear voltage circle. */
    c->pi_d.out_min = -vmax;
    c->pi_d.out_max = vmax;
    float vd = foc_pi_step(&c->pi_d, i_ref.d - i_dq.d, (float)TS_S);
    float vq_max = sqrtf(fmaxf(vmax * vmax - vd * vd, 0.0f));

    c->pi_q.out_min = -vq_max;
    c->pi_q.out_max = vq_max;
    float vq = foc_pi_step(&c->pi_q, i_ref.q - i_dq.q, (float)TS_S);

    foc_dq_t v_dq = {.d = vd, .q = vq};
    foc_abc_t duty;

    (void)foc_svpwm(foc_inv_park(v_dq, sc), vbus, &duty);
    *i_meas = i_dq;
    *v_cmd = v_dq;
    return duty;
}

// static foc_dq_t reference_at(double t) {
//     foc_dq_t ref = {.d = 0.0f, .q = 0.0f};

//     if (t >= 0.040) {
//         ref.q = -1.0f;
//     } else if (t >= 0.005) {
//         ref.q = 1.0f;
//     }
//     return ref;
// }

static float speed_reference_at(double t) {
    if (t >= 0.120) {
        return 100.0f;
    }
    if (t >= 0.005) {
        return 300.0f;
    }
    return 0.0f;
}

static bool finite_all(const double* v, int n) {
    for (int i = 0; i < n; i++) {
        if (!isfinite(v[i])) {
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    bool csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    motor_t motor = {0};
    current_loop_t loop = {
        .pi_d = {.kp = LOOP_KP, .ki = LOOP_KI},
        .pi_q = {.kp = LOOP_KP, .ki = LOOP_KI},
    };
    foc_abc_t duty_applied = {0.5f, 0.5f, 0.5f};
    int steps = (int)(SIM_END_S / TS_S);
    int failures = 0;
    double iq_peak = 0.0;
    double id_abs_max = 0.0;
    double iq_err_max = 0.0;

    foc_pi_reset(&loop.pi_d);
    foc_pi_reset(&loop.pi_q);
    /* Speed loop: iq -> speed is Kt / (J s); see the study guide for the tuning. */
    const double kt = 1.5 * MOTOR_POLE_PAIRS * MOTOR_PSI_WB; /* N·m per A */
    const double wsc = 2.0 * SIM_PI * 50.0;                  /* 50 Hz bandwidth */
    const double speed_kp = wsc * MOTOR_J_KGM2 / kt;
    foc_pi_t speed_pi = {
        .kp = (float)speed_kp,
        .ki = (float)(speed_kp * wsc / 4.0),
        .out_min = -3.0f, /* the PI output IS iq_ref, so these are the current limits */
        .out_max = 3.0f,
    };
    foc_pi_reset(&speed_pi);
    float theta_prev = encoder_theta_e(&motor); /* angle at the previous speed update */
    float omega_ref = 0.0f;
    float iq_cmd = 0.0f; /* held constant between speed updates */

    if (csv) {
        printf("t_s,id_ref,iq_ref,id,iq,vd,vq,duty_a,duty_b,duty_c,omega_m\n");
    }

    for (int k = 0; k < steps; k++) {
        double t = k * TS_S;
        double ia;
        double ic;
        if (k % 20 == 0) {
            float theta = encoder_theta_e(&motor);
            float dtheta = theta - theta_prev;
            /* Wrap dtheta into (-pi, pi] */
            if (dtheta <= -SIM_PI) {
                dtheta += 2.0f * (float)SIM_PI;
            }
            if (dtheta > SIM_PI) {
                dtheta -= 2.0f * (float)SIM_PI;
            }
            theta_prev = theta;
            float omega_meas = (float)(dtheta / (20.0 * TS_S) / MOTOR_POLE_PAIRS);
            omega_ref = speed_reference_at(t);
            iq_cmd = foc_pi_step(&speed_pi, omega_ref - omega_meas, (float)(TS_S * 20.0));
        }
        foc_dq_t i_ref = {.d = 0.0f, .q = iq_cmd};
        foc_dq_t i_meas;
        foc_dq_t v_cmd;

        motor_phase_currents(&motor, &ia, &ic);
        foc_abc_t duty =
            current_loop_step(&loop, (float)ia, (float)ic, encoder_theta_e(&motor), VBUS_V, i_ref, &i_meas, &v_cmd);

        /* This period still runs with last period's shadow-register duties. */
        for (int s = 0; s < PLANT_SUBSTEPS; s++) {
            motor_step(&motor, duty_applied, VBUS_V, TS_S / PLANT_SUBSTEPS);
        }
        duty_applied = duty;

        double sample[] = {motor.id, motor.iq, motor.omega_m, v_cmd.d, v_cmd.q, duty.a, duty.b, duty.c};

        if (!finite_all(sample, (int)(sizeof(sample) / sizeof(sample[0])))) {
            fprintf(stderr, "non-finite value at t=%.6f s\n", t);
            return 1;
        }
        if (duty.a < 0.0f || duty.a > 1.0f || duty.b < 0.0f || duty.b > 1.0f || duty.c < 0.0f || duty.c > 1.0f) {
            failures++;
        }
        if (t >= 0.005 && t < 0.040 && motor.iq > iq_peak) {
            iq_peak = motor.iq;
        }
        if (fabs(motor.id) > id_abs_max) {
            id_abs_max = fabs(motor.id);
        }
        /* Settled windows: 2 ms after each reference step. */
        if ((t >= 0.007 && t < 0.040) || t >= 0.042) {
            double err = fabs(motor.iq - i_ref.q);

            if (err > iq_err_max) {
                iq_err_max = err;
            }
        }
        if (csv && k % CSV_DECIMATION == 0) {
            printf("%.6f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f\n", t, (double)i_ref.d, (double)i_ref.q,
                   motor.id, motor.iq, (double)v_cmd.d, (double)v_cmd.q, (double)duty.a, (double)duty.b, (double)duty.c,
                   motor.omega_m);
        }
    }

    FILE* out = csv ? stderr : stdout;

    fprintf(out, "kp=%.4f V/A ki=%.1f V/(A*s) bw=%.0f Hz\n", (double)LOOP_KP, (double)LOOP_KI, LOOP_BW_HZ);
    fprintf(out, "iq peak after +1 A step: %.4f A\n", iq_peak);
    fprintf(out, "max |iq - iq_ref| when settled: %.4f A\n", iq_err_max);
    fprintf(out, "max |id|: %.4f A\n", id_abs_max);
    fprintf(out, "final speed: %.2f rad/s mech\n", motor.omega_m);

    if (failures != 0) {
        fprintf(stderr, "FAIL: %d period(s) with duty outside [0, 1]\n", failures);
    }
    if (iq_peak < 0.9 || iq_peak > 1.25) {
        fprintf(stderr, "FAIL: iq does not track the step (peak %.4f A)\n", iq_peak);
        failures++;
    }
    if (iq_err_max > 0.05) {
        fprintf(stderr, "FAIL: settled iq error %.4f A > 0.05 A\n", iq_err_max);
        failures++;
    }
    /*
	 * The +1 A -> -1 A step at ~350 rad/s electrical disturbs id through
	 * the omega_e * L * iq cross-coupling (~0.1 A here without decoupling).
	 */
    if (id_abs_max > 0.15) {
        fprintf(stderr, "FAIL: |id| reached %.4f A > 0.15 A\n", id_abs_max);
        failures++;
    }

    return failures == 0 ? 0 : 1;
}
