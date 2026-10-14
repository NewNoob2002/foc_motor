/* SPDX-License-Identifier: Apache-2.0 */

/*
 * SVPWM exercise visualizer on the host. No hardware.
 *
 * Sweeps a voltage vector through one electrical turn at several lengths,
 * runs foc_svpwm() and reports which stage of exercise 2 already works
 * (docs/study/foc_study_guide.md). test_svpwm stays the pass gate; this program
 * only shows progress.
 *
 * Usage: sim_svpwm          print the stage checklist
 *        sim_svpwm --csv    also print CSV (see plot_svpwm.py)
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

#define SIM_PI   3.14159265358979323846
#define VBUS_V   12.0f
#define TOL_V    1e-3 /* applied voltage error, V */
#define TOL_DUTY 1e-5

/*
 * Vector lengths as a fraction of the linear limit Vbus / sqrt(3).
 * 0.866 = Vbus / 2, the limit of sine PWM without zero-sequence injection.
 */
static const double plot_ratios[] = {0.5, 0.866, 1.0, 1.3};

typedef struct {
    foc_ab_t cmd;
    bool ret;
    foc_abc_t duty;
    foc_ab_t out; /* voltage the bridge applies, back in alpha/beta */
} point_t;

/* First failure of a stage, printed as a debugging hint. */
typedef struct {
    int fails;
    char hint[160];
} stage_t;

static double vlim(void) {
    return (double)VBUS_V / sqrt(3.0);
}

static point_t run(double ratio, int theta_deg) {
    double mag = ratio * vlim();
    double theta = SIM_PI * theta_deg / 180.0;
    point_t p = {.cmd = {(float)(mag * cos(theta)), (float)(mag * sin(theta))}};

    p.ret = foc_svpwm(p.cmd, VBUS_V, &p.duty);
    p.out = foc_clarke((foc_abc_t){p.duty.a * VBUS_V, p.duty.b * VBUS_V, p.duty.c * VBUS_V});
    return p;
}

/* Written as !(in range) so NaN duties fail too. */
static bool duty_bad(float d) {
    return !((double)d >= -TOL_DUTY && (double)d <= 1.0 + TOL_DUTY);
}

static void fail(stage_t* s, int theta_deg, const char* msg) {
    if (s->fails++ == 0) {
        snprintf(s->hint, sizeof(s->hint), "theta=%d deg: %s", theta_deg, msg);
    }
}

static void check_point(stage_t* s, const point_t* p, int theta_deg, foc_ab_t expect, bool expect_ret) {
    const float d[3] = {p->duty.a, p->duty.b, p->duty.c};
    char msg[120];

    for (int k = 0; k < 3; k++) {
        if (duty_bad(d[k])) {
            snprintf(msg, sizeof(msg), "duty_%c = %.4f is outside [0, 1]", 'a' + k, (double)d[k]);
            fail(s, theta_deg, msg);
            return;
        }
    }
    if (fabs((double)(p->out.alpha - expect.alpha)) > TOL_V || fabs((double)(p->out.beta - expect.beta)) > TOL_V) {
        snprintf(msg, sizeof(msg), "applied (%.3f, %.3f) V, expected (%.3f, %.3f) V", (double)p->out.alpha,
                 (double)p->out.beta, (double)expect.alpha, (double)expect.beta);
        fail(s, theta_deg, msg);
        return;
    }
    if (p->ret != expect_ret) {
        snprintf(msg, sizeof(msg), "returned %s, expected %s", p->ret ? "true" : "false",
                 expect_ret ? "true" : "false");
        fail(s, theta_deg, msg);
    }
}

/* Inside the linear range: applied voltage equals the command, returns false. */
static stage_t stage_linear(double ratio) {
    stage_t s = {0};

    for (int deg = 0; deg < 360; deg++) {
        point_t p = run(ratio, deg);

        check_point(&s, &p, deg, p.cmd, false);
    }
    return s;
}

static stage_t stage_sine(void) {
    /* Just below Vbus / 2, so plain sine PWM fits. */
    return stage_linear(0.85);
}

static stage_t stage_minmax(void) {
    stage_t s = stage_linear(0.999);

    /* Zero vectors split evenly: max and min duty centered on 0.5. */
    for (int deg = 0; deg < 360; deg++) {
        point_t p = run(0.5, deg);
        double hi = fmax(p.duty.a, fmax(p.duty.b, p.duty.c));
        double lo = fmin(p.duty.a, fmin(p.duty.b, p.duty.c));

        if (!(fabs(0.5 * (hi + lo) - 0.5) <= 1e-4)) {
            char msg[64];

            snprintf(msg, sizeof(msg), "(max + min) / 2 = %.4f at |v| = 0.5 limit, expected 0.5", 0.5 * (hi + lo));
            fail(&s, deg, msg);
        }
    }
    return s;
}

static stage_t stage_overmod(void) {
    stage_t s = {0};
    const double ratios[] = {1.3, 2.0};

    for (unsigned int i = 0; i < sizeof(ratios) / sizeof(ratios[0]); i++) {
        for (int deg = 0; deg < 360; deg++) {
            point_t p = run(ratios[i], deg);
            double theta = SIM_PI * deg / 180.0;
            foc_ab_t expect = {(float)(vlim() * cos(theta)), (float)(vlim() * sin(theta))};

            check_point(&s, &p, deg, expect, true);
        }
    }
    return s;
}

static stage_t stage_bad_bus(void) {
    stage_t s = {0};
    const float bad[] = {0.0f, -5.0f, NAN};

    for (unsigned int i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        foc_abc_t d = {0};
        bool ret = foc_svpwm((foc_ab_t){1.0f, 1.0f}, bad[i], &d);

        if (!ret || fabsf(d.a - 0.5f) > 1e-6f || fabsf(d.b - 0.5f) > 1e-6f || fabsf(d.c - 0.5f) > 1e-6f) {
            s.fails++;
            if (s.fails == 1) {
                snprintf(s.hint, sizeof(s.hint), "vbus=%g: returned %s, duty (%.3f, %.3f, %.3f)", (double)bad[i],
                         ret ? "true" : "false", (double)d.a, (double)d.b, (double)d.c);
            }
        }
    }
    return s;
}

static void print_csv(void) {
    printf("ratio,theta_deg,ret,v_alpha,v_beta,duty_a,duty_b,duty_c,out_alpha,out_beta\n");
    for (unsigned int i = 0; i < sizeof(plot_ratios) / sizeof(plot_ratios[0]); i++) {
        for (int deg = 0; deg <= 360; deg++) {
            point_t p = run(plot_ratios[i], deg);

            printf("%.3f,%d,%d,%.5f,%.5f,%.6f,%.6f,%.6f,%.5f,%.5f\n", plot_ratios[i], deg, p.ret ? 1 : 0,
                   (double)p.cmd.alpha, (double)p.cmd.beta, (double)p.duty.a, (double)p.duty.b, (double)p.duty.c,
                   (double)p.out.alpha, (double)p.out.beta);
        }
    }
}

int main(int argc, char** argv) {
    bool csv = argc > 1 && strcmp(argv[1], "--csv") == 0;
    FILE* out = csv ? stderr : stdout;
    const struct {
        const char* name;
        stage_t (*run)(void);
    } stages[] = {
        {"Step 1  sine PWM: |v| <= Vbus/2 is applied exactly", stage_sine},
        {"Step 2  min-max injection: |v| up to Vbus/sqrt(3), centered", stage_minmax},
        {"Step 3  over-modulation: scaled to Vbus/sqrt(3), returns true", stage_overmod},
        {"Step 4  invalid vbus: duties 0.5, returns true", stage_bad_bus},
    };
    int failed = 0;

    if (csv) {
        print_csv();
    }

    fprintf(out, "SVPWM progress (Vbus = %.1f V, linear limit %.3f V)\n", (double)VBUS_V, vlim());
    for (unsigned int i = 0; i < sizeof(stages) / sizeof(stages[0]); i++) {
        stage_t s = stages[i].run();

        fprintf(out, "[%s] %s\n", s.fails == 0 ? "PASS" : "FAIL", stages[i].name);
        if (s.fails != 0) {
            fprintf(out, "       %d failing point(s); first: %s\n", s.fails, s.hint);
            failed++;
        }
    }
    return failed == 0 ? 0 : 1;
}
