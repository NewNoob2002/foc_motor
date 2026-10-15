/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_bench.h"

#include "cycles.h"
#include "foc_vectors.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "foc/foc_pi.h"
#include "foc/foc_svpwm.h"
#include "foc/foc_transform.h"

LOG_MODULE_REGISTER(foc_bench, LOG_LEVEL_INF);

/* Pass 0 runs with a cold flash cache; passes 1.. are the warm figures. */
#define BENCH_PASSES       8U

/*
 * One result line is about 150 characters, 1.6 ms at 921600 baud. Pausing
 * between lines keeps the deferred log buffer (1 KiB) from dropping them.
 */
#define DUMP_PAUSE_MS      3

/* Stage 5 "done when": one step well under 20 us at 170 MHz. */
#define STEP_BUDGET_CYCLES (20U * CYCLES_PER_US)

#define INV_SQRT3          0.57735027f

typedef enum {
    STAGE_EMPTY,      /* call overhead of a timed stage, nothing inside */
    STAGE_SINCOS,     /* sinf + cosf */
    STAGE_CLARKE,     /* b = -(a + c), Clarke */
    STAGE_PARK,       /* Park */
    STAGE_PI,         /* two PI steps, d and q */
    STAGE_INV_PARK,   /* inverse Park */
    STAGE_SVPWM,      /* SVPWM */
    STAGE_STEP,       /* foc_vector_step: all of the above in one call */
    STAGE_SINCOS_DBL, /* sin + cos in double, for comparison */
    STAGE_COUNT,
} stage_t;

static const char* const stage_names[STAGE_COUNT] = {
    [STAGE_EMPTY] = "empty call", [STAGE_SINCOS] = "sinf+cosf", [STAGE_CLARKE] = "clarke",
    [STAGE_PARK] = "park",        [STAGE_PI] = "2x pi",         [STAGE_INV_PARK] = "inv park",
    [STAGE_SVPWM] = "svpwm",      [STAGE_STEP] = "full step",   [STAGE_SINCOS_DBL] = "sin+cos double",
};

typedef struct {
    uint32_t min;
    uint32_t max_warm;
    uint32_t max_cold;
} span_t;

/* Values passed between the timed stages; static so every stage writes memory the barriers cover. */
static struct {
    const foc_vector_in_t* in;
    foc_sincos_t sc;
    foc_ab_t i_ab;
    foc_dq_t i_dq;
    foc_dq_t v_dq;
    foc_ab_t v_ab;
    foc_abc_t duty;
    bool limited;
    foc_vector_out_t out;
    float sink;
} ctx;

static span_t spans[STAGE_COUNT];

static __noinline void run_empty(void) {
    compiler_barrier();
}

static __noinline void run_sincos(void) {
    ctx.sc.sin_theta = sinf(ctx.in->theta);
    ctx.sc.cos_theta = cosf(ctx.in->theta);
}

static __noinline void run_clarke(void) {
    foc_abc_t i_abc = {.a = ctx.in->ia, .b = -(ctx.in->ia + ctx.in->ic), .c = ctx.in->ic};

    ctx.i_ab = foc_clarke(i_abc);
}

static __noinline void run_park(void) {
    ctx.i_dq = foc_park(ctx.i_ab, ctx.sc);
}

static __noinline void run_pi(void) {
    float v_lim = ctx.in->vbus * INV_SQRT3;
    foc_pi_t pi_d = {
        .kp = FOC_VECTORS_KP, .ki = FOC_VECTORS_KI, .out_min = -v_lim, .out_max = v_lim, .integral = ctx.in->int_d};
    foc_pi_t pi_q = {
        .kp = FOC_VECTORS_KP, .ki = FOC_VECTORS_KI, .out_min = -v_lim, .out_max = v_lim, .integral = ctx.in->int_q};

    ctx.v_dq.d = foc_pi_step(&pi_d, ctx.in->id_ref - ctx.i_dq.d, FOC_VECTORS_DT_S);
    ctx.v_dq.q = foc_pi_step(&pi_q, ctx.in->iq_ref - ctx.i_dq.q, FOC_VECTORS_DT_S);
}

static __noinline void run_inv_park(void) {
    ctx.v_ab = foc_inv_park(ctx.v_dq, ctx.sc);
}

static __noinline void run_svpwm(void) {
    ctx.limited = foc_svpwm(ctx.v_ab, ctx.in->vbus, &ctx.duty);
}

static __noinline void run_step(void) {
    foc_vector_step(ctx.in, &ctx.out);
}

/* sin/cos take and return double: on the single-precision FPU this is software arithmetic. */
static __noinline void run_sincos_dbl(void) {
    double theta = (double)ctx.in->theta;

    ctx.sink = (float)(sin(theta) + cos(theta));
}

static void (*const stage_fns[STAGE_COUNT])(void) = {
    [STAGE_EMPTY] = run_empty, [STAGE_SINCOS] = run_sincos, [STAGE_CLARKE] = run_clarke,
    [STAGE_PARK] = run_park,   [STAGE_PI] = run_pi,         [STAGE_INV_PARK] = run_inv_park,
    [STAGE_SVPWM] = run_svpwm, [STAGE_STEP] = run_step,     [STAGE_SINCOS_DBL] = run_sincos_dbl,
};

/* The memory clobbers keep the stage call between the two counter reads. */
static uint32_t time_stage(stage_t s) {
    uint32_t t0 = cycles_now();

    compiler_barrier();
    stage_fns[s]();
    compiler_barrier();
    return cycles_now() - t0;
}

static void record(stage_t s, uint32_t pass, uint32_t cycles) {
    span_t* sp = &spans[s];

    sp->min = MIN(sp->min, cycles);
    if (pass == 0U) {
        sp->max_cold = MAX(sp->max_cold, cycles);
    } else {
        sp->max_warm = MAX(sp->max_warm, cycles);
    }
}

static void measure(void) {
    for (size_t s = 0; s < STAGE_COUNT; s++) {
        spans[s] = (span_t){.min = UINT32_MAX};
    }
    for (uint32_t pass = 0; pass < BENCH_PASSES; pass++) {
        for (size_t i = 0; i < FOC_VECTORS_COUNT; i++) {
            /* Stages run in data-flow order, so each one sees this vector's real intermediate values. */
            unsigned int key = irq_lock();

            ctx.in = &foc_vectors_in[i];
            for (size_t s = 0; s < STAGE_COUNT; s++) {
                record((stage_t)s, pass, time_stage((stage_t)s));
            }
            irq_unlock(key);
        }
    }
}

static uint32_t float_bits(float f) {
    uint32_t u;

    (void)memcpy(&u, &f, sizeof(u));
    return u;
}

/* Kept out of line so the log packages live on this frame only. */
static __noinline void print_spans(void) {
    LOG_INF("stage            min  max(warm)  max(cold)  cycles at 170 MHz");
    for (size_t s = 0; s < STAGE_COUNT; s++) {
        LOG_INF("%-15s %5u %10u %10u", stage_names[s], spans[s].min, spans[s].max_warm, spans[s].max_cold);
        k_msleep(1);
    }
}

static __noinline void print_vector(uint32_t i, const foc_vector_out_t* out) {
    float f[FOC_VECTORS_OUT_FLOATS];

    foc_vector_fields(out, f);
    LOG_INF(FOC_VECTORS_TAG " %u %08x %08x %08x %08x %08x %08x %08x %08x %08x %u", i, float_bits(f[0]),
            float_bits(f[1]), float_bits(f[2]), float_bits(f[3]), float_bits(f[4]), float_bits(f[5]), float_bits(f[6]),
            float_bits(f[7]), float_bits(f[8]), out->limited ? 1U : 0U);
}

void foc_bench_run(void) {
    (void)cycles_start();
    LOG_INF("Stage 5 FOC bench: FPU %s, %u vectors x %u passes, IRQs locked per vector",
            IS_ENABLED(CONFIG_FPU) ? "on (hard-float)" : "off (soft-float)", FOC_VECTORS_COUNT, BENCH_PASSES);
    measure();
    /* A J-Link disconnect during the measurement stops CYCCNT (seen on the first stage 5 run: every min read 0). */
    if (!cycles_start()) {
        LOG_WRN("DWT counter stopped during the measurement (debugger disconnect?); the cycle figures are invalid, "
                "reset with the debugger attached or unplugged");
    }
    print_spans();

    uint32_t step_max = MAX(spans[STAGE_STEP].max_warm, spans[STAGE_STEP].max_cold);

    LOG_INF("full step max %u cycles = %u.%02u us; budget %u cycles (20 us): %s", step_max, step_max / CYCLES_PER_US,
            (step_max % CYCLES_PER_US) * 100U / CYCLES_PER_US, STEP_BUDGET_CYCLES,
            step_max < STEP_BUDGET_CYCLES ? "under" : "OVER");

    /* The results are recomputed here, outside the timing loop, so the dump does not depend on it. */
    for (uint32_t i = 0; i < FOC_VECTORS_COUNT; i++) {
        foc_vector_out_t out;

        foc_vector_step(&foc_vectors_in[i], &out);
        print_vector(i, &out);
        k_msleep(DUMP_PAUSE_MS);
    }
    LOG_INF("Stage 5 FOC bench done; compare with: vec_compare <this log>");
}
