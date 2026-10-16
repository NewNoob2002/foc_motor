/* SPDX-License-Identifier: Apache-2.0 */

#include "foc_bench.h"

#include "cycles.h"
#include "foc_vectors.h"
#include "motor_ident.h"

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

/* Pass 0 is reported as cold; preparation and earlier stages may already warm shared code. */
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
    STAGE_IDENT_START,
    STAGE_IDENT_SERVO,
    STAGE_IDENT_R_FIT,
    STAGE_IDENT_L,
    STAGE_IDENT_COPY,
    STAGE_COUNT,
} stage_t;

static const char* const stage_names[STAGE_COUNT] = {
    [STAGE_EMPTY] = "empty call",
    [STAGE_SINCOS] = "sinf+cosf",
    [STAGE_CLARKE] = "clarke",
    [STAGE_PARK] = "park",
    [STAGE_PI] = "2x pi",
    [STAGE_INV_PARK] = "inv park",
    [STAGE_SVPWM] = "svpwm",
    [STAGE_STEP] = "full step",
    [STAGE_SINCOS_DBL] = "sin+cos double",
    [STAGE_IDENT_START] = "ident start",
    [STAGE_IDENT_SERVO] = "ident servo",
    [STAGE_IDENT_R_FIT] = "ident R fit",
    [STAGE_IDENT_L] = "ident L",
    [STAGE_IDENT_COPY] = "ident copy",
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

/* Synthetic numeric state only: these stages never call the BSP or change commands. */
static struct {
    motor_ident_t ident;
    motor_ident_in_t in;
    motor_ident_out_t out;
    motor_ident_result_t copy;
} ident_ctx;

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

/** Time the same bounded reset used on entry to identification in the ISR. */
static __noinline void run_ident_start(void) {
    motor_ident_start(&ident_ctx.ident);
}

/** Time one identification phase using state prepared outside the timing window. */
static __noinline void run_ident_step(void) {
    motor_ident_step(&ident_ctx.ident, &ident_ctx.in, &ident_ctx.out);
}

/** Time the exact scalar copy used by ident_publish, without altering its ownership state. */
static __noinline void run_ident_copy(void) {
    motor_ident_result_copy(&ident_ctx.copy, &ident_ctx.ident.result);
}

/** Prepare reachable servo, last R sample, and injection states without accessing hardware. */
static void prepare_ident(stage_t s, size_t vector) {
    static const float targets[IDENT_R_POINTS] = IDENT_R_TARGETS_A;
    motor_ident_t* m = &ident_ctx.ident;

    if (s == STAGE_IDENT_START) {
        (void)memset(m, 0xa5, sizeof(*m));
        return;
    }
    motor_ident_start(m);
    ident_ctx.in.i_d = 1.0f;
    ident_ctx.in.enc_count = 3745U;
    ident_ctx.in.power_on = true; /* Synthetic permission, never passed to the BSP. */
    m->filtered_i = 1.0f;
    m->servo_v = 0.25f;
    if (s == STAGE_IDENT_SERVO) {
        /* Include arrival, dwell, and both saturation branches across repetitions. */
        m->reached_n = IDENT_REACHED_N - 1U;
        m->reached = (vector & 1U) != 0U;
        m->servo_v = vector % 3U == 0U ? -0.001f : (vector % 3U == 1U ? IDENT_V_MAX : 0.25f);
        ident_ctx.in.i_d = vector % 3U == 1U ? 0.0f : 1.0f;
    } else {
        for (uint32_t k = 0U; k < IDENT_R_POINTS; k++) {
            m->result.points[k].i = targets[k];
            m->result.points[k].v = 0.15f + 0.1f * targets[k];
        }
        if (s == STAGE_IDENT_R_FIT) {
            m->result.state = IDENT_RESISTANCE;
            m->point = IDENT_R_POINTS - 1U;
            m->reached = true;
            m->hold_n = IDENT_R_SETTLE_N;
            m->avg_n = IDENT_R_AVG_N - 1U;
            ident_ctx.in.i_d = targets[m->point];
            m->filtered_i = ident_ctx.in.i_d;
            m->servo_v = m->result.points[m->point].v;
            m->sum_i = ident_ctx.in.i_d * (float)m->avg_n;
            m->sum_v = m->servo_v * (float)m->avg_n;
        } else if (s == STAGE_IDENT_L) {
            m->result.state = IDENT_INDUCTANCE;
            m->l_n = IDENT_L_DISCARD * 2U * IDENT_L_HALF + 2U + (uint32_t)(vector % (2U * IDENT_L_HALF));
            m->prev_i = 0.99f;
            if ((vector & 8U) != 0U) {
                /* Include the once-only L calculation and its three FPU divisions. */
                m->l_n = IDENT_L_CYCLES * 2U * IDENT_L_HALF + 1U;
                m->increments = (IDENT_L_CYCLES - IDENT_L_DISCARD) * 2U * IDENT_L_HALF - 1U;
                m->sum_delta = (float)m->increments * 0.1f;
                m->prev_i = ident_ctx.in.i_d;
            }
        } else {
            m->result.state = IDENT_DONE;
            m->result.r = 0.1f;
            m->result.l = 100e-6f;
        }
    }
}

static void (*const stage_fns[STAGE_COUNT])(void) = {
    [STAGE_EMPTY] = run_empty,
    [STAGE_SINCOS] = run_sincos,
    [STAGE_CLARKE] = run_clarke,
    [STAGE_PARK] = run_park,
    [STAGE_PI] = run_pi,
    [STAGE_INV_PARK] = run_inv_park,
    [STAGE_SVPWM] = run_svpwm,
    [STAGE_STEP] = run_step,
    [STAGE_SINCOS_DBL] = run_sincos_dbl,
    [STAGE_IDENT_START] = run_ident_start,
    [STAGE_IDENT_SERVO] = run_ident_step,
    [STAGE_IDENT_R_FIT] = run_ident_step,
    [STAGE_IDENT_L] = run_ident_step,
    [STAGE_IDENT_COPY] = run_ident_copy,
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
                if (s >= STAGE_IDENT_START) {
                    prepare_ident((stage_t)s, i);
                }
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
