/* SPDX-License-Identifier: Apache-2.0 */
#ifndef FOC_MOTOR_MOTOR_IDENT_H
#define FOC_MOTOR_MOTOR_IDENT_H

#include <stdbool.h>
#include <stdint.h>

#define IDENT_PERIOD_S        50e-6f /* s, exactly one PWM period */
#define IDENT_COUNTS          4096U  /* encoder/electrical counts per revolution */
#define IDENT_POLE_PAIRS      7U
#define IDENT_ALIGN_90        1024U /* electrical counts */
#define IDENT_ALIGN_A         1.0f  /* A */
#define IDENT_ALIGN_HOLD_S    0.5f  /* s after reaching the target */
#define IDENT_ALIGN_TOL_E     200   /* electrical counts */
#define IDENT_R_POINTS        5U
#define IDENT_R_TARGETS_A     {0.5f, 0.75f, 1.0f, 1.25f, 1.5f} /* A */
#define IDENT_R_FIT_FIRST     1U                               /* omit the nonlinear 0.5 A point */
#define IDENT_R_BIAS_POINT    2U                               /* 1.0 A point */
#define IDENT_R_SETTLE_S      0.2f                             /* s after reaching each target */
#define IDENT_R_AVG_N         2000U                            /* PWM periods */
#define IDENT_R_RESIDUAL_FRAC 0.05f
#define IDENT_R_SLOPE_FRAC    0.20f
#define IDENT_R_TOP_A         1.5f  /* A */
#define IDENT_L_VH            0.2f  /* V, square-wave amplitude */
#define IDENT_L_HALF          4U    /* PWM periods per half cycle */
#define IDENT_L_CYCLES        2500U /* square cycles */
#define IDENT_L_DISCARD       50U   /* initial square cycles */
#define IDENT_L_IPP_MAX_A     0.8f  /* A, peak-to-peak */
#define IDENT_SERVO_KI        5.0f  /* V/(A s) */
#define IDENT_V_MAX           0.9f  /* V, servo clamp */
#define IDENT_I_ABORT_A       1.8f  /* A, instantaneous d-axis abort */
#define IDENT_SERVO_TIMEOUT_S 1.0f  /* s per target */
#define IDENT_REACHED_FRAC    0.05f
#define IDENT_REACHED_S       0.02f /* s continuously within the band */
#define IDENT_FILTER_K        0.01f /* dimensionless; tau about 5 ms */
#define IDENT_ZERO_BAND_A     0.05f /* A, decay before switching angle */
#define IDENT_RUN_TIMEOUT_S   10.0f /* s, absolute run bound */

/** Identification phases; terminal states remain latched until start. */
typedef enum {
    IDENT_IDLE = 0,
    IDENT_ALIGN_FIRST,
    IDENT_ALIGN_DECAY,
    IDENT_ALIGN_ZERO,
    IDENT_RESISTANCE,
    IDENT_L_BIAS,
    IDENT_INDUCTANCE,
    IDENT_DONE,
    IDENT_FAILED,
} motor_ident_phase_t;

typedef enum {
    IDENT_FAIL_NONE = 0,
    IDENT_FAIL_NOT_POWERED,
    IDENT_FAIL_NUMERIC,
    IDENT_FAIL_OVERCURRENT,
    IDENT_FAIL_SERVO_TIMEOUT,
    IDENT_FAIL_ALIGNMENT,
    IDENT_FAIL_RESISTANCE,
    IDENT_FAIL_INDUCTANCE,
    IDENT_FAIL_RIPPLE,
    IDENT_FAIL_RUN_TIMEOUT,
    IDENT_FAIL_CANCELLED,
} motor_ident_fail_t;

typedef struct {
    float v; /* V */
    float i; /* A */
} motor_ident_point_t;

typedef struct {
    motor_ident_phase_t state;
    motor_ident_fail_t fail_reason;
    uint32_t enc_90; /* raw TIM3 counts; wrap handled modulo one mechanical turn */
    uint32_t enc_0;
    int32_t delta_mech; /* signed shortest displacement, counts */
    int32_t direction;  /* +1 or -1 */
    uint32_t offset_e;  /* theta_e = (direction * enc * 7 - offset_e) mod 4096 */
    motor_ident_point_t points[IDENT_R_POINTS];
    float r;            /* ohm */
    float v0;           /* V, fitted inverter loss */
    float max_residual; /* V */
    bool r_nonlinear;
    float l;   /* H */
    float ipp; /* A, estimated peak-to-peak ripple */
} motor_ident_result_t;

typedef struct {
    float i_d;          /* A, Park at the angle requested on the preceding call */
    uint32_t enc_count; /* raw encoder count */
    bool power_on;      /* actual power AND permission to remain powered */
} motor_ident_in_t;

typedef struct {
    float v_d;              /* V, q is always zero */
    uint32_t theta_e_count; /* electrical counts, 0..4095 */
    bool done;
    bool failed;
} motor_ident_out_t;

/** Fixed storage owned exclusively by the caller of motor_ident_step(). */
typedef struct {
    motor_ident_result_t result;
    uint32_t theta_e_count;
    uint32_t run_n;
    uint32_t target_n;
    uint32_t reached_n;
    uint32_t hold_n;
    uint32_t point;
    uint32_t avg_n;
    uint32_t l_n;
    uint32_t increments;
    bool reached;
    float filtered_i;
    float servo_v;
    float sum_v;
    float sum_i;
    float prev_i;
    float sum_delta;
} motor_ident_t;

/** Reset all results and begin the 90-degree alignment at zero voltage. */
void motor_ident_start(motor_ident_t* ident);

/**
 * Advance exactly once per 50 us PWM period; no RTOS or hardware dependencies.
 * Output u[k] starts at top k+1. Thus i[k+2]-i[k+1] belongs to u[k],
 * not u[k+1]. The L accumulator explicitly indexes that two-call history.
 * DONE/FAILED always output zero and remain latched until restarted.
 */
void motor_ident_step(motor_ident_t* ident, const motor_ident_in_t* in, motor_ident_out_t* out);

/** Abort a running identification, preserving all partial measurements. */
void motor_ident_abort(motor_ident_t* ident, motor_ident_fail_t reason);

#endif /* FOC_MOTOR_MOTOR_IDENT_H */
