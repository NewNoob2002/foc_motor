/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Stage 7 power-stage policy: when the gate drivers may be switched on, and
 * when they must trip off. Pure C11 (no Zephyr, CMSIS or board headers) so
 * tests/host/foc/test_power_guard.c can check it; runs inside the FOC ISR.
 *
 * Software only: it acts within one PWM period (50 us) of a bad sample, but
 * it is not the CPU-independent shutdown that R-02 asks for (accepted for
 * the Stage 7 learning bench only, AGENTS.md).
 *
 * Contract:
 * - The request turns on only when all of these hold in the same step:
 *   the arm key is set, the current offsets are calibrated, VBUS has been
 *   inside [POWER_GUARD_VBUS_ARM_MIN_V, POWER_GUARD_VBUS_ARM_MAX_V] for
 *   POWER_GUARD_VBUS_ARM_PERIODS steps in a row, explicitly allowed idle
 *   has held for POWER_GUARD_IDLE_PERIODS consecutive steps, and nothing
 *   trips. This lets zero duty propagate through CCR preload before power
 *   switches on; any non-idle step restarts the idle count.
 * - The request turns off when the key is cleared or anything trips; it
 *   turns on again only by meeting all the conditions again.
 * - Overcurrent: once calibrated, |ia|, |ic| or |ib| = |ia + ic| above
 *   POWER_GUARD_I_TRIP_A (or not a number) trips at once, powered or not.
 * - VBUS: while the stage is on, POWER_GUARD_VBUS_TRIP_PERIODS steps in a
 *   row outside [POWER_GUARD_VBUS_TRIP_MIN_V, POWER_GUARD_VBUS_TRIP_MAX_V]
 *   (or not a number) trip.
 */

#ifndef FOC_MOTOR_POWER_GUARD_H
#define FOC_MOTOR_POWER_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define POWER_GUARD_I_TRIP_A          2.0f
#define POWER_GUARD_VBUS_ARM_MIN_V    14.0f
#define POWER_GUARD_VBUS_ARM_MAX_V    16.5f
#define POWER_GUARD_VBUS_TRIP_MIN_V   12.0f
#define POWER_GUARD_VBUS_TRIP_MAX_V   17.0f
#define POWER_GUARD_VBUS_ARM_PERIODS  2000U /* 100 ms at 20 kHz */
#define POWER_GUARD_VBUS_TRIP_PERIODS 3U
#define POWER_GUARD_IDLE_PERIODS      20U /* 1 ms at 20 kHz; flush the previous duty */

/* Largest voltage vector the control may apply while the stage is on. */
#define POWER_GUARD_V_MAX             1.0f

/* Trip reasons (bits). */
#define POWER_GUARD_TRIP_OVERCURRENT  0x1U
#define POWER_GUARD_TRIP_VBUS         0x2U

typedef struct {
    float ia;        /* A, phase A, offset removed */
    float ic;        /* A, phase C */
    float vbus;      /* V */
    bool calibrated; /* current offsets measured */
    bool arm_key;    /* the arm command holds the key */
    bool idle;       /* FOC_MODE_IDLE explicitly commanded and allowed, not a refused fallback */
    bool power_on;   /* the power stage is on now (BSP state) */
} power_guard_in_t;

typedef struct {
    uint32_t vbus_ok_n;  /* steps in a row inside the arm window */
    uint32_t vbus_bad_n; /* steps in a row outside the trip window while on */
    uint32_t idle_n;     /* consecutive idle steps, saturating at POWER_GUARD_IDLE_PERIODS */
    bool request;
} power_guard_t;

typedef struct {
    bool power_on; /* request to the BSP */
    uint32_t trip; /* POWER_GUARD_TRIP_* bits; non-zero means switch off now and latch */
} power_guard_out_t;

void power_guard_reset(power_guard_t* g);

power_guard_out_t power_guard_step(power_guard_t* g, const power_guard_in_t* in);

#endif /* FOC_MOTOR_POWER_GUARD_H */
