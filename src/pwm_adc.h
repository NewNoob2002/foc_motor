/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: TIM1 center-aligned PWM timebase, ADC2 [IA, IC] triggered from TIM1,
 * DMA2 channel 1 moving both results, and the DMA transfer-complete ISR
 * that is the FOC ISR (docs/design/zephyr_architecture.md 2.3, 4.1).
 *
 * PC0-PC3 (the gate-driver IN pins) always stay in analog mode: with no
 * DCBUS the EG2104 VCC is 0 V and IN must stay below VCC + 0.3 V
 * (DOC-GATE-DS 7.1, HW-PWM-009). For scope checks CH1-3 come out only on
 * the unconnected pins PA8/PA9/PA10, and CH3N (CH3 inverted) on PB9 at the
 * U7-4 header, the one probe-able pin (both user-approved 2026-10-14).
 * pwm_adc_outputs_safe() checks the invariant; pwm_adc_init() refuses to
 * start without it.
 *
 * Timing: ADC2 converts at the counter top (TRGO2), the ISR computes the
 * duties and writes CCR1-3 into the preload registers; RCR = 1 makes the
 * only update event the next top, so a sample taken at top k drives the
 * period that starts at top k + 1 (one period, 50 us, of delay).
 */

#ifndef FOC_MOTOR_PWM_ADC_H
#define FOC_MOTOR_PWM_ADC_H

#include <stdbool.h>
#include <stdint.h>

/* 170 MHz / (2 * 4250) = 20 kHz, center-aligned. */
#define PWM_ARR                     4250U

/* AGENTS.md: FOC ISR WCET at most 20 us. A longer run is an overrun. */
#define PWM_ADC_EXEC_BUDGET         3400U

/* Overruns in a row that latch the fault. */
#define PWM_ADC_OVERRUN_FAULT_COUNT 3U

/* Sticky fault reasons (pwm_adc_stats_t.fault). */
#define PWM_ADC_FAULT_OVERRUN       0x1U /* PWM_ADC_OVERRUN_FAULT_COUNT overruns in a row */
#define PWM_ADC_FAULT_DMA_ERROR     0x2U

/* What the ISR hands to the control step each period. */
typedef struct {
    uint16_t ia_code;   /* ADC2 IN3, sampled at the counter top */
    uint16_t ic_code;   /* ADC2 IN17, sampled right after IA */
    uint32_t enc_count; /* TIM3 count read in the ISR */
} pwm_adc_sample_t;

/* High-side duty per phase, 0..1; values outside are clamped. */
typedef struct {
    float a;
    float b;
    float c;
} pwm_adc_duty_t;

/*
 * Runs inside the zero-latency ISR: no kernel, device or logging API, no
 * locks, no waiting (AGENTS.md).
 */
typedef void (*pwm_adc_step_fn)(const pwm_adc_sample_t* sample, pwm_adc_duty_t* duty);

/* ISR measurements over one window, written only by the ISR. */
typedef struct {
    uint32_t isr_count;
    uint32_t period_min; /* DWT cycles between ISR entries */
    uint32_t period_max;
    uint32_t exec_max; /* DWT cycles from ISR entry to exit */
    uint16_t cnt_min;  /* TIM1 CNT at ISR entry */
    uint16_t cnt_max;
    uint32_t counting_up; /* ISRs that saw TIM1 counting up */
    uint32_t ia_sum;
    uint32_t ic_sum;
    uint16_t ccr[3]; /* CCR1-3 written by the latest ISR */
    /* Sticky across windows: */
    uint32_t overruns; /* ISRs longer than PWM_ADC_EXEC_BUDGET */
    uint32_t missed;   /* ISRs that ended after the next ADC result had arrived */
    uint32_t fault;    /* PWM_ADC_FAULT_* bits; latched until reset */
    uint32_t dma_errors;
    uint32_t adc_overruns;
} pwm_adc_stats_t;

/*
 * Starts TIM1 (CH1-3 on PA8-PA10, CH3N on PB9 only, all at 50 %), ADC2 and
 * DMA; from the first ISR on, step() sets the duties. Returns 0 or -errno.
 */
int pwm_adc_init(pwm_adc_step_fn step);

/*
 * True while PC0-PC3 are analog, CH4, CH1N and CH2N are off (CH2N could
 * reach PB0 = SD1) and only CH1-3 and CH3N are enabled. If not, the caller
 * must call pwm_adc_stop().
 */
bool pwm_adc_outputs_safe(void);

/* Disables all TIM1 outputs (MOE = 0) and stops the counter. */
void pwm_adc_stop(void);

/*
 * Copies the current window and asks the ISR to start a new one at its next
 * entry. The copy is consistent: it retries if the ISR ran during it.
 * Returns false if the DWT cycle counter had been stopped (a debugger
 * disconnect does this); it is restarted, and the window's cycle figures
 * (period, exec_max) are invalid.
 */
bool pwm_adc_stats_take(pwm_adc_stats_t* out);

#endif /* FOC_MOTOR_PWM_ADC_H */
