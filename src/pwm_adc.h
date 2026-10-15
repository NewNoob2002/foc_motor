/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: TIM1 center-aligned PWM timebase, ADC2 [IA, IC] triggered from TIM1,
 * DMA2 channel 1 moving both results, and the DMA transfer-complete ISR
 * (the future FOC ISR, docs/design/zephyr_architecture.md 2.3).
 *
 * PC0-PC3 (the gate-driver IN pins) always stay in analog mode: with no
 * DCBUS the EG2104 VCC is 0 V and IN must stay below VCC + 0.3 V
 * (DOC-GATE-DS 7.1, HW-PWM-009). For scope checks CH1-3 come out only on
 * the unconnected probe pins PA8/PA9/PA10 (user-approved 2026-10-14), and
 * PB9 (U7-4) is high while the ISR runs. pwm_adc_outputs_safe() checks the
 * invariant; pwm_adc_init() refuses to start without it.
 */

#ifndef FOC_MOTOR_PWM_ADC_H
#define FOC_MOTOR_PWM_ADC_H

#include <stdbool.h>
#include <stdint.h>

/* 170 MHz / (2 * 4250) = 20 kHz, center-aligned. */
#define PWM_ARR 4250U

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
    uint16_t ia_last;
    uint16_t ic_last;
    uint32_t dma_errors; /* sticky across windows */
    uint32_t adc_overruns;
} pwm_adc_stats_t;

/* Stage 4 scope test modes, applied to CH1 only. */
typedef enum {
    PWM_MODE_FIXED,    /* CH1-3 at 25 / 50 / 75 %, updates at top and bottom (RCR 0) */
    PWM_MODE_ALT_RCR0, /* CH1 alternates 25 / 75 % every period; updates at top and bottom */
    PWM_MODE_ALT_RCR1, /* the same, but one update per period (RCR 1) */
} pwm_mode_t;

/* Starts TIM1 (CH1-3 on PA8-PA10 only), ADC2 and DMA. Returns 0 or -errno. */
int pwm_adc_init(void);

/*
 * Selects a test mode. Restarts the counter so the repetition counter starts
 * in a known phase (RM0440: RCR loaded by UG before the counter starts).
 * Thread context only.
 */
void pwm_adc_set_mode(pwm_mode_t mode);

/*
 * True while PC0-PC3 are analog, CH4 and the complementary outputs are off
 * and CH1-3 can reach only PA8-PA10. If not, the caller must call
 * pwm_adc_stop().
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
