/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: TIM1 center-aligned PWM timebase, ADC2 [IA, IC] triggered from TIM1,
 * DMA2 channel 1 moving both results, and the DMA transfer-complete ISR
 * (the future FOC ISR, docs/design/zephyr_architecture.md 2.3).
 *
 * Stage 4 runs TIM1 with every output disabled (CCER = 0, MOE = 0) and
 * PC0-PC3 left in analog mode: with no DCBUS the EG2104 VCC is 0 V and
 * its IN pin must stay below VCC + 0.3 V (DOC-GATE-DS 7.1, HW-PWM-009).
 * pwm_adc_init() checks this and refuses to start otherwise.
 */

#ifndef FOC_MOTOR_PWM_ADC_H
#define FOC_MOTOR_PWM_ADC_H

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

/* Starts TIM1 (outputs off), ADC2 and DMA. Returns 0 or -errno. */
int pwm_adc_init(void);

/*
 * Copies the current window and asks the ISR to start a new one at its next
 * entry. The copy is consistent: it retries if the ISR ran during it.
 */
void pwm_adc_stats_take(pwm_adc_stats_t* out);

#endif /* FOC_MOTOR_PWM_ADC_H */
