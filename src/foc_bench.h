/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Stage 5: lib/foc on the chip, with no ISR and no PWM. Times each stage of
 * one current-loop step with the DWT cycle counter and prints the results of
 * the fixed vectors (tests/foc_vectors) for tests/host/foc/vec_compare.
 */

#ifndef FOC_MOTOR_FOC_BENCH_H
#define FOC_MOTOR_FOC_BENCH_H

/*
 * Runs with interrupts locked for one vector at a time (a few microseconds),
 * so call it before the TIM1/ADC2 ISR is started: that ISR is zero-latency
 * and would still preempt. Thread context only; takes about 0.5 s, most of it
 * waiting for the log to drain.
 */
void foc_bench_run(void);

#endif /* FOC_MOTOR_FOC_BENCH_H */
