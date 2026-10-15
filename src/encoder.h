/* SPDX-License-Identifier: Apache-2.0 */

/*
 * BSP: KTH7823 ABZ encoder on TIM3 (A/B, x4 quadrature) and PB3 (Z).
 *
 * The Zephyr st,stm32-qdec driver configures TIM3 at boot; this module only
 * reads the counter register, so encoder_count() is safe in any context,
 * including the future FOC ISR.
 */

#ifndef FOC_MOTOR_ENCODER_H
#define FOC_MOTOR_ENCODER_H

#include <stdint.h>

#include <zephyr/devicetree.h>

#define ENCODER_COUNTS_PER_REV DT_PROP(DT_NODELABEL(encoder), st_counts_per_revolution)

/* Latest Z rising edge, written only by the Z interrupt. */
typedef struct {
    uint32_t events; /* Z edges since encoder_init() */
    uint32_t count;  /* TIM3 count latched at the latest edge */
} encoder_index_t;

/* Checks the qdec device and enables the Z interrupt. Returns 0 or -errno. */
int encoder_init(void);

/* Raw TIM3 count in [0, encoder_period()). */
uint32_t encoder_count(void);

/*
 * Counter wrap period (ARR + 1). The qdec driver sets it to a multiple of
 * ENCODER_COUNTS_PER_REV, so count % ENCODER_COUNTS_PER_REV is the position
 * within one turn without tracking wraps.
 */
uint32_t encoder_period(void);

encoder_index_t encoder_index_get(void);

#endif /* FOC_MOTOR_ENCODER_H */
