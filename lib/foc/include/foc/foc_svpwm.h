/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Space-vector modulation: alpha/beta voltage command -> phase duty cycles.
 *
 * Contract checked by tests/host/foc/test_svpwm.c:
 * - duty is the high-side on-time fraction of each phase, in [0, 1].
 * - The applied line-to-line voltages equal those of v_ab:
 *   foc_clarke(duty * vbus) == v_ab within float tolerance.
 * - The linear range is |v_ab| <= vbus / sqrt(3). A longer vector is scaled
 *   down to that length with its angle kept, and the function returns true.
 * - If vbus is not > 0 (including NaN), all duties are 0.5 (zero line
 *   voltage) and the function returns true.
 * - Otherwise it returns false.
 *
 * Bootstrap and sampling-window duty limits are board constraints applied
 * by the caller; they are not part of this function.
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdbool.h>

#include "foc/foc_transform.h"

bool foc_svpwm(foc_ab_t v_ab, float vbus, foc_abc_t* duty);

#endif /* FOC_SVPWM_H */
