/* SPDX-License-Identifier: Apache-2.0 */

#include "power_guard.h"

/* True only for a number inside [lo, hi]; NaN is outside. */
static bool inside(float x, float lo, float hi) {
    return x >= lo && x <= hi;
}

void power_guard_reset(power_guard_t* g) {
    g->vbus_ok_n = 0U;
    g->vbus_bad_n = 0U;
    g->request = false;
}

power_guard_out_t power_guard_step(power_guard_t* g, const power_guard_in_t* in) {
    power_guard_out_t out = {.power_on = false, .trip = 0U};
    float ib = -(in->ia + in->ic);

    if (in->calibrated
        && !(inside(in->ia, -POWER_GUARD_I_TRIP_A, POWER_GUARD_I_TRIP_A)
             && inside(in->ic, -POWER_GUARD_I_TRIP_A, POWER_GUARD_I_TRIP_A)
             && inside(ib, -POWER_GUARD_I_TRIP_A, POWER_GUARD_I_TRIP_A))) {
        out.trip |= POWER_GUARD_TRIP_OVERCURRENT;
    }

    if (inside(in->vbus, POWER_GUARD_VBUS_ARM_MIN_V, POWER_GUARD_VBUS_ARM_MAX_V)) {
        if (g->vbus_ok_n < POWER_GUARD_VBUS_ARM_PERIODS) {
            g->vbus_ok_n++;
        }
    } else {
        g->vbus_ok_n = 0U;
    }

    if (in->power_on && !inside(in->vbus, POWER_GUARD_VBUS_TRIP_MIN_V, POWER_GUARD_VBUS_TRIP_MAX_V)) {
        g->vbus_bad_n++;
        if (g->vbus_bad_n >= POWER_GUARD_VBUS_TRIP_PERIODS) {
            out.trip |= POWER_GUARD_TRIP_VBUS;
        }
    } else {
        g->vbus_bad_n = 0U;
    }

    if (!in->arm_key || out.trip != 0U) {
        g->request = false;
    } else if (!g->request && in->calibrated && in->idle && g->vbus_ok_n >= POWER_GUARD_VBUS_ARM_PERIODS) {
        g->request = true;
    }
    out.power_on = g->request;
    return out;
}
