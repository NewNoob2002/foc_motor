/* SPDX-License-Identifier: Apache-2.0 */

#include "analog.h"

#include <zephyr/devicetree.h>

/* Board constants from DOC-MAIN-SCH. */
#define NTC_PULLUP_OHM 10000

int32_t analog_code_x100_to_mv(int32_t code_x100) {
    int64_t num = (int64_t)code_x100 * ANALOG_VDDA_MV;

    return (int32_t)(num / (100LL * ANALOG_FULL_SCALE));
}

int32_t analog_phase_current_ma(int32_t code_x100, int32_t offset_x100) {
    int64_t num = (int64_t)(code_x100 - offset_x100) * ANALOG_VDDA_MV * ANALOG_SHUNT_MA_PER_V;

    /* mV * (mA/V) / 1000 = mA */
    return (int32_t)(num / (100LL * ANALOG_FULL_SCALE * 1000));
}

int32_t analog_vbus_mv(int32_t code_x100) {
    int64_t pin_mv = analog_code_x100_to_mv(code_x100);

    return (int32_t)(pin_mv * (ANALOG_VBUS_DIV_TOP_OHM + ANALOG_VBUS_DIV_BOT_OHM) / ANALOG_VBUS_DIV_BOT_OHM);
}

int32_t analog_ntc_ohm(int32_t code_x100) {
    int64_t den = 100LL * ANALOG_FULL_SCALE - code_x100;

    if (den <= 0) {
        return -1;
    }
    return (int32_t)((int64_t)NTC_PULLUP_OHM * code_x100 / den);
}
