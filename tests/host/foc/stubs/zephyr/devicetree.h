/* SPDX-License-Identifier: Apache-2.0 */

#ifndef FOC_TEST_STUB_DEVICETREE_H
#define FOC_TEST_STUB_DEVICETREE_H

/* Only board constants used by foc_ctrl; unknown properties fail compilation. */
#define DT_NODELABEL(node)                       node
#define DT_PROP(node, prop)                      TEST_DT_PROP(node, prop)
#define TEST_DT_PROP(node, prop)                 TEST_DT_##node##_##prop
#define TEST_DT_adc2_vref_mv                     3300
#define TEST_DT_encoder_st_counts_per_revolution 4096

#endif
