/* SPDX-License-Identifier: Apache-2.0 */

#ifndef FOC_TEST_STUB_UTIL_H
#define FOC_TEST_STUB_UTIL_H

#include <stddef.h>

#define BUILD_ASSERT(condition, message) _Static_assert(condition, message)
#define MIN(a, b)                        ((a) < (b) ? (a) : (b))
#define MAX(a, b)                        ((a) > (b) ? (a) : (b))

#endif
