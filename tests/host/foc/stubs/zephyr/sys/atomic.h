/* SPDX-License-Identifier: Apache-2.0 */

#ifndef FOC_TEST_STUB_ATOMIC_H
#define FOC_TEST_STUB_ATOMIC_H

#include <stdatomic.h>
#include <stdbool.h>

typedef atomic_int atomic_t;
typedef int atomic_val_t;

/* Match the Zephyr atomics used by control snapshots with standard C11 atomics. */
#define atomic_get(target)        atomic_load(target)
#define atomic_set(target, value) atomic_exchange(target, value)
#define atomic_inc(target)        atomic_fetch_add(target, 1)

/* Provide Zephyr's by-value expected argument using C11 compare/exchange. */
static inline bool atomic_cas(atomic_t* target, atomic_val_t expected, atomic_val_t desired) {
    return atomic_compare_exchange_strong(target, &expected, desired);
}

#endif
