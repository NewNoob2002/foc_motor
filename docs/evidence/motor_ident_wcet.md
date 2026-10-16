# Motor identification WCET work

Software verification against source baseline `70de69a` (its firmware code is the same as parent `ebf9ea8`). No hardware was accessed or flashed. The user-provided baseline ISR figures are 4109 cycles on entry, 2718–3179 in alignment/R, 3378 in the report containing the R fit, 2707 in L injection, and 2245 after completion. The target is every identification ISR period at most 2500 cycles, below the mandatory 3400-cycle / 20 us budget at 170 MHz. Neither bound is hardware-verified by this change.

## Baseline ARM findings

The inventory below comes from the linked baseline `zephyr.elf`, built with the requested Zephyr v4.4.2 / SDK 1.0.1 command. Addresses refer to that baseline image. Calls inside inlined `resistance_step`, `fit_resistance`, and `inductance_step` appear under `motor_ident_step`.

| Caller | Target | All static call sites (hex) | Dynamic work |
| --- | --- | --- | --- |
| `motor_ident_start` | `memset` | `0800b872` | 156-byte clear on entry |
| `ident_publish` | `memcpy` | `0800374c` | 88-byte terminal result copy |
| `servo` | `fmaxf` | `080040d4` | Once per servo period |
| `servo` | `fminf` | `080040dc` | Once per servo period |
| `motor_ident_step` (R fit) | `fmaxf` | `080044b0` | Four calls, one per residual |
| `fmaxf` | `__issignalingf` | `0800d8fc`, `0800d914` | Two per finite max |
| `fmaxf` | `__isnanf` | `0800d920`, `0800d92c` | Two per finite max |
| `fminf` | `__issignalingf` | `0800d956`, `0800d96e` | Two per finite min |
| `fminf` | `__isnanf` | `0800d97a`, `0800d986` | Two per finite min |
| `foc_ctrl_step` | `_sinf` | `08003b42`, `08003e66` | Initial transform; second only on angle change |
| `foc_ctrl_step` | `_cosf` | `08003b52`, `08003e72` | Initial transform; second only on angle change |
| `_sinf` | `__kernel_sinf` | `0800ae48` (tail), `0800ae7a`, `0800ae8c` | Argument-dependent branches |
| `_sinf` | `__kernel_cosf` | `0800ae84`, `0800ae96` | Argument-dependent branches |
| `_sinf` | `__math_invalidf` | `0800ae58` (tail) | Nonfinite argument only |
| `_sinf` | `__rem_pio2f` | `0800ae5e` | Argument reduction |
| `_cosf` | `__kernel_cosf` | `0800ab7e` (tail), `0800abae`, `0800abc2` | Argument-dependent branches |
| `_cosf` | `__kernel_sinf` | `0800abb8`, `0800abca` | Argument-dependent branches |
| `_cosf` | `__math_invalidf` | `0800ab8e` (tail) | Nonfinite argument only |
| `_cosf` | `__rem_pio2f` | `0800ab94` | Argument reduction |
| `__kernel_sinf` | `__math_inexactf` | `0800b45a` (tail) | Tiny-argument branch |
| `__math_invalidf` | `__isnanf` | `0800aeae` | Nonfinite argument only |
| `__rem_pio2f` | `__kernel_rem_pio2f` | `0800adc8` | Large arguments; outside the bounded control-angle range |
| `__kernel_rem_pio2f` | `ldexpf` | `0800b066`, `0800b0ee`, `0800b16c`, `0800b286` | Large-argument reducer |
| `__kernel_rem_pio2f` | `floorf` | `0800b076` | Large-argument reducer |
| `ldexpf` | `__math_uflowf` | `0800b5da` (tail) | Underflow branch |
| `ldexpf` | `__math_oflowf` | `0800b5f2` (tail) | Overflow branch |

There are no `__aeabi_*` calls anywhere in this reachable call graph. `isfinite` lowers to inline absolute-value/compare instructions; `fabsf` and `sqrtf` also lower to FPU instructions. There is no `memset` or `memcpy` call in baseline `foc_ctrl_step` itself: the designated `ident_in` initializer already lowered to scalar stores in this exact build. Its rewrite prevents reliance on aggregate initialization for future compiler choices.

For completeness, the other project/FOC call edges reachable from these roots are:

| Caller | Callees and static call sites (hex) |
| --- | --- |
| `foc_ctrl_step` | `motor_ident_abort` `080038e0`; `ident_publish` `080038e4`, `08003bc2`; `motor_ident_start` `080038ee`; `pi_reset` `08003902`; `atomic_get` `08003908`, `08003cac`, `08003cf2`, `08003d8a`; `atomic_set.isra.0` `08003914`, `08003cec`, `08003d80`; `power_guard_step` `08003958`; `foc_current_step` `08003b90`; `foc_clarke` `08003e0c`; `foc_park` `08003e20`; `motor_ident_step` `08003e48`; `foc_inv_park` `08003eca`; `foc_svpwm` `08003edc` |
| `motor_ident_step` | `motor_ident_abort` `08004182`, `0800418e`, `080041c2`, `080042a4`, `0800460a`; `servo` `08004218`, `080042ec`, `0800435a`, `08004558` |
| `servo` | `motor_ident_abort` `08004130` (tail) |
| `ident_publish` | `atomic_get` `0800373e`; `atomic_set.isra.0` `0800375c` (tail) |
| `pi_reset` | `foc_pi_reset` `08003710`, `0800371c` (tail) |
| `foc_current_step` | `foc_clarke` `080047b0`; `foc_park` `080047bc`; `foc_pi_step` `08004822`, `0800483e`; `foc_inv_park` `08004856`; `foc_svpwm` `08004860` |
| `foc_svpwm` | `foc_inv_clarke` `080046f8` |

The remaining listed callees are leaves. The atomic wrappers use inline barriers/exclusive instructions, not kernel services. No ISR kernel, allocation, device, or logging calls were introduced.

### Every baseline float division

| Instruction address (hex) | Source operation | Frequency / condition |
| --- | --- | --- |
| `08003ea6` | `cap_vector`: `max_v / len` | Only if the powered voltage vector exceeds the cap; one shared ratio |
| `080043ac`, `080043bc` | R averages: `sum_i / 2000`, `sum_v / 2000` | At each completed R point, including the final fit period |
| `08004482` | R fit: `covariance / variance` | Once per successful fit |
| `08004514` | R fit: final-point slope division | Once per fit; compiler can skip it if residuals already imply nonlinearity |
| `080045ea` | L finish: `sum_delta / increments` | Final L period |
| `08004614` | L finish: voltage-time product / mean | Final L period |
| `08004620` | L finish: voltage-time product / L | Final L period |
| `080046d8` | SVPWM: `vbus / sqrt(3)` | Every positive-bus SVPWM period |
| `080046e6` | SVPWM: voltage limit / magnitude | Conditional vector limiting |
| `08004748`, `0800474c`, `08004750` | SVPWM: three phase voltages / vbus | Every positive-bus SVPWM period |
| `0800aec2` | `__math_invalidf`: `0 / 0` | Only nonfinite trig arguments; unreachable for bounded encoder/command angles |

`periods(...)` already constant-folded in the baseline: it emitted integer comparisons, no conversions or float divides. The R-fit means divided by four also compiled to multiplication by 0.25. Ordinary servo and L-injection periods have no division inside the identification module. The normal-mode SVPWM and cap divisions are preserved.

## Changes and generated-code evidence

- `motor_ident_start` explicitly resets all semantic fields. Volatile-qualified scalar stores prevent GCC from synthesizing `memset` again. Padding has no semantic meaning and is neither compared nor interpreted. ARM output has a straight sequence of scalar stores, no library calls.
- `motor_ident_result_copy` copies each semantic result field. ARM uses word loads/stores for 32-bit values and byte accesses only for enums/bools. The destination's volatile qualification prevents a synthesized byte-loop `memcpy`; there are no casts that violate aliasing. The five-point loop is bounded.
- `ident_publish` retains the original single-writer ownership: the ISR writes only when `ident_ready == 0`, then publishes via `atomic_set`. The reader acquires through `atomic_get` and releases after reading. Volatile controls code generation; the existing atomics provide synchronization. A reader-held result is still immutable across restarts.
- The duration thresholds are explicit integer PWM counts (400, 4000, 10000, 20000, 200000), with their unchanged durations documented. `ident_in` uses field assignments. Two R-average divisions become multiplication by a float32 reciprocal; ordinary phases, thresholds, tolerances, and safety rules are unchanged.
- Servo saturation uses comparisons, including the previous NaN-to-zero clamp behavior. Fit residual maxima use comparisons. The fit still runs exactly once with its three four-point loops and two remaining divisions. Both `motor_ident_step` and `servo` lose their high floating-point register saves/restores after removal of library calls. There are no libc calls in the identification module.
- Control's identification angle is always zero or `IDENT_ALIGN_90`. `ident_sincos` uses constant-argument expressions at these angles, preserving the original rounded float32 angle and the small cosine residual at 90 degrees. ARM GCC inlines it and folds both trig functions to constants, including the angle-transition site. Only the normal-mode `_sinf`/`_cosf` pair remains in `foc_ctrl_step`; other control modes retain their existing math.

Expected savings are on the order of 1000 cycles for entry's byte clear, several hundred for terminal copying, and several hundred per servo period from avoiding trig plus two library clamps (each of which previously called four classification helpers). The R-fit period additionally removes four `fmaxf` calls and sixteen classification-helper calls, plus two hardware divisions for averaging. These are estimates from removed work, not measured timing or a static WCET proof. The reported 800-cycle identification increase cannot be attributed entirely to one cause from report maxima alone: the 90-degree trig path differs from the zero-angle path, and report maxima can include transitions/publication. No exact predicted ISR maximum is asserted.

## Boot benchmark

The existing bench gains `ident start`, `ident servo`, `ident R fit`, `ident L`, and `ident copy` lines in its existing min/max(warm)/max(cold) format. Each runs 64 repetitions per pass over eight passes (512 timings). State setup is outside the DWT timing window. The servo covers arrival/dwell and both clamps. The R fixture ends the 2000th sample of the fifth R point and actually runs the fit. L covers both pulse signs and the final L calculation. Copy times the same function called by `ident_publish`; atomic ownership checks are outside this microbenchmark.

All fixtures use static numeric storage and synthetic power permission only. They do not write `foc_ctrl_cmd`, invoke the BSP, or touch the power stage. `report_loop` already runs the bench before `foc_ctrl_init` / `pwm_adc_init`. The existing cold label means first pass, not guaranteed cold execution: setup and earlier stages can warm shared code.

A temporary host harness extracted the actual `prepare_ident` function and storage from `src/foc_bench.c`, compiled them with the production identification module and ASan/UBSan, and exercised all 512 repetitions per stage. All fixtures reached their intended states; the R step reached L_BIAS with R approximately 0.1 ohm and V0 approximately 0.15 V, and the terminal L step reached DONE near 100 uH. This validates fixture setup, not target cycles.

## Commands and observed results

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-wcet . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d build/arm-wcet/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-wcet/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm build/arm-wcet/zephyr/zephyr.elf | rg foc_ctrl_cmd
cmake -S tests/host/foc -B build/host-wcet
cmake --build build/host-wcet
ctest --test-dir build/host-wcet --output-on-failure
clang-format -i src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/foc_bench.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-format --dry-run --Werror src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/foc_bench.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-tidy -p build/host-wcet src/motor_ident.c src/foc_ctrl.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-tidy -p build/wcet-tidy src/motor_ident.c src/foc_ctrl.c src/foc_bench.c
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -Isrc build/wcet-evidence/bench-fixtures.c src/motor_ident.c -lm -o build/wcet-evidence/bench-fixtures
ASAN_OPTIONS=detect_leaks=0 build/wcet-evidence/bench-fixtures
git diff --check
```

Both baseline and final ARM builds pass. Final size: text 70060, data 564, bss 10288 bytes. `text + data = 70624 <= 98304`; `data + bss = 10852 <= 24576`. All CI artifact, allocation-symbol, four shutdown-hog, heap, stack-size, stack-protection, and build-version gates also pass. `foc_ctrl_cmd` is at **`0x200013d0`** (`B`).

All seven registered host tests pass with warnings-as-errors and ASan/UBSan. Added tests compare every semantic field after dirty versus zeroed restart and check every result-copy field, including both nonlinear-flag values. The controller test now drives synthetic current through alignment/decay and compares Park currents at both identification angles with the original `sinf`/`cosf` reference to 1e-7 A. Existing accuracy/safety/delay tolerances are unchanged.

Formatting and whitespace checks pass. Host clang-tidy has no project-code diagnostics. ARM clang-tidy exits zero with no project-code diagnostics; it emits seven `bugprone-signed-bitwise` warnings from unchanged Zephyr logging expansions (`cbprintf_internal.h:542`, attributed to the existing `LOG_*` sites in `foc_bench.c`). Running the baseline bench source through the same database reproduces the same seven warnings. These remain an exception to the requested fully clean output; no diagnostics were filtered or disabled. `src/main.c` was not changed or analyzed in this task.

The ARM analysis database under ignored `build/wcet-tidy` copies the real build database and removes only Clang-unsupported GCC switches: `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, `-specs=picolibc.specs`. It preserves target/includes/defines and all repository check policy. One overlapping pristine rebuild briefly removed generated headers during analysis; the final scan ran after the completed build and passed. Raw build/disassembly/analysis logs and the temporary fixture harness remain under ignored `build/wcet-evidence`.

The referenced `/home/gtc/.pi/agent/references/embedded-development.md` was absent. The repository AGENTS.md was read first and followed. Target DWT output, complete ISR WCET (including entry, alignment-angle transition, fit, and terminal publication), and the <=2500-cycle objective remain unmeasured because hardware access was prohibited.
