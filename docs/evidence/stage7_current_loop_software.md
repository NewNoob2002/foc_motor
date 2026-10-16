# Stage 7 current loop and two-sided alignment: software evidence

Implemented against source baseline `388da31` in task worktree `3527846f67b0`. No hardware was accessed or flashed. The host and ARM builds use the same pure-C FOC library. This report establishes software behavior and generated-code checks; it does not establish target timing or powered behavior.

## Implemented behavior

Identification now parks at 0, 120, 240, 360, 120, 0 electrical degrees, with verified zero-current decay between moves. The original `enc_360` is park A; `enc_zero_back` is park B. The two extra moves must have the original forward sign and the opposite sign respectively, each within the existing 200 electrical-count magnitude tolerance. The offset is the circular mean of `(enc * 7) mod 4096` at A/B, directed into the calibrated frame. `friction_half_e` records the signed shortest electrical B-A displacement divided by two. R/L identification still starts at zero degrees, with the rotor at B. The absolute run bound is now 460000 periods (23 s), exceeding the conservative phase sum of 454002 periods.

Powered mode 4 requires a successful per-boot calibration, strict `0.02 < R < 2` ohm and `10 uH < L < 5 mH`. Calibration retains R/L and precomputes `kp = L * 2*pi*1000`, `ki = R * 2*pi*1000`. For the specified motor these are 0.439823 V/A and 1256.637 V/(A s). Both transforms use the calibrated rotor angle. Mode 3 remains refused while powered. Off-state mode 4 uses raw encoder angle and placeholder gains, including after disarming without changing mode; its PI history is cleared while power is off.

`foc_current_step_lim` limits each PI output/integral to `+/-min(v_lim, vbus/sqrt(3))`, then scales the dq vector together to `v_lim`. Firmware passes the existing 1 V guard cap. The existing `foc_current_step` implementation and contract are preserved. The new function reports PI/vector saturation as well as SVPWM limiting.

The debugger block retains offsets +0 through +20, appending `id_ma` at +24 and `iq_ma` at +28. Mode entry clears references/integrals and takes the current sequence as its baseline. Write both references before changing `step_seq`; the accepted request clamps id to +/-1000 mA and iq to +/-500 mA and starts a kind-1 capture. As with the existing voltage capture, a busy or reader-owned buffer defers another sequence until release. Later reference edits do not affect the active capture. Header old/new references are signed mA. The shared current arrays mean id/iq for kind 1; `capdq` reports those samples. Kind-0 output is unchanged.

The PRE timing follows the existing PWM preload contract: sample `i[k]` is already measured when `u[k]` is computed, and `u[k]` starts at top `k+1`. New reference control is therefore computed at PRE-1 and applied at top PRE. Samples 0 through PRE-1 retain the old excitation; in the locked RL plant the first changed current sample is PRE+1. The capture header identifies PRE as the applied command boundary.

Speed sums signed shortest encoder increments over 200 periods, including mechanical/raw-counter wrap, using integer arithmetic. A powered mode-4 estimate strictly outside +/-205 counts/10 ms emits `PWM_ADC_FAULT_OVERSPEED = 0x100` through `cmd->trip`. BSP review confirms `trip`, `fault_latch(uint32_t)` and `stats.fault` are 32-bit, with no 8-bit mask: `cmd.trip != 0` calls `fault_latch(cmd.trip)` before duty application, switching SD low, PC analog, MOE off and CCR to 50%. The arm/idle sequence, 2 A phase trip, VBUS checks and supervisor are unchanged. The 1 Hz report includes dq references/measurements, dq voltages, gains, signed speed in rpm and limiting.

## Host verification

Tests were added before implementing the limited math API. The unchanged old-function manual-chain tests still require bit-identical outputs and integrals. New tests cover the capped manual chain, per-axis/vector limits, saturated-integral recovery, invalid bus/cap behavior, calibration permission boundaries including NaN, both reference clamp polarities, sequence entry/latching, capture metadata/ownership/PRE timing, powered/off-state gains and integral reset, signed overspeed thresholds, encoder wrap, and two-sided alignment sign/magnitude/circular-mean validation. Dirty restart and result-copy checks cover the new alignment fields.

Commands:

```sh
cmake -S tests/host/foc -B build/host-cc
cmake --build build/host-cc
ctest --test-dir build/host-cc --output-on-failure
ASAN_OPTIONS=detect_leaks=0 build/host-cc/test_current
ASAN_OPTIONS=detect_leaks=0 build/host-cc/test_foc_ctrl_calib
ASAN_OPTIONS=detect_leaks=0 build/host-cc/test_motor_ident
```

Final CTest tail (warnings-as-errors, ASan/UBSan):

```text
    Start 6: test_foc_ctrl_calib
6/8 Test #6: test_foc_ctrl_calib ..............   Passed    0.06 sec
    Start 7: test_motor_ident
7/8 Test #7: test_motor_ident .................   Passed    0.49 sec
    Start 8: vec_compare_selftest
8/8 Test #8: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 8

Total Test time (real) =   1.26 sec
```

Locked-rotor plant: R=0.2 ohm, L=70 uH, exact discrete RL integration, 50 us sample period, one full PWM period of delay, zero back-EMF, 0.3 A reference step. Rise time includes the preload delay from command computation. The production fixture additionally quantizes phase currents with the board ADC scale.

```text
locked rotor iq: rise90 0.250 ms, overshoot 2.191 %
locked rotor id: rise90 0.250 ms, overshoot 2.191 %
test_current: all checks passed
production iq rise90 0.250 ms, overshoot 2.984 %
production id rise90 0.250 ms, overshoot 2.335 %
test_foc_ctrl_calib: all checks passed
```

The sinusoidal torque/stiction plant has a normalized static-friction threshold of 0.1. Across ten noisy starting-position/motor cases per encoder direction, the old forward-only error is 64–65 electrical counts and the two-sided error is 1–2 counts. Tests require the old error >=50 counts, the new error <=8 counts (7-count encoder quantum), and an improvement greater than 5x. Observed improvement is at least 32x for both directions. Representative tail:

```text
two-sided dir 1: one-sided 65.0 -> mean 2.0 electrical counts, band 63
R 0.200 L 300 uH dir 1 start 300 deg: state 11 reason 0, R 0.201499 L 294.345 uH, ipp 0.136 A, 7.678 s
two-sided dir 1: one-sided 65.0 -> mean 1.0 electrical counts, band 66
test_motor_ident: all checks passed
```

## ARM build and generated code

Commands:

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-cc . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-cc/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm build/arm-cc/zephyr/zephyr.elf | grep foc_ctrl_cmd
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d build/arm-cc/zephyr/zephyr.elf > build/arm-cc.dis
```

Build/size/symbol tails:

```text
[183/183] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       73856 B       128 KB     56.35%
             RAM:         11 KB        32 KB     34.38%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
   text    data     bss     dec     hex
  73288     564   10608   84460   149ec
20001504 B foc_ctrl_cmd
```

`text+data = 73852 <= 98304`; `data+bss = 11172 <= 24576`. The CI artifact, allocation-symbol, four shutdown-hog, heap, stack-size, stack-protection and build-version checks all pass. FLASH region usage includes four additional alignment bytes, which explains its difference from text+data.

Disassembly inspection covers `foc_ctrl_step`, `foc_current_step_lim`, `motor_ident_start`, `motor_ident_result_copy`, `motor_ident_step`, `run_current` and `dma_isr`. None contains memset/memcpy, fminf/fmaxf, or `__aeabi_*` calls. The limited step calls only Clarke, Park, two PI steps, inverse Park and SVPWM. Its conditional vector clamp lowers to hardware `vsqrt.f32` and `vdiv.f32`. The speed/angle path is integer arithmetic; gain calculation is multiplication only when storing calibration. There is no new periodic gain/speed division. Existing voltage-mode cap division remains outside the mode-4 branch.

The boot bench adds `current capped`, timing the production limited current kernel with the same calibrated-angle and speed-window helpers used by the ISR. Static synthetic fixtures cover ordinary/rollover windows, the full electrical angle range and both unsaturated/saturated numeric inputs; setup is outside timing. It does not call the BSP or modify debugger commands. The benchmark reports its maximum alongside the full ISR budget, while normal ISR telemetry continues to record the complete execution maximum.

An engineering estimate for the worst mode-4 ISR is approximately **3000–3300 cycles (17.6–19.4 us at 170 MHz)**. This uses the existing Stage 6 measured 2344-cycle maximum as an anchor and allows roughly 650–950 additional cycles for Stage 7 guard checks, calibration/gain selection, speed rollover, capture start/sample bookkeeping, extra snapshot fields and saturated vector limiting. Entry and capture start are separate paths because entry baselines the sequence. This is an estimate, not a static WCET proof: flash fetch state, bus stalls, exception/FPU stacking and all transition paths still require target DWT evidence. The mandatory <=3400-cycle complete-ISR limit remains unmeasured under this task's hardware prohibition.

## Formatting and static analysis

```sh
clang-format --dry-run --Werror lib/foc/include/foc/foc_current.h lib/foc/src/foc_current.c src/foc_ctrl.c src/foc_ctrl.h src/foc_ctrl_motion.h src/motor_ident.c src/motor_ident.h src/pwm_adc.h src/main.c src/foc_bench.c tests/host/foc/test_current.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/test_motor_ident.c
clang-tidy -p build/host-cc lib/foc/src/foc_current.c src/foc_ctrl.c src/motor_ident.c tests/host/foc/test_current.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/test_motor_ident.c
clang-tidy -p build/cc-tidy src/foc_ctrl.c src/motor_ident.c src/main.c src/foc_bench.c lib/foc/src/foc_current.c
git diff --check
```

Formatting and whitespace checks pass. Host clang-tidy exits zero without project diagnostics. ARM clang-tidy exits zero with only the allowed existing `main.c:55` branch-clone and Zephyr logging-macro signed-bitwise diagnostics. No diagnostic policy was changed or disabled. For Clang compatibility, `build/cc-tidy/compile_commands.json` copies the ARM database and removes only GCC-only `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, and `-specs=picolibc.specs`; target, includes, defines and repository check configuration are retained.

Raw logs/disassembly remain under ignored `build/host-cc.log`, `build/arm-cc.log`, `build/host-cc-tidy.log`, `build/arm-cc-tidy.log`, `build/ident-cc.log`, and `build/arm-cc.dis`.

## Scope and remaining evidence

Changed files: `lib/foc/include/foc/foc_current.h`, `lib/foc/src/foc_current.c`, `src/foc_ctrl.c`, `src/foc_ctrl.h`, new `src/foc_ctrl_motion.h`, `src/motor_ident.c`, `src/motor_ident.h`, `src/pwm_adc.h`, `src/main.c`, `src/foc_bench.c`, `tests/host/foc/test_current.c`, `tests/host/foc/test_foc_ctrl_calib.c`, `tests/host/foc/test_motor_ident.c`, and this evidence file.

No functional deviation from the specification is intended. Capture requests follow the existing buffer ownership rule described above. Target DWT timing, boot benchmark output and powered/HIL verification were deliberately not performed. The globally referenced `/home/gtc/.pi/agent/references/embedded-development.md` was absent; repository AGENTS.md and existing evidence were followed. No verification policy, main checkout, other worktree, credentials or hardware state was changed.
