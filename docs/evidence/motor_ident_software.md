# Motor identification: software verification

This change implements `FOC_MODE_IDENT = 5`: 90-degree alignment, current decay before switching to zero degrees, five current-regulated R points, a return to the 1 A bias, and delayed square-wave L measurement. It does not change `power_guard` limits or arm conditions. No firmware was flashed and no hardware was accessed. These are software/model results, not HIL or measured ISR timing evidence.

## Contracts and bounded work

`motor_ident_step()` owns fixed storage and takes one sample per 50 us PWM period. The current passed by control is Park-transformed at the angle requested on the previous call. The output voltage of call k starts at counter top k+1, so the increment i[k+2]-i[k+1] belongs to output k. Injection keeps the last two samples for its final increments, then returns zero. DONE and FAILED remain latched until the next mode-entry restart.

The servo uses Ki = 5 V/(A s), 0..0.9 V saturation, a 5 ms current filter, a continuous 20 ms arrival band, and a 1 s arrival timeout for every target. A 10 s absolute timeout also bounds the whole run. A d-axis sample above 1.8 A, nonfinite current, disarm or loss of power aborts with zero voltage. The unchanged phase-current guard remains the separate 2 A protection. Leaving IDENT aborts with CANCELLED; results retain completed measurements.

Terminal results use a separate fixed buffer and atomic get/release handoff. A new run can restart on a mode edge without modifying a result held by the report thread. If a completed run is waiting behind an older unread result, it publishes after release; repeated further mode-edge restarts can supersede that unpublished run, since this is a single-result interface rather than a queue.

Normal servo/injection steps use roughly 8–15 single-precision arithmetic operations, plus comparisons, clamp operations and fixed integer bookkeeping. There are no per-period array loops or transcendental operations in the module. The R fit runs once: three loops of four points (12 bounded iterations), roughly 60 arithmetic operations including divisions. Reset/terminal publication also have fixed-size zeroing/copy costs. The ARM object has only `fminf`, `fmaxf`, and compiler-generated `memset` as undefined symbols, with no RTOS or allocation dependency. Existing Clarke/Park, sine/cosine, inverse Park and SVPWM costs remain in control. Actual total ISR WCET, including fit, reset and publication, still needs DWT measurement on the target; no timing budget is claimed as hardware-verified.

## Specification discrepancies and implementation choices

- The required successful `(1 ohm, 1 mH)` test conflicts with the required 0.9 V servo clamp. ALIGN alone needs about 1.15 V with the specified inverter loss at 1 A, and the final R point needs about 1.65 V. Both encoder directions therefore test SERVO_TIMEOUT at 1 s with zero output. The 5 ohm unreachable case also times out. Protection limits were preserved.
- The L injection begins only after restoring and settling the current to 1 A with the servo. Starting directly from the final 1.5 A R point caused the specified 100 uH plant to exceed the 1.8 A abort threshold on its first positive pulses, as shown by the initial host run. The additional `IDENT_L_BIAS` phase prevents that transient; the frozen injection bias still comes from the recorded 1 A R point.
- The fit always uses the four nominal targets at or above 0.75 A. Selecting by noisy measured averages could drop the 0.75 A point and conflict with the specified four-point fit. All five measured points remain in the result.
- Reverse encoder offsets follow `theta_e = (direction * enc * 7 - offset_e) mod 4096`; this reduces to the specified `(enc0 * 7) mod 4096` for positive direction. Both raw TIM3 readings are preserved; wrapping is used only for displacement and offset arithmetic.
- ADC noise is fixed-seed uniform noise of SD 0.1 A followed by 16 mA quantization. The specification gives an SD but does not require Gaussian noise. The plant uses exact exponential RL propagation per period with inverter loss evaluated at the initial current of that period.
- The referenced `/home/gtc/.pi/agent/references/embedded-development.md` was absent. Repository AGENTS.md and the existing firmware interfaces were inspected instead.

## Host verification

Executed:

```sh
cmake -S tests/host/foc -B build/host-ident && cmake --build build/host-ident && ctest --test-dir build/host-ident --output-on-failure
```

Tail:

```text
    Start 5: test_foc_ctrl
5/7 Test #5: test_foc_ctrl ....................   Passed    0.03 sec
    Start 6: test_motor_ident
6/7 Test #6: test_motor_ident .................   Passed    0.05 sec
    Start 7: vec_compare_selftest
7/7 Test #7: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 7

Total Test time (real) =   0.11 sec
```

All six preexisting registered tests and the new test pass with `-Wall -Wextra -Werror`, ASan and UBSan. Existing optional exercise tests retain their original registration policy.

Direct `build/host-ident/test_motor_ident` output:

```text
R 0.100 L 100 uH dir -1: state 7 reason 0, R 0.102267 L 99.788 uH, ipp 0.401 A, 4.409 s
R 0.200 L 300 uH dir -1: state 7 reason 0, R 0.203813 L 307.273 uH, ipp 0.130 A, 4.783 s
R 0.100 L 100 uH dir 1: state 7 reason 0, R 0.102267 L 99.788 uH, ipp 0.401 A, 4.409 s
R 0.200 L 300 uH dir 1: state 7 reason 0, R 0.203813 L 307.273 uH, ipp 0.130 A, 4.783 s
test_motor_ident: all checks passed
```

The independent delay test uses an exact integrator with a known L, checks the total accepted increment count, and verifies L within 0.1% (allowing float32 summation). Correlation with either adjacent wrong sign history gives half the correct sum, so either off-by-one error would approximately double the inferred L. Other cases cover nonfinite current, overcurrent, power loss, terminal latching/restart, total timeout, stuck encoder, nonpositive L response, excessive ripple and the nonlinear R flag. Controller tests cover powered IDENT permission, immediate NOT_POWERED while off, disarm, cancellation, result ownership and unchanged refusal of Stage 6/unknown modes while powered.

## ARM verification

Executed:

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-ident . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-ident/zephyr/zephyr.elf
```

Tail:

```text
[183/183] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       70080 B       128 KB     53.47%
             RAM:       10624 B        32 KB     32.42%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
Generating files from .../build/arm-ident/zephyr/zephyr.elf for board: foc_motor/stm32g431xx
   text    data     bss     dec     hex
  69508     564    9956   80028   1389c
```

`text + data = 70,072 B <= 98,304 B`; `data + bss = 10,520 B <= 24,576 B`. The linker region totals include alignment/region accounting beyond these size-tool sums.

## Format and static analysis

Executed formatting and its required dry run on every changed C/header file:

```sh
clang-format -i src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/foc_ctrl.h src/main.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-format --dry-run --Werror src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/foc_ctrl.h src/main.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
git diff --check
```

The dry run and whitespace check both exit 0 with no diagnostic output.

Direct `clang-tidy -p build/arm-ident src/motor_ident.c src/foc_ctrl.c src/main.c` initially fails because Clang 23 does not accept GCC-only arguments. For analysis only, `build/ident-tidy/compile_commands.json` is a copy of the ARM database with these five arguments removed: `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, `-specs=picolibc.specs`. The ARM build database and all repository verification policy remain unchanged.

Executed against that copied database:

```sh
python3 build/ident-tidy/prepare.py
clang-tidy -p build/ident-tidy src/motor_ident.c src/foc_ctrl.c src/main.c
python3 build/ident-tidy/filter_macros.py
clang-tidy -p build/ident-tidy --line-filter="$(< build/ident-tidy/line-filter.json)" src/motor_ident.c src/foc_ctrl.c src/main.c
```

The unfiltered scan exits 0 with the existing `main.c:55` stack warning and 26 `bugprone-signed-bitwise` diagnostics expanded from Zephyr logging macros. Each of the latter was verified to originate from `cbprintf_internal.h:542` and a `LOG_*` invocation. A second scan uses a line filter excluding only those verified invocation lines. All check groups remain enabled; the full first scan also checks their arguments. Preparation/filter scripts and full logs remain local under ignored `build/`.

Relevant filtered tail (exit 0):

```text
.../src/main.c:55:1: warning: conditional operator with identical true and false expressions [bugprone-branch-clone]
   55 | K_THREAD_STACK_DEFINE(report_stack, REPORT_STACK_SIZE);
      | ^
...
Suppressed 8615 warnings (371 in non-user code, 8244 due to line filter).
Use -header-filter=.* or leave it as default to display errors from all non-system headers. Use -system-headers to display errors from system headers as well.
```

No new first-party source diagnostic remains. Hardware alignment/current-sense polarity, real inverter/model accuracy, waveforms and target WCET are unverified because hardware access was explicitly prohibited.
