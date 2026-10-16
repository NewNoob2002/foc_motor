# Motor identification: software verification

## Cogging-tolerant alignment (2026-10-15)

This section supersedes the alignment limits and timing described in the historical verification below. Bench firmware `eeaddaf` failed twice at raw counts 61401, 61160, 61036: mechanical deltas -241/-124, or about 148/76 electrical degrees. With 12 slots / 14 poles, the 84 cogging detents are spaced 30 electrical degrees apart. Alignment only establishes direction and a rough park offset; `IDENT_OFFSET_ROT` subsequently removes the park bias.

Alignment now uses **1.5 A**. Every forced move (120→240, 240→360, 360→120 forward, 120→0 backward) must retain its expected sign and satisfy `abs(7 * abs(delta) - 1365) <= 600`. The first two must additionally satisfy `abs(7 * abs(delta_a + delta_b) - 2730) <= 400`. The wider individual band admits unequal detent landings; the total-travel band rejects insufficient or excessive travel even when individual magnitudes pass. Rotation permits `abs(c) <= 600`, retaining the 1024-count per-sample slip limit and 0..600 load-angle band. The 1.8 A/four-sample abort, global 2 A trip, servo clamp, per-target timeout and 31 s overall timeout are unchanged. `IDENT_L_BIAS_A` keeps the pre-injection target at **1 A**, independently of alignment current.

Host tests retain all existing R/L and offset accuracy bounds, including the original 200-count move checks for ordinary plants. New cases cover the exact bench deltas in both polarities, earlier successful bench deltas, individual and combined tolerance edges, no motion, half travel (~120 degrees total), and the relaxed correction boundaries. The full torque/stiction plant adds an 84-detent sinusoidal term plus a zero-mean lower spatial harmonic to produce unequal parks. It integrates torque throughout alignment and rotation; it never inserts encoder errors or assigns the rotor to target positions. This regression plant approximates the bench pattern, rather than identifying the physical motor's cogging waveform.

Executed:

```sh
cmake -S tests/host/foc -B build/host-al2 && cmake --build build/host-al2 && ctest --test-dir build/host-al2 --output-on-failure
build/host-al2/test_motor_ident
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-al2 . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-size -A build/arm-al2/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm build/arm-al2/zephyr/zephyr.elf | grep foc_ctrl_cmd
clang-format --dry-run --Werror src/motor_ident.c src/motor_ident.h tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-tidy -p build/host-al2 src/motor_ident.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
git diff --check
```

Observed tails:

```text
8/9 Test #8: test_motor_ident .................   Passed    1.47 sec
9/9 Test #9: vec_compare_selftest .............   Passed    0.01 sec
100% tests passed, 0 tests failed out of 9
Total Test time (real) =   2.28 sec
strong cogging dir -1: deltas -234 -124 (144.0, 76.3 deg), offset error 14.0 counts, 14.955 s
strong cogging dir 1: deltas 234 124 (144.0, 76.3 deg), offset error 14.0 counts, 14.955 s
test_motor_ident: all checks passed
[187/187] Linking C executable zephyr/zephyr.elf
FLASH: 77264 B / 128 KB; RAM: 11392 B / 22 KB; CCM: 9112 B / 10 KB
budgets: flash 77260/98304 B; SRAM data+bss 11340/22528 B; CCM 9112/10240 B
200015a8 B foc_ctrl_cmd
```

The host suite runs with warnings-as-errors and ASan/UBSan. Both strong-cogging runs finish with the correct direction and <30-count refined offset error; ordinary noisy plants also reach 1.5 A and finish within the existing bound. Format and whitespace checks pass; clang-tidy exits 0 with seven suppressed non-user-code warnings and no first-party diagnostics. All CI artifact/resource, allocation, GPIO-default and configuration gates pass. ARM object disassembly shows only module-internal call relocations (`motor_ident_abort`, `servo`, `align_delta`), with no library calls or byte-loop memory functions. No hardware was accessed or flashed; physical alignment and target WCET remain unverified.

## Historical alignment verification

`FOC_MODE_IDENT = 5` now aligns at 0, 120, 240 and 360 electrical degrees, with current decay before every angle switch, then runs the unchanged five current-regulated R points, return to the 1 A bias, and delayed square-wave L measurement from the final zero-degree position. This corrects the direction ambiguity observed in two bench runs of firmware `a96018a`. It does not change `power_guard` limits or arm conditions. No firmware was flashed and no hardware was accessed for this correction. These are software/model results, not HIL or measured ISR timing evidence.

## Contracts and bounded work

`motor_ident_step()` owns fixed storage and takes one sample per 50 us PWM period. The current passed by control is Park-transformed at the angle requested on the previous call. The output voltage of call k starts at counter top k+1, so the increment i[k+2]-i[k+1] belongs to output k. Injection keeps the last two samples for its final increments, then returns zero. DONE and FAILED remain latched until the next mode-entry restart.

The servo uses Ki = 5 V/(A s), 0..0.9 V saturation, a 5 ms current filter, a continuous 20 ms arrival band, and a 1 s arrival timeout for every target. An 18 s absolute timeout (360000 PWM periods) also bounds the whole run. Four consecutive d-axis samples with |i_d| > 1.8 A (200 us) abort with zero voltage on the fourth step; any sample at or below the threshold resets the counter. Nonfinite current, disarm or loss of power still aborts immediately. Single-sample protection remains with the unchanged global 2 A per-phase ISR trip in `power_guard` (about 5 sigma above 1.5 A at SD 0.1 A); the identification limit only stops a slow runaway of its own servo without reacting to isolated ADC noise spikes. Leaving IDENT aborts with CANCELLED; results retain completed measurements.

Terminal results use a separate fixed buffer and atomic get/release handoff. A new run can restart on a mode edge without modifying a result held by the report thread. If a completed run is waiting behind an older unread result, it publishes after release; repeated further mode-edge restarts can supersede that unpublished run, since this is a single-result interface rather than a queue.

Ordinary servo/injection steps use float32 arithmetic, comparisons, clamps and fixed integer bookkeeping. There are no library trig calls, byte-loop `memset`/`memcpy`, or per-period divisions in the identification module. Reset and publication retain the volatile scalar-store pattern from the WCET fix. The R fit runs once with three four-point loops and two divisions; terminal L calculation has three divisions. Control uses explicit sine/cosine constants for the module's three possible angles; unsupported angles return `(0, 0)`, suppressing the applied voltage through inverse Park. Existing SVPWM and vector-cap math is unchanged. Actual total ISR WCET, including alignment validation, fit, reset and publication, still needs DWT measurement on the target; no timing budget is claimed as hardware-verified.

## Encoder alignment correction

The command counts are `round(k * 4096 / 3)` for k = 0, 1, 2, 3: **0, 1365, 2731, 0** modulo 4096. This rounds each absolute angle consistently rather than accumulating a rounded 1365-count increment. The command gaps are 1365, 1366 and 1365 electrical counts. Each hold uses the existing 1.0 A servo and 10000-period (0.5 s) dwell after arrival. Three `IDENT_ALIGN_DECAY` visits remove voltage and verify the zero-current band before switching angles.

The first zero-degree hold only parks the rotor. With sinusoidal torque and adequate settling, it leaves the rotor near stable 0 degrees or friction-held unstable 180 degrees. Switching to 120 degrees gives a +120-degree or -60-degree path to the same stable point; neither parked position is the second command's unstable equilibrium (approximately 300 degrees). A rotor initially at 300 degrees has torque magnitude about 0.866 of full torque during the first zero park, so it cannot remain there in the host model with static-friction threshold 0.1. The claim that the 120-degree pull always moves at most +/-60 degrees is incorrect; the capture argument relies on the preceding park. Finite settling, load and actual friction still require bench evidence.

Only the final three raw TIM3 readings are retained: `enc_120`, `enc_240`, `enc_360` (mechanical encoder counts). `delta_a` is the signed shortest mechanical displacement from 120 to 240 degrees; `delta_b` is from 240 to 360 degrees, each wrapped to [-2048, 2048] counts. Both must be nonzero and have the same sign, and each must satisfy `abs(7 * abs(delta) - 1365) <= 200` electrical counts. The one-count difference of the middle command gap is inside this tolerance. Positive deltas mean `direction = +1` (encoder increases with electrical angle); negative deltas mean `direction = -1`. This reverses the old sign decision for its negative 90-to-0-degree command. The known bench direction is -1; no new hardware result is asserted.

`offset_e = (direction * enc_360 * 7) mod 4096`, normalized to [0, 4095] electrical counts, so `theta_e = (direction * enc * 7 - offset_e) mod 4096` is exactly zero at the recorded final position. The report prints the three raw readings and two signed deltas with mechanical-count units, direction, and offset with electrical-count units. R/L phases, targets, fit and sample-delay association are unchanged. `src/foc_bench.c` fixtures use the initial servo and R/L states without old angle/result-field dependencies, so require no edits.

The new absolute bound covers even late successful arrivals: 13 arrival budgets (four alignment holds, three decays, five R targets, one bias target), four alignment dwells, five R settling/averaging windows, the bias dwell, and L injection including its two flush samples total at most `13*20000 + 4*10000 + 5*(4000+2000) + 4000 + 20002 = 354002` periods (17.7001 s, conservatively counting the arrival period again). `IDENT_RUN_TIMEOUT_N` increases from 200000 to 360000 periods (18 s). A host assertion also checks this bound against the phase constants.

The host plant integrates an overdamped rotor with per-period motion proportional to `sin(theta_cmd - theta_rotor) * i / I_align`, stopping when the normalized torque magnitude is below 0.1. It no longer snaps to commanded angles. Tests cover both encoder directions at starts of 0, 90, 180, 270 degrees and exactly 180 degrees from the quantized 120-degree command (3413 electrical counts = 299.970703125 degrees). The 180-degree start explicitly remains stationary through the first park, reproducing the problematic unstable start; the second command still captures it. The last start moves away during the first park, and every successful run checks capture at 120 degrees. Both plant parameter sets retain R accuracy within 5%, L within 10%, and the existing nonlinear-resistance and inverter-loss assertions. Offset must be exact at the recorded final position and within 70 electrical counts of the physical zero (the `asin(0.1)` friction band plus encoder quantization). A stationary rotor and a rotor physically stopped halfway through each move fail ALIGNMENT in both directions. Additional direct state-machine cases check independent magnitude failures, tolerance edges, opposing signs, zero deltas, raw-count preservation and wrapping.

## Alignment correction verification

Executed in the task worktree:

```sh
cmake -S tests/host/foc -B build/host-align && cmake --build build/host-align && ctest --test-dir build/host-align --output-on-failure
build/host-align/test_motor_ident
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-align . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-align/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm -S build/arm-align/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d build/arm-align/zephyr/zephyr.elf > build/arm-align-disassembly.txt
clang-format -i src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/main.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-format --dry-run --Werror src/motor_ident.c src/motor_ident.h src/foc_ctrl.c src/main.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-tidy -p build/host-align src/motor_ident.c src/foc_ctrl.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl.c
clang-tidy -p build/align-tidy src/motor_ident.c src/foc_ctrl.c src/main.c
git diff --check
```

Host tail (warnings-as-errors, ASan/UBSan):

```text
    Start 5: test_foc_ctrl
5/7 Test #5: test_foc_ctrl ....................   Passed
    Start 6: test_motor_ident
6/7 Test #6: test_motor_ident .................   Passed
    Start 7: vec_compare_selftest
7/7 Test #7: vec_compare_selftest .............   Passed

100% tests passed, 0 tests failed out of 7
```

All 20 noisy plant success cases complete in 5.735 s (0.1 ohm, 100 uH) or 6.264 s (0.2 ohm, 300 uH), with correct direction and offset. Representative output:

```text
R 0.100 L 100 uH dir -1 start 180 deg: state 9 reason 0, R 0.103664 L 99.444 uH, ipp 0.402 A, 5.735 s
R 0.200 L 300 uH dir -1 start 180 deg: state 9 reason 0, R 0.201526 L 298.004 uH, ipp 0.134 A, 6.264 s
R 0.100 L 100 uH dir 1 start 180 deg: state 9 reason 0, R 0.103664 L 99.444 uH, ipp 0.402 A, 5.735 s
R 0.200 L 300 uH dir 1 start 180 deg: state 9 reason 0, R 0.201526 L 298.004 uH, ipp 0.134 A, 6.264 s
test_motor_ident: all checks passed
```

A temporary C11/ASan/UBSan harness under ignored `build/` includes production `foc_ctrl.c` to test its private `ident_sincos` directly, without a new firmware or public test interface:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -Itests/host/foc/stubs -Isrc -Ilib/foc/include build/align-contract.c src/motor_ident.c src/power_guard.c build/host-align/libfoc.a -lm -o build/align-contract
build/align-contract
```

```text
ident_sincos contract: 3 valid and 8 invalid angles passed
```

ARM tail and sizes:

```text
[183/183] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       70992 B       128 KB     54.16%
             RAM:       10880 B        32 KB     33.20%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
   text    data     bss     dec     hex
  70420     564   10320   81304   13d98
200013f0 00000014 B foc_ctrl_cmd
```

`text + data = 70984 B`; `data + bss = 10884 B`. Disassembly shows no `sinf/cosf/memset/memcpy/fminf/fmaxf` calls in the identification module or its shared guard/transform/publication path. IDENT's mode-dispatch table selects `0x08003f9e`, which loads angle constants and bypasses normal-mode trig at `0x08003d22` / `0x08003d32`. The angle-transition path also uses constants, including its zero-vector default. Module `vdiv.f32` appears only at `0x08004740` / `0x080047dc` (once-only R fit) and `0x080048b8` / `0x080048e2` / `0x080048ee` (terminal L calculation). Reset and result copy have no calls. Existing normal-mode trig and thread/init memory functions remain outside this identification ISR path.

Format and whitespace checks exit 0 with no diagnostics. Host clang-tidy exits 0 with only suppressed non-user-code warnings. For ARM clang-tidy, `build/align-tidy/compile_commands.json` copies the real ARM database, adds the explicit Clang ARM target, and removes only the five unsupported GCC flags listed in the historical static-analysis section below; build flags and verification policies are unchanged. ARM analysis exits 0 with the existing Zephyr stack macro branch-clone warning and 27 signed-bitwise warnings from Zephyr logging macros (including the added report line), all expanding to `cbprintf_internal.h:542`. These macro diagnostics are retained in the log, not disabled. Real ISR timing and hardware alignment remain unverified.

## Specification discrepancies and implementation choices

- The required successful `(1 ohm, 1 mH)` test conflicts with the required 0.9 V servo clamp. ALIGN alone needs about 1.15 V with the specified inverter loss at 1 A, and the final R point needs about 1.65 V. Both encoder directions therefore test SERVO_TIMEOUT at 1 s with zero output. The 5 ohm unreachable case also times out. Protection limits were preserved.
- The L injection begins only after restoring and settling the current to 1 A with the servo. Starting directly from the final 1.5 A R point caused the specified 100 uH plant to exceed the 1.8 A abort threshold on its first positive pulses, as shown by the initial host run. The additional `IDENT_L_BIAS` phase prevents that transient; the frozen injection bias still comes from the recorded 1 A R point.
- The fit always uses the four nominal targets at or above 0.75 A. Selecting by noisy measured averages could drop the 0.75 A point and conflict with the specified four-point fit. All five measured points remain in the result.
- Reverse encoder offsets follow `theta_e = (direction * enc * 7 - offset_e) mod 4096`; this reduces to `(enc_360 * 7) mod 4096` for positive direction. All three alignment result readings preserve raw TIM3 counts; wrapping is used only for displacement and offset arithmetic.
- ADC noise is fixed-seed approximately Gaussian noise: the sum of 12 independent uniforms in [-0.5, 0.5], scaled by 0.1 A, then quantized to 16 mA. Its pre-quantization SD is 0.1 A, with tails beyond 3 sigma; the previous single uniform was bounded below the 1.8 A threshold at the 1.5 A target. The plant uses exact exponential RL propagation per period with inverter loss evaluated at the initial current of that period.
- The referenced `/home/gtc/.pi/agent/references/embedded-development.md` was absent. Repository AGENTS.md and the existing firmware interfaces were inspected instead.

## Historical consecutive-overcurrent fix verification

The following results predate the alignment correction above and describe the old two-angle alignment.

Executed against the revised abort rule and noise model:

```sh
cmake -S tests/host/foc -B build/host-fix && cmake --build build/host-fix && ctest --test-dir build/host-fix --output-on-failure
build/host-fix/test_motor_ident
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-fix . -- -DBUILD_VERSION=v4.4.2
clang-format --dry-run --Werror src/motor_ident.c src/motor_ident.h tests/host/foc/test_motor_ident.c
clang-tidy -p build/host-fix src/motor_ident.c tests/host/foc/test_motor_ident.c
git diff --check
```

Host tail (warnings-as-errors, ASan/UBSan; R 5% and L 10% tolerances unchanged):

```text
    Start 6: test_motor_ident
6/7 Test #6: test_motor_ident .................   Passed    0.09 sec
    Start 7: vec_compare_selftest
7/7 Test #7: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 7

Total Test time (real) =   0.19 sec
```

Accuracy output:

```text
R 0.100 L 100 uH dir -1: state 7 reason 0, R 0.102429 L 100.867 uH, ipp 0.397 A, 4.428 s
R 0.200 L 300 uH dir -1: state 7 reason 0, R 0.202654 L 302.507 uH, ipp 0.132 A, 4.798 s
R 0.100 L 100 uH dir 1: state 7 reason 0, R 0.102429 L 100.867 uH, ipp 0.397 A, 4.428 s
R 0.200 L 300 uH dir 1: state 7 reason 0, R 0.202654 L 302.507 uH, ipp 0.132 A, 4.798 s
test_motor_ident: all checks passed
```

New plant cases repeatedly inject isolated 1.9 A samples or three-sample bursts during R and require the complete run to succeed. A four-sample burst must abort on its fourth sample with OVERCURRENT and zero voltage. Additional checks cover both current signs, resets at exactly 1.8 A and at zero, counter reset on restart, and immediate NaN failure during a burst.

ARM tail:

```text
[183/183] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       70096 B       128 KB     53.48%
             RAM:       10624 B        32 KB     32.42%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
```

Format and whitespace checks exit 0 without diagnostics. clang-tidy uses the host compilation database and repository policy, exits 0 with four suppressed non-user-code warnings and no first-party diagnostics. No hardware was accessed; target timing and HIL remain unverified.

## Original implementation verification

The following sections retain the original implementation's build and uniform-noise results; the fix verification above supersedes them for the revised code.

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
