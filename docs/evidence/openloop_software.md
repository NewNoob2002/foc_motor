# Open-loop rotation: software verification

`FOC_MODE_OPENLOOP = 6` applies a rotating d-axis voltage vector while reporting the rotor angle from this boot's identification. It is permitted with power on or off, as DC is; the existing explicit-idle arming sequence, trips, and 1.0 V powered guard cap still apply. Unknown modes remain refused. This change was built and tested only in the task worktree; no hardware was accessed or firmware flashed.

## Calibration and angle integration

The ISR saves `calib_direction` and `calib_offset_e` when identification reaches `IDENT_DONE`, independently of the terminal-result buffer's report-thread ownership. `calib_valid` starts false, becomes true on success, and clears on every new IDENT entry and controller initialization. Partial/failed identification cannot validate the calibration. There is no persistence across boots: the incremental encoder has a new origin after power-up.

The measured angle is `(direction * enc_count * 7 - offset_e) mod 4096`, normalized to [0, 4095]. Control first reduces the encoder modulo 4096, then uses unsigned subtraction and a 4095 mask, so negative direction and offset subtraction wrap without signed overflow. The snapshot publishes validity, direction, offset, and `theta_meas_count`; the latter three are meaningful only when valid. The measured angle is diagnostic; other modes retain their previous angle behavior.

On OPENLOOP entry, the ISR loads `cmd.theta_e mod 4096` into a 32-bit Q16 electrical-count phase and resets its signed fractional remainder and voltage ramp. The first output uses that angle; subsequent outputs advance once per 50 us period. Each step clamps the signed rate to +/-4096 counts/s, adds `rate * 65536` to the remainder, divides by 20000 for the Q16 increment, retains the division remainder, and wraps the phase modulo `4096 * 65536`. This keeps the rate exact over time instead of rounding each period's increment; all intermediate signed arithmetic fits int32. Runtime rate edits preserve phase, whereas theta edits take effect on the next mode entry. The ARM compiler emits one integer `sdiv` and a multiply/subtract for the remainder; there is no floating-point division in angle integration.

The ramp shares DC's 1 V/s update and clamps the target to `FOC_CTRL_V_MAX_MV = 1000`. Its state starts at zero, so the first period applies at most 50 uV, then ramps toward the requested voltage; target reductions also ramp. The existing guard cap still limits the powered vector. Output q voltage is zero.

## Debugger command layout and telemetry

`foc_ctrl_cmd` remains a debugger command block. Compile-time assertions protect every field offset. Its final ARM symbol is `0x20001410`, size 24 bytes; addresses are specific to this build.

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| +0 | uint32_t | arm | Existing `0x41524D21` arm key |
| +4 | uint32_t | mode | 6 selects open-loop rotation |
| +8 | uint32_t | v_mv | Vector magnitude in mV, clamped to 1000 |
| +12 | uint32_t | theta_e | Initial electrical angle on mode entry |
| +16 | uint32_t | step_seq | Existing STEP capture sequence, unchanged |
| +20 | int32_t | rate_e | Signed electrical counts/s, clamped to +/-4096 |

4096 counts/s is one electrical revolution/s, or 1/7 mechanical revolution/s for this motor. Rate zero holds the initial vector angle. The startup log now prints `rate_e +20` as part of the layout.

The existing `rot <steps> <enc_mech> <theta_e> <duty_a> <duty_b> <duty_c>` line is unchanged. At its nominal 10 Hz cadence, OPENLOOP adds `ol <steps> <theta_cmd> <theta_meas> <err>`, where err wraps command minus measured angle into [-2048, 2047]. Before calibration it prints `ol <steps> <theta_cmd> invalid`. The nominal 1 Hz report also prints calibration validity, direction and offset, including while current-offset calibration is pending or rejected.

Illustrative expected output, **not bench evidence** (timestamps/log prefixes omitted):

```text
ol 20000 100 invalid
  calib_valid 0, direction 0, offset_e 0 electrical counts
ol 240000 100 96 4
  calib_valid 1, direction -1, offset_e 123 electrical counts
ol 242000 20 4090 26
```

A successful rotation check should show the measured angle following the command through wraps, with bounded tracking error. Load, friction and loss of synchronization can create a real angle error; the software tests do not establish physical tracking.

## Executed checks

```sh
cmake -S tests/host/foc -B build/host-ol && cmake --build build/host-ol && ctest --test-dir build/host-ol --output-on-failure
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-ol . -- -DBUILD_VERSION=v4.4.2
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-ol/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm -S build/arm-ol/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d build/arm-ol/zephyr/zephyr.elf
clang-format -i src/foc_ctrl.c src/foc_ctrl.h src/main.c tests/host/foc/test_foc_ctrl.c tests/host/foc/test_foc_ctrl_calib.c
clang-format --dry-run --Werror src/foc_ctrl.c src/foc_ctrl.h src/main.c tests/host/foc/test_foc_ctrl.c tests/host/foc/test_foc_ctrl_calib.c
clang-tidy -p build/host-ol src/foc_ctrl.c tests/host/foc/test_foc_ctrl.c tests/host/foc/test_foc_ctrl_calib.c
python3 build/prepare-ol-tidy.py
clang-tidy -p build/ol-tidy src/foc_ctrl.c src/main.c
python3 build/check-ol-evidence.py
git diff --check
```

Host tail, with warnings-as-errors and ASan/UBSan:

```text
    Start 5: test_foc_ctrl
5/8 Test #5: test_foc_ctrl ....................   Passed    0.68 sec
    Start 6: test_foc_ctrl_calib
6/8 Test #6: test_foc_ctrl_calib ..............   Passed    0.02 sec
    Start 7: test_motor_ident
7/8 Test #7: test_motor_ident .................   Passed    0.41 sec
    Start 8: vec_compare_selftest
8/8 Test #8: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 8

Total Test time (real) =   1.13 sec
```

The rate test compares every output over ten simulated seconds with an independent elapsed-time calculation, requiring at most one electrical count of error. It covers +4096, -4096, +100, -100, zero, +/-100000, INT32_MIN and INT32_MAX; initial wrapping, live theta edits, rate reversal and reentry are also covered. Powered and unpowered tests check the ramp, lower target, one-volt cap, d-axis vector and mode permission. Calibration tests cover both directions, offsets 0/1/2048/4095, encoder wrap and raw counts up to UINT32_MAX, invalid/running/failed states, validity after success, restarting while an old result is held, replacement calibration behind an unread result, and initialization clearing valid calibration.

The calibration test uses a linker wrapper to supply one minimal successful `motor_ident_step` result at the module boundary. All other calls use the real module. It tests the production controller and its private angle helper through snapshots without adding a firmware test API. It does not simulate a successful physical identification; the existing `test_motor_ident` plant tests remain responsible for module accuracy.

ARM tail:

```text
[183/183] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       71456 B       128 KB     54.52%
             RAM:       11008 B        32 KB     33.59%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
   text    data     bss     dec     hex
  70884     564   10356   81804   13f8c
20001410 00000018 B foc_ctrl_cmd
```

`text + data = 71448 / 98304 B`; `data + bss = 10920 / 24576 B`: both gates pass. Disassembly finds no `memset`, `memcpy`, `memmove` or `__aeabi_mem*` calls in `foc_ctrl_step` or its 32-function direct-call/tail-branch graph. New ISR state and snapshot updates use scalar stores; no aggregate initialization was added to the ISR path.

Formatting and whitespace checks pass. Host clang-tidy reports no first-party diagnostics. ARM clang-tidy reports only the known `main.c:55` stack-macro branch-clone warning and 30 signed-bitwise warnings from Zephyr logging macros, each checked against its `cbprintf_internal.h:542` expansion. No checks were disabled. As in prior evidence, an analysis-only copy of the ARM compilation database adds `--target=arm-zephyr-eabi` and removes the five Clang-unsupported GCC flags: `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, `-specs=picolibc.specs`. Build flags and verification policy are unchanged. Full logs and analysis scripts remain in ignored `build/`.

The referenced `/home/gtc/.pi/agent/references/embedded-development.md` is absent; repository rules and source contracts were used. No target execution was authorized: ISR WCET <=3400 cycles and physical rotor tracking remain unmeasured. The existing runtime timing budget and overrun protection are unchanged; software build/disassembly evidence cannot establish target WCET.
