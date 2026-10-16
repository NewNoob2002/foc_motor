# CORDIC, dead-time compensation and rotating offset refinement

Software-only evidence for task `930251dae592`, based on `8c0cd9a`. No firmware was flashed and no hardware state was changed. The task specification was read completely from `/home/gtc/.claude/jobs/932f74da/tmp/dtc_cordic_offset_spec.md`.

## Changes

- `src/sincos.c`, `src/sincos.h`: BSP-owned CORDIC, reset/unity modulus, COSINE, six clocks/24 iterations, scale zero, one q1.31 argument and two q1.31 results. The inline transaction writes WDATA once and reads RDATA twice. RM0440 configuration/zero-overhead behavior and LL definitions are cited in the code. Ownership is exclusive to the boot bench before the ISR starts, then the FOC ISR.
- `src/foc_ctrl.c`, `src/foc_ctrl.h`: every control angle uses the BSP API; identification constants are removed. Calibration retains V0 and the refined offset. DTC defaults off, requires strictly `0.05 < V0 < 0.6 V`, uses reference-current phase signs and a 50 mA band, and latches with references at top PRE. Command offsets +0 through +28 are unchanged; `dtc` is +32. Capture/snapshot report accepted/refused state.
- `lib/foc/include/foc/foc_current.h`, `lib/foc/src/foc_current.c`: pure-C dead-time compensation, a small inline reciprocal wrapper and a shared bounded-PI helper. The new feed-forward path reserves `max(0.2 V, 1 V - |comp|)` for PI, adds compensation before one SVPWM operation, and caps the complete applied vector. Existing current-loop functions retain their contracts. The production reciprocal is constant-folded to 20 A^-1; no division occurs in the per-phase compensation implementation.
- `src/motor_ident.c`, `src/motor_ident.h`: after L, rotate forward/backward exactly two electrical turns per direction at 2048 count/s. Discard the first quarter-turn in each direction. Wrapped error accumulation, phase integration and final correction are integer operations. The accumulator bounds justify int32 sums without int64 helpers. Slip, correction and load checks fail with IDENT_FAIL_ALIGNMENT. Result publication/reset includes correction, load and refined offset. Calibration consumes `offset_e_new`.
- `src/foc_bench.c`, `src/main.c`: initialize CORDIC before bench/ISR; sweep all 4096 counts over eight passes for timing and errors against float/double libm; benchmark the CORDIC+DTC capped kernel and complete unpowered mode-4 control; report DTC and rotation diagnostics with units.
- `CMakeLists.txt`, `tests/host/foc/CMakeLists.txt`: register BSP source and new host test; host control selects the same API from `tests/host/foc/stubs/sincos.h`.
- `tests/host/foc/test_deadtime.c`: per-phase signs, linear zone, saturation, zero vector, 4/3 Vdt alpha vector, randomized manual feed-forward comparison, total cap, reduced-limit anti-windup, and delayed per-phase inverter plant.
- `tests/host/foc/test_motor_ident.c`: rotation timing/settling/sign/wrap, strict integer plausibility boundaries, slip edges, publication/reset, L-to-rotation transition, extended timeout and cogging offset fixture for both encoder polarities.
- `tests/host/foc/test_foc_ctrl_calib.c`: refined calibration handoff, V0 acceptance/rejection, DTC defaults, sequence/PRE timing and capture state.

Every newly implemented C function has English Doxygen documentation. No verification policy or orchestrator configuration was changed.

## Commands and observed results

The required host command was executed, with ASan/UBSan and warnings-as-errors supplied by the existing host CMake configuration:

```sh
cmake -S tests/host/foc -B build/host-dtc && cmake --build build/host-dtc && ctest --test-dir build/host-dtc --output-on-failure
```

Final tail:

```text
9/9 Test #9: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 9

Total Test time (real) =   2.08 sec
```

Simulation output was printed separately with:

```sh
ASAN_OPTIONS=detect_leaks=0 build/host-dtc/test_deadtime
ASAN_OPTIONS=detect_leaks=0 build/host-dtc/test_motor_ident
```

Relevant output:

```text
inverter DTC off: rise90 1.100 ms, overshoot -0.000 %
inverter DTC on: rise90 0.150 ms, overshoot 1.349 %
test_deadtime: all checks passed
cogging dir -1: park error -112.0 -> rotation error 0.0 counts; c -112 load 84, state 12 fail 0
cogging dir 1: park error -112.0 -> rotation error 0.0 counts; c -112 load 84, state 12 fail 0
...
test_motor_ident: all checks passed
```

The inverter parameters were fixed at R=0.2 ohm, L=70 uH, Vdt=0.175 V, tanh band 0.03 A, one full period of command delay and a 0.5 A id step. They were not adjusted to match the measured 1.05 ms rise. The compensated rise satisfies <=0.5 ms and <=20% overshoot.

The cogging fixture extends the existing torque/stiction plant with `0.25*sin(12*theta_e + 2.7)` torque: 12 electrical detents equal 84 mechanical detents for seven pole pairs. It parks physically in a detent first, then exercises the real post-L refinement with either encoder polarity. Friction threshold is 0.01 for this fixture, versus 0.1 in the existing stiction tests. It isolates offset refinement from earlier alignment/R/L phases; it does not assert that every initial condition in this strong cogging model passes the existing two-sided alignment checks. The separate complete identification tests still exercise both encoder polarities and multiple initial rotor positions.

The required pristine ARM command was executed:

```sh
source /home/gtc/zephyrproject/.venv/bin/activate; ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-dtc . -- -DBUILD_VERSION=v4.4.2
```

Final tail:

```text
[184/184] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       76812 B       128 KB     58.60%
             RAM:       11392 B        32 KB     34.77%
           SRAM0:           0 B        32 KB      0.00%
        IDT_LIST:           0 B        32 KB      0.00%
```

Size and command-address commands:

```sh
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-size build/arm-dtc/zephyr/zephyr.elf
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm build/arm-dtc/zephyr/zephyr.elf | grep foc_ctrl_cmd
```

```text
   text    data     bss     dec     hex
  76244     564   10764   87572   15614
2000159c B foc_ctrl_cmd
```

`text+data = 76808 <= 98304 B`; `data+bss = 11328 <= 24576 B`. The same existing CI artifact, allocation-symbol, four shutdown-hog, heap, stack-size, stack-protection and build-version gates pass (`python3 build/dtc-gates.py`).

Final-image audit:

```sh
/home/gtc/zephyr-sdk-1.0.1/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump -d build/arm-dtc/zephyr/zephyr.elf > build/dtc-arm-disassembly.txt
python3 build/dtc-audit.py
```

The direct/tail-call graph rooted at foc_ctrl_step contains no sinf, cosf, sincosf, memset, memcpy, fminf or fmaxf calls. It has no indirect function calls; switch table branches stay within their containing functions. The inline CORDIC path uses CMSIS base `0x40020c00`, a shift left by 20, one store at +4 (WDATA), two volatile loads at +8 (RDATA), signed-int-to-float conversions and multiplication by 2^-31. Compensation uses the precomputed reciprocal, with no vdiv in foc_deadtime_comp_inv. Startup/bench libm and memory calls remain outside the ISR call graph.

Formatting and analysis commands:

```sh
clang-format --dry-run --Werror src/sincos.c src/sincos.h src/foc_ctrl.c src/foc_ctrl.h src/foc_bench.c src/main.c src/motor_ident.c src/motor_ident.h lib/foc/src/foc_current.c lib/foc/include/foc/foc_current.h tests/host/foc/test_deadtime.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/stubs/sincos.h
clang-tidy -p build/dtc-tidy src/sincos.c src/foc_ctrl.c src/motor_ident.c src/foc_bench.c src/main.c lib/foc/src/foc_current.c
clang-tidy -p build/host-dtc tests/host/foc/test_deadtime.c tests/host/foc/test_motor_ident.c tests/host/foc/test_foc_ctrl_calib.c
git diff --check
```

Formatting/whitespace checks pass. Host tidy has no project diagnostics. ARM tidy has only 47 permitted warnings: 46 signed-bitwise diagnostics from Zephyr logging expansions and the existing stack-definition branch-clone diagnostic (main.c:55 in the baseline, now :56 due to the added include). No check was disabled. The analysis database is a copy of the ARM build database with an explicit Clang ARM target and only these unsupported GCC switches removed: `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, `-specs=picolibc.specs`.

Raw logs, analysis database and audit scripts are retained under ignored `build/dtc-*`; the build outputs are `build/host-dtc` and `build/arm-dtc`. Earlier development failures (missing new API before implementation, and strong-cogging full-alignment fixture rejection) were resolved; the final full host suite passes.

## Deviations and remaining validation

1. The explicit speed/turn requirements imply **8 s**, not approximately 4 s: 2048/4096 = 0.5 electrical turns/s, and four turns total take eight seconds. The implementation preserves the requested speed and turns, adds 160000 periods, and increases the absolute run timeout to 620000 periods (31 s; phase maxima <614002).
2. Boot benchmarking measures the complete **unpowered** mode-4 control function and separately the production capped CORDIC+DTC kernel with identified gains, angle and speed helpers. It does not manufacture a successful powered calibration in the production controller for benchmarking.
3. CORDIC error/latency sweep and ISR WCET instrumentation are implemented but were **not run on target**. The <=3400-cycle requirement, peripheral accuracy, actual current response and physical offset improvement remain HIL checks. Host tests and ARM disassembly do not establish those measurements.
4. The global guidelines reference `/home/gtc/.pi/agent/references/embedded-development.md`; that file is absent in this environment. Repository AGENTS.md and existing project conventions were followed.
