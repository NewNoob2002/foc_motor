# Stage 7 speed loop: software evidence

Implemented from baseline `b5f8127` in task worktree `860e323dfda8`. No board, debugger, power supply or other hardware was accessed. This evidence establishes host behavior, ARM build/resource compliance and generated code; target execution time remains unmeasured.

## Behavior and debugger interface

`FOC_MODE_SPEED = 7` shares mode 4's powered calibration/R/L permission, calibrated angle, current gains, DTC plausibility/switch and 1 V total vector cap. Its d reference is zero and its q reference is the speed PI output. Mode entry clears current/speed integrals, both speed references, held iq and the encoder estimator, and baselines `step_seq`. Unpowered checks use the same raw-angle/placeholder-current-gain path as mode 4. The current integrals are cleared while unpowered; the speed PI continues running for unpowered checks. Arming still requires explicit idle, so a later powered speed-mode entry resets its history.

The final ELF places `foc_ctrl_cmd` at **0x20001654**, size 44 bytes. Offsets are asserted at compile time:

| Field | Offset | Absolute address |
| --- | ---: | --- |
| arm | +0 | 0x20001654 |
| mode | +4 | 0x20001658 |
| v_mv | +8 | 0x2000165c |
| theta_e | +12 | 0x20001660 |
| step_seq | +16 | 0x20001664 |
| rate_e | +20 | 0x20001668 |
| id_ma | +24 | 0x2000166c |
| iq_ma | +28 | 0x20001670 |
| dtc | +32 | 0x20001674 |
| spd_rpm | +36 | 0x20001678 |
| spd_step | +40 | 0x2000167c |

Write `spd_rpm`, `dtc`, and `spd_step` before changing `step_seq`. `spd_rpm` clamps to +/-200 rpm. `spd_step=0` selects the ordinary 400 rpm/s ramp; `spd_step=1` requests a kind-2 step capture. Other values select the ordinary ramp. DTC retains its existing 0=off, 1=on meaning. Accepted references and DTC are latched on sequence changes; running or reader-owned captures defer subsequent sequences until release, as in mode 4.

The estimator accumulates shortest signed modulo-4096 deltas every PWM period, replaces one of ten int16 1 ms buckets every 20 periods and maintains their running sum. The resulting estimate updates at 1 kHz with 1.46484375 rpm/count resolution and about 5 ms effective window delay. Startup zero-pads the buckets. The overspeed decision retains the existing 200-period/10 ms cadence and strict +/-205-count threshold, in both modes 4 and 7. `trip_counts` holds each decision window between checks; `counts` exposes the rolling sum every millisecond. The feedback rpm is multiplied by the calibrated direction while powered, so the identified direction=-1 still gives negative feedback. Raw counts and the magnitude of the overspeed threshold are unchanged.

The pure-C speed module uses compile-time gains from **assumed, unmeasured** J=1.5e-5 kg m^2 and Kt=0.0217 Nm/A:

- ws=2*pi*10 rad/s.
- kp=(J*ws/Kt)*(2*pi/60)=0.0045482 A/rpm.
- ki=kp*ws/4=0.0714430 A/(rpm s).
- `foc_pi_step` clamps integral and output to +/-0.5 A; dt=0.001 s. The output holds for the intervening 19 PWM periods.

A kind-2 capture reuses the two existing arrays for deci-rpm and iq reference in mA. Its 256 samples share the PI's 1 kHz update. The first sample is at the next estimator tick; sample PRE=16 applies the new reference as a step instead of a ramp. The pre-trigger samples hold the previous ramped reference. Header old/new values are the old/new sequence-latched rpm targets. Recorded iq_ref is the PI command computed at that sample; PWM preload applies that command one counter top (50 us) later. The capture retains power validity across every PWM period, including between recorded samples. Buffer ownership and kinds 0/1 output are preserved.

Mode 7 adds 10 Hz `spd <steps> <ramped_ref_rpm> <rpm_x10> <iq_ref_ma> <iq_ma>` telemetry, the 1 Hz gain/latched/ramped reference report, and kind-2 `capspd <seq> <k> <decirpm> <iq_ref_ma>` output. Speed telemetry checks its 100 ms deadline during capture dumping as well as the regular report loop; the regular loop resumes on the next 100 ms clock boundary. Actual scheduling jitter remains subject to the existing report/log thread priorities.

## Host validation

Commands executed (all inside this worktree):

```sh
cmake -S tests/host/foc -B build/host-spd
cmake --build build/host-spd
ctest --test-dir build/host-spd --output-on-failure
ASAN_OPTIONS=detect_leaks=0 build/host-spd/sim_stage7_speed
cmake -S tests/host/foc -B build/host-spd-ci -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host-spd-ci
ctest --test-dir build/host-spd-ci --output-on-failure
```

Both configurations use the existing warnings-as-errors and ASan/UBSan settings. Final specified-command tail:

```text
      Start 10: test_motor_ident
10/11 Test #10: test_motor_ident .................   Passed    1.36 sec
      Start 11: vec_compare_selftest
11/11 Test #11: vec_compare_selftest .............   Passed    0.00 sec

100% tests passed, 0 tests failed out of 11

Total Test time (real) =   2.78 sec
```

The CI Debug configuration also passed all 11 tests in 2.78 s. Existing tests remain enabled. New tests cover +/-60, +/-150 and +/-200 rpm quantized encoder ramps within one count per 10 ms, both wraps, exact ring replacement, reset, 1 kHz update/hold, gains, both output limits, integral-clamping recovery, reference sequence/clamps/ramp/reentry, powered calibration rejection and unpowered permission, calibrated feedback direction, exact capture PRE/length/decimation, immutable reader ownership, deferred capture requests, DTC latching and mode-7 powered/unpowered overspeed thresholds. The existing powered-refusal fixture already includes mode 7 without calibration.

## Closed-loop plant and results

`sim_stage7_speed` is an additional registered test; the existing study simulation is preserved. It uses the production float speed estimator/PI and capped current loop with DTC. The independent double plant integrates dq cross coupling and back EMF at 1 us substeps, R=0.2 ohm, L=70 uH, Kt=0.0217, psi=Kt/(1.5*7), 15 V supply, one complete PWM-period delay, 0.175 V per-phase inverter loss, 4096-count encoder quantization, B=1e-6 Nm s, 84-detent sinusoidal cogging with 2 mNm amplitude, and 0.5 mNm Coulomb/static friction. The inverter's sign change has a 1 mA continuous transition for numerical integration; it retains the full 0.175 V loss outside that band.

Each inertia runs 0->100 rpm for one second, then 100->150 rpm for one second, carrying the complete plant/controller state between steps. Rise and peak use true mechanical speed. Overshoot is normalized to the change in reference. Settling is the end of the last out-of-band nonoverlapping 20 ms mean; 0 ms means every 20 ms mean was within +/-5 rpm, even if instantaneous startup speed was outside it. Mean and ripple RMS use 0.3–1.0 s after each step. RMS is relative to the settled mean and must remain below the explicitly selected **8 rpm** bound. The test checks the nominal rise/overshoot/settle limits, all-inertia settled means/ripple, finite plant values, voltage cap, iq_ref and measured iq <=0.5 A, and no overspeed decision in any period.

Raw simulation tail:

```text
J 1.50e-05, 0->100 rpm: rise10-90 16.95 ms, overshoot 17.34 %, settle(mean20ms) 120.0 ms, mean 100.00 rpm, ripple RMS 1.05 rpm (bound 8.0), peak |iq_ref| 0.4766 A, |iq| 0.4702 A
J 1.50e-05, 100->150 rpm: rise10-90 14.10 ms, overshoot 19.15 %, settle(mean20ms) 80.0 ms, mean 150.00 rpm, ripple RMS 0.69 rpm (bound 8.0), peak |iq_ref| 0.2602 A, |iq| 0.4138 A
J 5.00e-06, 0->100 rpm: rise10-90 5.50 ms, overshoot 50.52 %, settle(mean20ms) 0.0 ms, mean 100.00 rpm, ripple RMS 3.29 rpm (bound 8.0), peak |iq_ref| 0.4691 A, |iq| 0.4673 A
J 5.00e-06, 100->150 rpm: rise10-90 3.65 ms, overshoot 59.46 %, settle(mean20ms) 20.0 ms, mean 150.01 rpm, ripple RMS 2.08 rpm (bound 8.0), peak |iq_ref| 0.2634 A, |iq| 0.4544 A
J 4.50e-05, 0->100 rpm: rise10-90 46.25 ms, overshoot 29.32 %, settle(mean20ms) 220.0 ms, mean 99.62 rpm, ripple RMS 1.19 rpm (bound 8.0), peak |iq_ref| 0.4913 A, |iq| 0.4898 A
J 4.50e-05, 100->150 rpm: rise10-90 43.95 ms, overshoot 30.02 %, settle(mean20ms) 200.0 ms, mean 149.83 rpm, ripple RMS 0.59 rpm (bound 8.0), peak |iq_ref| 0.2672 A, |iq| 0.4142 A
sim_stage7_speed: all checks passed
```

The +/-25% overshoot requirement applies to nominal inertia only. The off-nominal transients have larger peaks, especially the one-third inertia case; the specified stability/ripple and no-overspeed criteria pass.

## ARM build, CI gates and disassembly

Commands executed:

```sh
source /home/gtc/zephyrproject/.venv/bin/activate
ZEPHYR_BASE=/home/gtc/zephyrproject/zephyr west build -p always -b foc_motor -d build/arm-spd . -- -DBUILD_VERSION=v4.4.2
test "$(git -C /home/gtc/zephyrproject/zephyr rev-parse HEAD)" = dccb09599635bdff17633fa7e9dab014b91dce90
bash build/ci-spd-gates.sh
python3 build/audit_spd.py
```

The scratch gate script extracts the complete `Check artifacts and budgets` shell block from `.github/workflows/ci.yml`, changing only `build/arm/` to `build/arm-spd/`. All artifact-presence, allocation-symbol, four output-low hogs, zero-heap, stack sizes, hardware stack protection and build-version gates pass. No workflow or verification policy was edited. The workflow's `west list zephyr` baseline lookup was attempted but this worktree's manifest returned `unknown project name/path: zephyr`; the exact required commit was instead verified directly in the explicitly specified Zephyr checkout.

Final build/size/gate tails:

```text
[188/188] Linking C executable zephyr/zephyr.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       79216 B       128 KB     60.44%
             RAM:       11520 B        22 KB     51.14%
           SRAM0:           0 B        22 KB      0.00%
             CCM:        9920 B        10 KB     96.88%
        IDT_LIST:           0 B        32 KB      0.00%
   text    data     bss     dec     hex
  78648     564   10956   90168   16038
budgets: flash 79212/98304 B; SRAM data+bss 11520/22528 B; CCM 9920/10240 B
```

Executable CCM text remains charged to flash. The CCM margin is 320 B. The FLASH memory-region figure includes four linker alignment bytes beyond the size-tool sum used by CI.

The audit disassembles the ELF with the SDK's `arm-zephyr-eabi-objdump -d`, traverses every direct call/tail branch reachable from `foc_ctrl_step`, and checks every destination is in CCM with no memset/memcpy, software arithmetic, fminf or fmaxf calls. It also checks the speed helpers contain no floating division instructions. Output:

```text
ISR control call graph: 30 reachable functions; all in CCM; no memcpy/memset, __aeabi, min/max library calls.
foc_ctrl_step: 0x1000014c
foc_ctrl_speed_reset: 0x100014ba
foc_ctrl_speed_step: 0x100014d8
foc_speed_loop_reset: 0x10001534
foc_speed_loop_update: 0x1000155c
100015a0: f7ff ff55 bl 1000144e <foc_pi_step>
Speed helpers: no floating-point division instructions. speed_prepare/speed_update are inlined into foc_ctrl_step.
20001654 B foc_ctrl_cmd
```

The estimator reset uses ten volatile halfword stores because GCC initially converted a normal zeroing loop into picolibc's byte-loop memset. No such call remains on mode entry or the periodic ISR path. Existing thread-only `foc_ctrl_init`/snapshot operations may still use memset/memcpy and are outside that call graph. Existing current-vector limiting retains its hardware FPU square root/division; no new periodic division or library call was added by the speed helpers.

The boot benchmark adds `speed 1ms` (production estimator rollover plus PI) and `mode7 full off` (the full production controller with synthetic, unpowered inputs, covering ordinary and 20th-period updates). Preparation is outside timing, all outputs stay in static numeric buffers, and debugger commands are saved/restored. Neither benchmark sends commands to the PWM BSP. Target timing values and the <=3400-cycle full ISR requirement need subsequent hardware evidence; this task cannot establish WCET by compiling or disassembling.

## Formatting and analysis

Commands executed:

```sh
clang-format -i src/foc_ctrl.c src/foc_ctrl.h src/foc_ctrl_motion.h src/foc_speed.c src/foc_speed.h src/main.c src/foc_bench.c tests/host/foc/test_speed.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/sim/sim_stage7_speed.c
clang-format --dry-run --Werror src/foc_ctrl.c src/foc_ctrl.h src/foc_ctrl_motion.h src/foc_speed.c src/foc_speed.h src/main.c src/foc_bench.c tests/host/foc/test_speed.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/sim/sim_stage7_speed.c
clang-tidy -p build/host-spd src/foc_ctrl.c src/foc_speed.c tests/host/foc/test_speed.c tests/host/foc/test_foc_ctrl_calib.c tests/host/foc/sim/sim_stage7_speed.c
clang-tidy -p build/spd-tidy src/foc_ctrl.c src/foc_speed.c src/main.c src/foc_bench.c
git diff --check
```

Formatting and whitespace pass. Host analysis exits zero with no project diagnostics (12 non-user warnings suppressed by the existing configuration). ARM analysis exits zero with only 52 allowed Zephyr logging-macro signed-bitwise diagnostics and the existing `main.c:55` branch-clone warning, now at line 56 after adding the mode name. The scratch Clang database preserves target/includes/defines but omits GCC-only `-fno-defer-pop`, `-fno-reorder-functions`, `-mfp16-format=ieee`, `--param=min-pagesize=0`, and `-specs=picolibc.specs`, matching prior repository verification practice. No diagnostic configuration was changed.

## Scope, deviations and remaining evidence

Touched files: `CMakeLists.txt`, `src/foc_ctrl.c`, `src/foc_ctrl.h`, `src/foc_ctrl_motion.h`, new `src/foc_speed.c`, new `src/foc_speed.h`, `src/foc_bench.c`, `src/main.c`, `tests/host/foc/CMakeLists.txt`, `tests/host/foc/test_foc_ctrl_calib.c`, new `tests/host/foc/test_speed.c`, new `tests/host/foc/sim/sim_stage7_speed.c`, and this report.

Implementation choices/qualifications: a separate +40 `spd_step` field rather than overloading DTC; calibrated-direction speed feedback for correct negative feedback; 8 rpm settled RMS bound for off-nominal stability; explicit 20 ms mean definition for settling; 1 mA numerical inverter-loss transition. No change to gains or requested loop/reference/capture rates was made.

Environment adaptations: the globally referenced `/home/gtc/.pi/agent/references/embedded-development.md` was absent (a scoped search found no replacement); repository AGENTS.md and existing implementation/evidence were followed. The west manifest lookup was replaced with the direct pinned-checkout baseline check described above, and Clang's compile database was adapted only for unsupported GCC flags.

Hardware/boot timing, powered/HIL behavior, actual motor inertia and the <=3400-cycle ISR bound remain unverified under the explicit hardware prohibition. Raw build/test/tidy logs, resource-gate scratch script and disassembly audit are retained under ignored `build/` in this worktree. No main checkout, other worktree, credentials, hardware state, orchestrator policy, push or merge was touched.
