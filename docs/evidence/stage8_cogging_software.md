# Stage 8 cogging feed-forward: software evidence

Branch `stage/8-cogging-ff`. Spec: gain check, ripple map and feed-forward, from the stage 7 finding of a 2×-electrical speed ripple (±15–24 rpm at 60–100 rpm, position-locked, about 3–4 mNm).

**Authorship.** A GPT worker wrote most of the code. Its first run was interrupted, and the provider then hit its usage limit (HTTP 429). With the user's agreement (2026-10-15), Claude finished the work: reclaiming CCM and SRAM, the boot-bench fixture, the audit, this record and the commit.

**No hardware result yet.**

## Design

**Mode 8 (`FOC_MODE_MAP`, `src/cog_map.{c,h}`, ISR side):**
- Needs the power stage on and a valid, plausible calibration. Otherwise it publishes a failure at once (`COG_NOT_POWERED` / `COG_CALIBRATION`).
- Bounded by 55 s.
- Aborts on a fault, disarm, power loss, NaN, a slip (|rpm − ref| > 5 rpm for 1 s), count saturation or timeout.

The sequence:
1. **Clear:** one bin per period, which bounds the mode-entry WCET.
2. **Gain check:** an open-loop voltage vector of the identified 1 A R-point voltage at 12 electrical angles (calibrated frame), with no current loop. Ramp at 1 V/s, settle 0.3 s, average 2000 samples of i_α / i_β, ramp down, then 20 ms at 0 V before the next angle, so there is never an energised angle jump.
   - With a constant voltage magnitude, the true current magnitude is the same at every angle. A 2θ pattern in the measured magnitude indicates a sensor gain mismatch, and 1θ an offset.
   - Phase-resistance imbalance also gives 2θ; this is a stated limitation.
3. **Ripple map: a position-loop sweep** (revised after bench run 1, where the 10 rpm speed-loop map went stick-slip):
   - A PID on the calibrated position error (target − measured, counts) sets iq; the current loop runs at the calibrated angle with DTC on and no feed-forward.
   - The target moves at 410 counts/s (about 6 rpm): hold 0.5 s, then 2 turns forward; hold 0.5 s, then 2 turns back.
   - Gains: kp = J·w²/Kt, kd = 2ζ·J·w/Kt, ki = kp·w/5, with the assumed J = 1.5e-5, w = 2π·10 Hz, ζ = 0.8. That gives 4.19 mA/count, 7.28 mA/rpm and 52.6 mA/(count·s); integral ±0.3 A, output ±0.5 A.
   - **J is not measured.** A pulse test is dominated by the 3–4 mNm cogging at any current that stays below the overspeed trip for an unknown J. The simulation covers J × 1/3 and × 3 instead.
   - iq is accumulated per mechanical bin of the *measured* position (256 bins × 2 directions; int32 mA sums, uint16 counts).
   - The run aborts on a position error above 400 counts (`COG_SLIP`).
   - Publication is by an atomic ready flag; the ISR does not touch the tables until the thread releases them.

**Thread side (`src/cog_proc.{c,h}`, pure C):**
- cog = (fwd + rev)/2 minus its mean, and friction = the mean of (fwd − rev)/2;
- harmonics for mechanical orders 1, 2, 7, 14, 28, 42 and 84 (84 is at the resolution limit of 256 bins);
- a gain fit |i|(θ) = I0·(1 + e1·cos(θ − p1) + e2·cos(2θ − p2));
- the int16 mA feed-forward table.
- The `ff_valid` flag protocol: the thread writes the table only while `ff_valid` is 0, then sets it to 1. Starting a new map clears it, and an identification invalidates it (value 2).

**Feed-forward:**
- `cff` at +44 (0/1), latched on `step_seq`. It is refused without a valid map of this boot (the encoder is incremental, so a map is valid for one boot only).
- In mode 7, iq_ref_total = speed PI + ff(enc); in mode 4, it is the latched iq ref + ff(enc).
- The interpolation is between bin centres, and the total is clamped to the existing iq limits. The feed-forward is off during the map itself.

**Output:** `gainchk`, `gainfit`, `cogmap`, `cogharm`, `friction` and `cogmap_ok` lines (formats in `src/main.c`).

## Command block

`foc_ctrl_cmd` at **`0x20002808`** (firmware with the position sweep):

| Field | Offset |
|---|---|
| arm | +0 |
| mode | +4 |
| v_mv | +8 |
| theta_e | +12 |
| step_seq | +16 |
| rate_e | +20 |
| id_ma | +24 |
| iq_ma | +28 |
| dtc | +32 |
| spd_rpm | +36 |
| spd_step | +40 |
| **cff** | **+44** |

`test_spin_count` / `test_spin_cycles` are at `0x20000E10` / `0x20000E14`.

## Resources and the CCM split

| | Worker state | Final |
|---|---|---|
| CCM `.ccm_text_reloc` | 10200 / 10240 B | **9680 / 10240 B (560 B free)** |
| SRAM data+bss | 20044 / 22528 B | **16568 / 22528 B** |
| Flash text+data | 85940 | 85832 / 98304 B |

**SRAM:** the boot bench kept a permanent 3988 B fixture (a full `cog_map_t` plus a processed copy) just to time the map period and the thread processing. It was replaced by a 512 B feed-forward table. The map period is measured on the target by the ISR segment timing in mode 8 instead.

**CCM:** functions moved to flash because they run once per event. The CCM → flash veneers found in the disassembly of `.ccm_text_reloc`:

| Function | Caller | When |
|---|---|---|
| `mode_enter` (new, the former mode-entry block of `foc_ctrl_step`) | foc_ctrl_step | once per mode change |
| `pi_reset` | foc_ctrl_step | mode entry and power-state change |
| `calibrate` | foc_ctrl_step | the first 4096 periods after boot, stage off |
| `speed_latch` | foc_ctrl_step | a `step_seq` edge |
| `align_result`, `align_two_sided`, `fit_resistance`, `inductance_result`, `rotation_result` | motor_ident_step | once per identification phase |
| `motor_ident_result_copy` | ident_publish | once per identification run |

**No per-period call target is in flash.** Run-once calls cost a flash fetch in the period where they happen; that is acceptable, and the bench ISR telemetry will show it.

## Host results (warnings-as-errors, ASan/UBSan)

`ctest`: **12/12 passed** (test_cog: processing, feed-forward, sequence, aborts; test_foc_ctrl_calib: map permission/abort, cff latching/refusal/invalidation, plus all earlier tests).

Closed-loop simulation (`sim_stage7_speed`): a 14/turn cogging term of 3 mNm, and **2 mNm stiction** (breakaway; the earlier model had only 0.5 mNm, which is why its 10 rpm speed map did not show the bench's stick-slip). The full mode-8 sequencer runs in simulation (about 56.5 s).

| Simulated J | Map | Max position error |
|---|---|---|
| 0.5e-5 | done | 50 counts |
| 1.5e-5 | done | 57 counts |
| 4.5e-5 | done | 74 counts |

The 14/turn harmonic is recovered as 145 mA (true 138 mA, 5 % high).

| Speed | Ripple RMS, ff off | Ripple RMS, ff on | Reduction |
|---|---|---|---|
| 60 rpm | 18.82 rpm | 2.30 rpm | 8.19× |
| 100 rpm | 12.26 rpm | 1.58 rpm | 7.75× |

## Checks

| Check | Result |
|---|---|
| ARM pristine build (`west build -p always … -DBUILD_VERSION=v4.4.2`) | OK |
| CI gates from `.github/workflows/ci.yml`, run locally | rc 0: `budgets: flash 85832/98304 B; SRAM data+bss 16568/22528 B; CCM 9680/10240 B`, plus the allocation, GPIO and config gates |
| clang-format `--dry-run --Werror` on all changed files | OK |
| clang-tidy on the changed `src/*.c` | only `bugprone-signed-bitwise` inside Zephyr `LOG_*` macro expansions (main.c, foc_bench.c), the pre-existing GPIO/clock macro warnings in pwm_adc.c, and the known `K_THREAD_STACK_DEFINE` branch-clone; none in cog_map.c, cog_proc.c, foc_ctrl.c, motor_ident.c |

## Not verified

- On the target: WCET of mode 8 and of modes 4/7 with the feed-forward.
- The gain-check result and the real ripple map.
- The ripple reduction on the bench.
