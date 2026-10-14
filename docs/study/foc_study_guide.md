# FOC Study Guide (host-only, no hardware)

Status: 2026-10-12. The user authorized host-only M3 study work. Everything here runs on the PC. Nothing is wired into the Zephyr firmware, any ISR or the power stage; M2 and the hardware gates in `AGENTS.md` are unchanged.

The consolidated reference (workflow, design rules, measured results, float vs fixed point, path to the board) is [foc_knowledge_base.md](./foc_knowledge_base.md).

## 1. Layout

| Path | What it is | Status |
|---|---|---|
| `lib/foc/include/foc/foc_transform.h`, `src/foc_transform.c` | Clarke, inverse Clarke, Park, inverse Park | Worked example, tested |
| `lib/foc/include/foc/foc_pi.h`, `src/foc_pi.c` | PI regulator with anti-windup | **Exercise 1**: done (integral clamping), `test_pi` passes |
| `lib/foc/include/foc/foc_svpwm.h`, `src/foc_svpwm.c` | Space-vector modulation | **Exercise 2**: done (min-max injection), `test_svpwm` passes |
| `tests/host/foc/test_*.c` | Unit tests; `foc_test.h` is a tiny CHECK helper | |
| `tests/host/foc/sim/sim_current_loop.c` | Simulated PMSM + closed d/q current loop | **Exercise 3**: loop closes with your PI and SVPWM; experiments open |
| `tests/host/foc/sim/plot_sim.py` | Plot the simulation | |
| `tests/host/foc/sim/sim_svpwm.c`, `plot_svpwm.py` | SVPWM progress checklist and plots for exercise 2 | Learning aid, not a CTest |

`lib/foc` is pure C11 `float`: no Zephyr, CMSIS, HAL or board headers (standard `<math.h>` such as `sqrtf` is fine). The board build will compile these same files. The headers carry the contract; the tests enforce it.

## 2. Build and run

```sh
# Default (what CI runs): smoke + transform tests
cmake -S tests/host/foc -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host
ctest --test-dir build/host --output-on-failure

# Include the exercises (fail until you implement them)
cmake -S tests/host/foc -B build/host -DFOC_EXERCISES=ON
cmake --build build/host && ctest --test-dir build/host --output-on-failure

# Simulation summary / plot
build/host/sim_current_loop
build/host/sim_current_loop --csv > build/host/sim.csv
uv run tests/host/foc/sim/plot_sim.py build/host/sim.csv

# SVPWM progress checklist / plot (exercise 2)
build/host/sim_svpwm
build/host/sim_svpwm --csv > build/host/svpwm.csv
uv run tests/host/foc/sim/plot_svpwm.py build/host/svpwm.csv
```

Builds use `-Wall -Wextra -Werror` with ASan/UBSan. `lib/foc` also uses `-Wdouble-promotion`: on the Cortex-M4F, `double` math is emulated in software, so write `0.5f`, `sqrtf`, `fabsf`.

## 3. Frozen conventions

| Item | Convention |
|---|---|
| Units | SI: A, V, rad, s. Duty is the high-side on-time fraction, 0..1 |
| Phase order | Positive sequence A -> B -> C |
| Clarke | Amplitude-invariant: `alpha = (2a - b - c)/3`, `beta = (b - c)/sqrt(3)`. A balanced set with peak X gives a vector of length X |
| Two shunts | The board measures A and C only: `b = -(a + c)` before Clarke |
| Park | `d = alpha cos + beta sin`, `q = -alpha sin + beta cos`; theta is the electrical angle of d from alpha; q leads d by 90 deg |
| Angle source | Callers pass sin/cos, so the board can use the CORDIC unit |
| Electrical angle | `theta_e = pole_pairs * theta_m - offset`; this motor has 7 pole pairs |
| Torque | `Te = 1.5 * p * psi * iq` (surface PMSM, amplitude-invariant) |
| Voltage limit | Linear SVPWM circle `|v| <= Vbus / sqrt(3)` |

Changing a convention means changing its test first.

## 4. Exercises

### Exercise 1 — PI regulator (`foc_pi.c`)

Read `foc_pi.h`. Implement `output = clamp(kp * e + integral)`, with `integral += ki * e * dt` and anti-windup.

- Two common anti-windup methods: clamp the integral to the output limits, or stop integrating while saturated in the direction of the error (conditional integration). Both satisfy the tests; think about how they differ after a long saturation.
- Ask yourself: why does the test drive the PI saturated for 1 s and then expect it to leave the limit immediately? What happens to a real motor's current if it doesn't?

Pass: `test_pi`.

### Exercise 2 — SVPWM (`foc_svpwm.c`)

Read `foc_svpwm.h`. The simplest correct method is min-max (zero-sequence) injection, which gives the same switching as classic six-sector SVPWM. Build it in four steps; after each one, run the checklist and look at the plot.

```sh
cmake --build build/host && build/host/sim_svpwm      # checklist, prints the first failing point
build/host/sim_svpwm --csv > build/host/svpwm.csv    # exits non-zero until all steps pass; the CSV is still written
uv run tests/host/foc/sim/plot_svpwm.py build/host/svpwm.csv                 # opens a window
uv run tests/host/foc/sim/plot_svpwm.py build/host/svpwm.csv --ratio 0.5 -o build/host/svpwm.png
```

`sim_svpwm` sweeps the command vector through 360 electrical degrees at lengths 0.5, 0.866, 1.0 and 1.3 times the linear limit `Vbus/sqrt(3)` (0.866 of it is `Vbus/2`). The plot has four panels:

| Panel | What it shows |
|---|---|
| Top left | The three duties at the `--ratio` length (default 1.0). Gray = outside `[0, 1]`, which the bridge cannot produce |
| Top right | The applied vector (`foc_clarke(duty * vbus)`) in the alpha/beta plane, with the reachable hexagon, the `Vbus/2` circle and the `Vbus/sqrt(3)` circle |
| Bottom left | `duty_a` at every length: watch the waveform shape change |
| Bottom right | One center-aligned PWM period at `--angle` (default 20 deg), with the switching state and space vector (V0..V7) of every segment |

Remember `-Wdouble-promotion`: use `fmaxf`, `fminf`, `sqrtf` and `f` literals.

**Step 1 — sine PWM.** Convert the vector to phase voltages with `foc_inv_clarke`, then `duty = 0.5 + v / vbus` for each phase. Return `false`.

- Checklist: step 1 PASS. Step 2 FAIL with something like `duty_a = 1.0768 is outside [0, 1]`. Step 4 now also FAILs (`vbus = 0` divides by zero). The placeholder only passed it by accident; step 4 below fixes it.
- Plot: the duties are pure sinusoids and leave `[0, 1]` at ratio 1.0. Note that the plane panel looks fine because it uses the unclipped duties; a real bridge would clip them and distort the voltage.
- Question: why is the sine-PWM limit exactly `Vbus/2`?

**Step 2 — min-max injection.** Before computing duties, subtract the same offset `(max + min) / 2` of the three phase voltages from each phase.

- Checklist: steps 1 and 2 PASS.
- Plot: the duties become the SVPWM "saddle"; at ratio 1.0 they just touch 0 and 1. In the bottom-right panel with `--ratio 0.5`, you see the 7-segment sequence V0-V1-V2-V7-V2-V1-V0 with V0 and V7 of equal length.
- Questions: why does `foc_clarke(duty * vbus)` still give back the command even though you added an offset to all three phases? Run with `--ratio 1.0`: how long is the V0 (000, all low sides on) segment now? That is the only time the low-side shunts see the phase currents (risk R-03).

**Step 3 — over-modulation limit.** `|v|` is not a parameter: compute the vector length from `v_ab` as `sqrtf(alpha * alpha + beta * beta)`. Before `foc_inv_clarke`, if `|v| > vbus / sqrt(3)`, multiply alpha and beta by the same factor `(vbus / sqrt(3)) / |v|` so that `|v|` equals the limit. Do not return yet: remember that you limited (a `bool`), let the rest of the steps write the duties, and return that `bool` at the end.

- Checklist: step 3 PASS.
- Plot: the 1.3 trace now lies on the dashed `Vbus/sqrt(3)` circle, on top of the 1.0 trace.
- Question: why scale the vector (keep its angle) instead of clipping each duty to `[0, 1]`? What would clipping do to the voltage angle, and to the torque?

**Step 4 — invalid bus.** At the very top, if `!(vbus > 0.0f)` (this also catches NaN), set all duties to 0.5 and return `true`.

- Checklist: all four steps PASS. Then run `ctest` with `-DFOC_EXERCISES=ON`: `test_svpwm` passes, and so should `sim_current_loop`, since it only needs your PI and SVPWM.

Final order inside the function: step 4 (guard), step 3 (limit), step 1 (`foc_inv_clarke`), step 2 (offset), duty.

**Optional — six-sector SVPWM.** Implement the classic version (find the sector, compute the T1/T2 times of the two adjacent active vectors, split the rest evenly between V0 and V7) as a second function and check that it gives the same duties as your min-max version over a 360-degree sweep.

Also answer: why is the linear limit `Vbus/sqrt(3)` and not `Vbus/2`? That is 15% more voltage. (Hint: compare the two circles in the plane panel with the hexagon.)

Pass: `test_svpwm`.

### Exercise 3 — Close the current loop (`sim_current_loop.c`)

With exercises 1 and 2 done, `sim_current_loop` passes: your PI and SVPWM already close the loop. This exercise is about understanding and tuning it. Experiments 1, 2 and 4 change `#define`s; experiments 3 and 5 are code you write.

**Before you start.** The experiments edit `tests/host/foc/sim/sim_current_loop.c`. Commit it (or copy it) first so you can go back to the default after each experiment. While an experiment is active, the summary may print `FAIL:` lines: the pass limits are set for the default values, so that is expected. Restore the default and check that `ctest` passes again when you are done.

**Workflow.** Name each CSV after the experiment; `plot_sim.py` overlays up to four files and labels them by file name. `--xlim` zooms in (in ms) and rescales the y axes.

```sh
cmake --build build/host && build/host/sim_current_loop --csv > build/host/bw1000.csv   # summary goes to the terminal
uv run tests/host/foc/sim/plot_sim.py build/host/bw1000.csv build/host/bw2000.csv --xlim 4.5 8
```

Answers are folded; look at your own plot before opening them. The numbers were measured with your `foc_pi.c` and `foc_svpwm.c`.

#### Part A — Read the loop

`main()` is the stand-in for the hardware; `current_loop_step()` is what will later run in the 20 kHz FOC ISR.

| Simulation code | What it is | On the board |
|---|---|---|
| `motor_phase_currents()` | Phase currents A and C | ADC2 injected IA/IC, triggered by TIM1 |
| `encoder_theta_e()` | Electrical angle, quantized to 4096 counts/rev | TIM3 encoder count minus alignment offset |
| `current_loop_step()` | b = −(a + c), Clarke, Park, PI d, PI q, inverse Park, SVPWM | The FOC ISR |
| `duty_applied` | Duties computed now are used one period later | TIM1 shadow registers, loaded at the update event |
| `motor_step()` × 25 | The motor, integrated in 2 µs steps | The real motor |
| `reference_at()` | iq_ref: 0, then +1 A at 5 ms, then −1 A at 40 ms | The speed loop (experiment 5) |

Run it once and plot it. Baseline: iq overshoot 2.1% (peak 1.021 A), settled error 0.020 A, max |id| 0.104 A, about 50 rad/s at 40 ms.

Question: `vq_max` is computed *after* the d PI runs, from what is left of the voltage circle. Why does d get priority?

<details><summary>Answer</summary>

id controls the flux. Holding it at its reference (0 here, negative in field weakening) matters more than getting all of the requested torque, so d takes the voltage it needs and q gets the remainder of `Vbus/sqrt(3)`. With the opposite priority, q could use up all the voltage at high speed and id would drift.
</details>

#### Experiment 1 — How fast can the loop be?

Change `LOOP_BW_HZ` to 1500, 2000, 3000 and 3500 (save `bw1500.csv` and so on). Predict first: does a faster loop simply follow the step faster? Plot with `--xlim 4.5 8`.

<details><summary>Answer</summary>

| `LOOP_BW_HZ` | iq peak after the +1 A step | Behaviour |
|---|---|---|
| 1000 | 1.021 A (2%) | Clean first-order response |
| 1500 | 1.231 A (23%) | Overshoot |
| 2000 | 1.528 A (53%) | Rings for about 1 ms |
| 3000 | 1.970 A | Rings at about 3 kHz for about 20 ms |
| 3500 | 4.5 A | Never settles; only the voltage limit keeps it bounded |

Why: pole-zero cancellation makes the open loop an ideal integrator `wc/s`, which has 90° of phase margin. The loop also contains about 1.5 PWM periods of delay (75 µs: one period from the shadow registers, plus half a period because the PWM voltage acts as an average over its period), and a delay costs phase `wc * 75 µs`. The phase margin is therefore 90° − 360° · f · 75 µs: 63° at 1 kHz, 36° at 2 kHz, 9° at 3 kHz, and negative (−4.5°) at 3.5 kHz, which is unstable. Rule of thumb: bandwidth up to about `fpwm / 20` without delay compensation.
</details>

#### Experiment 2 — The controller believes the wrong R or L

`TUNE_R_OHM` and `TUNE_L_H` are the values the gains are computed from; the motor still uses `MOTOR_R_OHM` and `MOTOR_L_H`. One at a time, set `TUNE_L_H` to `(0.5 * MOTOR_L_H)`, `(2.0 * MOTOR_L_H)` and `(3.0 * MOTOR_L_H)`, then `TUNE_R_OHM` to `(0.5 * MOTOR_R_OHM)` and `(2.0 * MOTOR_R_OHM)`. Predict: which mistake is more dangerous, a wrong R or a wrong L?

<details><summary>Answer</summary>

| Tuned with | iq peak | Settled error | What you see |
|---|---|---|---|
| correct | 1.021 A | 0.020 A | Baseline |
| L × 0.5 | 1.109 A | 0.026 A | Slower, small overshoot |
| L × 2 | 1.415 A | 0.037 A | Like the 2 kHz case of experiment 1 |
| L × 3 | 1.810 A | 0.401 A | Oscillates |
| R × 0.5 | 0.962 A | 0.077 A | Slow tail: iq is still up to 0.08 A off 2 ms after a step |
| R × 2 | 1.203 A | 0.010 A | Overshoot, then fast |

Why: `Kp = L * wc`, so an L error scales the real bandwidth by the same factor; L × 3 tuning is a 3 kHz loop, and experiment 1 showed what that does. R only places the PI zero; a wrong R leaves a slow pole-zero pair, which costs settling time but not stability. Wrong L hurts stability, wrong R hurts accuracy. Measure both on the real motor before the first closed-loop run (`docs/hardware/hardware_reference.md`, section 10: the spec's 170 Ω is probably 170 mΩ).
</details>

#### Experiment 3 — Decoupling feed-forward (you write it)

In d/q, the motor equations are coupled: the d axis sees `−we*L*iq`, the q axis sees `+we*L*id + we*psi` (the back-EMF). The PIs see these terms as disturbances they must integrate away. Feed-forward adds them directly, so the PIs only handle the error that remains.

1. Give `current_loop_step()` one more parameter, the electrical speed `float we`.
2. Compute `vd_ff = -we * L * iq` and `vq_ff = we * L * id + we * psi` from the measured `i_dq`, using the values the controller believes (`TUNE_L_H`, `MOTOR_PSI_WB`).
3. Output `vd = vd_ff + PI_d` and `vq = vq_ff + PI_q`. Shift the PI limits so the total stays inside the circle: `out_min = -vmax - vd_ff`, `out_max = vmax - vd_ff`, and the same for q with `vq_max` and `vq_ff`.
4. First pass the true speed from the plant, `MOTOR_POLE_PAIRS * motor.omega_m`. This is cheating, because the board does not know it, but it shows the best case.
5. Then estimate it from the encoder like the board must: the change of `encoder_theta_e()` since the previous period, wrapped into (−π, π], divided by `TS_S`. Then try the change over the last 20 periods (keep the last 20 angles in a small array), divided by `20 * TS_S`.

Compare `max |id|` and `max |iq - iq_ref| when settled` with the baseline.

<details><summary>Answer</summary>

| Speed used for feed-forward | max \|id\| | Settled iq error |
|---|---|---|
| None (baseline) | 0.104 A | 0.020 A |
| True plant speed | 0.077 A | 0.001 A |
| Encoder, 1 period | 0.079 A | 0.207 A (worse than none) |
| Encoder, 20 periods (1 ms) | 0.076 A | 0.028 A |

The 1-period estimate is mostly noise. One encoder count is 2π/4096 mechanical, or 0.0107 rad electrical with 7 pole pairs. Over 50 µs, a single count reads as 215 rad/s electrical, while the motor turns at about 350 rad/s electrical, about 1.6 counts per period. So the estimate jumps between about 215 and 430 rad/s, and the feed-forward injects that noise as voltage. Averaging over 1 ms divides the quantization step by 20, at the price of about 0.5 ms of lag, which is why the iq error is still a little above the baseline while the motor accelerates. Speed estimation is its own design problem; a PLL or observer is the usual next step.
</details>

#### Experiment 4 — Base speed

Hold a large torque long enough to reach top speed: in `reference_at()` set both `ref.q` values to `3.0f`, and set `SIM_END_S` to `0.300`. Plot the full run. Predict: what stops the motor from going faster?

<details><summary>Answer</summary>

Around 0.107 s, at about 435 rad/s, the voltage vector reaches the `Vbus/sqrt(3)` = 6.93 V circle (vd ≈ −1.75 V, vq ≈ 6.70 V). From then on iq falls from 2.94 A to about 0.02 A, and the speed stops at 478 rad/s.

Back-EMF is `we * psi` with psi = 60 / (sqrt(3) · 2π · KV · p) = 2.07 mWb. At 478 rad/s mechanical, we = 3346 rad/s electrical and the back-EMF is 6.93 V: all of the available voltage. No voltage is left to push current, so there is no torque, and the speed stops. This is the base speed at 12 V. Going faster needs field weakening: negative id reduces the effective flux, and with it the back-EMF.

Also look before saturation: iq sits at 2.94 A, not 3.0 A. The back-EMF is a ramp that the PI keeps chasing; the `we * psi` feed-forward from experiment 3 removes this error.
</details>

#### Experiment 5 — A speed loop on top (you write it)

This is the `control_thread` from the architecture document: a slower loop that turns a speed error into `iq_ref`.

1. Every 20 current-loop steps (`k % 20 == 0`, so 1 kHz), measure the mechanical speed from the encoder over the last 1 ms (the experiment 3 method, divided by `MOTOR_POLE_PAIRS`).
2. Run a second `foc_pi_t` with `dt = 20 * TS_S`. Its output is `iq_ref`; hold it constant between updates. Limit it to ±3 A.
3. Gains: from iq to speed, the motor is `Kt / (J * s)` with `Kt = 1.5 * p * psi` = 0.0218 N·m/A and J = 1.5e-5 kg·m². For a 50 Hz speed bandwidth (`wsc = 2π · 50`), use `Kp = wsc * J / Kt` ≈ 0.217 A/(rad/s) and `Ki = Kp * wsc / 4` ≈ 17 A/rad. 50 Hz is 20 times below both the 1 kHz update rate and the 1 kHz current loop.
4. Speed reference: 0, then 300 rad/s at 5 ms, then 100 rad/s at 120 ms. Set `SIM_END_S` to `0.200`.

<details><summary>Answer</summary>

iq_ref sits at the +3 A limit from 5 ms to about 77 ms: the acceleration is current-limited. Speed reaches 300 rad/s at 76 ms, overshoots to 311.7 rad/s (4%) and is back at 300 by 119 ms. After the step down, it dips to 88 rad/s and settles at 100.

The overshoot comes from your exercise 1 anti-windup: during 70 ms at the limit, the integral sat at the clamp (3 A), and it has to unwind before iq_ref can drop. Try conditional integration (stop integrating while the output is saturated in the direction of the error) and compare the overshoot.
</details>

## 5. Theory checklist

Make sure you can explain each item before moving to the board:

- Why rotating-frame (d/q) currents are DC in steady state, and why that makes PI control work.
- Why `id = 0` gives maximum torque per ampere for a surface PMSM, and what negative `id` does (field weakening).
- Low-side shunt sampling: why the ADC must sample in the middle of the low-side-on window (TIM1 counter at its top in center-aligned mode), and why two shunts fail near 100% duty (risk R-03).
- The PMSM voltage equations used in `motor_step()`; derive them yourself from the stator in alpha/beta.
- Why the one-period delay limits bandwidth, and why compensating the angle by about `1.5 * we * Ts` helps at high speed.

Reading: SimpleFOC documentation (beginner theory), ST MCSDK and TI motor-control application notes (practical sampling and timing), and De Doncker et al., *Advanced Electrical Drives* (rigorous derivations).

## 6. From the simulation to the board

The simulation stands in for the BSP. On hardware, these parts change, in this order. Each step needs its own gate per `AGENTS.md` and the roadmap.

| Simulation | Board (M2 BSP) | Note |
|---|---|---|
| `motor_phase_currents()` | ADC2 injected IA/IC triggered by TIM1, offset-calibrated | Gains: 5 mΩ x 10 -> 50 mV/A around 1.65 V (HW-ADC-010, ideal) |
| `encoder_theta_e()` | TIM3 encoder count -> angle, minus the measured alignment offset | Align by holding `Vd` at angle 0 |
| `duty_applied` | TIM1 CCR1..3 shadow registers, update event | SD1-SD3 control; D phase stays off |
| `sinf/cosf` | CORDIC or a polynomial in the BSP | Kernel takes `foc_sincos_t` |
| Illustrative R/L/psi | Measured values | 170 Ω in the spec is probably 170 mΩ — measure |
| — | DWT cycle counts, WCET < 20 µs | Required by `AGENTS.md` |

First power-on test (M4 rules apply): low-voltage current-limited supply, open-loop voltage vector, then encoder alignment, then the current loop with a small `iq_ref`.
