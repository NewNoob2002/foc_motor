# FOC Knowledge Base

Status: 2026-10-14. One place for what the M3 host-only study has established: the full FOC workflow, the design rules behind it, the numbers measured in simulation, the number format, and the path to the board. Exercises and step-by-step instructions stay in [the study guide](./foc_study_guide.md); hardware evidence in [hardware_reference.md](../hardware/hardware_reference.md); stage gates in [development_roadmap.md](../plan/development_roadmap.md); risks in [risk_list.md](../plan/risk_list.md).

Everything below "measured" was measured in the host simulation with illustrative motor values. Nothing here has been verified on hardware.

## 1. The system in one page

| Part | What it is | Evidence / status |
|---|---|---|
| MCU | STM32G431RBT6, Cortex-M4F with single-precision FPU, 170 MHz, 128 KiB Flash, 32 KiB SRAM, CORDIC/FMAC | HW-MCU-001 |
| PWM | TIM1 CH1/2/3 -> phases A/B/C, center-aligned, 20 kHz (10–25 kHz allowed). D leg exists but stays off (`SD4 = 0`) | HW-PWM-001..006 |
| Gate driver | EG2104 per leg, one `IN` + active-low `SD`, internal dead time 50–300 ns. `SD = 1, IN = 0` turns the **low** side on, so only `SD = 0` is "off" | HW-PWM-007/008 |
| Current sense | 5 mΩ low-side shunts on A and C only, gain 10, 1.65 V offset (ideal 50 mV/A, about ±33 A full scale). Both on ADC2, so sampled one after the other | HW-ADC-001/002/007/010, R-03, R-06 |
| Position | KTH7823 A/B, 4096 counts/rev -> TIM3 encoder mode; SPI only when stopped | HW-ENC-003/005/006, R-08 |
| Motor (inputs, not measured) | 4006, 12 slots / 14 poles = 7 pole pairs, KV380 -> psi = 2.07 mWb. Simulation uses R = 0.17 Ω, L = 100 µH, J = 1.5e-5 kg·m² as illustrative values | hardware_reference §10 |
| Safety gap | No CPU-independent hardware shutdown path to `SD` | **R-02, blocks powered closed loop (M4)** |

Software layers (dependencies only point down): Application -> Control (`control_thread`, 1 kHz) -> FOC Kernel (`lib/foc`, pure C11 `float`) -> BSP (TIM1, ADC2/DMA, TIM3, SD, DWT) -> Hardware.

## 2. The complete FOC workflow

### 2.1 Block diagram

```text
=== control_thread, 1 kHz (Control layer) ==================================
                                                    id_ref = 0 (MTPA)
  w_ref --->(+)---> speed PI ---> iq_ref, limited to +-I_max
             ^ -                       |
             |                         | single-writer snapshot, no locks
             +--- w_m = (TIM3 count change over 1 ms) <----------------+
                                       |                               |
=== FOC ISR, 20 kHz, triggered by ADC2 DMA complete (FOC Kernel + BSP) |
                                       v                               |
  1. ADC2 [IA, IC] -> amps (offset, gain); ib = -(ia + ic)             |
  2. TIM3 count -> theta_m -> theta_e = p * theta_m - theta_offset ----+
  3. sin/cos(theta_e)                      (CORDIC on the board)
  4. Clarke -> Park                        -> id, iq
  5. PI d:  vd = PI(id_ref - id), |vd| <= Vmax          (d has priority)
     PI q:  vq = PI(iq_ref - iq), |vq| <= sqrt(Vmax^2 - vd^2)
     optional feed-forward: vd += -we*L*iq ; vq += we*L*id + we*psi
  6. inverse Park at theta_e + 1.5*we*Ts   -> v_alpha, v_beta
  7. SVPWM: scale |v| to <= Vbus/sqrt(3), min-max injection -> duties
  8. clamp duties to [d_min, d_max]        (R-03 sample window, R-05 bootstrap)
  9. write TIM1 CCR1..3 (shadow registers), DWT end stamp, telemetry snapshot
                                       |
                                       v  loaded at the next TIM1 update event
=== Power stage and sensors ================================================
  TIM1 CH1-3 -> EG2104 x3 (SD1-3) -> MOSFET bridge -> PMSM
                                        |              |
            ADC2 <- amp x10 <- low-side shunts A, C    +-> KTH7823 A/B -> TIM3
```

Vmax = Vbus/sqrt(3), the largest circle inside the SVPWM hexagon (6.93 V at 12 V).

### 2.2 Timing of one PWM period (planned TIM1 configuration, verified in M2)

```text
TIM1 counter   0 ------ up ------> ARR ------ down ------> 0 ------ up ---->
high sides     ON while counter < CCRx (around 0)          ON around 0
low sides      all three ON around ARR = zero vector V0 (000)
ADC2 trigger                        ^ at ARR: IA, then IC (sequential)
FOC ISR                              [ DMA complete -> compute, WCET < 20 us ]
CCRx write                                          ^ into shadow registers
update event                                           loads CCRx -> new duties act
```

Why sample at the counter top: it is the middle of the window where all low sides conduct, so the shunts carry the phase currents; it is as far as possible from switching edges and their ringing; and with symmetric center-aligned PWM the current ripple crosses its period average there, so one sample equals the average current the controller wants.

Delay: the simulation assumes the new duties act during the whole next period, about **1.5 Ts = 75 µs** after the sample (1 period of shadow-register wait + 0.5 period because the PWM voltage acts as an average over its period). If M2 configures TIM1 to load the shadow registers at the counter bottom right after the top sample, the delay can shrink to about 1 Ts, but then the ISR must finish within half a period. Which one applies is an M2 measurement, not a fact yet.

### 2.3 Start-up and run sequence (architecture §6)

1. Reset: SD1–SD4 low (EG2104 internal pull-downs, then BSP drives them low explicitly). D leg stays off forever.
2. Clocks, DWT, TIM1 with outputs disabled, ADC2/DMA, TIM3.
3. ADC self-calibration; measure IA/IC zero offsets with PWM off; check bus voltage, temperature, encoder.
4. State machine stays `NOT_READY` until all checks and the external enable pass.
5. Rotor alignment (first time per power-up with an incremental encoder): hold a voltage vector on the d axis at angle 0, record the encoder count as `theta_offset`, check direction.
6. Start PWM in a safe state, release SD1–SD3 together, run the current loop with small `iq_ref`, then the speed loop.
7. Any fault: hardware turns the gates off first (needs R-02), then software latches the cause. Recovery only through an explicit reset and the full pre-check, never automatic retry.

## 3. Frozen conventions and key equations

| Item | Convention |
|---|---|
| Units | SI (A, V, rad, s). Duty = high-side on-time fraction, 0..1 |
| Clarke | Amplitude-invariant: `alpha = (2a - b - c)/3`, `beta = (b - c)/sqrt(3)`; with two shunts `b = -(a + c)` |
| Park | `d = alpha*cos + beta*sin`, `q = -alpha*sin + beta*cos`; q leads d by 90° |
| Electrical angle | `theta_e = p * theta_m - theta_offset`, p = 7 |
| Torque | `Te = 1.5 * p * psi * iq` (surface PMSM, Ld = Lq) |
| Voltage limit | Linear SVPWM: `|v| <= Vbus / sqrt(3)`; sine PWM only reaches `Vbus / 2` (15% less) |
| Kernel inputs | sin/cos are passed in (`foc_sincos_t`), so the board can use CORDIC |

PMSM in d/q (derive it: `v_ab = R*i_ab + d(psi_ab)/dt`, `psi_ab = (L*i_dq + psi)*e^(j*theta)`, differentiate, divide by `e^(j*theta)`):

```text
vd = R*id + L*did/dt - we*L*iq
vq = R*iq + L*diq/dt + we*L*id + we*psi        (we*psi = back-EMF)
Te = 1.5 * p * psi * iq ;  J*dwm/dt = Te - B*wm - T_load
```

Rotating the frame creates the `j*we` term: it holds both the cross-coupling (`+-we*L*i`) and the back-EMF.

## 4. Theory you should be able to explain

**d/q currents are DC.** Phase currents are sinusoids at the electrical frequency (334 Hz at 300 rad/s mechanical). Park measures the current vector relative to the rotor; in steady state the vector turns with the rotor, so id and iq are constants. A PI reaches zero steady-state error only for constant references (its integrator stops only when the error is exactly zero), so d/q is what makes plain PI control work at any speed.

**id = 0 is maximum torque per ampere.** Torque depends only on iq for a surface PMSM; id only adds copper loss `R*(id² + iq²)`. **Negative id** weakens the d-axis flux `psi + L*id`, so the back-EMF is smaller at a given speed and the motor can run above base speed (field weakening). Cost: current without torque. Here, 10% less flux needs `id = -0.1*psi/L ≈ -2 A`, so field weakening is expensive on this low-L motor.

**Low-side shunt sampling and R-03.** Phase current flows through a low-side shunt only while that low side conducts, which is the `(1 - d)*Ts` window around the counter top. At `d = 0.95` and 20 kHz that is 2.5 µs for dead time, amplifier settling and the ADC sample. Three shunts could always pick the two widest windows; two fixed shunts (A, C) cannot, so high modulation and some sector positions are blind, and IA/IC are not even sampled at the same instant (one ADC). Mitigations: cap the duty, define what the controller does in the blind zone (never reuse stale samples as if fresh), or change the hardware (dual ADC or three shunts). Min-max SVPWM helps by centering the duties.

**Delay limits bandwidth.** A delay `T` adds phase lag `360° * f * T` without changing gain. Pole-zero-cancelled current loops are an integrator with 90° margin, so phase margin = `90° - 360° * f * 75 µs`: 63° at 1 kHz, 36° at 2 kHz, 9° at 3 kHz, negative at 3.5 kHz. Rule of thumb: current bandwidth about `f_pwm/20` without delay compensation.

**Angle compensation.** The voltage computed with the angle at the sample acts about 1.5 Ts later, when the rotor has moved `1.5*we*Ts`: at 478 rad/s mechanical that is 3346 rad/s × 75 µs = 0.25 rad ≈ 14°. Part of vq then lands on the d axis. Use `theta_e + 1.5*we*Ts` for the **inverse** Park only; the forward Park keeps the sample angle, because that is where the current was measured. (Not yet tried in the simulation.)

## 5. Critical design considerations

| Area | Rule | Why / where it came from |
|---|---|---|
| Current-loop tuning | `Kp = L*wc`, `Ki = R*wc` (pole-zero cancellation), `wc = 2π*f_bw` | Exp. 1: start at 1 kHz; 2 kHz already overshoots 53% |
| Parameter errors | Measure **L** carefully; R errors only slow settling | Exp. 2: L×3 oscillates, R×0.5 only leaves a slow tail. Spec's 170 Ω is probably 170 mΩ: measure |
| Voltage limit | d PI first, q gets `sqrt(Vmax² - vd²)` | Flux control (id) matters more than full torque |
| Anti-windup | Every PI that can saturate needs it | Integral clamp: simple, but after long saturation it overshoots (speed loop 4%); conditional integration recovers faster |
| Decoupling feed-forward | Only with a clean speed estimate | Exp. 3: true speed cuts iq error 0.020 -> 0.001 A; a 1-period encoder estimate makes it 10× worse (0.207 A) |
| Speed estimation | Differentiate counts over 1 ms or use a PLL/observer; never 1 period | 1 count in 50 µs = 215 rad/s electrical of noise |
| Base speed | At 12 V, voltage runs out at 478 rad/s mechanical | Exp. 4: iq collapses; above needs field weakening or more Vbus |
| Cascade | Each outer loop ≥ 10–20× slower than the inner one | Exp. 5: 50 Hz speed loop on a 1 kHz current loop, updated at 1 kHz |
| Units in the slow loop | `dt = 20*Ts`; mechanical vs electrical rad/s; divide by `p` | The two bugs found in Exp. 5 |
| Angle wrap | Wrap angle differences into (−π, π]; better, difference the raw TIM3 count as `int16_t` | Electrical-angle differences alias above 449 rad/s mechanical at 1 ms (see §7) |
| Duty limits | Clamp to `[d_min, d_max]` from measured sample window and bootstrap refresh | R-03, R-05; values come from M2 gate waveforms |
| Invalid inputs | Reject NaN/≤0 Vbus with `!(vbus > 0.0f)`; never output NaN duties | A plain `vbus <= 0` lets NaN through |
| Real time | FOC ISR: no kernel/device API, logging, locks, allocation; WCET < 20 µs; measured with DWT | AGENTS.md |
| Safety | Stopping TIM1 or `IN = 0` is not "off" (low side conducts). Only `SD = 0` is | HW-PWM-007; independent hardware path needed (R-02) |

## 6. Measured simulation results (illustrative motor, 12 V, 20 kHz)

| Experiment | Setting | Result |
|---|---|---|
| Baseline current loop (1 kHz) | +1 A then −1 A iq steps | Overshoot 2.1% (1.021 A), settled error 0.020 A, max \|id\| 0.104 A |
| 1. Bandwidth | 1000 / 1500 / 2000 / 3000 / 3500 Hz | Peak 1.021 / 1.231 / 1.528 / 1.970 A / unstable (4.5 A, voltage-bounded) |
| 2. Wrong tuning (peak / settled error) | L×0.5, L×2, L×3, R×0.5, R×2 | 1.109/0.026, 1.415/0.037, 1.810/0.401 (oscillates), 0.962/0.077, 1.203/0.010 A |
| 3. Feed-forward (max \|id\| / settled error) | none, true speed, 1-period, 20-period estimate | 0.104/0.020, 0.077/0.001, 0.079/0.207, 0.076/0.028 A |
| 4. Base speed | iq_ref = 3 A for 0.3 s | Voltage limit at about 107 ms; speed stops at 478 rad/s; vd ≈ −1.75 V, vq ≈ 6.70 V |
| 5. Speed loop (integral clamp) | 50 Hz, Kp 0.2165, Ki 17.0, ±3 A, 0 -> 300 -> 100 rad/s | +3 A until 77 ms; 300 reached at 75.5 ms; peak 311.7 (3.9%); within ±1% from 96 ms; dip to 88.2 after the step down; −0.87 A id spike at the −3 A iq step (cross-coupling) |

## 7. Lessons from the exercises

- **NaN guard:** `if (vbus <= 0)` is false for NaN; `if (!(vbus > 0.0f))` catches it.
- **Operator precedence:** `dtheta / 20 * TS_S * p` multiplies by `TS_S` and `p`. Write `dtheta / (20.0 * TS_S) / p`.
- **Broken feedback looks like open loop:** with speed measured as ~0, the speed PI pinned +3 A and the motor ran to base speed (478 rad/s). A loop stuck at its limit points at a sign or scale error in the feedback.
- **Wrong constant:** `LOOP_KP` (current loop, V/A) in the speed PI (A per rad/s). Compute each loop's gains from its own plant.
- **Precision:** keep `float` math in `float` (`2.0f * (float)SIM_PI`); `lib/foc` builds with `-Wdouble-promotion`.
- **CSV header and row must match**; `plot_sim.py` now says so instead of crashing.
- **Self-check answers.** The 0.33 A iq steps at 300 rad/s: one count = 2π/4096 rad = 1.53 rad/s over 1 ms, × Kp 0.2165 = 0.33 A. The wrap limit: (−π, π] electrical per 1 ms -> `we < 3142 rad/s` -> `wm < 449 rad/s`, below the 478 rad/s base speed; differencing the raw 16-bit TIM3 count avoids it.

## 8. Floating point or fixed point?

**Decision: `float32`**, set by `AGENTS.md` ("C11 and float32 as the baseline; fixed point only when WCET or numerical evidence shows float is not enough"). The study is not float-only by accident.

**Why ST's MCSDK uses fixed point.** Its FOC core uses 16-bit integer quantities (Q15-style "digit" units) and PI gains written as integer numerator/power-of-two divisor. One reason is that the same library runs on parts without an FPU (Cortex-M0/M0+, M3), where float is emulated in software and far too slow. Integer math is also natural for 12-bit ADC data and timer compare values.

**Why float is fine here.** The G431's Cortex-M4F has a single-precision FPU: add and multiply take 1 cycle, multiply-accumulate 3, divide and square root 14 (Cortex-M4 TRM). A FOC step is a few dozen such operations, so float costs about the same as Q15 and removes the hard parts of fixed point: choosing per-unit bases, guarding every multiply against overflow, saturating arithmetic, wide integral accumulators, and debugging scaled numbers. It also matches the host simulation exactly. The ISR time must still be **measured** (M3 ARM WCET benchmark); that number, not a belief, closes the question.

**Float rules on the M4F.**

- Never `double`: it is software-emulated (hence `-Wdouble-promotion`, `0.5f`, `sqrtf`).
- Avoid divisions in the ISR: precompute `1/Vbus` once per period and multiply.
- `sinf`/`cosf` from libm are slow; use CORDIC (it works in Q1.31 / Q1.15, so convert at the BSP boundary) or a polynomial.
- Integers stay integers at the edges: ADC counts -> amps is one multiply-add in the BSP; position stays an integer count (extend TIM3 with signed 16-bit deltas) and is converted to a float angle only modulo one electrical turn. A float has a 24-bit mantissa, so an ever-growing float angle loses resolution.
- The FOC ISR uses the FPU, so FPU context saving must be configured and included in the WCET measurement (Cortex-M lazy stacking; Zephyr FPU options). Verify in M3/M2, do not assume.

**When fixed point would come back:** a measured WCET that misses the 20 µs budget after the above, a port to an FPU-less MCU, or using the FMAC/CORDIC units on a hot path.

## 9. Path to the board

Hardware tests need explicit authorization, a current-limited supply, an emergency stop, confirmed board identity and a recovery plan. Host results never count as hardware verification.

| Gate | Status today | What closes it |
|---|---|---|
| M1 close | Open. **R-18 BLOCKER** (no real SD1–SD4/PC0–PC3 waveforms at reset/boot), M1-09 pre-check missing (supply voltage, current limit, e-stop), UART3 repeat capture, R-16 documents, CI evidence for the current revision | Scope/logic-analyzer captures with no bus voltage; pre-check record; evidence summary |
| Enter M2 | Needs your explicit gate confirmation and a frozen encoder harness (**R-08**: 5 V vs 3.3 V, pinouts differ) | Confirmation + harness/fixture |
| M2 BSP, no-power HIL | Not started | PWM (frequency, centering, sync update, dead time, bootstrap), ADC2/DMA (trigger phase, `[IA, IC]` skew, offsets, gains), encoder (direction, counts, Z), DWT timing; raw evidence |
| M3 close | Host work done for Clarke/Park/PI/SVPWM; no ARM build or WCET yet | ARM build + WCET benchmark of the same `lib/foc` sources, no power output; needs your confirmation |
| M4 powered closed loop | **Blocked by R-02**: no CPU-independent hardware shutdown on this board revision. Also R-12 (no layout/BOM/SOA), unknown bus voltage and current limits, unmeasured R/L/psi | Hardware path (comparator/fault logic pulling SD1–SD3 low, latched) with measured turn-off time; parameter and power reviews |

So spinning the motor under FOC is not allowed on the current board revision, whatever the software state, until R-02 has a hardware fix. The board work that is possible next is M1 closure (no bus voltage), then M2 signal-level tests.

First powered sequence, once M4 is open: low-voltage current-limited supply -> open-loop voltage vector at low amplitude (check phase order and encoder direction) -> alignment and `theta_offset` -> current loop at 1 kHz bandwidth with small `iq_ref`, rotor free or locked -> raise limits step by step -> speed loop.

## 10. Files and commands

| Path | Purpose |
|---|---|
| `lib/foc/` | Kernel: `foc_transform` (given), `foc_pi` (Exercise 1), `foc_svpwm` (Exercise 2) |
| `tests/host/foc/test_*.c` | Unit tests (`-DFOC_EXERCISES=ON` adds PI, SVPWM, `sim_current_loop`) |
| `tests/host/foc/sim/sim_current_loop.c` | Plant + current loop, the CTest reference version |
| `tests/host/foc/sim/sim_speed_loop.c` | Your Exercise 3 / Exp. 5 copy with the speed loop (not a CTest) |
| `tests/host/foc/sim/sim_svpwm.c`, `plot_svpwm.py` | SVPWM checklist and plots |
| `tests/host/foc/sim/plot_sim.py` | Overlay up to 4 CSVs; `--xlim START END` (ms); `-o file.png`; dashed `omega_ref` if present |

```sh
cmake -S tests/host/foc -B build/host -DCMAKE_BUILD_TYPE=Debug -DFOC_EXERCISES=ON
cmake --build build/host && ctest --test-dir build/host --output-on-failure
build/host/sim_speed_loop --csv > build/host/speed.csv
uv run tests/host/foc/sim/plot_sim.py build/host/speed.csv
```

Reading: SimpleFOC documentation (intuition), ST MCSDK and TI motor-control application notes (sampling and timing practice), De Doncker et al., *Advanced Electrical Drives* (derivations).
