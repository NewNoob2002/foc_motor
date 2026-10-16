# Stage 7 — First power

Status: **entered 2026-10-15** on the user's explicit go ("confirm and go", recorded in `AGENTS.md`). The firmware and the checks without DCBUS pass, and R-18 passes for this firmware (`r18_boot_capture.md`, run 2).

## Decisions recorded before Stage 7

| Item | Decision (2026-10-15) |
|---|---|
| R-02 (no CPU-independent shutdown) | Option (b): accepted for this learning bench only. Still a blocker for M4 (`risk_list.md`) |
| Supply | Bench supply, **500 mA limit with OCP on**, DCBUS **15 V** (+12V regulates from about 14.1 V, `HW-PWR-002`) |
| E-stop | The bench supply's output button |
| Logic power | J-Link power off (VTref sense only). USB and DCBUS are **never connected together**: USB_5V reaches +5V through R35 (0 Ω) (`HW-PWR-003`) |

## Firmware `f2460b5`

Written by Claude; reviewed by a GPT model, which found 9 issues, all confirmed and fixed in `f2460b5` (see its commit message); merged as PR #10.

| Part | Behaviour |
|---|---|
| Boot | Power stage off: SD1–SD4 low (hogs), PC0–PC3 analog. ADC2 samples [IA, IC, VBUS] at the counter top |
| Arming (`src/power_guard.c`, host test `test_power_guard`) | All of these: arm key `0x41524D21` in `foc_ctrl_cmd.arm`; current offsets calibrated **and plausible** (within 200 counts of mid-scale, spread ≥ 2 counts); VBUS 14.0–16.5 V for 100 ms; **explicitly commanded idle for 1 ms** (so the preloaded duty is the zero vector) |
| Power on / off (`src/pwm_adc.c`) | On: PC0–PC2 to TIM1 AF2, then SD1–SD3 high. Off: SD1–SD3 low first, then PC0–PC2 analog. SD4/PC3 never touched. Duty limited to 5–95 % (bootstrap) |
| Trips (latched until reset) | ISR faults: \|IA\|, \|IB\|, \|IC\| > 2 A or not a number (after calibration); VBUS outside 12–17 V for 3 powered periods; 3 overruns in a row; DMA error; ADC overrun; a duty outside [0, 1] (NaN); a powered ISR gap > 17000 DWT cycles; the fixed-pin check failing at switch-on. These set SD1–SD3 low, PC0–PC2 analog, MOE off and CCR1–3 2125 (50 %). The pin check in the report thread (10 Hz) instead calls `pwm_adc_stop()`: SD1–SD3 low, PC0–PC2 analog, DMA IRQ and TIM1 stopped, MOE off until reset; it does not rewrite the CCRs or set the PINS fault bit |
| Supervisor | 10 ms kernel timer: while powered, fewer than 150 of 200 ISRs → `pwm_adc_stop()` (final until reset) |
| Voltage | Vector capped at 1.0 V while powered; duty scale from the latest raw VBUS sample |
| Modes (`foc_ctrl_cmd.mode`) | 0 idle; 1 dc (vector `v_mv` at `theta_e`, ramped 1 V/s: R, alignment); 2 step (each `step_seq` change: 16 zero, 128 on, 112 zero periods, IA/IC captured and printed: L); 3/4 the Stage 6 modes, only while off; unknown values run idle and cannot arm |
| Command block | `foc_ctrl_cmd` at `0x2000118C`: arm +0, mode +4, v_mv +8, theta_e +12, step_seq +16 (address printed at boot). J-Link `w4` values are hex |

## Checks without DCBUS (2026-10-15)

Setup: board UID `001D0050 42535013 20363130`; firmware `f2460b5` flashed and verified; **USB power only** (VTref 3.320 V), no DCBUS; UART H2 921600. Raw files in `build/hil/stage7/` (`uart_main.log`, `jlink_checks.txt`, `checks.jlink`, `vec_compare.txt`).

| Check | Result |
|---|---|
| Boot | "power stage off"; VBUS 35–53 mV; offsets IA 1962.33, IC 1996.41 counts (accepted); Stage 5 vectors still match the host (`vec_compare` on `uart_main.log` lines 1–80 with ANSI codes stripped: PASS, saved as `vec_compare.txt`) |
| ISR | 20 kHz. Idle after boot (reports 1–3): period 8489–8511 cycles, exec max 1901 cycles (11.2 µs). During T1–T4: exec max 2441 cycles (14.4 µs, report 45), period up to 8514 cycles (report 44); report 36 has one 4229-cycle interval. Before the forced overruns: 0 overruns, 0 DMA errors, 0 ADC overruns; the overrun count is 1 after T5 and 4 after T6 |
| T1: arm key written, no DCBUS | Power stayed **off** in every report ("request 0, on 0 times"). Readback: GPIOB ODR 0 and GPIOA ODR 0 (SD1–SD4 low; PA0–PA2 output mode), GPIOC MODER `0xFFFFFFFF` (PC0–PC3 analog) |
| T2: mode 99 (unknown) | "refused, idle runs", 50 % duties |
| T3: dc mode, 500 mV then 2000 mV | v_d 500 mV; 2000 mV clamped to **1000 mV** (ramping through 880 mV); CCR 2391/1859/1859 at 1 V (nominal 12 V scale, no bus) |
| T4: step capture, 800 mV at theta_e 1024 | `cap_begin`, 256 `cap` lines, `cap_end`; labelled "power OFF"; currents are ADC noise (no bus) |
| T5: one 4000-cycle forced overrun | overruns 1, no fault |
| T6: three in a row | fault 0x1 at the third; BDTR MOE 0; SD low, PC analog; CCR 2125 |

Observation: one missed conversion, first reported in report 4, which also says DWT had stopped after the flash session (also seen in Stage 6). This fits debugger interference; the stall length was not measured. The ISR-gap fault (> 17000 DWT cycles between powered ISR entries) and the 10 ms supervisor both count time only while the core runs: DWT CYCCNT and the kernel tick stop during a debug halt, while TIM1 may keep switching at the last duty. So **neither protects against a halt**. **Rule for Stage 7: no halts, no flash sessions and no halting debugger operations while the power stage is on.**

## First power (2026-10-15)

Setup: board UID `001D0050 42535013 20363130`, firmware `f2460b5`; USB unplugged; DCBUS from the bench supply, 15 V, 500 mA limit, OCP on; motor body clamped, shaft free, nothing on it; the user at the supply output button. J-Link (power off, SWD only, JLinkExe V9.84) in **one session kept open** from before arming to after disarming (`build/hil/stage7/first_power/jl.sh`); UART H2 921600. Raw files in `build/hil/stage7/first_power/` (`uart.log`, `jl_session1.out`, `r_steps.csv`, `r_marks.txt`, `arm_idle_marks.txt`, `vec_compare_dcbus.txt`).

J-Link behaviour, measured on USB power before DCBUS: connecting cost 2 missed periods (one 1920-cycle interval); reads over the open session cost nothing; disconnecting stops DWT (cycle figures invalid until the report thread restarts it, up to 1 s). Connecting on DCBUS (stage off) cost none. Hence: connect only with the stage off, disconnect only after disarming.

### Step 1: DCBUS on, stage off

| Check | Result |
|---|---|
| Supply | 15.008 V, **0.028 A** (user, supply display) |
| +12V at U14 pin 1 | **12.5 V** (user, DMM; nominal 12.6 V from the divider) |
| Firmware | boot clean, no warnings; VBUS 15.03 V (15.007–15.043); offsets IA 1962.74, IC 1996.94 counts (accepted); worst ISR 1918 cycles, 0 overruns, 0 missed; `vec_compare` PASS on this boot |
| Pins (J-Link) | GPIOB ODR 0, GPIOA ODR 0 (SD1–SD4 low), GPIOC MODER `0xFFFFFFFF` (PC0–PC3 analog). TIM1 BDTR `0x8000` (MOE 1) and CCER `0x511` (CH1–3, CH3N): set by `pwm_adc_init()` by design; with the stage off no pin carries TIM1 (PC0–PC2 analog, SD low). MOE is cleared only by a fault |

### Step 2: arm in idle (about 6 s)

| | Before | Armed | After disarm |
|---|---|---|---|
| Firmware | power off | **power ON** (request 1) | power off |
| SD1 (PB0) / SD2–SD3 (PA1–PA2) | low | **high** | low |
| PC0–PC2 | analog | TIM1 AF (MODER `0xFFFFFFEA`) | analog |
| CCR1–3 | 2125 | 2125 (50 %) | 2125 |
| avg IA / IB / IC | 9 / −18 / 9 mA | 7 / −11 / 3 mA | 7 / −15 / 8 mA |
| VBUS; worst ISR; faults | 15.03 V; 1917 cycles; none | 15.03 V; 2001 cycles; none | 15.03 V; 1917 cycles; none |

Nothing moved; the current averages did not change.

### Step 3: DC steps at theta_e 0 (R attempt, alignment)

Each step: arm in idle, mode dc at `v_mv`, about 3.5 s, idle, disarm. 0 faults, 0 overruns, 0 missed. Steady-state averages (firmware low-pass, tau 51 ms); baseline IA about 7 mA. B and C each carried about half of A.

| v_mv | avg IA (mA) | ΔIA (mA) | slope over the step (Ω) | encoder |
|---|---|---|---|---|
| 50 | 45 | 38 | 1.32 | 3840 |
| 100 | 72 | 65 | 1.85 | 3839 |
| 200 | 166 | 159 | 1.06 | 3833 |
| 300 | 402 | 395 | 0.42 | 3747 |
| 400 | 840 | 833 | 0.23 | 3742 |

**R not measured.** The current per step keeps growing: the inverter (EG2104 fixed dead time, switching delays) loses a voltage that depends on the current at these levels, comparable to the commanded 0.05–0.4 V. The slope never became constant below 0.85 A, and the next step could have reached the 2 A trip. The supply's 500 mA limit played no part (the motor needs under 0.1 W here).

Cross-check by the user with the supply off: DMM between two phase wires **0.3–0.4 Ω** (not recorded: which pairs, whether the motor was unplugged from the board, whether the lead resistance was subtracted). That suggests 0.15–0.3 Ω line to line, about 0.08–0.15 Ω per phase, in line with the spec's "170" read as 170 mΩ.

Alignment (first estimate): at 300–400 mV the rotor pulled to encoder 3742–3747 (3754 after release): electrical offset about (3745 × 7) mod 4096 = **1639 counts** (144°), to be confirmed at a second angle.

Decision (user, 2026-10-15): identify R, L and the encoder alignment automatically in firmware (current-regulated multi-point R, square-wave L around a DC bias, 90°/0° alignment), with `MOTOR/User/MotorControl/motor_identify.c` as the reference for the idea.

## Identification firmware `ebf9ea8` (2026-10-15)

`src/motor_ident.{c,h}`, mode 5 (`FOC_MODE_IDENT`), design and host evidence in `docs/evidence/motor_ident_software.md`. Order: align at 90° then 0° electrical (1.0 A), R from a current servo at 0.5–1.5 A (line fit over the top four points; reports all five and flags curvature), L from a ±0.2 V square wave around the 1.0 A bias. Bounded to 10 s; aborts on disarm, NaN, 4 consecutive samples above 1.8 A, or a servo target not reached in 1 s; the global 2 A trip and the 1.0 V cap are unchanged. **Command block moved to `0x20001288`** (arm +0, mode +4 …).

Flash: USB power only (supply output off), VTref 3.320 V, UID `001D0050 42535013 20363130`; `fw_ebf9ea8.bin` SHA-256 `71084933f93101c36a63b2843240f1253c0ee054f3349a655890040dbf5059c8`, `verifybin` OK. Boot clean; offsets IA 1962.28, IC 1996.09 counts; worst ISR 1942 cycles; `vec_compare` PASS. Mode 5 with the stage off: result state 8 (failed), reason 1 (not powered), all values 0; SD1–SD4 low, PC0–PC3 analog throughout. Raw files in `build/hil/stage7/ident/`.

### Identification run 1 (DCBUS 15 V, 2026-10-15)

Setup as in "First power" (USB unplugged, 500 mA, OCP on, motor clamped, user at the output button); one J-Link session opened with the stage off, closed after disarming. Boot: VBUS 15.02 V, offsets IA 1962.53, IC 1996.47 counts. Arm in idle → power ON, then mode 5. Raw: `build/hil/stage7/ident/` (`uart.log`, `jl_ident_run1.out`, `ident_run1_result.txt`, `ident_marks.txt`).

| Result | Value |
|---|---|
| State | 7 (done), no failure, no fault; VBUS 15.01–15.04 V throughout |
| Alignment | enc 61438 at 90°, 61305 at 0°: Δ −133 counts; reported direction +1, offset 3151. **Wrong** (see run 2): the 10 Hz `rot` lines show the rotor did not move at the 90° pull (1 → 0 → 4095 → 1), so it started about 180° away and the "90° → 0°" move was really 270° → 360° |
| R points (v_d, i_d) | 331 mV / 500 mA, 385 / 749, 436 / 999, 487 / 1249, 535 / 1500 |
| R (fit 0.75–1.5 A) | **0.200 Ω**, residual 1 mV, not flagged nonlinear |
| V0 (inverter loss) | **235 mV** |
| L (±0.2 V square wave around 1.0 A) | **69 µH**, ripple 572 mA p-p |

R is everything in the current path seen from the controller: the motor phase plus the MOSFETs, shunt, board and wiring. Line to line that is 0.40 Ω, consistent with the user's DMM reading of 0.3–0.4 Ω. V0 explains the failed open-loop R attempt: (400 − 235) mV / 0.200 Ω = 0.83 A, the 833 mA measured at 400 mV; below about 0.5 A the loss is smaller, so the open-loop slopes were wrong. τ = L/R = 0.35 ms.

**WCET violation:** ISR exec max was 4109 cycles (24.2 µs, one overrun) in the report where mode 5 started, 2707–3378 cycles during the run (3378 in the report with the R fit), 2245 after it, against the 3400-cycle (20 µs) budget; idle is about 1950. No missed period during the run and no fault; one missed period at the J-Link disconnect after disarming (stage off). Fix in progress before further powered runs.

### WCET fix `a96018a` (2026-10-15)

Causes (disassembly, `docs/evidence/motor_ident_wcet.md`): byte-loop `memset` of the 156-byte state at mode entry, byte-loop `memcpy` of the 88-byte result at publish, library `fminf`/`fmaxf` in the servo and fit, and a `sinf`/`cosf` pair every period. Fixed with explicit field resets and copies, inline clamps, integer period constants, and constant trig for the two fixed angles (0° and 90°).

Flashed on USB power (VBUS 0 mV, VTref 3.320 V, UID `001D0050 42535013 20363130`) through probe S/N 63728769 (a second probe, S/N 20781318, is also plugged in; selected by `-USB`). `fw_a96018a.bin` SHA-256 `0f232611e3e750faa5de1c8d0621e497ef227a9afeac1a7ad3804b01d35bb390`, verify OK. **Command block now at `0x200013D0`.** Boot benchmarks on the target (max cycles, 512 repetitions): ident start 112, servo step 376, R fit step 794, L step 397, result copy 140; `sinf+cosf` (772) is no longer on the ident path. Idle ISR 1946–1951 cycles; offsets IA 1963.04, IC 1996.94; `vec_compare` PASS. The powered ISR maximum in mode 5 is still to be measured.

### Identification run 2 (`a96018a`, DCBUS 15 V, 2026-10-15)

Same setup; probe S/N 63728769; connect with the stage off cost 2 missed periods (as before). Raw: `run2.log`, `jl_ident_run2.out`, `ident_run2_result.txt`.

| Result | Value |
|---|---|
| ISR | exec max **2585 cycles (15.2 µs)** over the run (reports 36–42: 2069–2585), **0 overruns**, no new missed period; after the run about 2040, idle 1963 |
| State | 7 (done), no failure, no fault |
| R points | 329 mV / 502 mA, 384 / 750, 436 / 1000, 487 / 1250, 534 / 1499 |
| R, V0, residual | **0.201 Ω**, 233 mV, 1 mV (run 1: 0.200 Ω, 235 mV) |
| L, ripple | **69 µH**, 573 mA p-p (run 1: 69 µH) |
| Alignment | enc 61189 at 90°, 61332 at 0°: Δ +143 counts, direction −1, offset 756 (this boot). The `rot` lines show a proper first pull (4095 → 3845) |

Direction is **−1** (the count falls as the electrical angle rises): run 2 directly, and run 1 once its 270° start is taken into account (270° → 360° is +90°, measured −136 counts). Design flaw: the alignment never checks that the first pull moved the rotor. Fix in progress: steps 0° → 120° → 240° → 360°, direction from the two forced +120° moves, which must agree.

### Alignment fix `9d385a0` (2026-10-15)

Alignment now parks at 0°, then pulls to 120°, 240° and 360° (0°) at 1.0 A with the current decayed to zero between steps. Direction comes from the 120° → 240° and 240° → 360° moves, which must agree in sign and each be 1365 ± 200 electrical counts; offset from the final 0° position. Host plant with torque and stiction, five start angles including the 180° unstable start, both directions. Phase numbering changed: done = 9, failed = 10. **Command block now at `0x200013F0`.**

Flashed on USB power (VBUS 0 mV, VTref 3.320 V, UID as above, probe 63728769); `fw_9d385a0.bin` SHA-256 `3c9c20dc7a09828dd153a9498760e4dc46ac39fa8645bf8aae6dda0e312da812`, verify OK. Boot benchmarks (max cycles): start 112, servo 406, R fit 795, L 395, copy 142; idle ISR 1963–1966; offsets IA 1962.34, IC 1996.45; `vec_compare` PASS. Mode 5 with the stage off: state 10, reason 1 (not powered); SD1–SD4 low, PC0–PC3 analog.

### Identification run 3 (`9d385a0`, DCBUS 15 V, 2026-10-15)

Same setup; probe 63728769. Raw: `run3.log`, `jl_ident_run3.out`, `ident_run3_result.txt`.

| Result | Value |
|---|---|
| ISR | exec max **2606 cycles (15.3 µs)** over the run, **0 overruns**; the one missed period was at the J-Link connect (uptime 8 s, stage off), none while powered (29–42 s) |
| State | 9 (done), no failure, no fault |
| Alignment | enc 61248 at 120°, 61055 at 240°, 60865 at 360°: **Δ −193 and −190 counts** (ideal 195), **direction −1**, offset 4025 electrical counts (this boot). `rot` trace: park 0 → 0, 120°: 4094 → 3905, 240°: 3898 → 3711, 360°: → 3521 |
| R points | 331 mV / 500 mA, 384 / 749, 436 / 1000, 487 / 1249, 535 / 1499 |
| R, V0, residual | **0.201 Ω**, 234 mV, 1 mV |
| L, ripple | **70 µH**, 566 mA p-p |

Over three runs: R 0.200 / 0.201 / 0.201 Ω, V0 235 / 233 / 234 mV, L 69 / 69 / 70 µH. Direction −1 (runs 2 and 3; run 1's +1 was the alignment flaw). The three forced +120° steps also confirm the phase order against the encoder: the field rotating forward (A → B → C) turns the rotor with a falling count, every step. The offset is per boot (incremental encoder): run identification after each power-up, or reference it to Z later.

Roadmap stage 7 status: step 1 (first power) done; step 2 (R, L) done; step 4 (alignment) done; step 3 (open-loop rotation) partly covered by the three forced steps; steps 5 (current loop, gains from R and L: kp = L·ωc ≈ 0.43 V/A, ki = R·ωc ≈ 1260 V/(A·s) at 1 kHz) and 6 (speed loop) remain.

### Open-loop check `c03b6ad` (DCBUS 15 V, 2026-10-15)

Firmware `c03b6ad` (mode 6: a voltage vector of `v_mv` rotating at `rate_e` electrical counts/s; per-boot calibration from mode 5; 10 Hz `ol <steps> <theta_cmd> <theta_meas> <err>` with err = cmd − meas). Flashed on USB power (VBUS 71 mV, probe 63728769), SHA-256 `9914e0bf7ad2f042e0125b19f14a068df2cac9e5ae1023ce7d6aaa58de086461`, verify OK; command block `0x20001410` (rate_e +20). Unpowered: the commanded angle advanced 205 counts per 0.1 s at 2048 counts/s, stage off throughout. Raw: `build/hil/stage7/openloop/` (`ident.log`, `ol_fwd.log`, `ol_bwd.log`, `ol_*_err.txt`, `jl_openloop.out`).

On DCBUS: identification run 4 first (direction −1 from −204 / −179 counts, offset 3605, R 0.199 Ω, V0 235 mV, L 71 µH, ISR max 2649 cycles). Then 440 mV (about 1 A) at +2048 and −2048 counts/s (half an electrical turn per second, 14 s per mechanical turn), 16 s each, from theta_cmd 0:

| | Forward | Backward |
|---|---|---|
| err after the ramp (counts) | mean −56, sd 152, −332 … +442 | mean −259, sd 133, −598 … +86 |
| First / last third mean | −95 / −11 | −210 / −282 |
| ISR max; faults | 2666 cycles; none | 2680 cycles; none |

- **The calibrated angle tracks the rotor** through about 8 electrical turns (more than one mechanical turn) each way. A wrong direction or pole-pair count would accumulate thousands of counts; the error stays bounded, with a position-dependent ±80-count variation between thirds.
- **Load angle** (fwd − bwd)/2 = 101 counts = **8.9° electrical lag** in both directions, as expected.
- **Offset bias** (fwd + bwd)/2 = −157 counts = **−13.8° electrical**: the alignment's final park always arrives from the same side (forward from 240°), so friction stops the rotor short. Fix planned: approach 0° from both sides and average.
- The swing (sd about 12°) is cogging under a 1 A open-loop drive.

### Current-loop firmware `89f28a3` (2026-10-15)

Mode 4 while powered: only with a valid, plausible identification this boot (0.02 < R < 2 Ω, 10 µH < L < 5 mH); calibrated angle; gains kp = L·ωc, ki = R·ωc at 1 kHz; PI and vector limited to the 1.0 V cap (`foc_current_step_lim`, host-tested); id/iq references (`id_ma` +24, `iq_ma` +28) latched on a `step_seq` edge, |id| ≤ 1.0 A, |iq| ≤ 0.5 A, applied at capture sample 16 of a 256-sample dq capture; overspeed trip above 205 counts per 10 ms (300 rpm, fault 0x100). Alignment now ends with a two-sided park at 0° (arrive forward, then backward, circular mean). Host: iq/id step 90 % in 0.25 ms, overshoot 2–3 % (R 0.2 Ω, L 70 µH, locked rotor, one-period delay); two-sided park cuts the stiction bias from 64–65 to 1–2 counts. Design in `docs/evidence/stage7_current_loop_software.md`.

Flashed on USB power (VBUS 89 mV, probe 63728769), SHA-256 `afb57f28e154db0ec308f322596cded196d4bf82ae7263cd4d38e5f0ef43536b`, verify OK; **command block `0x20001504`**. Boot benchmarks (max cycles): mode-4 kernel + angle + speed 1547, full Stage 5 step 1724, ident steps ≤ 828. Mode 4 with the stage off (same arithmetic, stage 6 path): ISR exec max **2603–2683 cycles (15.3–15.8 µs)** against 3400; stage off and pins safe throughout. Raw: `build/hil/stage7/current/`.

### First closed current loop (`89f28a3`, DCBUS 15 V, 2026-10-15)

Same setup, shaft free (user's choice for the iq tests). Raw: `build/hil/stage7/current/` (`ident.log`, `id_steps.log`, `iq_steps.log`, `jl_current_run1.out`, `marks.txt`).

**Identification run 5:**
- R 0.200 Ω, V0 234 mV, L 69 µH, direction −1 (Δ −219 / −186), offset 3080, ISR max 2785 cycles.
- Two-sided park: A = B = 61000 exactly (return through 120° at 60820), friction band 0.
- The open-loop offset bias is therefore **not friction but a cogging detent**: 84 detents per turn, one every 30° electrical, so the rotor settles up to ±15° from the field. A two-sided park cannot remove it; the forward/backward open-loop average can (−157 counts last time).

**Current loop:**

| | Bench | Host simulation |
|---|---|---|
| Gains | kp 435 mV/A, ki 1258 V/(A·s) | same |
| id 0 → 500 mA | final 500 mA (noise sd 11 mA), iq −1 mA (sd 12); 90 % at **1.05 ms**, overshoot 3 % | 90 % at 0.25 ms, overshoot 2 % |
| id 500 → 0 mA | final 1 mA; 90 % at 1.0 ms, overshoot 0 % | 0.25 ms |
| iq 0 → +50 mA, then disarm after 100 ms | final 50 mA (sd 17); 90 % at 1.3 ms; shaft did not turn (encoder 3655–3656) | — |
| iq 0 → −50 mA, same | final −50 mA (sd 11); 90 % at 1.1 ms; shaft did not turn | — |
| Faults; overruns; speed | none; 0; 0 rpm | — |
| ISR exec max (powered mode 4) | **3049 cycles (17.9 µs)**, 350 cycles under the budget | — |

- **The loop is stable and holds the references** with no cross-coupling, but it rises about 4× slower than the simulation.
- The response jumps to about 35 % in 3 samples, then creeps at the integral rate. At a 0.5 A error, kp gives 0.22 V, almost exactly the measured inverter loss V0 = 234 mV, so the proportional action is largely lost and the integrator builds the rest.
- The simulation has no dead-time model. **Remedy: dead-time compensation** (feed-forward of the measured V0 per phase in the direction of that phase's current, smoothed near zero current).
- 50 mA × 0.022 Nm/A ≈ 1.1 mNm is below the cogging/stiction torque, so the free shaft stayed still.
- WCET margin is thin; the largest single item is `sinf`+`cosf` (up to 772 cycles). A table or a combined sin/cos would recover margin.

### CORDIC, dead-time compensation, rotation offset `f396936` (2026-10-15)

- **Sin/cos:** the BSP CORDIC (`src/sincos.{c,h}`: COSINE, q1.31, 24 iterations, one argument, two results) replaces every sinf/cosf in the control step; host builds use a sinf/cosf stub with the same angle mapping.
- **Dead-time compensation** (mode 4, `dtc` +32, latched on `step_seq`, default off): a per-phase feed-forward of 0.75·V0 times clamp(i_ref,x / 50 mA), from the reference current. The PI gets the remaining headroom (at least 0.2 V); the total vector stays capped at 1.0 V.
- **Rotation offset refinement** in mode 5: after L, 2 electrical turns forward then back at the 1 A voltage. offset_new = offset − (mean_f + mean_b)/2; fails on a slip (> 90°), |c| > 400 or an implausible load angle.
- **Host:** DTC off 1.10 ms / on 0.15 ms (90 %, 1.3 % overshoot) with a per-phase dead-time plant; 84-detent cogging park bias −112 → 0 counts.
- Design: `docs/evidence/dtc_cordic_offset_software.md`.

**Incident before the flash: USB and DCBUS were connected together** for about a minute.
- The user plugged in USB while the supply output was still on; the board kept running from DCBUS (VBUS 15.03 V live, uptime 31 min).
- Flashing was held; the user switched the supply off; VBUS fell to 89–107 mV and the board kept running on USB without a reboot.
- This breaks `HW-PWR-003` (USB_5V is tied to +5V through R35, 0 Ω). The board, the J-Link (VTref 3.316–3.320 V) and the USB port kept working; whether any part got warm is to be confirmed by the user.
- Rule kept: before every flash, check VBUS < 1 V in the live UART log, not only the user's word.

**Flash:** USB power, probe 63728769, SHA-256 `4b899fdf77b4f1edeed0943326595ddd58797923bd090eafead02d6b96c3f67d`, verify OK; **command block `0x2000159C`** (dtc +32). Boot benchmarks (max cycles):

| Item | Max cycles |
|---|---|
| CORDIC sin+cos | **27** (sinf+cosf: 772) |
| CORDIC max error (4096 angles) | 1.61e-6 vs sinf/cosf, 1.58e-6 vs double |
| Mode-4 kernel + DTC + angle + speed | 1344 |
| Mode 4 complete, unpowered path | 2082 |
| Ident steps | ≤ 817 |

Idle ISR 1890–1908 cycles (was about 2050); mode 4 with the stage off 2454–2522 (was 2603–2683); `vec_compare` PASS. Raw: `build/hil/stage7/dtc/`.

### Dead-time compensation on the bench (`f396936`, DCBUS 15 V, 2026-10-15)

Fresh boot on DCBUS (USB unplugged first), probe 63728769. Raw: `build/hil/stage7/dtc/` (`ident.log`, `id_steps.log`, `jl_dtc_run1.out`, `marks.txt`).

**Identification run 6:**
- Direction −1 (Δ −213 / −190), park A 60857 / B 60854 (friction half −10 counts).
- **Rotation correction c = +58 counts** (+5°), load angle 124 counts (11°), final offset 4033. This boot parked in a different detent than the open-loop test (−157 then): the per-boot correction this step exists for.
- R 0.200 Ω, V0 234 mV, L 71 µH; ISR max 2785 cycles.

**Current loop:**

| id step | DTC off | **DTC on** | Host (DTC on) |
|---|---|---|---|
| 0 → 0.5 A, 90 % | 1.05 ms | **0.10 ms** (86 % after 2 samples) | 0.15 ms |
| 0.5 → 0 A, 90 % | 1.6 ms | **0.05 ms** | — |
| Final / noise sd | 499 / 12 mA | 500 / 12 mA | — |
| iq cross-coupling | −3 mA | −3 mA | 0 |

- With compensation the loop is about 10× faster and matches the simulation.
- A small tail remains from 86 % to 100 % over about 1 ms (residual dead-time mismatch).
- No fault, 0 overruns, 0 missed periods.

**ISR timing:**
- Exec max in powered mode 4 with DTC reached **3279 cycles (19.3 µs)**, only 121 cycles under the budget (3049 before DTC/CORDIC).
- Unpowered mode 4 measures about 2500 cycles; the boot bench gives 1344 for the control step.
- `FLASH_ACR` = `0x00040704`: 4 wait states, prefetch, I-cache and D-cache all on. The remaining gap is most likely flash/cache misses: the ISR path no longer stays cached between periods. Next: measure the ISR segments with DWT and run the ISR path from CCM SRAM.

### FOC ISR path in CCM SRAM `eeaddaf` (2026-10-15)

- **Design:** `dma_isr`, `foc_ctrl_step`, `motor_ident`, `power_guard`, `lib/foc` and the per-period `pwm_adc` helpers execute from the 10 KiB CCM SRAM (0x10000000, I-code bus, zero wait states), copied before the kernel starts. sram0 is reduced to 22 KiB so the CCM alias at 0x20005800 is not used twice. DMA buffers stay in SRAM1/2. CI counts CCM separately (≤ 10240 B) and tightens the SRAM gate to 22528 B. The ISR reports pre/step/post segment maxima. Design: `docs/evidence/isr_ccm_software.md`.
- **Flash:** on USB power, VBUS 0 mV checked live by Claude (fresh boot), probe 63728769; SHA-256 `f128c313c7f3490274a38500ea0db861d1444c38a1c0f4ba994d8c3a69422d3d`, verify OK; command block **`0x200015A8`**. Usage: flash 77204 / 98304, SRAM 11340 / 22528, CCM 9056 / 10240 B.

| ISR (cycles, max) | Before (flash, `f396936`) | **CCM (`eeaddaf`)** |
|---|---|---|
| Idle | 1890–1908 | **1159–1167** (segments 169 / 781 / 239) |
| Mode 4, stage off | 2454–2522 | **1516–1547** (segments 170 / 1111–1168 / 238) |
| Boot bench mode-4 complete | 2082 | 1280 |

`vec_compare` PASS; stage off and pins safe throughout. The powered mode-4 figure with DTC is still to be measured. Raw: `build/hil/stage7/ccm/`.

### Powered run with the CCM firmware (`eeaddaf`, DCBUS 15 V, 2026-10-15)

Fresh boot on DCBUS, probe 63728769. Raw: `build/hil/stage7/ccm/` (`run.log`, `run2.log`, `jl_ccm_run.out`).

- **Powered identification ISR max 1641 cycles (9.7 µs)** (segments up to 174 / 1259 / 236), against 2785 from flash.
- **Identification failed twice with alignment (reason 5) at identical positions:**
  - enc 61401 / 61160 / 61036, so Δ −241 and −124 counts = 148° and 76° electrical, against the 120° ± 17.6° tolerance.
  - The two moves still sum to about 224°.
  - The `rot` trace shows the 0° → 120° step moving about 154°.
  - Cause: at 1.0 A the alignment loses to the cogging detents (84 per turn, one every 30° electrical). Friction held the rotor in a detent past 240°, and the next step fell short.
  - The positions repeat because the detents are deterministic and the rotor started where the previous run ended.
- With the calibration invalid, **mode 4 was refused while powered** ("mode 4 (refused, idle runs)"), so no id steps ran. Powered mode-4 timing is still to be measured.
- No fault; one missed period at the J-Link connect (stage off).
- **Fix in progress:** alignment at 1.5 A; per-move tolerance 67°–173° plus a check on the two moves together (205°–275°); rotation correction limit 600 counts. The rotation step makes the final offset precise.

### Cogging-tolerant alignment `50f17c3`: powered timing goal met (DCBUS 15 V, 2026-10-15)

- **Firmware `50f17c3`:**
  - alignment at 1.5 A;
  - per-move check 1365 ± 600 electrical counts plus a two-move sum check of 2730 ± 400;
  - rotation-correction limit 600 counts; L bias kept at 1.0 A.
  - Host strong-cogging case (144° / 76° moves) passes, with a final offset error of 14 counts.
- **Flash:** on USB (VBUS 71 mV checked live), SHA-256 `dc3c9c99f01bafebba1c3ffdf532dcdbfab1e893767b3b0134a491668f7e721b`, verify OK; command block `0x200015A8`; `vec_compare` PASS.
- **Identification run 9** (raw `build/hil/stage7/al2/`):
  - Δ −202 / −190, direction −1;
  - park A 60711 / B 60709;
  - rotation c +56, load 125 counts, offset 958;
  - R 0.199 Ω, V0 235 mV, L 69 µH;
  - ISR max 1733 cycles.
- **Powered mode 4 with DTC:**
  - id 0 → 0.5 A 90 % in **0.10 ms**, 0.5 → 0 A in **0.05 ms**; overshoot 3 %, noise sd 8–9 mA, iq ±5 mA (sd 17–18).
  - **ISR exec max 2013 cycles (11.8 µs)**, segments 174 / 1631 / 237, against 3279 before CCM and the 3400 budget.
  - No fault.

### Speed-loop firmware `6d856c1` (2026-10-15)

Mode 7 (`FOC_MODE_SPEED`):
- **Speed estimate:** a rolling 10 ms window updated every 1 ms (1.46 rpm/count, about 5 ms delay).
- **Speed PI** every 1 ms → iq_ref, limited to ±0.5 A. Gains from an *assumed* J = 1.5e-5 kg·m² at 10 Hz: kp 4.55 mA/rpm, ki 71.4 mA/(rpm·s).
- **Reference:** `spd_rpm` (+36), at most ±200 rpm, ramped at 400 rpm/s, or applied as a step with `spd_step` (+40) = 1 together with a 1 kHz speed capture.
- Same current loop, DTC and caps as mode 4; overspeed trip unchanged.

Host simulation (cogging 2 mNm, friction, dead time, quantisation): 0 → 100 rpm rise 17 ms, overshoot 17 %, settled 0.12 s at J = 1.5e-5; stable at J × 1/3 (overshoot about 50–60 %) and × 3 (30 %). Bench steps are therefore kept at ≤ 100 rpm until J is known. Design: `docs/evidence/stage7_speed_loop_software.md`.

Flash on USB power (VBUS 35 mV checked live), SHA-256 `000468a86dffb1b87f6665d0286961fd6f1b80a8ad6bad52f1c8b15f00022fdc`, verify OK. **Command block `0x20001654`** (spd_rpm +36, spd_step +40); test_spin_count/cycles at `0x20000B5C` / `0x20000B60`.

| Resource / timing | Value |
|---|---|
| Flash | 79212 B |
| SRAM | 11520 B |
| CCM | 9920 / 10240 B (320 B free) |
| Boot bench: speed 1 ms update | 221 cycles |
| Boot bench: mode-7 period, unpowered | 1281 cycles |
| Mode 7 with the stage off: ISR max | 1646 cycles; stage off, pins safe |

`vec_compare` PASS.

### Speed loop and safe stops (`6d856c1`, DCBUS 15 V, 2026-10-15)

Shaft free, motor body clamped, probe 63728769. Raw: `build/hil/stage7/speed/` (`ident.log`, `speed.log`, `ripple.log`, `demo1.log`, `demo2.log`, `ident3.log`, `demo3.log`, `jl_speed_run.out`, `marks.txt`).

**Identification runs 10 and 11:**

| Run | Moves | Rotation c | R | L |
|---|---|---|---|---|
| 10 | −192 / −191 | +58 | 0.200 Ω | 70 µH |
| 11, after the reset | −199 / −182 | −113 | 0.198 Ω | 71 µH |

Both passed.

**Speed loop** (mode 7, DTC on). 10 Hz telemetry aliases the ripple; the 1 kHz captures are the reference:

| Reference | Mean (1 kHz capture) | Ripple | Dominant frequency | Electrical frequency |
|---|---|---|---|---|
| 60 rpm | 63.8 rpm | sd 19.3 rpm (±24) | 15 Hz | 7.4 Hz |
| 100 rpm | 99.6 rpm | sd 14.5 rpm (±18) | 23 Hz | 11.6 Hz |
| −60 rpm | −62.7 rpm (10 Hz) | sd 22.6 rpm | — | — |
| 100 rpm, 27 s hold | 103.1 rpm (10 Hz) | sd 15.8 rpm | — | — |

- Step 60 → 100 rpm: about 30 ms to the new level, then the same ripple. Stopped cleanly at 0 rpm.
- ISR max **2122 cycles**, 0 overruns, no fault.
- **The motor holds a speed in the mean**, forward and backward.
- **The ripple is position-locked at twice the electrical frequency**, at both speeds, so it is not a control-loop resonance.
- It implies a torque disturbance of about 3–4 mNm, comparable to the drive torque here (iq about 0.1 A ≈ 2 mNm) and to the stiction seen earlier.
- Likely source: magnet-to-magnet variation of the cogging torque (14 per turn = 2× electrical). An IA/IC gain mismatch also gives 2× electrical, but the measured mismatch (about 3 %) is far too small at 0.1 A.
- The simulation (cogging only at 84 per turn) predicted about 1 rpm and does not model this.
- **Known limitation.** The remedy is a learned cogging feed-forward (later work).

**Safe stops while running at 100 rpm:**

| Demo | Result |
|---|---|
| 1. Disarm (arm key cleared) | SD high while running, **low on the next J-Link read** after the write; PC0–PC2 analog; the rotor coasted from about 108 rpm to 0 within about 0.3 s; no fault |
| 2. Forced fault (3 consecutive forced 4000-cycle ISR overruns) | **fault 0x1 latched** within the 50 ms read window: overruns 3, SD low, PC analog, BDTR 0 (MOE off), CCR 2125; cleared only by reset (J-Link reset with the stage already off) |
| 3. Supply output button (user) | The motor stopped (all power lost). The UART log ends within about 0.1 s after the last line (VBUS still 15.04 V at the previous report). The board's logic, fed from the same bus, browned out before the next telemetry line, so **the firmware VBUS trip (< 12 V for 3 periods) could not be observed this way**. Physically safe: without the bus nothing can drive the motor. Showing the trip needs a scope on VBUS and an SD pin |

Findings:
- After a latched fault, the speed and rot telemetry freezes at the last value (99.6 rpm), because the skipped control step no longer updates the snapshot. Misleading, not unsafe; to fix.
- In mode 7 with the stage off, the speed PI keeps integrating (iq_ref 500 mA). Harmless (the stage is off; re-arming needs idle, which resets the loop).

## Next (needs the user's go)

Stage 7 first power, in the roadmap order, stopping at the first surprise:
1. USB unplugged; DCBUS 15 V from the bench supply (500 mA, OCP on); motor clamped, nothing on the shaft; the supply output button within reach. Check +12V (U14 pin 1) is about 12.6 V and that the firmware reports VBUS about 15 V, with the stage still off.
2. Arm (idle), check that nothing moves and the current reads 0; disarm.
3. R: with mode 0 and `v_mv` 0, write the arm key and confirm "power ON" (arming needs 20 periods of commanded idle; switching to dc first and arming later does not arm), then mode 1. Keep the key set. Small `v_mv` steps (50, 100, 150 mV …), averaged currents; R from the slope between steps (cancels a constant dead-time offset). Record the encoder count after the rotor aligns (theta offset). Back to idle, clear the key.
4. L: with `v_mv` 0, arm in idle again and confirm "power ON", then mode 2 and step captures at the same angle. Back to idle, clear the key.
