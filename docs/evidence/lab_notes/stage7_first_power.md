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

## Next (needs the user's go)

Stage 7 first power, in the roadmap order, stopping at the first surprise:
1. USB unplugged; DCBUS 15 V from the bench supply (500 mA, OCP on); motor clamped, nothing on the shaft; the supply output button within reach. Check +12V (U14 pin 1) is about 12.6 V and that the firmware reports VBUS about 15 V, with the stage still off.
2. Arm (idle), check that nothing moves and the current reads 0; disarm.
3. R: with mode 0 and `v_mv` 0, write the arm key and confirm "power ON" (arming needs 20 periods of commanded idle; switching to dc first and arming later does not arm), then mode 1. Keep the key set. Small `v_mv` steps (50, 100, 150 mV …), averaged currents; R from the slope between steps (cancels a constant dead-time offset). Record the encoder count after the rotor aligns (theta offset). Back to idle, clear the key.
4. L: with `v_mv` 0, arm in idle again and confirm "power ON", then mode 2 and step captures at the same angle. Back to idle, clear the key.
