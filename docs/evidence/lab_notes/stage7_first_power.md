# Stage 7 — First power

Status: **not entered** (2026-10-15). The firmware and the checks without DCBUS pass, and R-18 passes for this firmware (`r18_boot_capture.md`, run 2). Entering Stage 7 still needs the user's explicit go (`AGENTS.md`).

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
| Trips (latched until reset) | \|IA\|, \|IB\|, \|IC\| > 2 A; VBUS outside 12–17 V for 3 periods; 3 overruns in a row; DMA error; ADC overrun; a duty outside [0, 1] (NaN); ISR gap > 2 periods while powered; pin invariant. A fault: SD low, PC analog, MOE off, 50 % |
| Supervisor | 10 ms kernel timer: while powered, fewer than 150 of 200 ISRs → `pwm_adc_stop()` (final until reset) |
| Voltage | Vector capped at 1.0 V while powered; duty scale from the latest raw VBUS sample |
| Modes (`foc_ctrl_cmd.mode`) | 0 idle; 1 dc (vector `v_mv` at `theta_e`, ramped 1 V/s: R, alignment); 2 step (each `step_seq` change: 16 zero, 128 on, 112 zero periods, IA/IC captured and printed: L); 3/4 the Stage 6 modes, only while off; unknown values run idle and cannot arm |
| Command block | `foc_ctrl_cmd` at `0x2000118C`: arm +0, mode +4, v_mv +8, theta_e +12, step_seq +16 (address printed at boot). J-Link `w4` values are hex |

## Checks without DCBUS (2026-10-15)

Setup: board UID `001D0050 42535013 20363130`; firmware `f2460b5` flashed and verified; **USB power only** (VTref 3.320 V), no DCBUS; UART H2 921600. Raw files in `build/hil/stage7/` (`uart_main.log`, `jlink_checks.txt`, `checks.jlink`).

| Check | Result |
|---|---|
| Boot | "power stage off"; VBUS 35–53 mV; offsets IA 1962.33, IC 1996.41 counts (accepted); Stage 5 vectors still match the host (`vec_compare` PASS) |
| ISR | 20 kHz, period 8489–8511 cycles, worst 1901 cycles (11.2 µs), 0 overruns, 0 DMA errors, 0 ADC overruns |
| T1: arm key written, no DCBUS | Power stayed **off** in every report ("request 0, on 0 times"). Readback: GPIOB ODR 0 and GPIOA ODR 0 (SD1–SD4 low; PA0–PA2 output mode), GPIOC MODER `0xFFFFFFFF` (PC0–PC3 analog) |
| T2: mode 99 (unknown) | "refused, idle runs", 50 % duties |
| T3: dc mode, 500 mV then 2000 mV | v_d 500 mV; 2000 mV clamped to **1000 mV** (ramping through 880 mV); CCR 2391/1859/1859 at 1 V (nominal 12 V scale, no bus) |
| T4: step capture, 800 mV at theta_e 1024 | `cap_begin`, 256 `cap` lines, `cap_end`; labelled "power OFF"; currents are ADC noise (no bus) |
| T5: one 4000-cycle forced overrun | overruns 1, no fault |
| T6: three in a row | fault 0x1 at the third; BDTR MOE 0; SD low, PC analog; CCR 2125 |

Observation: one "missed" period again in the window where the J-Link closed the flash session (also seen in Stage 6). Closing a J-Link session that halted the core stalls the CPU briefly. Unpowered this is harmless; with power on the ISR-gap fault would trip. **Rule for Stage 7: no halts, no flash sessions and no halting debugger operations while the power stage is on.**

## Next (needs the user's go)

Stage 7 first power, in the roadmap order, stopping at the first surprise:
1. USB unplugged; DCBUS 15 V from the bench supply (500 mA, OCP on); motor clamped, nothing on the shaft; the supply output button within reach. Check +12V (U14 pin 1) is about 12.6 V and that the firmware reports VBUS about 15 V, with the stage still off.
2. Arm (idle), check that nothing moves and the current reads 0; disarm.
3. R: dc mode at small `v_mv` steps (50, 100, 150 mV …), averaged currents; R from the slope between steps (cancels the dead-time offset). Record the encoder count after the rotor aligns (theta offset).
4. L: step captures at the same angle.
