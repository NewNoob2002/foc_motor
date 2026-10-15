# R-18 — SD and PWM pins during power-up and reset (oscilloscope)

Status: **pass** for firmware `1d39b8c` (2026-10-14, run 1) and for the Stage 7 firmware `f2460b5` (2026-10-15, run 2, below): no net crossed 0.8 V on power-up, the reset button or SYSRESETREQ, nor (run 2) while the arm key was written. R-18 is closed for `f2460b5`; a later firmware that changes how these pins are driven needs the procedure again.

## What is checked

That no gate-driver control net goes above the EG2104 logic-low limit during power-up, the reset button, a debugger reset (SYSRESETREQ) and the firmware start. `DOC-GATE-DS` §7.2: input low ≤ 1.0 V, high ≥ 2.5 V; IN has an internal 200 kΩ pull-down.

| Net | MCU pin | Probe point (EG2104 SOP-8) |
|---|---|---|
| SD1 | PB0 | U14 pin 3 |
| SD2 | PA1 | U15 pin 3 |
| SD3 | PA2 | U16 pin 3 |
| SD4 | PA0 | U17 pin 3 |
| PWMA | PC0 | U14 pin 2 |
| PWMB | PC1 | U15 pin 2 |
| PWMC | PC2 | U16 pin 2 |
| PWMD | PC3 | U17 pin 2 |

Pins 2 (IN) and 3 (SD) are next to each other and pin 1 is VCC: touch only the intended pin. Probe ground: DGND at H2-4 or U7-6 (or U14 pin 4), nothing else.

## Setup

| Item | Value |
|---|---|
| Board | UID `001D0050 42535013 20363130` |
| Firmware on the board | `1d39b8c` (stage 4: TIM1 internal, outputs disabled, PC0–PC3 analog) |
| Power | J-Link VTref 3.3 V only; no DCBUS, no USB; U14 pin 1 (EG2104 VCC) measured 0 V by the user |
| Scope | ALIENTEK DS100 Mini (user manual V4.5): 2 channels, one probe used (CHA); probe switch ×10 and scope probe ratio X10 (user, 2026-10-15), so the 0.8 V threshold is 0.8 V at the pin; MCX probe at ×10: 140 MHz, 10 MΩ, 15 pF; max 250 MSa/s, 128 Kpts; triggers: auto / normal / single, rising or falling edge |

Scope settings for each net (DS100 Mini menu names in brackets): probe switch ×10 and channel probe ratio X10 [通道 CHA → 探头 X10]; 1 V/div [垂直档位]; trigger type CHA rising [触发类型 CHA-上]; **threshold 0.8 V** [触发阈值]; mode single [触发方式 单次]; timebase **20 µs/div**; only CHA on.

Why 20 µs/div: the DS100 samples at the rate the timebase sets (manual §4.3: 1 ms/div is only 5 MSa/s, one point per 200 ns; 20 µs/div is 200 MSa/s, one point per 5 ns; a second channel halves it). While armed it keeps sampling until the trigger, so a fast timebase still waits indefinitely and catches pulses down to about 10 ns. The display then shows only 200 µs around the trigger point.

## Procedure, per net

1. **Positive control (once per session):** probe the 3.3 V rail (H2-1) with the same trigger and power the board up: the scope must trigger. This shows the setup would catch a rising edge.
2. Probe the net. Auto mode: confirm it sits at about 0 V; save a screenshot.
3. Arm single trigger. Power up: unplug and replug the J-Link USB (the board's only supply). Note: triggered / not triggered.
4. Re-arm. Press the reset button SW1. Note.
5. Re-arm. Debugger reset: Claude runs a J-Link `r` + `g` (SYSRESETREQ). Note.
6. If it triggers at any step: save the screenshot and stop; that is a finding.

Pass for a net: no trigger at steps 3–5 and about 0 V in step 2.

## Positive control

2026-10-14: probe on the 3.3 V rail (H2-1), single rising trigger, J-Link USB power-cycled: **the scope triggered** (user report; timebase of that capture not recorded). To repeat at 20 µs/div before the net captures.

## Results

| Net | Idle level (step 2) | Power-up | SW1 reset | SYSRESETREQ | Screenshot |
|---|---|---|---|---|---|
| SD1 | 30 mV | no trigger | no trigger | no trigger (23:37:49) | — |
| SD2 | about 30 mV | no trigger | no trigger | no trigger (23:38:35) | — |
| SD3 | about 30 mV | no trigger | no trigger | no trigger (23:39:20) | — |
| SD4 | about 30 mV | no trigger | no trigger | no trigger (23:39:50) | — |
| PWMA | about 70 mV | no trigger | no trigger | no trigger (23:40:14) | — |
| PWMB | about 70 mV | no trigger | no trigger | no trigger (23:44:01) | — |
| PWMC | about 70 mV | no trigger | no trigger | no trigger (23:44:34) | — |
| PWMD | about 70 mV | no trigger | no trigger | no trigger (23:45:02) | — |

## Conclusion and limits

All 8 nets pass: SD1–SD4 idle at about 30 mV and PWMA–PWMD at about 70 mV (well under the EG2104 1.0 V low limit; the PWM lines are driven by nothing, PC0–PC3 being analog, so the probe sees the EG2104 input pull-down and noise), and none triggered at 0.8 V on power-up, SW1 or SYSRESETREQ.

Limits:
- One trial per event per net; the power-up and SW1 tests were done by the user without timestamps.
- Valid for firmware `1d39b8c` only. A firmware that configures these pins differently (Stage 6/7) needs the same captures again.
- Without DCBUS the EG2104 is unpowered; this shows what the MCU drives onto the nets, which is what R-18 asks. It does not show gate-driver outputs.
- Pulses shorter than about 10 ns (two samples at 200 MSa/s) could be missed.
- No screenshots were saved.

## Run 2 — Stage 7 firmware `f2460b5` (2026-10-15)

Why again: `f2460b5` is the first firmware that can drive these nets (power stage on: PC0–PC2 to TIM1 AF2, then SD1–SD3 high). This run checks that it does not at boot, at reset, or when the arm key is written without DCBUS.

| Item | Value |
|---|---|
| Board | UID `001D0050 42535013 20363130` (read at each J-Link session) |
| Firmware | `f2460b5` (merged as `5f7cc0c`), flashed and verified (`verifybin`) |
| Power | **USB only** (J-Link power off, VTref read 3.320 V = the board's own LP5907), no DCBUS (`HW-PWR-003`) |
| Scope | Same as run 1: DS100 Mini, ×10 / X10, 1 V/div, single trigger CHA rising at 0.8 V, 20 µs/div; probe ground H2-4 |

Procedure per net (the user ran steps 1–3, Claude step 4):
1. Idle level in auto mode.
2. Power-up: USB unplugged and replugged (the board's only supply).
3. SW1 reset.
4. Debugger reset with the arm key: J-Link `r`, `g`, 3 s (boot, Stage 5 bench, offset calibration), arm key `0x41524D21` written to `foc_ctrl_cmd.arm`, held 2 s, GPIO registers read back, key cleared. Without DCBUS (VBUS about 50 mV) the guard must refuse to arm.

Positive control: probe on H2-1 (3.3 V), USB replugged: **triggered**.

| Net | Idle | Power-up | SW1 | Step 4 (reset + arm key) | Readback during step 4 |
|---|---|---|---|---|---|
| SD1 (U14-3) | 30 mV | no trigger | no trigger | no trigger | GPIOB ODR 0, GPIOA ODR 0, GPIOC MODER `0xFFFFFFFF` (PC0–PC15 analog) |
| SD2 (U15-3) | 30 mV | no trigger | no trigger | no trigger | same |
| SD3 (U16-3) | 30 mV | no trigger | no trigger | no trigger | same |
| SD4 (U17-3) | 30 mV | no trigger | no trigger | no trigger | same |
| PWMA (U14-2) | about 70 mV | no trigger | no trigger | no trigger | same |
| PWMB (U15-2) | about 70 mV | no trigger | no trigger | no trigger | same |
| PWMC (U16-2) | about 70 mV | no trigger | no trigger | no trigger | same |
| PWMD (U17-2) | about 70 mV | no trigger | no trigger | no trigger | same |

Raw files: `build/hil/stage7/` (`r18_steps.log`, `r18_step4_<net>.txt`, `uart_main.log`).

Limits of run 2:
- For SD1–SD3 the step-4 result was given by the user after SD4 was done (from memory, not one by one); PWMA–PWMD were confirmed one by one right after each run.
- The PWM nets' idle levels and power-up/SW1 results were reported together as "same as before" (run 1: about 70 mV, no trigger).
- One trial per event per net; no screenshots.
- Without DCBUS the arm key cannot arm (VBUS window), so step 4 shows the refusal, not the switch-on itself. Switching on with DCBUS changes these nets by design (Stage 7).
