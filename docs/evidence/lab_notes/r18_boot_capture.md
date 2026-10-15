# R-18 — SD and PWM pins during power-up and reset (oscilloscope)

Status: **procedure ready** (2026-10-14). Closes R-18 if every capture passes; R-18 is a Stage 7 entry condition (`docs/plan/risk_list.md`).

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
| Scope | ALIENTEK DS100 Mini (user manual V4.5): 2 channels, one probe used (CHA); MCX probe at ×10: 140 MHz, 10 MΩ, 15 pF; max 250 MSa/s, 128 Kpts; triggers: auto / normal / single, rising or falling edge |

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
| SD2 | not reported | ? | ? | no trigger (23:38:35) | — |
| SD3 | not reported | ? | ? | no trigger (23:39:20) | — |
| SD4 | not reported | ? | ? | no trigger (23:39:50) | — |
| PWMA | not reported | ? | ? | no trigger (23:40:14) | — |
| PWMB | not reported | ? | ? | no trigger (23:44:01) | — |
| PWMC | not reported | ? | ? | no trigger (23:44:34) | — |
| PWMD | not reported | ? | ? | no trigger (23:45:02) | — |
