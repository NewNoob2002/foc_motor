# Stage 3 — Read the ADC

Status: **all "done when" criteria pass** (runs 1–2, 2026-10-14). Moving to Stage 4 needs the user's confirmation, recorded in `AGENTS.md`.

## Setup

| Item | Value |
|---|---|
| Board | Same as stages 1–2 (UID `001D0050 42535013 20363130`) |
| Power | Logic power only from the J-Link VTref pin (3.37 V measured in stage 2); DCBUS and USB not connected; SD1–SD4 low |
| Instruments | J-Link S/N 63728769, USB-UART on H2 (`/dev/ttyUSB0`), multimeter |

## Signals (from `DOC-MAIN-SCH`, see `HW-ADC-001/002/006/011/013`)

| Signal | Pin | ADC2 channel | Front end | Expected with no DCBUS, no current |
|---|---|---|---|---|
| IA | PA6 (fast) | IN3 | R28 5 mΩ shunt → COS722MR difference amplifier, gain 10 (1 kΩ in, 10 kΩ feedback), referenced to VREF1.65 → 100 Ω + 100 pF | Mid-scale, about 2048 |
| IC | PA4 (slow) | IN17 | R29, same chain | About 2048 |
| VBUS | PC5 | IN11 | DCBUS → 100 kΩ / 4.7 kΩ → buffer → 100 Ω + 100 pF | About 0 |
| TEMP | PB15 | IN15 | +3.3 V → R38 10 kΩ → TEMP → NTC R36 → GND, no capacitor | Depends on the NTC; half-scale if the NTC is 10 kΩ |

Schematic crops: [MCU ADC pins](./stage3/mcu_adc_pins.png), [current amplifier U25](./stage3/current_amp_u25.png), [VBUS and VREF1.65, U24](./stage3/vbus_vref_u24.png), [NTC](./stage3/ntc_temp.png).

VREF1.65 is +3.3 V halved by R2/R3 (10 kΩ/10 kΩ), and the ADC reference is the same +3.3 V rail, so the zero-current code does not depend on the exact rail voltage: ideally 4095 / 2 = 2047.5.

## Conversion formulas

`code` is the 12-bit ADC result, `VDDA` the ADC reference (the 3.3 V rail; the firmware assumes 3300 mV from the devicetree `vref-mv`), full scale 4095 (RM0440).

| Quantity | Formula | Per count at 3.3 V |
|---|---|---|
| Pin voltage | `V = code × VDDA / 4095` | 0.806 mV |
| Phase current | `I = (V − V_offset) / (5 mΩ × 10) = (code − offset) × VDDA / 4095 × 20 A/V` | 16.1 mA |
| DC bus | `V_bus = V × (100 k + 4.7 k) / 4.7 k = V × 22.28` | 17.95 mV; full scale ≈ 73.5 V |
| NTC resistance | `R_ntc = 10 kΩ × code / (4095 − code)` (ratiometric, no VDDA) | — |

Code: `analog_code_x100_to_mv`, `analog_phase_current_ma`, `analog_vbus_mv`, `analog_ntc_ohm` in `src/analog.c`. All integer (hundredths of a count in, mV/mA/Ω out); the FPU is a Stage 5 topic.

Limits of the formulas:
- The sign of the current (into or out of the motor) is not confirmed (`HW-ADC-010`); Stage 7 checks it.
- VDDA is the J-Link-supplied rail, measured at 3.37 V in stage 2, not 3.30 V: absolute mV and the VBUS volts read about 2 % low. IA/IC offsets in counts and the NTC resistance are ratiometric and unaffected.
- The NTC's nominal resistance and B constant are not on the schematic, so the firmware reports ohms, not °C.

## Firmware

Source: `src/analog.{c,h}` (BSP), `src/main.c`, ADC2 in `boards/others/foc_motor/foc_motor.dts`.

| Part | Setting | Why |
|---|---|---|
| ADC2 clock | Synchronous, HCLK / 4 = 42.5 MHz | Below the 60 MHz G4 limit |
| Calibration | Done by the Zephyr driver at boot (single-ended) | Removes the ADC's own offset |
| Sampling time | IA, IC, VBUS: 47.5 cycles (1.1 µs); TEMP: 247.5 cycles (5.8 µs) | IA/IC/VBUS come from op-amp outputs behind 100 Ω; TEMP is a 10 kΩ divider with no capacitor |
| Reading | Zephyr ADC API, one blocking conversion at a time, from a thread | Stage 4 moves IA/IC to a TIM1-triggered sequence |
| Report | Every second: 1000 back-to-back samples per channel; mean, sample standard deviation, min, max, and the converted value | Estimated 20–40 ms per 1000 samples (driver overhead dominates; not measured) |
| Thread | `report_loop` on its own static 1 KiB stack, priority 7 | The log path needs about 450 B, too close to the 512 B main stack that CI pins |

Output format (placeholders, not measurements):

```
report <n>, encoder mech <count>
IA   mean <counts.xx> sd <counts.xx> min <code> max <code>
     <pin mV> mV, <+/-mA> mA vs mid-scale, sd <mA> mA
VBUS mean ... (same first line)
     <pin mV> mV, bus <mV> mV
TEMP mean ... (same first line)
     <pin mV> mV, NTC <ohm> ohm
```

Build (2026-10-14, Zephyr v4.4.2, SDK 1.0.1): flash 42,576 B, RAM 7,936 B; CI size, heap, stack and `output-low` gates pass locally; `.clang-format` clean; clang-tidy findings only inside Zephyr macros.

## Test procedure

1. Flash; read back and compare.
2. Capture the UART for a few minutes with the board untouched.
3. Record per channel: mean (offset), standard deviation and min/max (noise) across several reports.
4. Multimeter (optional cross-check): IA, IC and VREF1.65 test points against GND, and the 3.3 V rail.

## Run 1 (2026-10-14, 20:43–20:53, firmware `dc99fb1`)

Raw files in `build/hil/stage3/` (not committed: J-Link scripts and logs, `uart_run1.log`, `uart_run2.log`, flashed binaries).

| Step | Result |
|---|---|
| Board identity | UID `001D0050 42535013 20363130`, J-Link S/N 63728769, VTref 3.375 V; bench as stage 2 (J-Link 3.3 V only, no DCBUS/USB) |
| Flash | `zephyr.hex` SHA-256 `0760b403…`, `zephyr.bin` `96cd49f0…`; `verifybin` OK (43,008 B). Recovery point: the stage 2 `preflash.bin` |
| Registers after start | SD1–SD4 low (`ODR` A/B = 0), TIM1 not clocked; ADC2 `CR = 0x10000001` (regulator on, enabled), `CALFACT = 0x39` (boot calibration ran), `SMPR1/2` = 47.5 cycles on IN3/IN11/IN17 and 247.5 on IN15; PA4, PA6, PB15, PC5 analog |
| UART | 10 minutes, about 560 reports of 1000 samples per channel, board untouched |
| Bug | `VDDA 0 mV`: every mV/mA value printed 0. `adc_dt_spec.vref_mv` is only set with `zephyr,vref-mv`; fixed in `f987f8d` (take it from `adc_ref_internal()`). Raw counts and NTC ohms (ratiometric) were unaffected |

Over the 10 minutes (statistics of the per-report values):

| Channel | Offset: mean of report means | Report means range | Noise: median SD of 1000 samples | Overall min–max |
|---|---|---|---|---|
| IA | **1963.46** counts | 1963.09–1963.80 | **6.30** counts | 1937–2011 |
| IC | **1997.38** counts | 1997.07–1997.73 | **6.33** counts | 1975–2022 |
| VBUS | 0.31 counts | 0.19–0.45 | 0.46 counts | 0–2 |
| TEMP | 1924.5 counts | 1922.6–1926.2 | 0.58 counts | 1921–1928 |

## Run 2 (2026-10-14, firmware `f987f8d`)

Flash `zephyr.hex` SHA-256 `01b9df53…`, `zephyr.bin` `616bb3dc…`, `verifybin` OK; same registers (SD low, TIM1 off, `CALFACT = 0x39`). First report:

```
1000 samples per channel per report, VDDA 3300 mV assumed
IA   mean 1962.98 sd 5.92 min 1947 max 1980
     1581 mV, -1362 mA vs mid-scale, sd 95 mA
IC   mean 1997.44 sd 6.27 min 1982 max 2015
     1609 mV, -806 mA vs mid-scale, sd 101 mA
VBUS mean    0.19 sd 0.39 min    0 max    1
        0 mV, bus 0 mV
TEMP mean 1922.66 sd 0.56 min 1921 max 1925
     1549 mV, NTC 8850 ohm
```

Hand check: IA 1962.98 × 3300 / 4095 = 1581.9 mV; (1962.98 − 2047.5) × 0.806 mV × 20 A/V = −1362 mA; NTC 10 kΩ × 1922.66 / (4095 − 1922.66) = 8850 Ω. All match the firmware.

## What the numbers mean

- **Offsets are not mid-scale.** IA sits 84 counts (68 mV) and IC 50 counts (40 mV) below 2047.5. Read as current, that is −1.36 A and −0.81 A with nothing flowing, so the firmware must measure each offset at start-up (motor off) and subtract it; a fixed 2048 would be badly wrong. The likely causes are the op-amp input offset (amplified by the noise gain of 11) and the 1 % resistor mismatch in the difference amplifier; this was not separated by measurement. The offsets moved by less than ±0.4 counts over 10 minutes.
- **Noise comes from the current-sense chain, not the ADC.** VBUS and TEMP show SD 0.5–0.6 counts, IA/IC 6.3 counts (about 100 mA RMS, about 1.2 A peak to peak in the min/max). The 100 Ω + 100 pF output filters (16 MHz corner) remove almost nothing at these frequencies. Where it comes from (amplifier noise, the J-Link-supplied 3.3 V rail, pickup) was not measured. For FOC this sets a floor: one sample has about ±0.3 A of noise (3 SD); averaging or the current-loop bandwidth reduce it.
- **VBUS reads 0.3 counts** (about 5 mV at the pin): no DCBUS, as expected.
- **TEMP: NTC about 8.85 kΩ.** If it is a 10 kΩ NTC with B ≈ 3950 (an assumption, not on the schematic) that is about 28 °C; not confirmed.
- **Absolute volts are about 2 % low** because VDDA is the J-Link's 3.375 V, not 3.300 V; counts, offsets in counts and NTC ohms are unaffected.

Not done: the optional multimeter cross-check of IA, IC, VREF1.65 and the 3.3 V rail.

## Done when (roadmap)

| Criterion | Result |
|---|---|
| IA/IC offsets recorded with their noise in counts | **Pass**: IA 1963.5, IC 1997.4 counts; SD 6.3 counts each (10 minutes) |
| VBUS reads near 0 | **Pass**: 0.3 counts |
| Conversion formulas in code and in the lab note | **Pass**: `src/analog.c` and the formulas section above, hand-checked against run 2 |
