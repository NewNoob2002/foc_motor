# Stage 3 — Read the ADC

Status: **in progress** (started 2026-10-14). Done so far: analog front ends read from the schematic, conversion formulas, firmware built (not flashed). Next: flash, record the readings.

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

## Done when (roadmap)

IA/IC offsets recorded (expect near 2048 counts) with their noise in counts; VBUS reads near 0; the conversion formulas are in code and in this note.
