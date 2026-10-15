# Stage 4 — The PWM timebase and the ADC trigger

Status: **in progress** (started 2026-10-14). Done so far: the gate-driver input limit found and the plan changed to an internal-only TIM1, firmware built (not flashed). Next: flash, record the register read-back and the ISR measurements.

## Why TIM1 runs with its outputs disabled

The roadmap first planned PWM on PC0–PC2 with no DCBUS. Reading the schematic and `DOC-GATE-DS` before writing the firmware showed that this would overstress the gate drivers (`HW-PWM-009`, R-19):

- PC0–PC3 (U3 pins 8–11) connect straight to the EG2104 IN pins (U14–U17 pin 2), with no series resistor or pull-down: [MCU pins](./stage4/mcu_pwm_pins.png), [U14](./stage4/u14_gate_driver_a.png).
- The EG2104 VCC is +12 V, made from DCBUS by the U9 buck; with no DCBUS it is 0 V (from the schematic; not yet measured).
- EG2104 absolute maximum (`DOC-GATE-DS` §7.1): IN from −0.3 V to **VCC + 0.3 V**, so 0.3 V when unpowered. A 3.3 V PWM high would exceed it by 3 V (SD has a fixed 6 V limit, so the SD pins are not affected).

On 2026-10-14 the user chose "registers only for now": TIM1 runs internally with `CCER = 0` and `MOE = 0`, PC0–PC3 keep their reset analog mode, and the firmware checks this at start-up and refuses to run otherwise. TIM1 CH1–3 can be moved to the unconnected pins PA8/PA9/PA10 later, with a logic analyzer and the user's go-ahead. `AGENTS.md`, the roadmap and the risk list were changed to match.

## Firmware

Source: `src/pwm_adc.{c,h}` (BSP), `src/main.c`. The devicetree keeps TIM1, ADC2 and DMA2 disabled so no Zephyr driver or pinctrl claims them; the BSP takes their addresses and clocks from the devicetree and programs them with the STM32 LL API. Design follows `docs/design/zephyr_architecture.md` §2.3.

| Part | Setting | Why |
|---|---|---|
| TIM1 | Center-aligned (CMS = 2), PSC 0, ARR 4250, ARR preload | 170 MHz / (2 × 4250) = 20 kHz |
| CH1–3 | PWM mode 1, preload on, CCR 1062 / 2125 / 3187 (25 / 50 / 75 %) | Preload: a new CCR takes effect at the next update event |
| Outputs | `CCER = 0`, `BDTR.MOE = 0`, PC0–PC3 analog; checked after setup and after start | `HW-PWM-009` |
| CH4 | Internal only: PWM mode 2, CCR4 = ARR − 1, `MMS2` = OC4REF → TRGO2 | OC4REF rises one tick (6 ns) before the counter top |
| ADC2 | BSP-owned (the Zephyr ADC driver is no longer used); HCLK/4 = 42.5 MHz, calibrated, regular sequence `[IN3 IA, IN17 IC]`, 47.5 cycles each, external trigger TIM1_TRGO2 rising, DMA circular | Low-side shunts carry current while the high sides are off, around the counter top |
| DMA2 channel 1 | ADC2 request (DMAMUX 36), circular, 2 half-words, transfer-complete and error interrupts, IRQ priority 1 | DMA1 belongs to the Zephyr DMA driver (USART3 TX). This ISR becomes the FOC ISR in Stage 6 |
| ISR | Reads DWT `CYCCNT`, TIM1 `CNT` and direction; keeps period min/max, execution max, CNT min/max, IA/IC sums; DMA error and ADC overrun counters. Single writer; the report thread copies under a sequence counter | `AGENTS.md`: every ISR measurable |
| Report | Every second from the report thread (static 1 KiB stack, priority 7) | |

VBUS and TEMP are not read in this firmware (ADC2 now belongs to the BSP); they can be added to the sequence later.

Expected values: 20,000 ISRs per second; period 8500 cycles (50 µs at 170 MHz); ISR entry a few µs after the top (trigger, two conversions of (47.5 + 12.5) / 42.5 MHz ≈ 1.4 µs each, DMA, interrupt entry), so TIM1 counting down with CNT a few hundred ticks below 4250; IA/IC near 1963 / 1997 counts (stage 3).

Build (2026-10-14, Zephyr v4.4.2, SDK 1.0.1): flash 40,272 B, RAM 7,872 B; CI size, heap, stack and `output-low` gates pass locally; `.clang-format` clean; clang-tidy findings only inside Zephyr macros. Stack use (`-fstack-usage`): ISR 32 B; report path about 550 B of 1 KiB.

## Test procedure

1. Flash; read back and compare.
2. J-Link register read-back: TIM1 `CR1`, `CR2`, `CCMR1/2`, `CCER`, `BDTR`, `ARR`, `CCR1–4`, `CNT` (twice, to see it move); ADC2 `CR`, `CFGR`, `SQR1`, `SMPR1/2`; DMA2 channel 1 `CCR`, `CNDTR`; DMAMUX channel 8; GPIOC `MODER`.
3. Capture the UART for several minutes; check ISR rate, period, jitter, execution time, ISR position, IA/IC, errors.
4. Optional: multimeter on the +12 V rail (U14 pin 1) to confirm 0 V without DCBUS.

## Done when (roadmap, revised 2026-10-14)

Register read-back shows the configuration; 20,000 ISRs per second with an 8500-cycle period and small jitter; the TIM1 count at ISR entry puts the sample at the top of the period; IA/IC match the Stage 3 offsets; no DMA errors or ADC overruns over several minutes. Deferred: the waveform checks (pulses centered, duty change only at an update event).
