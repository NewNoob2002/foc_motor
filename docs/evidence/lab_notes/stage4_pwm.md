# Stage 4 — The PWM timebase and the ADC trigger

Status: **all "done when" criteria pass** (runs 1–2, 2026-10-14) except the waveform checks, which are deferred by design. Moving to Stage 5 needs the user's confirmation, recorded in `AGENTS.md`.

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
| DMA2 channel 1 | ADC2 request (DMAMUX 36), circular, 2 half-words, transfer-complete and error interrupts; run 1: ordinary IRQ priority 1; from `1d39b8c`: zero-latency direct ISR (`IRQ_ZERO_LATENCY`, hardware priority 0) | DMA1 belongs to the Zephyr DMA driver (USART3 TX). This ISR becomes the FOC ISR in Stage 6 |
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

## Run 1 (2026-10-14, firmware `c593a5b`, ordinary IRQ)

Raw files in `build/hil/stage4/` (not committed: J-Link scripts and logs, UART logs, flashed binaries). `stage4/summarize.py` (committed) turns a UART log into the figures below.

| Step | Result |
|---|---|
| Board, bench | UID `001D0050 42535013 20363130`, J-Link S/N 63728769, VTref 3.374 V; J-Link 3.3 V only, no DCBUS/USB. **User measured U14 pin 1 (EG2104 VCC) = 0 V**, confirming the gate drivers are unpowered without DCBUS |
| Flash | `zephyr.hex` SHA-256 `87c71b24…`, `zephyr.bin` `e3c2aa45…`; `verifybin` OK (40,960 B) |
| Outputs | GPIOC `MODER = 0xFFFFFFFF` (PC0–PC3 analog), `ODR` C = 0; TIM1 `CCER = 0`, `BDTR = 0` (MOE = 0); SD1–SD4 low (`ODR` A/B = 0) |
| TIM1 | `CR1 = 0xD1` (CEN, CMS = 2, ARPE); `CR2 = 0x00700000` (MMS2 = OC4REF); `CCMR1 = 0x6868`, `CCMR2 = 0x7868` (CH1–3 PWM1 + preload, CH4 PWM2 + preload); `ARR = 4250`; `CCR1–4 = 1062 / 2125 / 3187 / 4249`; `CNT` read 7, 388, 2623 (running); `APB2ENR.TIM1EN = 1` |
| ADC2 | `CR = 0x10000005` (regulator, ADEN, ADSTART); `CFGR = 0x80000543` (DMA circular, 12-bit, EXTSEL 10 = TIM1_TRGO2, rising); `SQR1 = 0x000110C1` (2 ranks: IN3, IN17); `SMPR1/2` 47.5 cycles on IN3/IN17; `CALFACT = 0x39`; common `CCR` CKMODE = HCLK/4 |
| DMA | DMA2 channel 1 `CCR = 0x35AB` (enabled, TC + TE interrupts, circular, half-words, very high priority), `CNDTR = 2`, `CPAR` = ADC2 `DR`; DMAMUX channel 8 = request 36 (ADC2); NVIC IRQ 56 enabled |

UART, about 7 minutes (416 reports; reports 1–3 overlap the J-Link session and are left out):

| Figure | Result |
|---|---|
| ISRs per second | 20,001–20,004 |
| ISR entry after the counter top | **3.26 to 13.61 µs**; worst case per report typically 7.65 µs |
| IA / IC mean | 1963.04 / 1997.13 counts (stage 3: 1963.46 / 1997.38) |
| DMA errors, ADC overruns | 0, 0 |

Two findings:
1. **Entry latency jitter of about 10 µs.** The trigger is hardware (TIM1 CH4), so the earliest entry (3.26 µs = trigger + two conversions + DMA) is fixed; the rest is the CPU entering late while kernel critical sections mask ordinary interrupts (logging, UART, scheduler). For a 50 µs FOC period with a 20 µs budget this is too much, and it is why the architecture makes this ISR zero-latency.
2. **The DWT cycle counter stopped when the J-Link disconnected** (period and execution time read 0 from report 4 on). Fixed in `1d39b8c`: the report thread restarts it and marks that window's cycle figures invalid.

## Run 2 (2026-10-14, firmware `1d39b8c`, zero-latency ISR)

Flash `zephyr.hex` SHA-256 `34f5843b…`, `zephyr.bin` `81d59288…`, `verifybin` OK; outputs still off (PC0–PC3 analog, `CCER = 0`, `BDTR = 0`, SD low); NVIC priority byte of IRQ 56 = `0x00` (zero-latency), neighbours `0x20`; the vector table entry for IRQ 56 points straight at `dma_isr`.

UART, about 7 minutes (413 reports after leaving out reports 1–3):

| Figure | Run 1 (ordinary IRQ) | Run 2 (zero-latency) |
|---|---|---|
| ISRs per second | 20,001–20,004 | 20,001–20,006 |
| Period between ISR entries | 8,255–9,939 cycles (first 2 s only) | **8,477–8,525 cycles** (8,500 ± 0.15 µs) |
| ISR entry after the counter top | 3.26–13.61 µs | **3.19–3.41 µs** |
| ISR execution time, max | 494 cycles | 457 cycles (2.7 µs) |
| IA / IC mean | 1963.04 / 1997.13 | 1963.08 / 1997.16 |
| DMA errors, ADC overruns | 0, 0 | 0, 0 |

Report 2 flagged the stopped DWT after the J-Link disconnect and restarted it; report 3's minimum period of 4,265 cycles is the one interval that spans the stop (a known artifact after a debugger disconnect, not ISR behaviour).

Where the sample is: the trigger fires one TIM1 tick before the counter top; IA is sampled during the first 47.5 ADC cycles (1.1 µs) and IC 1.4 µs later; the ISR enters about 3.2 µs after the top, which matches trigger + 2 × (47.5 + 12.5) cycles at 42.5 MHz (2.8 µs) + DMA + interrupt entry. So the samples are taken at the top of the period, where the low sides would conduct. The IA–IC time skew (1.4 µs) is part of the phase-error budget for later.

## Done when (roadmap, revised 2026-10-14)

| Criterion | Result |
|---|---|
| Register read-back shows the configuration (center-aligned, ARR for 20 kHz, CCR1–3, preload, TRGO2, `CCER = 0`, `MOE = 0`, PC0–PC3 analog) | **Pass** (run 1 table) |
| 20,000 ISRs per second, period 8500 cycles, small jitter | **Pass** with the zero-latency ISR: 8,477–8,525 cycles |
| The sample is at the top of the period | **Pass**, inferred from the ISR entry time (3.19–3.41 µs after the top, matching the conversion time); the sample instant itself is not visible without a pin |
| IA/IC match the Stage 3 offsets | **Pass**: 1963.1 / 1997.2 vs 1963.5 / 1997.4 counts |
| No DMA errors or ADC overruns over several minutes | **Pass**: 0 / 0 in about 14 minutes over two runs |
| Deferred: pulses centered, a duty change taking effect only at an update event | **Not done** (needs TIM1 on PA8–PA10 and a logic analyzer, with the user's go-ahead). Preload is configured (`OCxPE = 1`, `ARPE = 1`), which is what makes CCR changes wait for the update event (RM0440), but this is not observed |
