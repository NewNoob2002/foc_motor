# Stage 4 — The PWM timebase and the ADC trigger

Status: **done** (2026-10-15). All "done when" criteria pass, including the waveform checks (run on TIM1 CH3N at PB9 / U7-4); the user confirmed Stage 4 done and Stage 5 entry on 2026-10-15 (`AGENTS.md`).

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

## Waveform preparation (2026-10-14)

The user now has an oscilloscope (ALIENTEK DS100 Mini, one probe) and approved TIM1 CH1–3 on the unconnected pins PA8/PA9/PA10 (LQFP pins 41–43) for the deferred waveform checks. R-18 was captured first (`r18_boot_capture.md`, pass).

PA8–PA10 float (J-Link, firmware `1d39b8c` running): switched from analog to input, pull-up → all three read 1 in 20/20 samples; pull-down → 0 in 20/20. Nothing on the board drives them, consistent with the schematic's no-connect marks. Restored afterwards to analog, no pull (`MODER = 0xABFFFFD5`, `PUPDR = 0x64000000`); SD outputs stayed low (`ODR` A = 0).

Scope build `85267e7` / `e57ca62` (2026-10-15): CH1–3 on PA8–PA10 and an ISR marker on PB9. Read-back on the board: PC0–PC3 analog, `CCER = 0x111`, MOE on; `85267e7` left PA9/PA10 analog because `LL_GPIO_SetPinMode/Speed/Pull` act on one pin (`POSITION_VAL`), fixed in `e57ca62` (`GPIOA MODER = 0xABEAFFD5`). User measurement on PB9 (ISR marker): **20 kHz, mean high time 940 ns** (the typical ISR time; the firmware reports the maximum per second, about 2.7 µs). The user then found PA8–PA10 too fine to probe.

Route change (user-approved 2026-10-15): TIM1 CH3N (AF12) on **PB9 at the U7-4 header**, replacing the ISR marker; the test patterns move to CH3 (CH1/CH2 fixed at 25 / 50 %). CCER may hold only CC1E–CC3E and CC3NE. Modes by UART key: 1 / 2 / 3 = CH3 at 25 / 50 / 75 % (CH3N 75 / 50 / 25 %), 4 = CH3 alternating 25 / 75 % per period with RCR 0, 5 = the same with RCR 1. What the probe on CH3N shows: frequency and (inverted) duty; center alignment (with alternating 25 / 75 % pulses centered on the period, the time between successive rising edges alternates 37.5 / 62.5 µs instead of a constant 50 µs); and the update event (RCR 1: each value applies to a whole period; RCR 0: the value written after the top loads at the bottom, mid-pulse).

## Waveform results (2026-10-15, firmware `7b106d0`)

Flash `zephyr.bin` SHA-256 `39999cc3…`, `verifybin` OK. Read-back: PC0–PC3 analog, PB9 alternate function 12 (`AFRH B = 0x77C0`), PB0 (SD1) a GPIO output driven low, `CCER = 0x511` (CC1E, CC2E, CC3E, CC3NE only), MOE on, SD1–SD4 low. Probe on U7-4 (TIM1 CH3N = CH3 inverted), ground U7-6; probe switch ×10, scope X10.

The keyboard route did not work: nothing drives PB11 (H2-5, USART3 RX), which read low in 31/31 samples while 40 bytes were sent, and USART3 saw no byte or error, so the adapter's TX is not connected. Modes were set over J-Link instead: writing the firmware's `mode` variable (`0x2000099C`), and for the RCR modes the same steps as `pwm_adc_set_mode()` (stop counter, RCR, CNT = 0, UG, start). CCR3 read back the expected values each time.

| Mode | Set | Measured on U7-4 (user) | Expected |
|---|---|---|---|
| 1 | CH3 25 % | 20 kHz, +Duty 75 % | 20 kHz, 75 % |
| 2 | CH3 50 % | 20 kHz, +Duty 50 % | 50 % |
| 3 | CH3 75 % | 20 kHz, +Duty 25 % | 25 % |
| 5 | CH3 alternating 25 / 75 % per period, RCR 1 | low pulses alternating **12.5 / 37.5 µs**, rising-edge spacing **62.5 / 37.5 µs** | One update per period at the top; pulses centered on the bottom |
| 4 | Same, RCR 0 | constant **25 µs** low pulses, +Duty 50 %, edges wobbling **37.5 / 62.5 µs** | The value written about 3 µs after the top loads at the bottom, mid-pulse: each pulse is half old, half new, and not centered |

What this shows:
- **Center-aligned:** in mode 5 the narrow and wide pulses share the same center (the counter bottom); edge-aligned PWM would give a constant 50 µs edge spacing.
- **Preload and the update event:** a new CCR never takes effect at the moment it is written, only at an update event. With RCR 0 there is an update event at the bottom too, in the middle of a high-side pulse, so a duty written after the top splits that pulse (mode 4). With RCR 1, loaded by UG before the counter starts, the single update is at the top (mode 5), where all high sides are off. **Stage 6 uses RCR 1, started this way.** The cost is one PWM period (50 µs) from sample to new duty.
- The three channels share one counter, ARR and update event, so CH1/CH2 behave the same as CH3 (not probed: PA8/PA9 are too fine to probe).

## Done when (roadmap, revised 2026-10-14)

| Criterion | Result |
|---|---|
| Register read-back shows the configuration (center-aligned, ARR for 20 kHz, CCR1–3, preload, TRGO2, `CCER = 0`, `MOE = 0`, PC0–PC3 analog) | **Pass** (run 1 table) |
| 20,000 ISRs per second, period 8500 cycles, small jitter | **Pass** with the zero-latency ISR: 8,477–8,525 cycles |
| The sample is at the top of the period | **Pass**, inferred from the ISR entry time (3.19–3.41 µs after the top, matching the conversion time); the sample instant itself is not visible without a pin |
| IA/IC match the Stage 3 offsets | **Pass**: 1963.1 / 1997.2 vs 1963.5 / 1997.4 counts |
| No DMA errors or ADC overruns over several minutes | **Pass**: 0 / 0 in about 14 minutes over two runs |
| Pulses centered; a duty change taking effect only at an update event (deferred on 2026-10-14) | **Pass** (2026-10-15, oscilloscope on CH3N at U7-4): modes 5 and 4 above |
