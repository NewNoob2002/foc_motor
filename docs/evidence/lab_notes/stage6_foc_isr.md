# Stage 6 — The real FOC interrupt, without motor power

Status: **done** (2026-10-15). All three "done when" criteria pass: the duty pattern follows the shaft at 7 electrical turns per mechanical turn (hand-turn with the real encoder, and a debugger-written sweep), the worst ISR is 13.8 µs with 0 overruns over 7.4 minutes, and forced overruns are detected. The user confirmed Stage 6 done on 2026-10-15 (`AGENTS.md`); Stage 7 is not entered.

## What was built

| Part | Where | What it does |
|---|---|---|
| Current-loop step | `lib/foc/src/foc_current.c`, `foc/foc_current.h` | Clarke, Park, a PI per axis limited to ±vbus/√3, inverse Park, SVPWM in one call. Host test `test_current`: bit-identical to the hand-written chain on 2000 random inputs, zero error → 50 %, saturation at the bus limit, limits follow vbus, no-bus behaviour. The Stage 5 vectors now run through it, and the Stage 5 chip log still compares identically |
| Control step | `src/foc_ctrl.{c,h}` | Runs inside the ISR. First 4096 periods (205 ms): IA/IC zero-current offsets, duties 50 %. Then the selected mode. **Voltage:** v_d = 1 V of a nominal 12 V bus at the encoder's electrical angle. **Current:** the full loop with iq_ref = 1 A and placeholder gains (kp 1 V/A, ki 1000 V/(A·s)). Electrical angle = (count mod 4096) × 7 mod 4096 (7 pole pairs, `theta_offset` 0 until the Stage 7 alignment). A single-writer snapshot gives the thread the latest values |
| FOC ISR | `src/pwm_adc.c` | DMA2 ch1 transfer-complete, zero-latency. Reads [IA, IC] and TIM3, calls the control step through a function pointer (the BSP does not include `lib/foc`), converts duty to CCR (duty × 4250, clamped) and writes CCR1–3 into the preload registers |
| Update timing | `src/pwm_adc.c` | **RCR = 1**, loaded by UG before the counter starts: one update event per period, at the top (Stage 4 scope check, mode 5). The ADC samples at top k; the ISR ends by about 17 µs after it; the new duties take effect at top k + 1. Sample-to-output delay: one period, 50 µs |
| Overrun detection | `src/pwm_adc.c` | **Overrun:** ISR longer than 3400 cycles (20 µs, `AGENTS.md`). **Missed:** the next ADC result had already arrived when the ISR ended (DMA TC flag set again). Three overruns in a row, or a DMA error, latch a **fault**: MOE = 0, CCR1–3 = 50 %, no more control steps; the ISR keeps its statistics. The fault clears only by reset |
| Forced overrun | `src/pwm_adc.c` | `test_spin_cycles`, `test_spin_count`: the debugger sets them; the next `count` ISRs busy-wait until `cycles` after entry. Both 0 in normal operation |
| Telemetry | `src/main.c` | A `rot` line every 100 ms (step, mechanical count, electrical count, three duties in 1/10000) and a report every second (timing, counters, fault, CCRs, mode, offsets, i_d/i_q, v_d/v_q) |
| Tools | `stage6/check_rotation.py`, `stage6/summarize.py` | Rebuild the voltage-vector angle from the three duties and compare it with 7 × the shaft angle per row (alias-free) and as an unwrapped ratio (slow turns only); summarize the one-second reports |

The Stage 5 bench still runs at boot before the ISR starts.

The ISR call tree was checked in the disassembly: it calls only `lib/foc`, `sinf`/`cosf` (picolibc), `memset` (once per second, at a statistics reset) and register reads. `sinf`/`cosf` can reach `__math_invalidf`, which sets errno, only for an infinite or NaN angle; the angle comes from an integer count. Stack (`CONFIG_STACK_USAGE`): ISR path about 350 B + 104 B FPU frame on the 1 KiB ISR stack; report thread about 650 B + 104 B on its 1 KiB stack.

## Setup

| Item | Value |
|---|---|
| Board | UID `001D0050 42535013 20363130` (read before flashing and at each J-Link session) |
| Firmware | `71f8f19`, flashed and read back (`verifybin` OK) |
| Tools | Zephyr v4.4.2, SDK 1.0.1 (GCC 14.3.0, picolibc), `-Os`, FPU on; J-Link Commander V9.84, S/N 63728769, SWD 1 MHz |
| Power | J-Link VTref 3.3 V only; no DCBUS, no USB. SD1–SD4 low (hogs), PC0–PC3 analog; TIM1 outputs only on PA8–PA10 (unconnected) and CH3N on PB9 (U7-4, nothing attached) |
| UART | H2 → `/dev/ttyUSB0`, 921600 8N1 |
| Raw files | `build/hil/stage6/` (not committed): binary/ELF, J-Link scripts and output, `uart_main.log` (one capture, split into `boot1.log` after the flash and `boot2.log` after a reset), `sweep_c.log`, `uart_handturn.log` (hand-turn, boot 2) |

Mode and test inputs were set with J-Link writes (`foc_ctrl_mode` at 0x20000CCC, `test_spin_count` 0x20000B1C, `test_spin_cycles` 0x20000B20, TIM3 CNT 0x40000424), since the UART adapter's TX is not wired. J-Link Commander reads `w4` values as hex.

## Runs

Timeline of boot 1 (report n is about n seconds after boot):

| Reports | What ran |
|---|---|
| 1–49 | Voltage mode, shaft still at count 0 |
| 50–~350 | Voltage mode; the J-Link wrote a random TIM3 count (0–61439) every 0.5 s, 600 positions, so `sinf`/`cosf` see many angles |
| ~350–~400 | Voltage mode; two-turn sweep: TIM3 CNT written 0, 64, …, 8192 (mod 61440), 0.3 s per step |
| ~401–415 | Current mode (`foc_ctrl_mode` = 1), one more sweep |
| 416–445 | Voltage mode again |
| 446–454 | Forced overrun, first try (see below) |

Boot 2 (after a J-Link reset): the corrected forced-overrun test.

### Timing and overruns

`summarize.py boot1.log <first> <last>`:

| Reports | Mode | ISR/s | exec max (cycles) | exec max (µs) | overruns |
|---|---|---|---:|---:|---:|
| 4–366 (6 min) | voltage, random angles | 20024–20029 | 2168 | 12.8 | 0 |
| 367–401 | sweep, then the switch to current | 20024–20029 | 2344 | 13.8 | 0 |
| 402–415 | current | 20024–20027 | 2342 | 13.8 | 0 |
| 416–445 | voltage | 20024–20025 | 1853 | 10.9 | 0 |

Over 445 one-second windows (7.4 minutes) before the forced test: **worst ISR 2344 cycles = 13.8 µs, 69 % of the 20 µs budget; 0 overruns; 0 DMA errors; 0 ADC overruns.** The ISR starts 3.28–3.41 µs after the counter top (TIM1 CNT 3670–3691, counting down), so it ends by about 17 µs after the top, long before the next update at 50 µs.

ISR/s reads 20020–20029 instead of 20000 because a window is ten 100 ms sleeps plus the report work, about 1.001 s; the ISR period itself is 8485–8516 cycles (50 µs ± 0.1 µs) in every undisturbed window.

The voltage-mode ISR at a fixed angle takes about 1585 cycles; with random angles up to 2168, because `sinf`/`cosf` time depends on the angle (227–761 cycles in Stage 5). Current mode adds the two PI steps.

### Debugger artifacts

- Every window where the J-Link connected or disconnected shows a too-short "period min" (reports 4, 49, 420, 459: 597–3034 cycles) or "DWT was stopped". The J-Link pauses or stops CYCCNT; the ISR period itself does not change (ISR/s stays 20024–20029). These windows are left out of the timing figures.
- **One unexplained missed period** in report 3 of boot 1: the window in which the J-Link closed the flash session (halt, program, reset, run, close). It did not repeat on the later disconnects of boot 1, nor after a reset session ending the same way (boot 2, reports 1–20: missed 0). The likely cause is the debugger stalling the core while it closed a session that had halted it; not proven. The `missed` counter stays in the report to watch for it.

### Duty pattern vs encoder angle (debugger-written encoder count)

`check_rotation.py boot1.log 6984935 7765953` (the voltage-mode sweep):

| Check | Result |
|---|---|
| rows | 387 (4 skipped: the first current-mode steps, still labelled voltage in the 1 s report) |
| mechanical travel | +8192 counts = **2.000 turns** |
| duty-angle travel | +57344 counts = **14.000 electrical turns** |
| ratio | **7.000** (expected 7) |
| duty angle vs encoder electrical angle | max 0.27 counts (0.02 electrical degrees) |

The angle is rebuilt from the three duties alone (Clarke of the duties; the SVPWM common mode cancels), so this checks the whole path count → electrical angle → `sinf`/`cosf` → inverse Park → SVPWM → duties. It does **not** check the encoder hardware: the debugger wrote the count. In one place the count read 3777 then 3776 right after the write of 3776, so the resting shaft sits on an encoder edge (one-count flicker). That is normal for a quadrature encoder at rest.

### Hand-turn with the real encoder

Boot 2, voltage mode, no debugger attached. The user turned the shaft by hand during a 120 s capture (`uart_handturn.log`, reports 1472–1548). The user reported "CW, about 2 turns". The encoder counted 7.7 turns forward (count up = CW from the shaft end, Stage 2) and 2.9 turns back, in four bursts of 3–6 s at about 0.4–0.5 turn/s. Why this differs from the user's count was not resolved; it does not affect the check below, which is per position.

At that speed the duty pattern moves up to about 5700 electrical counts between two 10 Hz `rot` lines, more than half an electrical turn. So the unwrapped electrical/mechanical ratio aliases (the script prints 0.311 and a warning) and is not used. The alias-free check compares every row's duty angle with 7 × the logged mechanical count:

`check_rotation.py uart_handturn.log`:

| Check | Result |
|---|---|
| rows | 1200, all voltage mode |
| shaft positions | 216 distinct, covering 60 of 64 sectors (5.6° each) of the mechanical circle |
| duty angle vs 7 × mechanical count | **max 0.41 counts (0.036 electrical degrees)** |
| control: the same comparison with 6 or 8 pole pairs | max 2047 counts (the worst possible), so the data rule out other ratios |

Timing during the hand-turn: worst ISR 2079 cycles (12.2 µs), 0 overruns, 0 missed (`summarize.py uart_handturn.log`).

### Current mode

With iq_ref = 1 A and no current possible, v_q sat at **6928 mV = 12 V/√3** (the limit), and v_d ran between the limits with the noise on the d current. SVPWM reported "limited" (the vector (v_d, v_q) is longer than the 6.93 V circle). Measured i_d/i_q: about ±0.4 A of noise around 0. This matches the Stage 3 ADC noise (SD 6.3 counts ≈ 0.1 A per phase), which Clarke/Park mix. As the roadmap expected: the PIs sit at their limits.

Offsets measured at boot: IA 1962.15, IC 1995.95 counts (Stage 3: 1963.5, 1997.4).

### Forced overrun

First try (boot 1). The intended values were 4000 and 9000 cycles, but J-Link read them as hex: 0x4000 = 16384 and 0x9000 = 36864.

| Step | Written | Seen | Detected |
|---|---|---|---|
| 1 ISR | spin 0x4000 | exec 16400 | overruns 0 → 1, missed 1 → 3 (the ISR spanned two more ADC results) |
| 1 ISR | spin 0x9000 | exec 36878 | overruns → 2, missed → 4 |
| 2 in a row | spin 0x4000 | | overruns → 4, missed → 7; no fault; BDTR MOE = 1 |
| 5 in a row | spin 0x4000 | | overruns → 9, missed → 12; **fault 0x1** at the third; **BDTR 0x8000 → 0x0000 (MOE off)**; CCR1–3 = 0x84D = 2125 (50 %); ISR exec afterwards about 586 cycles (statistics only) |

Second try (boot 2, values in hex as meant):

| Step | Written | Seen | Detected |
|---|---|---|---|
| 1 ISR | spin 0xFA0 = 4000 | exec 4014 | overruns 0 → 1, missed 0, no fault |
| 2 in a row | spin 4000 | exec 4014 | overruns → 3, missed 0, no fault; BDTR MOE = 1 |

Both detectors work. Overrun is "ISR over budget"; missed is "past the next sample". Below the fault count of three in a row the outputs stay on; at three they latch off.

## Bench timing note (Stage 5 bench)

The boot bench's "full step" grew from 866–1395 to 1209–1729 cycles (all other stages unchanged). The difference is in the test wrapper `foc_vector_step()`: it now builds a `foc_current_t`, and GCC zero-fills it with a `memset` call. picolibc's `memset` in this build is a byte loop (about 4 cycles per byte, 40 bytes). The ISR path does not call `memset` per step (checked in the disassembly), and the ISR is measured directly above.

## Against the "done when"

| Criterion | Result |
|---|---|
| Duty pattern rotates as you turn the shaft, 7 electrical per mechanical turn | **Pass**: hand-turn with the real encoder, duty angle = 7 × shaft angle within 0.04° at 216 positions around the circle; debugger-written sweep: 2.000 turns → 14.000 electrical turns |
| WCET under 20 µs and zero overruns over several minutes | **Pass**: 13.8 µs max over 7.4 min, 0 overruns (one unexplained missed period at the debugger close of the flash session, see above) |
| A forced overrun is detected | **Pass**: single overruns counted without a fault; three in a row latch the fault, outputs off |

## Limits

- No current flows (no DCBUS), so the current-loop numbers show the code path and the saturation behaviour, not regulation.
- PI gains, `theta_offset`, the current-sense sign (`HW-ADC-010`) and the phase order are placeholders until Stage 7.
- The nominal 12 V bus is a constant; VBUS is not sampled in the ISR yet.
- No duty limit for the bootstrap refresh window yet (a Stage 7 board constraint).
- The outputs were not probed in this session; Stage 4 checked the PWM itself on CH3N.
