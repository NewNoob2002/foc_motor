# Stage 5 — The FOC math on the chip

Status: **done** (2026-10-15). Both "done when" criteria pass: the chip's results match the host within float tolerance on all 64 vectors, and one full current-loop step takes at most 1395 cycles (8.2 µs, 41 % of the 3400-cycle / 20 µs budget). The user confirmed Stage 5 done and Stage 6 entry on 2026-10-15 (`AGENTS.md`).

## What was built

| Part | Where | What it does |
|---|---|---|
| Test vectors | `tests/foc_vectors/foc_vectors.{c,h}` | 64 fixed inputs (ia, ic, theta, id_ref, iq_ref, vbus, both PI integrals) and `foc_vector_step()`: b = −(a + c), Clarke, `sinf`/`cosf`, Park, a PI per axis limited to ±vbus/√3, inverse Park, SVPWM. Rows 0–7 are chosen cases (all zero, pure d current, saturation, SVPWM limit, integral clamp, theta near 0/π/2π); rows 8–63 are fixed random draws. The inputs are decimal literals, so both compilers produce the same float bits |
| Firmware bench | `src/foc_bench.{c,h}` | Times each stage with the DWT cycle counter, then prints the results of all 64 vectors as hex float bits (`foc_vec` lines) |
| Comparator | `tests/host/foc/vec_compare.c` | Reads a UART log, recomputes the same vectors on the host, prints the largest error per output field. `vec_compare --selftest` (CTest) checks the parser and hand-computed rows 0, 1, 3, 4, 5 |
| Cycle counter | `src/cycles.h` (BSP) | DWT start/read, moved out of `pwm_adc.c` so the bench and the ISR share it |
| FPU | `prj.conf`: `CONFIG_FPU=y` | Hard-float ABI; Zephyr selects `CONFIG_FPU_SHARING` with it, so threads (and later the ISR) can all use the FPU |

The bench runs at boot in the report thread, before `pwm_adc_init()`, so the zero-latency TIM1/ADC ISR (which `irq_lock()` cannot mask) is not running yet. After the bench the firmware continues exactly as in Stage 4.

### How the timing is taken

- Each stage is a `__noinline` function writing its result to a static context; the call sits between two `CYCCNT` reads with compiler memory barriers, so the compiler cannot move the work outside the window.
- The stages run in data-flow order for each vector, so each stage sees real intermediate values (e.g. the PI saturates where the vector says it should).
- Interrupts are locked for one vector (all stages), 64 vectors × 8 passes. "max(cold)" is pass 0, "max(warm)" passes 1–7.
- Every figure includes the call overhead of one timed call ("empty call", 29–39 cycles). The "full step" is one call to `foc_vector_step()`, which itself calls the lib/foc functions: this is the figure for the budget.
- `sin+cos double` is `sin()` + `cos()` on `(double)theta`, to show the cost of double precision on a single-precision FPU.

## Setup

| Item | Value |
|---|---|
| Board | UID `001D0050 42535013 20363130` (read over SWD before flashing) |
| Firmware | `99bbf2f` (FPU on), flashed and read back (`verifybin` OK). The next commit adds only a firmware warning for a stopped DWT counter and a log-parser fix; it was not flashed |
| Tools | Zephyr v4.4.2, Zephyr SDK 1.0.1 (arm-zephyr-eabi-gcc 14.3.0, picolibc), `-Os`; J-Link Commander V9.84, J-Link S/N 63728769, SWD 1 MHz |
| Power | J-Link VTref 3.3 V only; no DCBUS, no USB. SD1–SD4 low (hogs), PC0–PC3 analog |
| UART | H2 → `/dev/ttyUSB0`, 921600 8N1 |
| Raw files | `build/hil/stage5/` (not committed): flashed binaries and ELFs, J-Link scripts and output, `uart_fpu_on.log` (run 1), `uart_fpu_on_run2.log` (run 2) |

Before running the bench, the user chose to flash the FPU-on build only. The soft-float comparison build (`-DCONFIG_FPU=n`, `build/hil/stage5/fpu_off_99bbf2f.bin`) was built but not run.

## Run 1 — flash, reset, J-Link disconnect right away

The math results were complete, but every stage's **minimum was 0 cycles**. The J-Link script ends with `g` then `qc`. As seen in Stage 4, a J-Link disconnect clears TRCENA/CYCCNTENA. The bench starts at boot and takes 26 ms, so the counter stopped partway through: after that, every reading was 0. The maximum figures in this run happen to equal run 2's, which fits the counter running only during the early part of the bench. The cycle figures of run 1 are **invalid**; its numeric results are valid.

Fix: firmware now warns when the counter was stopped during the measurement (commit after `99bbf2f`); the procedure keeps the J-Link connected until the bench is done.

## Run 2 — reset with the J-Link held for 3 s

J-Link script: `r`, `g`, `Sleep 3000`, read `CYCCNT` (0x1E626387 = 509,764,487 cycles = 3.0 s at 170 MHz: running), `qc`.

| Stage | min | max (warm) | max (cold) | cycles (µs at 170 MHz for the max) |
|---|---:|---:|---:|---|
| empty call | 29 | 39 | 39 | call overhead, included in every row |
| sinf + cosf | 227 | 761 | 760 | 4.5 µs |
| Clarke (with b = −(a + c)) | 104 | 117 | 116 | |
| Park | 79 | 84 | 85 | |
| 2 × PI | 258 | 269 | 270 | |
| inverse Park | 72 | 72 | 73 | |
| SVPWM | 279 | 292 | 291 | |
| **full step** | **866** | **1395** | **1395** | **8.2 µs** |
| sin + cos in double | 451 | 6550 | 6553 | 38.5 µs |

Firmware verdict line: `full step max 1395 cycles = 8.20 us; budget 3400 cycles (20 us): under`.

Numeric comparison, `vec_compare uart_fpu_on_run2.log` (run 1 gives identical output):

| Field | max \|error\| | at vector | not bit-exact (of 64) | tolerance |
|---|---:|---:|---:|---:|
| i_d (A) | 9.54e-7 | 46 | 9 | 1e-4 |
| i_q (A) | 9.54e-7 | 13 | 10 | 1e-4 |
| v_d (V) | 1.91e-6 | 60 | 5 | 1e-4 |
| v_q (V) | 8.34e-7 | 10 | 1 | 1e-4 |
| int_d (V) | 4.77e-7 | 11 | 4 | 1e-4 |
| int_q (V) | 2.38e-7 | 24 | 5 | 1e-4 |
| duty a | 5.96e-8 | 24 | 6 | 1e-5 |
| duty b | 5.96e-8 | 10 | 5 | 1e-5 |
| duty c | 1.19e-7 | 28 | 6 | 1e-5 |

64 of 64 vectors found; the SVPWM limit flag matches on every vector. **PASS.**

## What the numbers say

- **Budget.** The worst step is 1395 cycles, 41 % of 3400. The Stage 4 ISR body (ADC read, statistics) took up to about 490 cycles; together that is about 1900 cycles, still well under 20 µs. Stage 6 measures the real ISR.
- **sin/cos is the biggest piece.** `sinf` + `cosf` is 227–761 cycles, more than half of the worst step, and it varies with the angle. Presumably picolibc takes a slower range-reduction path for larger angles (not checked). If Stage 6 needs the time, the G431 CORDIC unit or a lookup table can replace it; `lib/foc` takes sin/cos as inputs for exactly this reason.
- **Why `double` is slow.** The Cortex-M4F FPU is single-precision only. `sin()` + `cos()` in double run in software: up to 6553 cycles, 8.6 times `sinf` + `cosf`, and almost twice the whole 20 µs budget on its own. `lib/foc` and the vectors build with `-Wdouble-promotion` (host and ARM), so an accidental `double` (for example `0.5` instead of `0.5f`) is a warning, and an error under `-Werror`.
- **Float agreement.** The inputs are bit-identical, and the largest differences are 1–2 float ULPs (one ULP is 9.5e-7 for values between 8 and 16). Possible sources: libm `sinf`/`cosf` (glibc on the host, picolibc on the chip) and instruction selection (the ARM build uses `VMLA` multiply-accumulate). Which one it is was not checked. The tolerances (1e-4 A/V, 1e-5 duty = 0.04 TIM1 counts) are far above this and far below anything the board can resolve.
- **Cold vs warm.** Pass 0 gives the same maxima as later passes. Only pass 0's first vector runs with truly cold caches, so this is not a cold-cache worst case. In the ISR other code runs between steps; Stage 6 measures that directly.

## Other checks

- Host: `ctest` 3/3 pass (`foc_host_smoke`, `test_transform`, `vec_compare_selftest`); with `-DFOC_EXERCISES=ON` all exercise tests pass too.
- ARM build: flash 62,092 B, RAM 8,960 B (text+data 62,088 ≤ 98,304; data+bss 8,875 ≤ 24,576); no malloc symbols; four `output-low` hogs; stack sizes as CI pins them. `.clang-format` clean; clang-tidy: no new findings (the remaining ones are inside Zephyr macros).
- Stack (`CONFIG_STACK_USAGE`): report thread path through the bench is about 600 B of its 1 KiB stack: `report_loop` 152 + `foc_bench_run` 184 + `print_vector` 152 + logging. Add 104 B if an interrupt arrives while the FPU context is active (extended exception frame). It ran without a stack fault (`HW_STACK_PROTECTION` on).

## Limits

- Timing with interrupts locked and no ISR running; the zero-latency ISR in Stage 6 will preempt threads, but the step itself runs inside it.
- Code runs from flash through the ART accelerator. Running the step from RAM (CCM SRAM) is an option if Stage 6 shows cache misses.
- The soft-float build was not run (user's choice); the double-precision row shows the software-float cost instead.
- One board; two runs of the same firmware.
