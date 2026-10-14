# Stage 1 — Boot-safety check with the J-Link

Status: **partly done** (2026-10-14). The J-Link checks below pass. Open: the multimeter reset check and the power-source confirmation (see "Open items"). R-18 stays open by design; its waveform capture is a stage 7 entry condition.

## Setup

| Item | Value |
|---|---|
| Date | 2026-10-14 |
| Board | Main board V2.0; STM32G431RB UID `001D0050 42535013 20363130`, DBGMCU ID `0x20036468`, 128 KiB flash — same board as M1 |
| Firmware on the board | Not reflashed. Flash `0x0800_0000`, 36,916 B, SHA-256 `d1551241a7bdd8b7d8a010738ed7e2bda9405782d589e8a92c54d285cf143db3` = the UART3 `zephyr.bin` recorded in [uart3_validation.md](../uart3_validation.md) |
| Debug probe | SEGGER J-Link, S/N 63728769, hardware V9.70, firmware compiled 2021-05-07; SWD at 1 MHz |
| Tools | J-Link Commander and DLL V9.84 |
| Power | Reported by the user: board powered from the J-Link 3.3 V supply. VTref measured 3.374 V. DCBUS: to be confirmed (open item) |
| Wiring | J-Link SWD to the board's SWD header. No NRST line on the header |
| Instruments | None besides the J-Link (no scope or logic analyzer) |

## What was run

All scripts and raw logs are in [`stage1/`](./stage1/); `parse_run.py` turns a loop log into a table.

1. **Read-only pass** (`readonly.jlink`, core kept running: DHCSR `S_HALT = 0` before and after): ID registers, RCC clock enables, GPIOA/B/C and TIM1 registers, flash read-back.
2. **Run A, software reset** (`runA.jlink`): J-Link "Normal" reset, which sets `DEMCR.VC_CORERESET` and writes `AIRCR.SYSRESETREQ`; register snapshot at the first instruction; one data watchpoint on **every CPU write to `0x4800_0000`–`0x4800_07FF`** (all GPIOA and GPIOB registers, address mask `0x7FF`); then 80 × (go, wait 300 ms, record the stop reason, PC, GPIOA/B `MODER` and `ODR`).
3. **Run C, button reset** (`runC_button.jlink`): the same watchpoint plus vector catch; the user pressed the reset button (SW1); then 60 loop iterations.
4. **Final state** (`final_state.jlink`): read-only check after the runs.

## Results

| Check | Run A (SYSRESETREQ) | Run C (reset button) |
|---|---|---|
| Stop after reset | Vector catch, PC `0x0800_1AC8` (reset handler) | Same |
| Pins at the first instruction | `MODER` A `ABFFFFFF`, B `FFFFFEBF`, C `FFFFFFFF` (reset values: all motor pins analog, not driven); GPIO clocks off (`AHB2ENR = 0`) | Same |
| CPU writes to GPIOA/GPIOB during boot | 30 | 30, **identical** sequence (same PCs and register values) |
| Any write leaving an SD pin as an output driving high | **None** (`ODR` A and B = `0x00000000` after every write) | **None** |
| Writes after boot | None in 50 × 300 ms of running | None in 30 × 300 ms |

The 30 writes are 5 per pin, the last of each group setting `MODER`: PB10, PB11 (USART3 alternate function), then PA0 (SD4), PA1 (SD2), PA2 (SD3), PB0 (SD1) as outputs. The output data registers are 0 throughout, so each SD pin goes from analog (held low by the EG2104 pull-down) straight to driving low.

Running state (read-only pass and final state):

| Signal | Registers | Meaning |
|---|---|---|
| SD1–SD4 (PB0, PA1, PA2, PA0) | `MODER` A `ABFFFFD5`, B `FFAFFEBD`; `ODR` 0; `IDR` A `0x0000C000`, B `0x00000400` | Outputs, driving low, pins read low (the set bits are PA14/PA15 debug and PB10 UART TX idle) |
| PC0–PC3 (PWM A–D) | `MODER` C `FFFFFFFF` | Analog, not connected to TIM1 |
| TIM1 | `RCC_APB2ENR = 0x1` (TIM1EN = 0); `CR1`, `CCER`, `BDTR` read 0 | Timer not clocked: no PWM possible |
| Core after the runs | DHCSR `0x00050001`, DWT `FUNCTION3 = 0` | Firmware running, watchpoint removed |

### Pass / fail against "done when"

| Criterion | Result |
|---|---|
| Every recorded write keeps SD1–SD4 low | **Pass** (both resets) |
| At run time the SD pins are outputs driving low and `IDR` reads low | **Pass** |
| TIM1 stopped with main output off | **Pass** (not even clocked) |
| Multimeter about 0 V on SD1–SD4 during reset | **Open** |
| Lab note written | This file |

## Limits

- The watchpoint sees CPU writes only, not DMA writes. The only DMA in this firmware feeds USART3.
- No microsecond glitches, no power-up ramp, no pin voltages: that is what the R-18 capture before stage 7 is for.
- Power-on reset was not covered: it resets the debug logic, and the SWD header has no NRST line to hold the MCU in reset while connecting. Power-on runs the same boot code as the two resets above.
- PCs are not symbolized: the matching ELF of the UART3 build is no longer on this machine.

## Surprises and lessons

- J-Link's "Normal" reset already is a SYSRESETREQ reset, so run A covered the software reset; a separate run B was not needed.
- Writing 0 to the DWT `FUNCTION` registers does not remove a J-Link watchpoint: J-Link re-arms it on `go`, and the core stopped once more after the final reset. Use `ClearWP <handle>` before resuming (done in run C). J-Link removed the watchpoint on disconnect and the firmware ran normally, as the final-state read shows.

## Open items

1. Multimeter: hold the reset button, measure SD1–SD4 (PB0, PA1, PA2, PA0) to GND; expect about 0 V.
2. Confirm the power source (which J-Link pin feeds the board) and that DCBUS was disconnected.

## Raw evidence (SHA-256)

| File | SHA-256 |
|---|---|
| `stage1/readonly.log` | `e1a69c45b972ac9c8e36b1e709a43044fb9cb112b175ad5f8b3b4a1a54ef8e33` |
| `stage1/runA.log` | `7c7d37b13d09647b8faa5eedd5b4b3bd02ee92381fecc9cc6e615e5bc18c5fc8` |
| `stage1/runC_button.log` | `3291931bfc739117fe9106ef34e6fccf3847821052e062c1c4d9b1b52d30884a` |
| `stage1/final_state.log` | `d38d0c5c2ea9fdef18890401ebbc43c4f0c6940c591f5703f9737ef844436923` |
