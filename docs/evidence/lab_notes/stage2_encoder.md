# Stage 2 — Read the encoder by hand

Status: **all "done when" criteria pass** (run 2, 2026-10-14). Moving to Stage 3 needs the user's confirmation, recorded in `AGENTS.md`.

## Setup

| Item | Value |
|---|---|
| Board | Same as stage 1 (UID `001D0050 42535013 20363130`) |
| Encoder board | KTH7823 encoder board V1.0 (`DOC-ENC-SCH`), ABZ connector CN2 |
| Power | Logic power only, DCBUS disconnected |
| Instruments | Multimeter, J-Link S/N 63728769 |

## Connector pinouts (from the schematics)

`DOC-MAIN-SCH` (SHA-256 `e3d94ad3…`) rendered at 400 dpi; crop of H2 and U7: [`stage2/main_schematic_h2_u7.png`](./stage2/main_schematic_h2_u7.png).

| Pin | Main board U7 | Encoder CN2 | Main board H2 (SWD/UART) |
|---|---|---|---|
| 1 | VCC (net only on U7-1 and TP1; TP5 = +3.3V and TP4 = +5V next to it) | GND | +3.3V |
| 2 | SPI1_MISO / PB4 (TIM3_CH1) | B | SWDIO |
| 3 | SPI1_SCK / PB3 | not connected | SWCLK |
| 4 | SPI1_CS / PB9 | Z | DGND |
| 5 | SPI1_MOSI / PB5 (TIM3_CH2) | A | USART3_RX |
| 6 | DGND | VCC3.3 | USART3_TX |

Plugging CN2 into U7 directly would put VCC against GND. The schematic annotates PB4 as TIM3_CH1 and PB5 as TIM3_CH2.

## Measurements (multimeter, by the user, 2026-10-14)

| Point | Reading | Conclusion |
|---|---|---|
| U7-1 to GND, board powered | 3.35 V | VCC is on 3.3 V on this board (how TP1 is linked was not inspected) |
| H2-1 to GND, board powered | 3.34 V | H2-1 is +3.3V, as the schematic shows; the old `HW-UART-003` text ("H2-1 = DGND") was wrong and is corrected |

## Harness plan

| Encoder CN2 | Main board | MCU |
|---|---|---|
| 5 A | U7-2 | PB4 = TIM3_CH1 |
| 2 B | U7-5 | PB5 = TIM3_CH2 |
| 4 Z | U7-4 (plan) → **U7-3 as built** | Plan: PB9. As built: PB3, GPIO interrupt (firmware moved to PB3 in `6adcd13`; PB3 is also TRACESWO, unused here) |
| 1 GND | U7-6 | |
| 6 VCC3.3 | U7-1 (measured 3.3 V) | |

KTH7823 supply at 3.3 V: 3.0–3.6 V, 11.6 mA typical (`DOC-ENC-DS`, table 7), plus about 1 mA for the board LED. It needs a diametrically magnetized magnet giving 30–150 mT.

Harness: built and checked end to end by the user, 2026-10-14 (reported; the check method and readings were not recorded).

## Firmware

Source: `src/encoder.c` (BSP), `src/main.c`, TIM3/Z in `boards/others/foc_motor/foc_motor.dts`.

| Part | Setting | Why |
|---|---|---|
| TIM3 | Zephyr `st,stm32-qdec` driver, encoder mode 3 (x4, `SMS = 0b011`), input filter level 3 | x4 counts every A and B edge: 1024 periods → 4096 counts per turn. Filter: 8 samples at 170 MHz = 47 ns |
| TIM3 wrap | The driver sets `ARR = 61439`, so the count wraps at 61440 = 15 × 4096 | `count % 4096` is the angle within a turn, with no wrap tracking |
| PB4/PB5 | AF2, pull-up | KTH7823 holds A/B/Z high at power-up; the pull-ups keep unplugged inputs still. PB4 is NJTRST after reset; only SWD is used, so reassigning it is fine |
| PB3 (Z) | GPIO input, pull-up, interrupt on the rising edge (EXTI 3, priority 5); the ISR latches the TIM3 count. Run 1 used PB9 | The Z rising edge marks zero once per turn (`DOC-ENC-DS` §7.3) |
| Output | USART3 921600 on H2, one line every 100 ms; one extra line per Z | |

`theta_e` is computed in integers, `(count % 4096) × 7 % 4096 × 360 / 4096`, with no offset yet (alignment is Stage 7). The firmware does not use `lib/foc` (that is Stage 5) and does not touch TIM1 or the SD pins; the build-time checks for SD1–SD4 low and TIM1 disabled are unchanged.

Example lines:

```
cnt 12345 pos   +123 turn   +0 mech   57 theta_e  35.0 deg
Z #3 (+1) mech  812 pos  +4908, +4096 since last Z
```

Build (2026-10-14, Zephyr v4.4.2, SDK 1.0.1, `west build -p always -b foc_motor -d build/arm . -- -DBUILD_VERSION=v4.4.2`): flash 37,940 B, RAM 6,592 B; the CI size, heap, stack and `output-low` checks pass locally. `zephyr.bin` SHA-256 `5088e961…` and `zephyr.hex` SHA-256 `817117fa…` from the uncommitted tree; record the commit and hashes again for the binary that gets flashed.

## Test procedure

0. Diametrically magnetized magnet on the shaft end, centered over the KTH7823 (30–150 mT, see above).
1. Back up the full 128 KiB flash with the J-Link (recovery point); record the SHA-256.
2. Flash `zephyr.hex`; read back and compare.
3. USB-UART on H2 (RX → H2-6, TX → H2-5, GND → H2-4), 921600 8N1; log to a file.
4. Turn the shaft slowly by hand, checking each item:
   - Mark a start position; one full turn changes `pos` by 4096 (±a few counts for where you stop).
   - Turn one way, then the other: the sign of the change follows the direction. Note which direction (viewed from the shaft end) counts up. The datasheet says A leads B for CW rotation viewed from above the chip, and A is TIM3_CH1, so CW over the chip should count up.
   - Z: one Z line per turn, always the same `mech` count in the same direction. In the other direction the rising edge comes from the other side of the Z pulse, so `mech` can differ by the Z width; that is expected.
   - After 10+ turns back and forth, return to the start mark: `mech` is where it started and the Z `mech` has not drifted.
   - `theta_e` goes around 7 times per turn.

## Run 1 (2026-10-14, 18:52–19:05)

Raw files in `build/hil/stage2/flash/` (not committed: `preflash.bin`, `postflash.bin`, `uart_run1.log`, J-Link scripts and logs).

| Step | Result |
|---|---|
| Board identity | UID `001D0050 42535013 20363130`, J-Link S/N 63728769, VTref 3.367 V; bench as stage 1 (J-Link 3.3 V only, no DCBUS/USB) |
| Backup | `preflash.bin`, 128 KiB, SHA-256 `522685edcce6fc7e4e46740dd3c2be7b106ad53aa30e75baed40c39fa70dba21`. The first 36,916 B match the UART3 `zephyr.bin` (`d1551241…`); the rest still holds 51,372 non-erased bytes from older firmware |
| Flash | Commit `44124b6`, `zephyr.hex` SHA-256 `817117fa…`, `zephyr.bin` `5088e961…`; J-Link `loadfile` + `verifybin` OK (38,912 B programmed) |
| Registers after start | SD1–SD4 outputs low (`ODR` A/B = 0); TIM1 not clocked (`APB2ENR = 1`); TIM3 `CR1 = 0x11`, `SMCR = 3`, `ARR = 0xEFFF`; PB4/PB5 AF, PB9 input |
| UART | Boots and prints at 10 Hz. At rest the count dithers by one count across an edge (`cnt 0` ↔ `61439`, shown as `pos -1 turn -1 mech 4095`: the wrap handling works) |
| Turning (free-form, the user turned and stopped several times) | `pos` followed the shaft from 0 to +30,147 (7.4 turns net) with steps back; the one-count dither at rest is the only noise seen |
| Z | **0 events** in the whole run (`z_events` read over J-Link = 0, EXTI 9 → port B, rising edge, NVIC enabled). PB9 reads high in 400/400 fast samples at rest at `mech` 1475; with the PB9 pull-up switched to pull-down it reads low 50/50. **Nothing drives PB9: the Z line is open** between KTH7823 U1-3 → CN2-4 and U7-4 |

Not yet checked: 4096 counts per turn and the direction sign (both need a marked turn or a working Z).

Earlier J-Link GPIO sampling (`build/hil/stage2/*.jlink`, 04:12–04:21, before this firmware) saw PB9 toggle, so Z may have worked then; it is not driven now.

Cause, found by the user: the harness takes Z to U7-3 (PB3), not U7-4 (PB9). With J-Link, PB3 read low with no pull, pull-up and pull-down, so it is driven. Firmware moved Z to PB3 (`6adcd13`).

## Run 2 (2026-10-14, 19:15–19:20)

| Step | Result |
|---|---|
| Flash | Commit `6adcd13`, `zephyr.hex` SHA-256 `10392f58…`, `zephyr.bin` `533f26cc…`; `verifybin` OK. Same board, J-Link, bench as run 1 |
| Registers after start | SD1–SD4 low, TIM1 not clocked (unchanged); PB3 input, `SYSCFG_EXTICR1 = 0x1000` (EXTI3 → port B), `EXTI_IMR1`/`RTSR1` bit 3 set, NVIC IRQ 9 enabled |
| Turning | User: 5 turns clockwise, then counter-clockwise, viewed from the shaft end; magnet on the shaft end above the chip. Peak hand speed about 70 rpm (478 counts per 100 ms). UART log `build/hil/stage2/flash/uart_run2.log`, 2,200 count lines, no errors or warnings |

Z lines from the log:

```
Z #1  (+1) mech 1008 pos   +1008, +0 since last Z
Z #2  (+1) mech 1008 pos   +5104, +4096 since last Z
Z #3  (+1) mech 1008 pos   +9200, +4096 since last Z
Z #4  (+1) mech 1008 pos  +13296, +4096 since last Z
Z #5  (+1) mech 1008 pos  +17392, +4096 since last Z
Z #6  (+1) mech 1008 pos  +17392, +0 since last Z
Z #7  (+1) mech 1008 pos  +13296, -4096 since last Z
Z #8  (+1) mech 1008 pos   +9200, -4096 since last Z
Z #9  (+1) mech 1008 pos   +5104, -4096 since last Z
Z #10 (+1) mech 1008 pos   +1008, -4096 since last Z
Z #11 (+1) mech 1008 pos   -3088, -4096 since last Z
```

Z #6 is the shaft passing zero again just after the turn-around. The rising edge comes at the same count (1008) in both directions, so the Z pulse is narrower than one count at this speed; the width difference expected in the procedure did not show up.

## Done when (roadmap)

| Criterion | Result |
|---|---|
| One turn = 4096 counts | **Pass**: all 9 same-direction Z-to-Z intervals are exactly ±4096 |
| Sign matches the direction | **Pass**: clockwise viewed from the shaft end counts up, counter-clockwise counts down. This matches the datasheet (CW viewed from above the chip → A leads B) with A on TIM3_CH1 |
| Z once per turn at the same count | **Pass**: 11 Z edges, one per zero crossing, all at `mech` 1008 |
| No lost counts after many turns back and forth | **Pass**: after 5 turns up and 5 down, Z #10 lands on the same `pos` (+1008) as Z #1; 10+ turns at up to 70 rpm |

Limits: hand speed only (up to 70 rpm); the count at motor speed (A/B edges every few µs) is checked again in Stage 6/7. The Z ISR runs at normal priority with interrupt latency; at hand speed that is far below one count. `theta_e` is arithmetic on the count (spot check: `mech` 1014 → 263.8°, as computed by hand); it has no offset until the Stage 7 alignment.
