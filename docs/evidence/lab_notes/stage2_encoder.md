# Stage 2 — Read the encoder by hand

Status: **in progress** (started 2026-10-14). Done so far: connector pinouts and the harness plan, two supply measurements. Next: build and check the harness, magnet, firmware.

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
| 4 Z | U7-4 | PB9, GPIO interrupt (PB3 is kept free: it is also the SWO trace pin) |
| 1 GND | U7-6 | |
| 6 VCC3.3 | U7-1 (measured 3.3 V) | |

KTH7823 supply at 3.3 V: 3.0–3.6 V, 11.6 mA typical (`DOC-ENC-DS`, table 7), plus about 1 mA for the board LED. It needs a diametrically magnetized magnet giving 30–150 mT.

## To do

1. Build the harness; check each wire end to end and that no two adjacent wires are shorted; mark pin 1 on both connectors.
2. Magnet and mount over the shaft end.
3. Before the first flash: full 128 KiB flash backup as the recovery point.
4. Firmware: TIM3 in encoder mode, count and `theta_e` over UART at about 10 Hz, Z on PB9.
5. Done when (roadmap): 4096 counts per turn, correct sign, Z once per turn at the same count, no lost counts.
