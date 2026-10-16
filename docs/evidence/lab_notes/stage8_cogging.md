# Stage 8 — Cogging feed-forward

Status: in progress, started 2026-10-15 on the user's go (`AGENTS.md`). Software design and host evidence: `docs/evidence/stage8_cogging_software.md`. Power conditions as for stage 7: DCBUS 15 V, 500 mA, OCP on, supply output button as e-stop, USB and DCBUS never together, no core halts while powered.

## Starting point (stage 7)

| Reference | Mean | Ripple | Frequency |
|---|---|---|---|
| 60 rpm | 63.8 rpm | ±24 rpm | 15 Hz |
| 100 rpm | 99.6 rpm | ±18 rpm | 23 Hz |

The ripple is at 2× the electrical frequency and position-locked. It implies a torque disturbance of about 3–4 mNm. The suspects are magnet-to-magnet cogging (14/turn) and an IA/IC gain mismatch.

## Firmware `65f5435` flashed (2026-10-15)

- **Flash:** USB power, VBUS 0 mV checked live, probe S/N 63728769, UID `001D0050 42535013 20363130`.
- **Image:** `fw_65f5435.bin` SHA-256 `f59343bc883ccc838db64f34eadfed8280f3d57b17f618b9a5df5b7853329a31`; verify OK.
- **Command block:** `0x200027F4` (cff +44); test_spin at `0x20000E10` / `0x20000E14`.
- **Boot benchmarks:**

  | Item | Cycles |
  |---|---|
  | Feed-forward lookup | ≤ 101 |
  | Mode-7 period, unpowered | 1397 |
  | Full Stage 5 step | 1507 |
  | Idle ISR | 1254–1262 |

  `vec_compare` PASS.
- **Mode 8 with the stage off:** phase 11 (failed), reason 1 (not powered), `cogmap_ok 0`, ff_valid 0; SD1–SD4 low, PC0–PC3 analog.
- Raw: `build/hil/stage8/`.

## Run 1 (DCBUS 15 V, 2026-10-15)

Fresh boot on DCBUS, probe 63728769. Raw: `build/hil/stage8/` (`ident.log`, `map.log`, `jl_map_run1.out`, `marks.txt`).

**Identification:** direction −1 (Δ −218 / −181), rotation c −104, load 107, offset 3194, R 0.200 Ω, V0 234 mV, L 69 µH.

**Gain check** (open-loop voltage at the 1 A point, 12 electrical angles):

| k | θe (counts) | IA | IB | IC (mA) | \|i\| (mA) | angle error (counts) |
|---|---|---|---|---|---|---|
| 0 | 0 | 1003 | −514 | −489 | 1003 | −9 |
| 1 | 341 | 1019 | −9 | −1009 | 1171 | −4 |
| 2 | 682 | 524 | 493 | −1017 | 1018 | −10 |
| 3 | 1024 | 64 | 991 | −1055 | 1183 | −35 |
| 4 | 1365 | −462 | 1003 | −540 | 1004 | −28 |
| 5 | 1706 | −1007 | 995 | 12 | 1156 | 7 |
| 6 | 2048 | −1000 | 506 | 493 | 1000 | −5 |
| 7 | 2389 | −1025 | 30 | 995 | 1167 | −16 |
| 8 | 2730 | −514 | −505 | 1019 | 1019 | −2 |
| 9 | 3072 | −14 | −1023 | 1037 | 1189 | −8 |
| 10 | 3413 | 466 | −1009 | 542 | 1010 | −27 |
| 11 | 3754 | 1002 | −1017 | 15 | 1166 | −7 |

Fit: I0 1090 mA, **e1 0.3 %, e2 1.1 %**.

- The magnitude alternates between about 1000 mA (0°, 60°, …) and about 1170 mA (30°, 90°, …), a 6θ pattern: the inverter dead-time loss is smaller where one phase carries no current. That is not a sensor effect, and on 12 evenly spaced points it is orthogonal to the 1θ/2θ fit terms.
- **The sensor gain mismatch is about 1.1 %** (2θ, possibly also phase-resistance imbalance), and the offset effect 0.3 %. At iq about 0.1 A that is a few mA of torque ripple. **The current sensors are ruled out** as the source of the 3–4 mNm, 2×-electrical ripple; magnet cogging remains the suspect.

**Ripple map failed:** phase 11, reason 6 (slip).
- At +10 rpm the rotor moved **stick-slip** (encoder 3109 → 2953 → 2886 → 2860 → 2806 → 2571 → 2284, then stuck at 2273 for more than 1 s).
- The speed PI (kp 4.6 mA/rpm, ki 71 mA/(rpm·s)) builds iq too slowly against stiction and cogging at this speed; the rotor breaks free, overshoots, and sticks again, a limit cycle.
- A speed-loop map at crawl speed does not work on this motor.
- No fault; ISR max 2396 cycles (segments up to 174 / 2014 / 236).

## Firmware `fc34266` flashed (2026-10-15)

Phase B of the map is now a position-loop sweep (see `docs/evidence/stage8_cogging_software.md`).

- **Supply overlap before the flash (HW-PWR-003):** the board booted on DCBUS for run 1. Its VBUS reading fell from 15.06 V to 71 mV at uptime 00:20:57 without a reset, so USB (or J-Link power) was connected while DCBUS was still on. The overlap time is unknown. The board stayed healthy: no fault, normal reports, and current offsets unchanged.
- **Flash:** USB power, VBUS 53 mV checked live just before flashing, probe S/N 63728769, UID `001D0050 42535013 20363130`.
- **Image:** `fw_fc34266.bin` SHA-256 `663e364eb085895df9df3e984cb1275bc996b39e01319cd4cbda98985efc3539` (identical to `build/arm/zephyr/zephyr.bin`); verify OK.
- **Command block:** `0x20002808` (mode +4, cff +44).
- **Boot benchmarks:**

  | Item | Cycles |
  |---|---|
  | Feed-forward lookup | ≤ 101 |
  | Mode-7 period, unpowered | 1394 |
  | Full Stage 5 step | 1507 |
  | CORDIC | 25–29 |
  | Idle ISR | 1244–1258 |

  `vec_compare` PASS (64/64 vectors, max error 1.9e-6).
- **Mode 8 with the stage off:** phase 11 (failed), reason 1 (not powered), ff_valid 0. Register reads: GPIOB ODR 0 (SD1–SD4 low), GPIOA ODR 0, GPIOC MODER `0xFFFFFFFF` (PC0–PC3 analog).
- **Missed periods:** one, at the close of the J-Link session for the mode-8 check. This is the known disconnect artefact; there were no overruns and no fault.
- Raw: `build/hil/stage8/` (`jlink_flash_fc34266.txt`, `boot_fc34266.log`, `jlink_t_map_off_fc34266.txt`, `t_map_off_fc34266.log`).

## Run 2 (DCBUS 15 V, 2026-10-15)

Firmware `fc34266`. The board booted fresh on DCBUS after USB was unplugged (a new boot banner, with VBUS 15.0 V in the first report). Probe 63728769, one persistent J-Link session (connected with the stage off). Raw: `build/hil/stage8/` (`ident_run2.log`, `map_run2.log`, `jl_map_run2.out`, `marks_run2.txt`).

**Identification:** direction −1, rotation c 26, load 118, offset 3351, R 200 mΩ, V0 234 mV, L 70 µH. ISR max 1756 cycles.

**Gain check:** the same as run 1 within a few mA. Fit: I0 1092 mA, e1 0.3 %, e2 0.9 %.

**Ripple map failed:** phase 11, reason 6 (slip), 20 s after the start, in the forward sweep. Calibrated position vs the target (410 counts/s):

| t (s) | Position (counts) | Target (counts) |
|---|---|---|
| 44.5 | 264 | 287 |
| 45.0 | 499 | 492 |
| 45.3 | 676 (jump) | 615 |
| 46.0 | 967 | 902 |
| 46.6–47.4 | **1115, stuck** (raw 1968) | 1148 → 1476 |

- The P term (4.19 mA/count) reaches the 0.5 A output cap at about 120 counts of error.
- The duty pattern while stuck was constant: a vector of about 350 mV, which is R·0.5 A plus the DTC term. **iq was therefore about 0.5 A, and the rotor did not move.**
- The static resisting torque at that position is above about 11 mNm. That matches the strong 84/turn slot cogging seen in stage 7, where a 1.0 A alignment lost to the detents. At 6 rpm, momentum does not carry the rotor through the detents as it does at 60 rpm.
- While the rotor was still creeping (t = 46.5 s), the reported iq was 304 mA at an error of about 25 counts, so about 0.2 A came from the integrator.
- There was no fault and no overrun. ISR max 2309 cycles in phase B. One missed period, at the J-Link session connect, with the stage off.

**Decision (user, 2026-10-15):** raise the phase-B cap to 1.0 A (integral 0.8 A). See `docs/evidence/stage8_cogging_software.md`.
