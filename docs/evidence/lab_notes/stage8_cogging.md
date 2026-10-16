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

## Firmware `75821a6` flashed (2026-10-15)

Phase-B iq cap 1.0 A.

- **Supply change:** clean. The DC session ended at uptime 12:31 with VBUS 15 V, then the board booted again on USB with VBUS 71 mV. No overlap.
- **Flash:** USB power, VBUS 71 mV checked live just before flashing, probe 63728769, UID `001D0050 42535013 20363130`.
- **Image:** `fw_75821a6.bin` SHA-256 `c5a297220bc48d56d31b1b7890ce80be9f341456758f7e3979daf4465262dd60`; verify OK.
- **Command block:** `0x20002808` (unchanged).
- **Boot benchmarks:**

  | Item | Cycles |
  |---|---|
  | Full step | 1496 |
  | Mode-7 period, unpowered | 1398 |
  | Feed-forward lookup | 98 |
  | Idle ISR | ≤ 1254 |

  `vec_compare` PASS.
- **Mode 8 with the stage off:** phase 11, reason 1. GPIOB ODR 0, GPIOA ODR 0, GPIOC MODER `0xFFFFFFFF`. One missed period at the J-Link session close (the known artefact).
- Raw: `build/hil/stage8/*75821a6*`.

## Run 3 (DCBUS 15 V, 2026-10-15) — map done, then a fatal stack overflow

Firmware `75821a6` (phase-B cap 1.0 A). The board booted fresh on DCBUS after USB was unplugged. Probe 63728769, persistent J-Link session. Raw: `build/hil/stage8/` (`ident_run3.log`, `map_run3.log`, `jl_map_run3.out`, `marks_run3.txt`, `run3_map_ram.bin`, `run3_report_stack.bin`, `run3_ram_all.bin`).

**Identification:** direction −1, rotation c −103, load 107, offset 1449, R 199 mΩ, V0 235 mV, L 69 µH.

**Map:**
- The gain check completed, and phase B completed both 2-turn sweeps (forward 1:18–1:37, reverse 1:38–1:57 uptime) with no slip.
- Sampled iq (1 s snapshots) ranged from −443 to +467 mA; voltage limit never hit; ISR max 2373 cycles; no fault.

**Fatal error after the map:**
- At 1:58.2 the report thread (1 KiB stack) overflowed into its MPU guard: MPU FAULT, MMFAR `0x20002dec` (108 B into `report_stack` @ `0x20002d80`), "ZEPHYR FATAL ERROR 2: Stack overflow", "Halting system".
- This happened inside `dump_map()` → `cog_process()`, before the first `cogmap_result` line. It was the first time on the target that the `ok` processing path (libm atan2f/sinf/cosf) ran; in runs 1–2 the map failed.
- `-fstack-usage` frames: `report_loop` 280 B, `dump_map` 256 B, `cog_process` 240 B, `report` 672 B. These come on top of the log packaging, libm, the FP exception frame and the 128 B FP stack guard.

**Safety finding: the power stage stayed on after the halt.**
- About 35 s after the halt, J-Link (memory access, no core halt) read the following:
  - TIM1 BDTR `0x8000` (MOE = 1), CCER `0x511`;
  - GPIOB ODR `0x1` (SD1 high), GPIOC MODER `0xFFFFFFEA` (PC0–PC2 in AF);
  - CCR1–3 2125 (50 %, zero vector), timer counting.
- Writing arm = 0 / mode = 0 had no effect.
- **Cause:** Zephyr's fatal handler spins inside the fault exception at priority 0. That is the same priority as the zero-latency FOC ISR, so the ISR (with its trips and disarm) and the 10 ms k_timer supervisor never run again. The PWM stays frozen at its last duty.
- Here the last duty was the zero vector, because the map had just finished. A crash during a sweep would freeze a vector of up to 1 V, about 3.8 A DC in a winding with a bus current below 0.3 A, which the 500 mA OCP would not catch.
- **Recovery:** via J-Link, BDTR = 0 (MOE off), then SD1 (PB0) and SD2/SD3 (PA1/PA2) low through BSRR. Read back: BDTR 0, GPIOA/GPIOB ODR 0. The user then confirmed DC off.

**The map data survived in RAM** (`map` @ `0x20001400`, read with non-halting `savebin`): phase 10 (done), reason 0, ready 1. Samples per bin: 186–12401 forward, 152–13633 reverse. Offline analysis of the raw bins, (fwd + rev)/2 minus its mean:

| Mechanical order | Amplitude | Phase |
|---|---|---|
| 1 | 4.4 mA | −133° |
| 2 | 5.0 mA | −63° |
| 7 | 13.9 mA | 162° |
| **14** | **179.6 mA (≈ 3.9 mNm)** | 166° |
| 21 | 2.4 mA | 147° |
| 28 | 37.7 mA | −59° |
| 42 | 7.4 mA | 6° |
| 84 | 14.9 mA (attenuated by 16-count bins) | −164° |

Friction (fwd − rev)/2: 114 mA ≈ 2.5 mNm. **The stage 7 2×-electrical ripple (3–4 mNm) is confirmed as 14/turn cogging.** The encoder is incremental, so this table is valid only for the crashed boot; the feed-forward needs a fresh map in the next run.

**Fixes before the next powered run** (commit `5027673`, GPT worker task `a46fb236df14`, reviewed; details in `docs/evidence/stage8_fatal_off.md`):
- report stack 2 KiB, with stack high-water telemetry;
- a `k_sys_fatal_error_handler` override that first drives SD1–SD3 low, PC0–PC2 analog and MOE off, then halts;
- a test hook (`crash` key at cmd +48) to verify the safe-off on the bench.

## Firmware `5027673` flashed (2026-10-15)

Includes the fatal safe-off and the 2 KiB report stack.

- **Supply change:** clean. After run 3 the board rebooted on USB; VBUS 17 mV was checked live just before flashing.
- **Flash:** probe 63728769, UID `001D0050 42535013 20363130`.
- **Image:** `fw_5027673.bin` SHA-256 `c6cda238fed2f70f9a74a51fd950b7264d9bef4e1626417f776c3565251dfb38`; verify OK.
- **Command block:** `0x20002808`, with `crash` at +48 (`0x20002838`).
- **Boot:** full step 1545 cycles; `vec_compare` PASS; report stack unused 1304 B after the boot bench.
- **Mode 8 with the stage off:** phase 11, reason 1. GPIOB/GPIOA ODR 0, GPIOC MODER `0xFFFFFFFF`.
- **Crash hook with the stage off** (write `0x43524153` to +48):
  - the field reads back 0;
  - UART: "Kernel oops", then "fatal: power stage off (reason 3)";
  - TIM1 CR1 `0xC1` → `0xC0` (counter stopped), BDTR `0x8000` → `0` (MOE off);
  - SD pins low, PC0–PC3 analog.

  MOE = 1 is the normal stage-off state: PC0–PC2 analog and SD low keep the drivers off. The board was then reset through J-Link.
- **Still to verify with DCBUS:** the crash hook while armed (SD1–SD3 must go from high to low).
- Raw: `build/hil/stage8/*5027673*`.

## Run 4 (DCBUS 15 V, 2026-10-16) — crash safe-off verified, map, ripple reduction

Firmware `5027673`. The board booted fresh on DCBUS after USB was unplugged; the log shows the USB session ending and a new boot at VBUS 15.06 V. Probe 63728769, persistent J-Link session. Raw: `build/hil/stage8/` (`t_crash_armed_5027673.log`, `ident_run4.log`, `map_run4.log`, `speed_run4.log`, `run4_all.log`, `jl_run4.out`, `marks_run4.txt`).

**1. Crash hook while armed** (idle, zero vector):

| | SD1 (GPIOB ODR) | SD2/SD3 (GPIOA ODR) | GPIOC MODER | TIM1 BDTR | TIM1 CR1 |
|---|---|---|---|---|---|
| Armed | `0x1` | `0x6` | `0xFFFFFFEA` (PC0–PC2 AF) | `0x8000` (MOE) | `0xD1` |
| 0.4 s after writing `CRAS` | `0x0` | `0x0` | `0xFFFFFFFF` (analog) | `0x0` | `0xD0` (CNT frozen at `0x7BC`) |

- UART: "ZEPHYR FATAL ERROR 3: Kernel oops" at 29.800 s, then "fatal: power stage off (reason 3)" at 29.807 s. The 7 ms is the synchronous log flush, which runs after the register writes.
- **The fatal path now switches the power stage off.** The pin-level timing was not measured (no scope).
- The board was then reset through J-Link and came back with the stage off and no fault.

**2. Identification:** direction −1, offset 1681, R 200 mΩ, V0 234 mV, L 70 µH.

**3. Map:**
- `cogmap_result phase 10 reason 0`, `cogmap_ok 1`, ff_valid 1. Gain fit I0 1093 mA, e1 0.2 %, e2 1.2 %.

| Order | Amplitude (mA) | Phase (counts) |
|---|---|---|
| 1 | 2 | 2693 |
| 2 | 5 | 2894 |
| 7 | 17 | 1894 |
| **14** | **178** | 1806 |
| 28 | 38 | 3949 |
| 42 | 7 | 2685 |
| 84 | 12 | 3592 |

- Friction 114 mA.
- This matches the run 3 offline analysis (order 14: 180 mA, order 28: 38 mA, friction 114 mA), so **the map is repeatable**.
- **Report stack:** unused 888 B after `dump_map`, so the peak is 1160 B. That is above the old 1024 B, which confirms the cause of run 3. 888 B is left in the 2048 B stack.

**4. Speed loop** (mode 7, DTC on). Each line is a 1 kHz capture (`capspd`) of 256 samples at a constant reference. The feed-forward was switched with `cff` on a `step_seq` edge 2.5 s before its capture. "Order 14" is a least-squares sine fit at 14 × the measured rotation frequency.

| Reference | cff | Mean | SD | Peak-to-peak | Order 14 | Order 28 | Speed-PI iq SD |
|---|---|---|---|---|---|---|---|
| 60 rpm | off | 59.7 rpm | 25.0 rpm | 90.8 rpm | 31.3 rpm (13.9 Hz) | 4.6 rpm | 116 mA |
| 60 rpm | **on** | 59.4 rpm | **6.2 rpm** | 26.3 rpm | **2.8 rpm** | 2.1 rpm | 28 mA |
| 100 rpm | off | 100.0 rpm | 14.8 rpm | 63.0 rpm | 17.8 rpm (23.3 Hz) | 1.4 rpm | 68 mA |
| 100 rpm | **on** | 100.3 rpm | **5.3 rpm** | 24.9 rpm | **1.9 rpm** | 0.8 rpm | 24 mA |

- **Order-14 ripple reduction: 11× at 60 rpm, 9.4× at 100 rpm.** Total SD reduction: 4.0× and 2.8×.
- The "off" figures agree with stage 7 (SD 19.3 / 14.5 rpm).
- The speed PI no longer fights the cogging: its iq SD falls 4×.
- The remaining ripple has no single dominant line (35 Hz at 60 rpm; 55–62 Hz at 100 rpm; 3.9 Hz resolution). It is not analysed further here.

**Timing and faults:**
- ISR max: 1906 cycles in identification, 2368 in the map, 2270 in mode 7 with the feed-forward (budget 3400).
- 0 overruns, 0 missed periods, no fault.
- Disarmed at the end: SD low.
