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
