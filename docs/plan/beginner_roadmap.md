# Beginner Roadmap

Status: **adopted 2026-10-14** as the current execution path (`AGENTS.md`, "当前阶段"). **Current stage: 1.** [development_roadmap.md](./development_roadmap.md) stays as the long-term product plan; section 5 maps the two.

The M0–M6 plan is organized around formal stage gates. This plan is organized around **one new skill per stage, each with a result you can see**. Motor power comes only at the last stage. Theory and measured simulation results are in [foc_knowledge_base.md](../study/foc_knowledge_base.md).

## 1. Rules that do not change

These stay in every stage, whatever else is simplified:

1. **SD1–SD4 low at reset and whenever not running.** `SD = 0` is the only "off" state of the EG2104: `SD = 1, IN = 0` turns the low side **on** (HW-PWM-007).
2. **D leg always off:** `SD4 = 0`; PC3 never drives power PWM.
3. **No dynamic memory, nothing blocking in the FOC ISR** (no kernel API, logging, locks).
4. **Write down what you measured, not what you expect.** A build or a simulation is not a hardware result.
5. **Power tests:** current-limited supply, a way to cut power immediately, motor clamped with nothing attached to the shaft.

## 2. Board facts that shape the plan

Read from the main schematic (`DOC-MAIN-SCH`) and [hardware_reference.md](../hardware/hardware_reference.md):

| Fact | Consequence |
|---|---|
| The gate-driver supply (+12 V) comes from **DCBUS through a buck converter** (U9, XL7005A). USB appears to feed only +5 V / +3.3 V (through R35, 0 Ω) | Without DCBUS the MCU runs on USB but the gate drivers are unpowered: stages 1–6 can only look at **MCU pins**. Gate waveforms need DCBUS |
| DCBUS must be high enough for the buck to regulate 12 V | The lowest usable bus voltage comes from the XL7005A datasheet (not in the repo yet), not from the motor |
| USB 5 V and the regulated 5 V appear to be tied by R35 (0 Ω) | Check before connecting USB and DCBUS at the same time: the board may push 5 V back into the PC's USB port |
| Current sense: 5 mΩ × 10 around 1.65 V = 50 mV/A ≈ 62 ADC counts/A, zero at ≈ 2048 counts | Offsets and gains must be measured (R-06) |
| VBUS divider 100 kΩ / 4.7 kΩ = 0.0449 | 24 V reads ≈ 1.08 V ≈ 1337 counts; full scale ≈ 73 V |
| Encoder 4096 counts/rev, 7 pole pairs | 4096 / 7 = 585.14 counts per electrical turn: **not an integer**, so compute `theta_e` from the full count, never by counting electrical turns |
| TIM1 center-aligned 20 kHz at a 170 MHz timer clock | ARR = 170 MHz / (2 × 20 kHz) = 4250 |
| No CPU-independent shutdown (R-02) | Decides how stage 7 may run; see section 4 |

## 3. Stages

Each stage: **learn** (the concept), **do** (the code or setup), **done when** (what you can show). One short lab note per stage (section 6) and one branch per stage, named `stage/N-name` (for example `stage/1-boot-check`, `stage/2-encoder`). All of a stage's work goes on its branch: code, scripts, roadmap and rule wording, the lab note and evidence. Commit and push often; each push runs CI. Merge into `master` once, when the stage's "done when" is met, so `master` only gains finished stages. If `master` changes meanwhile, merge it into the stage branch before finishing.

### Stage 0 — Toolchain and FOC math on the PC (done)

- Zephyr v4.4.2 build, safe-boot firmware, UART3 console, CI (host + ARM) — done, CI run 38546477578.
- Clarke/Park, PI, SVPWM, current and speed loops in simulation — done (study guide exercises 1–3).

### Stage 1 — Bench setup and the boot-safety check with the J-Link

- **Learn:** the debugger (halt, step, data watchpoints), GPIO and TIM1 registers, what "safe at reset" means, and why a register snapshot is not a waveform.
- **Do:** USB power only, no DCBUS, no flashing: debug the firmware already on the board.
  1. Bench record: power source and current limit, how you cut power, J-Link serial number, board UID read back over SWD (M1 recorded `001D0050 42535013 20363130`).
  2. Multimeter: hold the reset button and measure SD1–SD4 to ground. While reset is held the MCU pins are high-impedance, so this checks the EG2104 internal pull-downs.
  3. Halt at the first instruction after reset. Read `MODER`, `ODR` and `IDR` for SD1 (PB0), SD2 (PA1), SD3 (PA2), SD4 (PA0) and PC0–PC3, plus TIM1 `CR1` and `BDTR`.
  4. Set data watchpoints on CPU writes to the GPIOA and GPIOB output registers (`BSRR`, `ODR`), the two ports that carry SD1–SD4. Run to `main()` and record every write and its value.
  5. Repeat steps 3–4 after a pin reset (button) and after a software reset (SYSRESETREQ).
- **Done when:** every recorded write keeps SD1–SD4 low; at `main()` the SD pins are outputs driving low and `IDR` reads low; TIM1 is stopped (`CR1.CEN = 0`) with its main output off (`BDTR.MOE = 0`); the multimeter reads about 0 V on SD1–SD4 during reset; the lab note is written.
- **Limits:** this checks the CPU's own writes and static levels only. It cannot see microsecond glitches, DMA writes or the power-up ramp, so **R-18 stays open** and becomes an entry condition for stage 7 (a logic-analyzer or scope capture). That is acceptable before stage 7 because the gate drivers have no supply without DCBUS (section 2).

Register addresses (RM0440, check against the reference manual): GPIOA `0x4800_0000`, GPIOB `0x4800_0400`, GPIOC `0x4800_0800`, offsets `MODER 0x00`, `IDR 0x10`, `ODR 0x14`, `BSRR 0x18`; TIM1 `0x4001_2C00`, offsets `CR1 0x00`, `BDTR 0x44`. The Cortex-M4 has 4 DWT comparators, enough for the 4 watched registers.

### Stage 2 — Read the encoder by hand

- **Learn:** quadrature (A/B), counts per turn, direction, the index pulse (Z), mechanical vs electrical angle.
- **Do:** build the 3.3 V encoder harness first (R-08: the board connector is labelled 5 V and the pinouts differ). TIM3 in encoder mode; print the count and `theta_e` over UART at about 10 Hz; turn the shaft by hand.
- **Done when:** one turn = 4096 counts, the sign matches the direction you turn, Z comes once per turn at the same count, no counts lost after many turns back and forth.

### Stage 3 — Read the ADC

- **Learn:** ADC calibration, converting counts to volts and amps, noise.
- **Do:** USB power only. Read IA, IC, VBUS and TEMP; average 1000 samples of IA/IC with no current.
- **Done when:** IA/IC offsets recorded (expect near 2048 counts) with their noise in counts; VBUS reads near 0; the conversion formulas are in code and in the lab note.

### Stage 4 — PWM on the MCU pins

- **Learn:** center-aligned PWM, shadow registers and the update event, triggering the ADC from TIM1.
- **Do:** USB power only, SD1–SD4 low. TIM1 CH1–3 at 20 kHz with fixed duties (for example 0.25 / 0.50 / 0.75); trigger ADC2 at the counter top; toggle a test pin in the ADC interrupt (AGENTS.md allows PC3 while SD4 stays low).
- **Done when:** the scope shows 20 kHz, the three pulses centered on each other, the ADC sample in the middle of the period, and a duty change taking effect at an update event, never mid-period.

### Stage 5 — The FOC math on the chip

- **Learn:** the FPU, why `double` is slow, measuring time with DWT.
- **Do:** build `lib/foc` into a test firmware (no ISR, no PWM); feed it fixed inputs from the host tests; print results and DWT cycle counts.
- **Done when:** results match the host within float tolerance, and one full current-loop step (Clarke, Park, 2 PI, inverse Park, SVPWM, sin/cos) takes well under 3400 cycles (20 µs at 170 MHz).

### Stage 6 — The real FOC interrupt, without motor power

- **Learn:** the 20 kHz ISR structure, the delay between sample and update, overrun detection.
- **Do:** combine stages 2–5. The ADC interrupt reads IA/IC (about 0 A) and TIM3, runs the full current-loop code path, and writes CCR1–3. No current can flow, so the PIs sit at their limits; that is expected. To see something meaningful, also run a mode that sends a fixed small voltage vector (`vd` only) through inverse Park at `theta_e` and SVPWM. SD1–SD4 stay low; look at PC0–PC2 on the scope and turn the shaft by hand.
- **Done when:** the duty pattern rotates as you turn the shaft (7 electrical turns per mechanical turn); DWT shows WCET under 20 µs and zero overruns over several minutes; a forced overrun is detected.

### Stage 7 — First power (needs the decisions in section 4 and the R-18 capture)

- **Learn:** motor parameters, alignment, closing the loops on real hardware.
- **Do, in order, stopping at the first surprise:**
  1. Lowest DCBUS that runs the 12 V buck; low current limit (for example a few hundred mA); motor clamped, nothing on the shaft.
  2. Measure R and L (4-wire milliohm and LCR meter, or a small DC/voltage-step test). The spec's 170 Ω is probably 170 mΩ.
  3. Open loop: a slowly rotating voltage vector of small amplitude. Check phase order and that the encoder counts the same way.
  4. Alignment: hold a voltage vector at angle 0, record `theta_offset`.
  5. Current loop at 1 kHz bandwidth with small `iq_ref`, gains from the measured R and L.
  6. Speed loop.
- **Done when:** iq follows small steps like the simulation (compare plots), the motor holds a speed, and turning the supply off or a forced fault stops it safely.

## 4. Decisions for you before stage 7

| Decision | Options |
|---|---|
| R-02 (no CPU-independent shutdown) | (a) keep the rule: add a hardware shutdown (comparator pulling SD1–SD3 low) first; (b) accept for a **learning bench only**, limited to a low current-limited supply, and record that acceptance in `AGENTS.md` and `risk_list.md`. A current limit reduces damage but does not replace a shutdown path |
| Bus voltage and supply | Bench supply with an adjustable current limit; minimum voltage set by the XL7005A |
| Instruments | Stage 1 runs with the J-Link and a multimeter. A logic analyzer (an 8-channel, 24 MHz USB one is enough) or scope is needed from stage 4, and for the R-18 capture before stage 7. Milliohm/LCR meter for stage 7 |
| Evidence level | Lab notes (section 6) instead of the full M-gate records; they must still carry the fields `AGENTS.md` requires for hardware tests |

## 5. Adoption (2026-10-14)

What adopting this plan changed:

- `AGENTS.md` "当前阶段": the current position is a stage number (now stage 1); moving to the next stage needs your explicit confirmation, recorded there with the date. Each stage's allowed scope is listed there: stages 1–4 USB power only with SD1–SD4 low, `lib/foc` in a test firmware from stage 5, the FOC ISR from stage 6, and stage 7 not allowed until the R-02 decision is recorded.
- `AGENTS.md` "安全默认值": one explicit exception, PWM on the MCU pins in stages 4–6 while DCBUS is disconnected and SD1–SD4 are low.
- `development_roadmap.md` stays as the long-term product plan. Mapping: stage 1 = M1-09 and the software-path part of M1-08 (the M1-08 waveform, R-18, moves to the stage 7 entry), stages 2–4 = M2, stage 5 = M3 ARM/WCET, stage 6 = M2/M3 ISR, stage 7 = M4.
- Unchanged: the hardware-test rules in `AGENTS.md` (board identity, supply, e-stop, recovery, evidence) and the R-02 rule for powered closed loop.

## 6. Lab note template

One file per stage in `docs/evidence/lab_notes/` (for example `stage2_encoder.md`):

```markdown
# Stage N — title
Date / firmware commit / board UID / tool versions:
Setup: power (USB / DCBUS V, current limit), probes, wiring:
Steps:
Results (numbers, screenshots or raw captures):
Pass / fail against "done when":
Surprises and open questions:
```
