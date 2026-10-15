#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Stage 6 hand-turn check: does the duty pattern turn 7 times per shaft turn?

Reads the firmware's "rot <step> <mech 0..4095> <theta_e 0..4095> <da> <db> <dc>"
lines (duties in 1/10000). For each line it rebuilds the voltage vector angle
from the three duties (Clarke; the SVPWM common mode cancels) and compares it
with the electrical angle the firmware derived from the encoder. Over the
whole log it unwraps both the mechanical count and the duty angle and prints
their ratio (expected: 7 pole pairs).

Only voltage-mode rows count: the duty vector there is 1 V of the nominal
12 V bus, 1/12 = 833 duty units (1/10000) long; rows of another length (the
current loop, the 50 % calibration) are skipped. Optional step bounds select
one sweep; the unwrap needs less than half an electrical turn between rows.

Usage: check_rotation.py <uart log> [first_step last_step]
"""

import math
import re
import sys

COUNTS = 4096
VOLTAGE_MODE_LEN = 10000 / 12  # duty units
LEN_TOL = 0.05
ROT = re.compile(r"\brot (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)")


def wrap(d, period=COUNTS):
    """Wrap a count difference into [-period/2, period/2)."""
    return (d + period // 2) % period - period // 2


def duty_angle_counts(da, db, dc):
    alpha = (2 * da - db - dc) / 3.0
    beta = (db - dc) / math.sqrt(3.0)
    return (math.atan2(beta, alpha) % (2 * math.pi)) * COUNTS / (2 * math.pi)


def duty_len(da, db, dc):
    return math.hypot((2 * da - db - dc) / 3.0, (db - dc) / math.sqrt(3.0))


def main(path, first=0, last=None):
    rows = []
    skipped = 0
    with open(path, errors="replace") as f:
        for line in f:
            m = ROT.search(line)
            if not m:
                continue
            row = tuple(int(x) for x in m.groups())
            if row[0] < first or (last is not None and row[0] > last):
                continue
            if abs(duty_len(*row[3:]) / VOLTAGE_MODE_LEN - 1) > LEN_TOL:
                skipped += 1
                continue
            rows.append(row)
    print(f"{skipped} rows skipped (not voltage mode)")
    if len(rows) < 2:
        print(f"{len(rows)} rot lines: not enough")
        return 1

    max_err = 0.0
    mech_travel = 0
    elec_travel = 0.0
    max_elec_step = 0.0
    prev = None
    for step, mech, theta_e, da, db, dc in rows:
        ang = duty_angle_counts(da, db, dc)
        err = abs(wrap(ang - theta_e))
        max_err = max(max_err, err)
        if prev is not None:
            mech_travel += wrap(mech - prev[0])
            d = wrap(ang - prev[1])
            elec_travel += d
            max_elec_step = max(max_elec_step, abs(d))
        prev = (mech, ang)

    mech_turns = mech_travel / COUNTS
    elec_turns = elec_travel / COUNTS
    print(f"{len(rows)} rot lines, steps {rows[0][0]}..{rows[-1][0]}")
    print(f"duty angle vs encoder electrical angle: max |error| {max_err:.2f} counts "
          f"({max_err * 360 / COUNTS:.3f} electrical deg)")
    print(f"mechanical travel {mech_travel:+d} counts = {mech_turns:+.3f} turns")
    print(f"duty-angle travel {elec_travel:+.0f} counts = {elec_turns:+.3f} electrical turns")
    if abs(mech_travel) > 0:
        print(f"ratio electrical / mechanical = {elec_turns / mech_turns:.3f} (expected 7)")
    if max_elec_step > COUNTS / 4:
        print(f"WARNING: largest electrical step between lines {max_elec_step:.0f} counts; "
              "turn slower or the unwrap may alias")
    return 0


if __name__ == "__main__":
    a = sys.argv
    if len(a) == 2:
        sys.exit(main(a[1]))
    if len(a) == 4:
        sys.exit(main(a[1], int(a[2]), int(a[3])))
    sys.exit(2)
