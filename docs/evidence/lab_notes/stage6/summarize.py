#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Summarize the Stage 6 one-second reports in a UART log.

Prints, over the selected reports: ISR rate, period range, worst execution
time (cycles and us), the sticky overrun / missed / fault / DMA / ADC
counters at the last report, and the modes seen. Reports whose DWT figures
were invalid (debugger disconnect) are counted and left out of the cycle
figures.

Usage: summarize.py <uart log> [first_report [last_report]]
"""

import re
import sys

CYC_PER_US = 170
HEAD = re.compile(r"report (\d+): (\d+) ISR/s, period (\d+)\.\.(\d+) cyc, exec max (\d+) cyc")
INVALID = re.compile(r"report (\d+): (\d+) ISR/s, DWT was stopped")
CNT = re.compile(r"overruns (\d+), missed (\d+), fault 0x([0-9a-f]+), DMA errors (\d+), ADC overruns (\d+)")
MODE = re.compile(r"mode (\w+) ")


def main(path, first=1, last=None):
    reports = []  # (n, isr, pmin, pmax, exec) or (n, isr, None...)
    counters = None
    modes = set()
    current = None
    with open(path, errors="replace") as f:
        for line in f:
            m = HEAD.search(line)
            if m:
                current = int(m.group(1))
                if current >= first and (last is None or current <= last):
                    reports.append(tuple(int(x) for x in m.groups()))
                continue
            m = INVALID.search(line)
            if m:
                current = int(m.group(1))
                if current >= first and (last is None or current <= last):
                    reports.append((current, int(m.group(2)), None, None, None))
                continue
            if current is None or current < first or (last is not None and current > last):
                continue
            m = CNT.search(line)
            if m:
                counters = m.groups()
            m = MODE.search(line)
            if m and "  mode " in line:
                modes.add(m.group(1))
    valid = [r for r in reports if r[2] is not None]
    if not reports:
        print("no reports in range")
        return 1
    print(f"reports {reports[0][0]}..{reports[-1][0]}: {len(reports)} ({len(reports) - len(valid)} with invalid DWT)")
    print(f"ISR/s {min(r[1] for r in reports)}..{max(r[1] for r in reports)}")
    if valid:
        wcet = max(r[4] for r in valid)
        print(f"period {min(r[2] for r in valid)}..{max(r[3] for r in valid)} cycles")
        print(f"exec max {wcet} cycles = {wcet / CYC_PER_US:.2f} us (budget 3400 = 20 us)")
    if counters:
        o, mi, fa, de, ao = counters
        print(f"at the last report: overruns {o}, missed {mi}, fault 0x{fa}, DMA errors {de}, ADC overruns {ao}")
    print(f"modes: {', '.join(sorted(modes)) or '-'}")
    return 0


if __name__ == "__main__":
    a = sys.argv
    if len(a) < 2:
        sys.exit(2)
    sys.exit(main(a[1], int(a[2]) if len(a) > 2 else 1, int(a[3]) if len(a) > 3 else None))
