"""Summarize a stage 4 UART log: ISR rate, period, exec time, ISR position, IA/IC, errors."""
import re, statistics as st, sys

ARR = 4250
rows = []
cur = {}
for line in open(sys.argv[1], errors="replace"):
    m = re.search(r"report (\d+): (\d+) ISR/s(?:, period (\d+)\.\.(\d+) cyc, exec max (\d+) cyc)?", line)
    if m:
        cur = {"n": int(m[1]), "isr": int(m[2]), "valid": m[3] is not None and "DWT was stopped" not in line}
        if m[3]:
            cur.update(pmin=int(m[3]), pmax=int(m[4]), exec=int(m[5]))
        rows.append(cur)
        continue
    m = re.search(r"TIM1 CNT at ISR (\d+)\.\.(\d+) .*counting up in (\d+)", line)
    if m and cur:
        cur.update(cmin=int(m[1]), cmax=int(m[2]), up=int(m[3]))
    m = re.search(r"IA mean (\d+)\.(\d+).*IC mean (\d+)\.(\d+)", line)
    if m and cur:
        cur.update(ia=int(m[1]) + int(m[2]) / 100, ic=int(m[3]) + int(m[4]) / 100)
    m = re.search(r"DMA errors (\d+), ADC overruns (\d+)", line)
    if m and cur:
        cur.update(dma=int(m[1]), ovr=int(m[2]))

full = [r for r in rows if "cmin" in r and "dma" in r]
valid = [r for r in full if r.get("valid") and r.get("pmin", 0) > 0]
print(f"reports {len(full)}; with valid DWT figures {len(valid)}")
print(f"ISR/s min {min(r['isr'] for r in full)} max {max(r['isr'] for r in full)}")
if valid:
    print(f"period min {min(r['pmin'] for r in valid)} max {max(r['pmax'] for r in valid)} cyc; "
          f"exec max {max(r['exec'] for r in valid)} cyc")
us = lambda c: (ARR - c) / 170
print(f"ISR entry after top: earliest {us(max(r['cmax'] for r in full)):.2f} us, latest {us(min(r['cmin'] for r in full)):.2f} us; "
      f"median per-report latest {us(st.median(r['cmin'] for r in full)):.2f} us; counting-up entries {sum(r['up'] for r in full)}")
print(f"IA mean {st.mean(r['ia'] for r in full):.2f} (range {min(r['ia'] for r in full):.2f}..{max(r['ia'] for r in full):.2f}); "
      f"IC mean {st.mean(r['ic'] for r in full):.2f} (range {min(r['ic'] for r in full):.2f}..{max(r['ic'] for r in full):.2f})")
print(f"DMA errors {max(r['dma'] for r in full)}, ADC overruns {max(r['ovr'] for r in full)} (sticky totals)")
