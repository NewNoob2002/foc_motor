# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.8"]
# ///
"""Plot sim_svpwm CSV output (exercise 2, docs/study/foc_study_guide.md).

Usage:
    build/host/sim_svpwm --csv > build/host/svpwm.csv
    uv run tests/host/foc/sim/plot_svpwm.py build/host/svpwm.csv
    uv run tests/host/foc/sim/plot_svpwm.py build/host/svpwm.csv --ratio 0.866 --angle 20 -o svpwm.png

--ratio picks the vector length (fraction of Vbus/sqrt(3)) for the duty and
PWM-period panels; --angle picks the electrical angle of the PWM period.
"""

import argparse
import csv
import itertools
import math
import sys

import matplotlib.pyplot as plt

VBUS = 12.0  # must match VBUS_V in sim_svpwm.c

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
# Switching state (a b c high side on) -> space-vector name.
VECTOR = {"000": "V0", "100": "V1", "110": "V2", "010": "V3", "011": "V4", "001": "V5", "101": "V6", "111": "V7"}
INK = "#0b0b0b"
INK_MUTED = "#52514e"
REF = "#8a8984"
BAND = "#ecebe8"


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    by_ratio = {}
    for r in rows:
        by_ratio.setdefault(float(r["ratio"]), []).append({k: float(v) for k, v in r.items()})
    return by_ratio


def nearest(keys, x):
    return min(keys, key=lambda k: abs(k - x))


def shade_outside_duty(ax):
    ax.axhspan(1.0, 2.0, color=BAND, zorder=0)
    ax.axhspan(-1.0, 0.0, color=BAND, zorder=0)
    ax.axhline(0.0, color=REF, lw=1)
    ax.axhline(1.0, color=REF, lw=1)
    ax.axhline(0.5, color=REF, lw=1, ls=":")


def plot_duties(ax, rows, ratio):
    deg = [r["theta_deg"] for r in rows]
    for color, ph in zip(SERIES, "abc"):
        ax.plot(deg, [r[f"duty_{ph}"] for r in rows], color=color, lw=2, label=f"duty_{ph}")
    shade_outside_duty(ax)
    ax.set_title(f"Phase duties, |v| = {ratio:g} x Vbus/sqrt(3)  (gray = impossible)", color=INK)
    ax.set_xlabel("electrical angle [deg]")
    ax.set_ylabel("duty")
    ax.set_xticks(range(0, 361, 60))
    ax.set_ylim(-0.25, 1.25)


def plot_duty_a_all(ax, by_ratio):
    for color, ratio in zip(SERIES, sorted(by_ratio)):
        rows = by_ratio[ratio]
        ax.plot([r["theta_deg"] for r in rows], [r["duty_a"] for r in rows], color=color, lw=2, label=f"{ratio:g}")
    shade_outside_duty(ax)
    ax.set_title("duty_a at every vector length (legend: fraction of Vbus/sqrt(3))", color=INK)
    ax.set_xlabel("electrical angle [deg]")
    ax.set_ylabel("duty")
    ax.set_xticks(range(0, 361, 60))
    ax.set_ylim(-0.25, 1.25)


def plot_plane(ax, by_ratio):
    # Reachable hexagon: the six active vectors have length 2/3 * Vbus.
    hexagon = [
        (2 / 3 * VBUS * math.cos(math.radians(a)), 2 / 3 * VBUS * math.sin(math.radians(a))) for a in range(0, 361, 60)
    ]
    ax.plot(*zip(*hexagon), color=REF, lw=1, label="hexagon (2/3 Vbus)")
    for bits, name in VECTOR.items():
        if name in ("V0", "V7"):
            continue
        i = int(name[1]) - 1
        x, y = hexagon[i]
        ax.annotate(
            f"{name} {bits}",
            (x, y),
            textcoords="offset points",
            xytext=(6 if x >= 0 else -6, 6),
            ha="left" if x >= 0 else "right",
            color=INK_MUTED,
            fontsize=8,
        )
    circle = [math.radians(a) for a in range(0, 361, 2)]
    for radius, style, name in (
        (VBUS / 2, ":", "Vbus/2 (sine PWM)"),
        (VBUS / math.sqrt(3), "--", "Vbus/sqrt(3) (SVPWM)"),
    ):
        ax.plot(
            [radius * math.cos(a) for a in circle],
            [radius * math.sin(a) for a in circle],
            color=REF,
            lw=1,
            ls=style,
            label=name,
        )
    for color, ratio in zip(SERIES, sorted(by_ratio)):
        rows = by_ratio[ratio]
        ax.plot(
            [r["out_alpha"] for r in rows],
            [r["out_beta"] for r in rows],
            color=color,
            lw=2,
            label=f"applied, {ratio:g}",
        )
    ax.set_title("Applied voltage vector (alpha/beta); limited vectors stay on the dashed circle", color=INK)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    ax.set_xlabel("alpha [V]")
    ax.set_ylabel("beta [V]")
    ax.set_aspect("equal")
    lim = 2 / 3 * VBUS * 1.25
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)


def plot_period(ax, rows, ratio, angle):
    row = min(rows, key=lambda r: abs(r["theta_deg"] - angle))
    duty = {ph: row[f"duty_{ph}"] for ph in "abc"}
    # Center-aligned PWM: the high side is on for duty * T around mid-period.
    edges = sorted({0.0, 1.0} | {min(max(0.5 + s * duty[ph] / 2, 0.0), 1.0) for ph in "abc" for s in (-1, 1)})
    for i, (color, ph) in enumerate(zip(SERIES, "abc")):
        base = 2 - i
        lo, hi = max(0.5 - duty[ph] / 2, 0.0), min(0.5 + duty[ph] / 2, 1.0)
        t = [0.0, lo, lo, hi, hi, 1.0]
        y = [base, base, base + 0.7, base + 0.7, base, base]
        ax.plot(t, y, color=color, lw=2, drawstyle="default", label=f"{ph}: duty {duty[ph]:.3f}")
    for t0, t1 in itertools.pairwise(edges):
        if t1 - t0 < 1e-4:
            continue
        mid = 0.5 * (t0 + t1)
        state = "".join("1" if abs(mid - 0.5) < duty[ph] / 2 else "0" for ph in "abc")
        ax.axvline(t0, color=REF, lw=0.5, ls=":")
        ax.text(
            mid, 3.05, f"{VECTOR[state]} {state}", ha="center", va="bottom", fontsize=8, color=INK_MUTED, rotation=90
        )
    title = f"One PWM period at {row['theta_deg']:g} deg, |v| = {ratio:g}  (labels: vector, bits a b c)"
    if any(not 0.0 <= d <= 1.0 for d in duty.values()):
        title += "\nduty outside [0, 1]: the bridge cannot do this, drawn clipped"
    ax.set_title(title, color=INK)
    ax.set_xlabel("time [fraction of PWM period]")
    ax.set_yticks([2.35, 1.35, 0.35], ["a", "b", "c"])
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.2, 3.9)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv")
    parser.add_argument("--ratio", type=float, default=1.0)
    parser.add_argument("--angle", type=float, default=20.0)
    parser.add_argument("-o", "--output", help="save to this image file instead of opening a window")
    args = parser.parse_args()

    by_ratio = load(args.csv)
    if not by_ratio:
        print("no data rows")
        return 1
    ratio = nearest(by_ratio, args.ratio)
    rows = by_ratio[ratio]

    fig, ax = plt.subplots(2, 2, figsize=(14, 10))
    plot_duties(ax[0][0], rows, ratio)
    plot_plane(ax[0][1], by_ratio)
    plot_duty_a_all(ax[1][0], by_ratio)
    plot_period(ax[1][1], rows, ratio, args.angle)
    for a in ax.flat:
        a.grid(True, color=BAND)
        a.tick_params(colors=INK_MUTED)
        if a.get_legend() is None:
            a.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=110)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
