# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib>=3.8"]
# ///
"""Plot sim_current_loop CSV output; several files are overlaid for comparison.

Usage:
    build/host/sim_current_loop --csv > build/host/sim.csv
    uv run tests/host/foc/plot_sim.py build/host/sim.csv
    uv run tests/host/foc/plot_sim.py build/host/bw1000.csv build/host/bw2000.csv --xlim 4 10 -o build/host/bw.png

Each file is labelled with its name, so name the CSVs after the experiment.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
REF = "#8a8984"


def load(path, xlim=None):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    # DictReader puts surplus row values under the key None.
    if rows and (None in rows[0] or None in rows[0].values()):
        sys.exit(f"{path}: data rows and the CSV header have different column counts; fix the printf")
    # Drop rows outside the zoom window so the y-axes rescale to what is shown.
    if xlim:
        rows = [r for r in rows if xlim[0] <= float(r["t_s"]) * 1e3 <= xlim[1]]
    if not rows:
        return None
    return {k: [float(r[k]) for r in rows] for k in rows[0]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+", help="one or more CSV files from sim_current_loop --csv (at most 4)")
    parser.add_argument("--xlim", nargs=2, type=float, metavar=("START_MS", "END_MS"), help="zoom the time axis")
    parser.add_argument("-o", "--output", help="save to this image file instead of opening a window")
    args = parser.parse_args()
    if len(args.csv) > len(SERIES):
        print(f"at most {len(SERIES)} files can be overlaid")
        return 2

    runs = []
    for path in args.csv:
        col = load(path, args.xlim)
        if col is None:
            print(f"{path}: no data rows" + (" in --xlim window" if args.xlim else ""))
            return 1
        runs.append((Path(path).stem, col))
    single = len(runs) == 1

    # Duties only make sense for one run; overlaid they are unreadable.
    fig, ax = plt.subplots(5 if single else 4, 1, sharex=True, figsize=(10, 12 if single else 10))
    first = runs[0][1]
    t_ms = [t * 1e3 for t in first["t_s"]]
    ax[0].plot(t_ms, first["iq_ref"], "--", color=REF, label="iq_ref")
    # Optional column, written by a speed loop (study guide experiment 5).
    if "omega_ref" in first:
        ax[3].plot(t_ms, first["omega_ref"], "--", color=REF, label="omega_ref")
    for color, (name, col) in zip(SERIES, runs):
        t = [x * 1e3 for x in col["t_s"]]
        tag = "" if single else f" {name}"
        ax[0].plot(t, col["iq"], color=color, label=f"iq{tag}")
        ax[1].plot(t, col["id"], color=color, label=f"id{tag}")
        ax[2].plot(t, col["vq"], color=color, label=f"vq{tag}")
        ax[2].plot(t, col["vd"], color=color, ls=":", label=f"vd{tag}")
        ax[3].plot(t, col["omega_m"], color=color, label=f"omega_m{tag}")
    if single:
        for color, phase in zip(SERIES, "abc"):
            ax[4].plot(t_ms, first[f"duty_{phase}"], color=color, label=f"duty_{phase}")
        ax[4].set_ylabel("duty")
    ax[0].set_ylabel("iq [A]")
    ax[1].set_ylabel("id [A]")
    ax[2].set_ylabel("voltage [V]")
    ax[3].set_ylabel("speed [rad/s mech]")
    ax[-1].set_xlabel("time [ms]")
    for a in ax:
        a.grid(True, color="#ecebe8")
        a.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=110)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
