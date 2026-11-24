"""Plot the raw camera path against the smoothed path.

Usage:
    python scripts/plot_smoothing.py results/walk_traj.csv [-o results/walk_traj.png]

The CSV comes from: ./build/stabilize <clip> -o out.mp4 --trajectory-csv <file>
Gray = raw path (includes the shake). Colored = smoothed path (what the output follows).
"""

import argparse
import math

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="trajectory CSV from --trajectory-csv")
    parser.add_argument("-o", "--output", help="save the plot here instead of showing it")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    df["raw_deg"] = df["raw_a"].apply(math.degrees)
    df["smooth_deg"] = df["smooth_a"].apply(math.degrees)

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    panels = [
        ("raw_x", "smooth_x", "x position (pixels)", "tab:blue"),
        ("raw_y", "smooth_y", "y position (pixels)", "tab:orange"),
        ("raw_deg", "smooth_deg", "angle (degrees)", "tab:green"),
    ]
    for ax, (raw, smooth, label, color) in zip(axes, panels):
        ax.plot(df["frame"], df[raw], color="gray", alpha=0.7, linewidth=1, label="raw")
        ax.plot(df["frame"], df[smooth], color=color, linewidth=2, label="smoothed")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel("frame")
    fig.suptitle(args.csv)
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=150)
        print(f"Saved {args.output}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
