"""Plot per-frame camera motion from the CSV written by ./stabilize --motion-csv.

Usage:
    python scripts/plot_trajectory.py results/pan_motion.csv [-o results/pan_motion.png]

Top three panels: dx, dy and rotation (degrees) for each frame pair.
Bottom panel: how many points agreed with the fit (RANSAC inliers). A sudden
drop means the estimate for that frame is less trustworthy.
"""

import argparse
import math

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", help="motion CSV from ./stabilize --motion-csv")
    parser.add_argument("-o", "--output", help="save the plot here instead of showing it")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    df["da_deg"] = df["da"].apply(math.degrees)

    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    panels = [
        ("dx", "dx (pixels/frame)", "tab:blue"),
        ("dy", "dy (pixels/frame)", "tab:orange"),
        ("da_deg", "rotation (degrees/frame)", "tab:green"),
        ("inliers", "RANSAC inliers", "tab:gray"),
    ]
    for ax, (column, label, color) in zip(axes, panels):
        ax.plot(df["frame"], df[column], color=color, linewidth=1)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)

    # Mark frames where the estimate failed.
    bad = df[df["valid"] == 0]
    for frame in bad["frame"]:
        for ax in axes:
            ax.axvline(frame, color="red", alpha=0.4)

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
