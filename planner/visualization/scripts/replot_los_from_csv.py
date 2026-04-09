#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(csv_path):
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    times = []
    los_vals = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        if "time" not in reader.fieldnames or "min_los" not in reader.fieldnames:
            raise ValueError(
                f"CSV must contain columns 'time' and 'min_los', got: {reader.fieldnames}"
            )

        for row in reader:
            try:
                t = float(row["time"])
                los = float(row["min_los"])
            except Exception:
                continue

            times.append(t)
            los_vals.append(los)

    if len(times) == 0:
        raise ValueError("No valid rows found in csv.")

    return np.array(times, dtype=float), np.array(los_vals, dtype=float)


def plot_neg_los(times, vals, out_png, threshold=3.2,
                 title="Negative Minimum LoS from CSV"):
    neg_vals = -vals  + 0.8

    fig, ax = plt.subplots(figsize=(12, 6))

    ax.plot(times, neg_vals, linewidth=2.0, label="LoS margin")
    ax.set_ylabel("LoS margin", fontsize=28)
    ax.scatter(times, neg_vals, s=12)
    ax.tick_params(axis='both', labelsize=24)
    # 红色阈值线 y=3.2
    ax.axhline(
        y=threshold,
        color="red",
        linestyle="--",
        linewidth=2.0,
        label=f"y={threshold}"
    )

    # ax.set_title(title)
    # ax.set_xlabel("Time [s]")
    ax.set_ylabel("LoS margin")
    ax.grid(True, linestyle="--", alpha=0.4)
    # ax.legend()

    # 保证红线能显示出来
    ymin = min(np.min(neg_vals) - 0.5, threshold - 1.0)
    ymax = max(np.max(neg_vals) + 0.5, threshold + 0.5)
    ax.set_ylim([ymin, ymax])

    fig.tight_layout()
    fig.savefig(out_png, dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Replot negative LoS curve from generated csv file."
    )
    parser.add_argument(
        "--csv",
        required=True,
        help="Input csv file path, e.g. /home/weijian/.ros/los_record/los_min.csv"
    )
    parser.add_argument(
        "--out",
        default="",
        help="Output png path. Default: same folder as csv, named los_min_neg.png"
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=3.2,
        help="Horizontal reference line value, default 3.2"
    )
    args = parser.parse_args()

    csv_path = os.path.abspath(args.csv)

    if args.out.strip() == "":
        out_png = os.path.join(
            os.path.dirname(csv_path),
            os.path.splitext(os.path.basename(csv_path))[0] + "_neg.png"
        )
    else:
        out_png = os.path.abspath(args.out)

    times, vals = read_csv(csv_path)
    plot_neg_los(times, vals, out_png, threshold=args.threshold)

    print(f"Saved figure to: {out_png}")


if __name__ == "__main__":
    main()