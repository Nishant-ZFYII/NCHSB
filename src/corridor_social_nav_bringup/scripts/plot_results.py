#!/usr/bin/env python3
"""
Post-experiment plotting: generate publication-quality figures from
experiment CSV data for the ICRA 2027 paper.

Usage:
  python3 plot_results.py --input results/all_trials.csv --output-dir results/figures/

Generates:
  1. Box plots: min separation, time-to-goal per controller
  2. Bar chart: success rate and collision rate per controller
  3. Ablation sweep: single-parameter sensitivity plots
  4. Shield timeline: intervention percentage across scenarios
  5. Per-scenario box plots
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from typing import Dict, List

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("Warning: matplotlib not found. Install with: pip install matplotlib")

CONTROLLER_ORDER = ["mppi_vanilla", "mppi_social_instant", "mppi_social_pred", "full"]
CONTROLLER_LABELS = {
    "mppi_vanilla": "MPPI",
    "mppi_social_instant": "MPPI+Social",
    "mppi_social_pred": "MPPI+Pred",
    "full": "Full (Ours)",
}
CONTROLLER_COLORS = {
    "mppi_vanilla": "#999999",
    "mppi_social_instant": "#5DA5DA",
    "mppi_social_pred": "#FAA43A",
    "full": "#60BD68",
}

SCENARIO_ORDER = [
    "head_on_single", "head_on_group", "bidirectional",
    "overtaking", "doorway_popout", "junction_crossing",
]
SCENARIO_SHORT = {
    "head_on_single": "H-on(1)",
    "head_on_group": "H-on(3)",
    "bidirectional": "Bidir",
    "overtaking": "Overt",
    "doorway_popout": "Door",
    "junction_crossing": "Junct",
}


def load_csv(path: str) -> List[dict]:
    with open(path, "r") as f:
        return list(csv.DictReader(f))


def group_by_field(rows: List[dict], field: str) -> Dict[str, List[dict]]:
    groups = defaultdict(list)
    for r in rows:
        groups[r[field]].append(r)
    return groups


def plot_boxplot_metric(
    rows: List[dict],
    metric: str,
    ylabel: str,
    title: str,
    out_path: str,
):
    """Box plot of a metric across controllers."""
    by_ctrl = group_by_field(rows, "controller")

    data = []
    labels = []
    colors = []
    for ctrl in CONTROLLER_ORDER:
        if ctrl not in by_ctrl:
            continue
        vals = [float(r[metric]) for r in by_ctrl[ctrl]]
        data.append(vals)
        labels.append(CONTROLLER_LABELS.get(ctrl, ctrl))
        colors.append(CONTROLLER_COLORS.get(ctrl, "#333333"))

    fig, ax = plt.subplots(figsize=(8, 5))
    bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.6)
    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  Wrote {out_path}")


def plot_bar_rates(rows: List[dict], out_path: str):
    """Grouped bar chart: success rate and collision rate."""
    by_ctrl = group_by_field(rows, "controller")

    ctrls = [c for c in CONTROLLER_ORDER if c in by_ctrl]
    success_rates = []
    collision_rates = []
    for ctrl in ctrls:
        trials = by_ctrl[ctrl]
        n = len(trials)
        s = sum(1 for t in trials if int(t["success"]) == 1)
        c = sum(1 for t in trials if int(t["collision_count"]) > 0)
        success_rates.append(100.0 * s / n if n > 0 else 0)
        collision_rates.append(100.0 * c / n if n > 0 else 0)

    x = np.arange(len(ctrls))
    width = 0.35

    fig, ax = plt.subplots(figsize=(8, 5))
    bars1 = ax.bar(x - width / 2, success_rates, width, label="Success %",
                   color="#60BD68", alpha=0.8)
    bars2 = ax.bar(x + width / 2, collision_rates, width, label="Collision %",
                   color="#F15854", alpha=0.8)

    ax.set_ylabel("Rate (%)", fontsize=12)
    ax.set_title("Success and Collision Rates by Controller", fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels([CONTROLLER_LABELS.get(c, c) for c in ctrls])
    ax.legend()
    ax.set_ylim(0, 105)
    ax.grid(axis="y", alpha=0.3)

    for bar in bars1:
        h = bar.get_height()
        ax.annotate(f"{h:.0f}", xy=(bar.get_x() + bar.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha="center", fontsize=9)
    for bar in bars2:
        h = bar.get_height()
        ax.annotate(f"{h:.0f}", xy=(bar.get_x() + bar.get_width() / 2, h),
                    xytext=(0, 3), textcoords="offset points", ha="center", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  Wrote {out_path}")


def plot_shield_by_scenario(rows: List[dict], out_path: str):
    """Bar chart: shield intervention % per scenario for 'full' controller."""
    full_rows = [r for r in rows if r["controller"] == "full"]
    by_sc = group_by_field(full_rows, "scenario")

    scenarios = [s for s in SCENARIO_ORDER if s in by_sc]
    means = []
    stds = []
    for sc in scenarios:
        vals = [float(r["shield_interventions_pct"]) for r in by_sc[sc]]
        means.append(np.mean(vals))
        stds.append(np.std(vals))

    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(scenarios))
    ax.bar(x, means, yerr=stds, capsize=4, color="#5DA5DA", alpha=0.8)
    ax.set_ylabel("Shield Intervention (%)", fontsize=12)
    ax.set_title("CBF Safety Shield Activity by Scenario (Full Controller)", fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels([SCENARIO_SHORT.get(s, s) for s in scenarios], fontsize=10)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"  Wrote {out_path}")


def plot_per_scenario_boxplots(rows: List[dict], metric: str, ylabel: str, out_dir: str):
    """One box plot per scenario, comparing controllers."""
    by_sc = group_by_field(rows, "scenario")

    for sc in SCENARIO_ORDER:
        if sc not in by_sc:
            continue
        sc_rows = by_sc[sc]
        by_ctrl = group_by_field(sc_rows, "controller")

        data = []
        labels = []
        colors = []
        for ctrl in CONTROLLER_ORDER:
            if ctrl not in by_ctrl:
                continue
            vals = [float(r[metric]) for r in by_ctrl[ctrl]]
            data.append(vals)
            labels.append(CONTROLLER_LABELS.get(ctrl, ctrl))
            colors.append(CONTROLLER_COLORS.get(ctrl, "#333333"))

        if not data:
            continue

        fig, ax = plt.subplots(figsize=(7, 4))
        bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.5)
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        sc_label = SCENARIO_SHORT.get(sc, sc)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(f"{sc_label}: {ylabel}", fontsize=13)
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()

        fname = f"boxplot_{metric}_{sc}.png"
        fig.savefig(os.path.join(out_dir, fname), dpi=200, bbox_inches="tight")
        plt.close(fig)
        print(f"  Wrote {fname}")


def main():
    if not HAS_MPL:
        print("Error: matplotlib is required. Install with: pip install matplotlib")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Plot corridor social nav results")
    parser.add_argument("--input", required=True, help="Path to all_trials.csv")
    parser.add_argument("--output-dir", default="results/figures", help="Output directory")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: input file not found: {args.input}")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)
    rows = load_csv(args.input)
    print(f"Loaded {len(rows)} rows from {args.input}")

    print("\nGenerating aggregate plots...")
    plot_boxplot_metric(
        rows, "min_separation_m", "Min Separation (m)",
        "Minimum Human-Robot Separation by Controller",
        os.path.join(args.output_dir, "boxplot_min_separation.png"),
    )
    plot_boxplot_metric(
        rows, "time_to_goal_s", "Time to Goal (s)",
        "Navigation Time by Controller",
        os.path.join(args.output_dir, "boxplot_time_to_goal.png"),
    )
    plot_boxplot_metric(
        rows, "path_length_m", "Path Length (m)",
        "Path Length by Controller",
        os.path.join(args.output_dir, "boxplot_path_length.png"),
    )
    plot_bar_rates(rows, os.path.join(args.output_dir, "bar_success_collision.png"))
    plot_shield_by_scenario(rows, os.path.join(args.output_dir, "bar_shield_by_scenario.png"))

    print("\nGenerating per-scenario plots...")
    plot_per_scenario_boxplots(rows, "min_separation_m", "Min Separation (m)", args.output_dir)
    plot_per_scenario_boxplots(rows, "time_to_goal_s", "Time to Goal (s)", args.output_dir)

    print(f"\nAll figures written to {args.output_dir}/")


if __name__ == "__main__":
    main()
