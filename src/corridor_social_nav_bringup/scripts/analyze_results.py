#!/usr/bin/env python3
"""
Post-experiment analysis: aggregate CSV trial data, compute statistics,
generate LaTeX tables for the ICRA 2027 paper.

Usage:
  python3 analyze_results.py --input results/all_trials.csv --output-dir results/tables/

Reads the CSV produced by MetricsLogger across all trials and generates:
  1. Main comparison table (4 controllers x 6 scenarios)
  2. Ablation tables
  3. Per-scenario breakdowns
  4. Statistical significance tests (Wilcoxon signed-rank, Fisher exact)
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import stats as sp_stats


@dataclass
class TrialRow:
    scenario: str
    controller: str
    seed: int
    success: bool
    collision_count: int
    min_separation_m: float
    ttc_violations: int
    shield_interventions_pct: float
    time_to_goal_s: float
    path_length_m: float
    stuck_time_s: float


def load_trials(csv_path: str) -> List[TrialRow]:
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(TrialRow(
                scenario=r["scenario"],
                controller=r["controller"],
                seed=int(r["seed"]),
                success=bool(int(r["success"])),
                collision_count=int(r["collision_count"]),
                min_separation_m=float(r["min_separation_m"]),
                ttc_violations=int(r["ttc_violations"]),
                shield_interventions_pct=float(r["shield_interventions_pct"]),
                time_to_goal_s=float(r["time_to_goal_s"]),
                path_length_m=float(r["path_length_m"]),
                stuck_time_s=float(r["stuck_time_s"]),
            ))
    return rows


def group_by(trials: List[TrialRow], *keys) -> Dict[tuple, List[TrialRow]]:
    groups = defaultdict(list)
    for t in trials:
        k = tuple(getattr(t, key) for key in keys)
        groups[k] = groups.get(k, [])
        groups[k].append(t)
    return groups


@dataclass
class GroupStats:
    n: int = 0
    success_rate: float = 0.0
    collision_rate: float = 0.0
    mean_min_sep: float = 0.0
    std_min_sep: float = 0.0
    mean_ttc_viol: float = 0.0
    mean_time: float = 0.0
    std_time: float = 0.0
    mean_path_len: float = 0.0
    mean_shield_pct: float = 0.0
    mean_stuck: float = 0.0


def compute_stats(trials: List[TrialRow]) -> GroupStats:
    n = len(trials)
    if n == 0:
        return GroupStats()

    successes = sum(1 for t in trials if t.success)
    collisions = sum(1 for t in trials if t.collision_count > 0)
    min_seps = [t.min_separation_m for t in trials if t.min_separation_m >= 0]
    ttc_viols = [t.ttc_violations for t in trials]
    times = [t.time_to_goal_s for t in trials if t.success]
    path_lens = [t.path_length_m for t in trials]
    shield_pcts = [t.shield_interventions_pct for t in trials]
    stuck_times = [t.stuck_time_s for t in trials]

    gs = GroupStats(n=n)
    gs.success_rate = 100.0 * successes / n
    gs.collision_rate = 100.0 * collisions / n
    gs.mean_min_sep = float(np.mean(min_seps)) if min_seps else -1.0
    gs.std_min_sep = float(np.std(min_seps)) if min_seps else 0.0
    gs.mean_ttc_viol = float(np.mean(ttc_viols))
    gs.mean_time = float(np.mean(times)) if times else 0.0
    gs.std_time = float(np.std(times)) if times else 0.0
    gs.mean_path_len = float(np.mean(path_lens))
    gs.mean_shield_pct = float(np.mean(shield_pcts))
    gs.mean_stuck = float(np.mean(stuck_times))
    return gs


def wilcoxon_test(
    trials_a: List[TrialRow],
    trials_b: List[TrialRow],
    metric: str,
) -> Tuple[float, float]:
    """Wilcoxon signed-rank test on paired trials (same seeds)."""
    seeds_a = {t.seed: t for t in trials_a}
    seeds_b = {t.seed: t for t in trials_b}
    common_seeds = sorted(set(seeds_a.keys()) & set(seeds_b.keys()))

    if len(common_seeds) < 5:
        return (np.nan, np.nan)

    vals_a = [getattr(seeds_a[s], metric) for s in common_seeds]
    vals_b = [getattr(seeds_b[s], metric) for s in common_seeds]

    diffs = [a - b for a, b in zip(vals_a, vals_b)]
    if all(d == 0 for d in diffs):
        return (0.0, 1.0)

    try:
        stat, p_val = sp_stats.wilcoxon(vals_a, vals_b)
        return (float(stat), float(p_val))
    except ValueError:
        return (np.nan, np.nan)


def fisher_exact_success(
    trials_a: List[TrialRow],
    trials_b: List[TrialRow],
) -> Tuple[float, float]:
    """Fisher's exact test on success rates."""
    s_a = sum(1 for t in trials_a if t.success)
    f_a = len(trials_a) - s_a
    s_b = sum(1 for t in trials_b if t.success)
    f_b = len(trials_b) - s_b

    table = [[s_a, f_a], [s_b, f_b]]
    try:
        odds, p_val = sp_stats.fisher_exact(table)
        return (float(odds), float(p_val))
    except ValueError:
        return (np.nan, np.nan)


def format_mean_std(mean: float, std: float, precision: int = 2) -> str:
    return f"${mean:.{precision}f} \\pm {std:.{precision}f}$"


def format_pct(val: float) -> str:
    return f"${val:.1f}\\%$"


CONTROLLER_ORDER = ["mppi_vanilla", "mppi_social_instant", "mppi_social_pred", "full"]
CONTROLLER_LABELS = {
    "mppi_vanilla": "MPPI",
    "mppi_social_instant": "MPPI+Social",
    "mppi_social_pred": "MPPI+Pred",
    "full": "Full (Ours)",
}

SCENARIO_ORDER = [
    "head_on_single", "head_on_group", "bidirectional",
    "overtaking", "doorway_popout", "junction_crossing",
]
SCENARIO_LABELS = {
    "head_on_single": "Head-on (1)",
    "head_on_group": "Head-on (3)",
    "bidirectional": "Bidir.",
    "overtaking": "Overtake",
    "doorway_popout": "Doorway",
    "junction_crossing": "Junction",
}


def generate_main_table(trials: List[TrialRow], out_dir: str):
    """Table 1: Main comparison across all scenarios (aggregated)."""
    by_ctrl = group_by(trials, "controller")

    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Main results: 4 controllers across all scenarios (N=30 per cell).}",
        r"\label{tab:main_results}",
        r"\begin{tabular}{l c c c c c}",
        r"\toprule",
        r"Controller & Succ.\,\% & Coll.\,\% & Min Sep (m) & TtG (s) & Shield\,\% \\",
        r"\midrule",
    ]

    for ctrl in CONTROLLER_ORDER:
        key = (ctrl,)
        if key not in by_ctrl:
            continue
        gs = compute_stats(by_ctrl[key])
        label = CONTROLLER_LABELS.get(ctrl, ctrl)
        lines.append(
            f"  {label} & {format_pct(gs.success_rate)} & {format_pct(gs.collision_rate)} & "
            f"{format_mean_std(gs.mean_min_sep, gs.std_min_sep)} & "
            f"{format_mean_std(gs.mean_time, gs.std_time)} & "
            f"{format_pct(gs.mean_shield_pct)} \\\\"
        )

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]

    path = os.path.join(out_dir, "table_main_results.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Wrote {path}")


def generate_per_scenario_table(trials: List[TrialRow], out_dir: str):
    """Table 2: Per-scenario breakdown."""
    by_sc_ctrl = group_by(trials, "scenario", "controller")

    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Per-scenario results.}",
        r"\label{tab:per_scenario}",
        r"\begin{tabular}{l l c c c c}",
        r"\toprule",
        r"Scenario & Controller & Succ.\,\% & Coll.\,\% & Min Sep (m) & TtG (s) \\",
        r"\midrule",
    ]

    for sc in SCENARIO_ORDER:
        sc_label = SCENARIO_LABELS.get(sc, sc)
        first = True
        for ctrl in CONTROLLER_ORDER:
            key = (sc, ctrl)
            if key not in by_sc_ctrl:
                continue
            gs = compute_stats(by_sc_ctrl[key])
            label = CONTROLLER_LABELS.get(ctrl, ctrl)
            sc_col = sc_label if first else ""
            lines.append(
                f"  {sc_col} & {label} & {format_pct(gs.success_rate)} & "
                f"{format_pct(gs.collision_rate)} & "
                f"{format_mean_std(gs.mean_min_sep, gs.std_min_sep)} & "
                f"{format_mean_std(gs.mean_time, gs.std_time)} \\\\"
            )
            first = False
        lines.append(r"\midrule")

    if lines[-1] == r"\midrule":
        lines[-1] = r"\bottomrule"

    lines += [
        r"\end{tabular}",
        r"\end{table*}",
    ]

    path = os.path.join(out_dir, "table_per_scenario.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Wrote {path}")


def generate_significance_table(trials: List[TrialRow], out_dir: str):
    """Table 3: Statistical significance (Full vs each baseline)."""
    by_ctrl = group_by(trials, "controller")
    full_key = ("full",)

    if full_key not in by_ctrl:
        print("Warning: 'full' controller not found, skipping significance table")
        return

    full_trials = by_ctrl[full_key]
    baselines = ["mppi_vanilla", "mppi_social_instant", "mppi_social_pred"]

    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Statistical significance: Full (Ours) vs baselines.}",
        r"\label{tab:significance}",
        r"\begin{tabular}{l c c c}",
        r"\toprule",
        r"Baseline & Fisher $p$ (success) & Wilcoxon $p$ (min sep) & Wilcoxon $p$ (TtG) \\",
        r"\midrule",
    ]

    for bl in baselines:
        bl_key = (bl,)
        if bl_key not in by_ctrl:
            continue
        bl_trials = by_ctrl[bl_key]

        _, p_fisher = fisher_exact_success(full_trials, bl_trials)
        _, p_sep = wilcoxon_test(full_trials, bl_trials, "min_separation_m")
        _, p_time = wilcoxon_test(full_trials, bl_trials, "time_to_goal_s")

        label = CONTROLLER_LABELS.get(bl, bl)

        def fmt_p(p):
            if np.isnan(p):
                return "---"
            if p < 0.001:
                return f"$<0.001$"
            return f"${p:.3f}$"

        lines.append(f"  {label} & {fmt_p(p_fisher)} & {fmt_p(p_sep)} & {fmt_p(p_time)} \\\\")

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]

    path = os.path.join(out_dir, "table_significance.tex")
    with open(path, "w") as f:
        f.write("\n".join(lines))
    print(f"Wrote {path}")


def generate_summary_csv(trials: List[TrialRow], out_dir: str):
    """Machine-readable summary CSV for further analysis."""
    by_sc_ctrl = group_by(trials, "scenario", "controller")
    path = os.path.join(out_dir, "summary_stats.csv")

    fieldnames = [
        "scenario", "controller", "n", "success_rate", "collision_rate",
        "mean_min_sep", "std_min_sep", "mean_ttc_viol",
        "mean_time", "std_time", "mean_path_len", "mean_shield_pct", "mean_stuck",
    ]

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for sc in SCENARIO_ORDER:
            for ctrl in CONTROLLER_ORDER:
                key = (sc, ctrl)
                if key not in by_sc_ctrl:
                    continue
                gs = compute_stats(by_sc_ctrl[key])
                writer.writerow({
                    "scenario": sc, "controller": ctrl, "n": gs.n,
                    "success_rate": round(gs.success_rate, 2),
                    "collision_rate": round(gs.collision_rate, 2),
                    "mean_min_sep": round(gs.mean_min_sep, 4),
                    "std_min_sep": round(gs.std_min_sep, 4),
                    "mean_ttc_viol": round(gs.mean_ttc_viol, 2),
                    "mean_time": round(gs.mean_time, 2),
                    "std_time": round(gs.std_time, 2),
                    "mean_path_len": round(gs.mean_path_len, 3),
                    "mean_shield_pct": round(gs.mean_shield_pct, 2),
                    "mean_stuck": round(gs.mean_stuck, 2),
                })
    print(f"Wrote {path}")


def main():
    parser = argparse.ArgumentParser(description="Analyze corridor social nav experiment results")
    parser.add_argument("--input", required=True, help="Path to all_trials.csv")
    parser.add_argument("--output-dir", default="results/tables", help="Output directory for tables")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: input file not found: {args.input}")
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    trials = load_trials(args.input)
    print(f"Loaded {len(trials)} trial rows from {args.input}")

    generate_main_table(trials, args.output_dir)
    generate_per_scenario_table(trials, args.output_dir)
    generate_significance_table(trials, args.output_dir)
    generate_summary_csv(trials, args.output_dir)

    print(f"\nAll tables written to {args.output_dir}/")


if __name__ == "__main__":
    main()
