"""Experiment CLI.

uv run awi-exp run scenarios/experiments/baseline_sweep.yaml --workers 4
uv run awi-exp report results/baseline_sweep --a independent_astar --b independent_bfs
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence
from pathlib import Path

import pandas as pd

from awi.experiments.runner import ExperimentSpec, RunResult, run_experiment
from awi.experiments.stats import compare, summarize

KEY_METRICS = [
    "throughput_per_1k",
    "delivery_time_mean",
    "conflicts_total",
    "distance_per_delivery",
    "idle_frac",
    "blocked_frac",
    "deadlock_events",
    "plan_ms_mean",
]


def strategy_column(runs: pd.DataFrame) -> str:
    """The column that tells strategies apart: planner, assigner, or both combined."""
    if "assigner" not in runs:
        return "planner"
    multi_planner = runs["planner"].nunique() > 1
    multi_assigner = runs["assigner"].nunique() > 1
    if multi_planner and multi_assigner:
        runs["strategy"] = runs["planner"] + "+" + runs["assigner"]
        return "strategy"
    return "assigner" if multi_assigner else "planner"


def _print_report(runs: pd.DataFrame, a: str | None, b: str | None) -> None:
    col = strategy_column(runs)
    summary = summarize(runs, KEY_METRICS, by=("num_robots", col))
    table = summary.pivot_table(
        index=["num_robots", col], columns="metric", values="mean", sort=False
    )[KEY_METRICS]
    with pd.option_context("display.width", 200, "display.max_columns", 20, "display.precision", 3):
        print(f"\nMean per (robots, {col}):")
        print(table.to_string())
        strategies = sorted(runs[col].unique())
        if a is None and b is None and len(strategies) == 2:
            a, b = strategies
        if a and b:
            cmp = compare(runs, a, b, KEY_METRICS, group_col=col)
            print(f"\nPaired comparison {a} vs {b} (diff = {a} - {b}):")
            cols = [
                "num_robots",
                "metric",
                "n_pairs",
                "diff",
                "rel_change",
                "wilcoxon_p",
                "cliffs_delta",
            ]
            print(cmp[cols].to_string(index=False))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="awi-exp", description="Batch experiments")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="run an experiment spec")
    p_run.add_argument("spec", type=Path)
    p_run.add_argument("--workers", type=int, default=None)
    p_run.add_argument("--out", type=Path, default=Path("results"))
    p_run.add_argument("--seeds", type=int, help="override the number of seeds")
    p_run.add_argument("--ticks", type=int, help="override ticks per run")

    p_rep = sub.add_parser("report", help="summarize a results directory")
    p_rep.add_argument("results", type=Path)
    for p in (p_run, p_rep):
        p.add_argument("--a", help="strategy A for the paired comparison")
        p.add_argument("--b", help="strategy B for the paired comparison")

    args = parser.parse_args(argv)
    if args.cmd == "run":
        spec = ExperimentSpec.from_yaml(args.spec)
        overrides = {k: v for k, v in (("seeds", args.seeds), ("ticks", args.ticks)) if v}
        if overrides:
            spec = ExperimentSpec.model_validate(spec.model_dump() | overrides)
        total = len(spec.runs())
        print(f"experiment {spec.name}: {total} runs")
        t0 = time.perf_counter()

        def progress(i: int, n: int, res: RunResult) -> None:
            r = res.row
            print(
                f"  [{i:>3}/{n}] {r['planner']:<18} {r['assigner']:<10} "
                f"robots={r['num_robots']:<3} "
                f"seed={r['seed']:<3} throughput={r['throughput_per_1k']:.1f}/1k "
                f"({r['wall_seconds']:.1f}s)",
                flush=True,
            )

        result = run_experiment(spec, args.out, workers=args.workers, progress=progress)
        print(f"done in {time.perf_counter() - t0:.1f}s -> {result.out_dir}")
        _print_report(result.runs, args.a, args.b)
    else:
        path = args.results / "runs.parquet"
        if not path.exists():
            print(f"no runs.parquet in {args.results}", file=sys.stderr)
            return 1
        _print_report(pd.read_parquet(path), args.a, args.b)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
