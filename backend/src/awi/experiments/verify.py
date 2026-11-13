"""Reproducibility check: re-run recorded experiment rows and compare exactly.

Every run is fully determined by its spec and seed, so re-running it with the same
code must give *identical* metrics. ``awi-exp verify results/<name>`` samples
recorded rows, rebuilds their :class:`RunSpec` from the saved ``spec.yaml``, re-runs
them and reports any difference. Timing columns (wall time, planner milliseconds)
are machine-dependent and are excluded.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from awi.experiments.runner import ExperimentSpec, RunSpec, run_single

TIMING_COLUMNS = {"wall_seconds", "plan_ms_total", "plan_ms_mean", "plan_ms_max"}


@dataclass(slots=True)
class VerifyReport:
    checked: int = 0
    mismatches: list[dict[str, Any]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.checked > 0 and not self.mismatches


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, float) or isinstance(b, float):
        fa, fb = float(a), float(b)
        return (math.isnan(fa) and math.isnan(fb)) or math.isclose(
            fa, fb, rel_tol=1e-9, abs_tol=1e-9
        )
    return bool(a == b)


def _find_run(spec: ExperimentSpec, row: pd.Series) -> RunSpec:
    for run in spec.runs():
        if (
            run.planner == row["planner"]
            and run.assigner == row.get("assigner", "greedy")
            and run.variant == row.get("variant", "default")
            and run.config["num_robots"] == row["num_robots"]
            and run.config["seed"] == row["seed"]
        ):
            return run
    raise KeyError(f"no run in spec for row {dict(row)}")


def verify_results(directory: Path, sample: int = 5, seed: int = 0) -> VerifyReport:
    spec = ExperimentSpec.from_yaml(directory / "spec.yaml")
    runs = pd.read_parquet(directory / "runs.parquet")
    rng = np.random.default_rng(seed)
    picks = rng.choice(len(runs), size=min(sample, len(runs)), replace=False)
    report = VerifyReport()
    for idx in sorted(picks.tolist()):
        recorded = runs.iloc[idx]
        fresh = run_single(_find_run(spec, recorded)).row
        report.checked += 1
        for col, value in fresh.items():
            if col in TIMING_COLUMNS or col not in recorded.index:
                continue
            if not _same(value, recorded[col]):
                report.mismatches.append(
                    {"row": int(idx), "column": col, "recorded": recorded[col], "now": value}
                )
    return report
