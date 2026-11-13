"""Headless batch experiment runner.

An :class:`ExperimentSpec` expands into the cross product
``planners x robot counts x seeds``. Every planner sees *the same seeds*, so the
task stream and robot spawn positions are identical across planners, and results
can be compared pairwise. Runs execute in parallel processes and the results are
written to Parquet::

    results/<name>/runs.parquet         one row per run (KPIs + run parameters)
    results/<name>/timeseries.parquet   one row per (run, window)
    results/<name>/spec.yaml            the spec, for reproducibility
"""

from __future__ import annotations

import os
import time
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd
import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.experiments.registry import PLANNERS, make_planner
from awi.metrics.collector import MetricsCollector, MetricsConfig


class ExperimentSpec(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(pattern=r"^[A-Za-z0-9_\-]+$")
    description: str = ""
    planners: list[str] = Field(min_length=1)
    robots: list[int] = Field(default_factory=lambda: [10], min_length=1)
    seeds: int | list[int] = Field(10, description="Count (0..n-1) or explicit list")
    ticks: int = Field(3000, ge=1)
    warmup_ticks: int = Field(300, ge=0)
    window: int = Field(100, ge=1)
    deadlock_ticks: int = Field(20, ge=1)
    task_rate_per_robot: float | None = Field(
        None, gt=0, description="If set, task_rate = this x num_robots (keeps load per robot fixed)"
    )
    base: SimConfig = Field(default_factory=SimConfig)

    @field_validator("planners")
    @classmethod
    def _known_planners(cls, v: list[str]) -> list[str]:
        unknown = [p for p in v if p not in PLANNERS]
        if unknown:
            raise ValueError(f"unknown planners {unknown}; choose from {sorted(PLANNERS)}")
        return v

    @property
    def seed_list(self) -> list[int]:
        return list(range(self.seeds)) if isinstance(self.seeds, int) else list(self.seeds)

    @classmethod
    def from_yaml(cls, path: str | Path) -> ExperimentSpec:
        return cls.model_validate(yaml.safe_load(Path(path).read_text(encoding="utf-8")))

    def runs(self) -> list[RunSpec]:
        out: list[RunSpec] = []
        for n in self.robots:
            for seed in self.seed_list:
                overrides: dict[str, Any] = {"num_robots": n, "seed": seed}
                if self.task_rate_per_robot is not None:
                    overrides["task_rate"] = self.task_rate_per_robot * n
                cfg = self.base.model_copy(update=overrides)
                SimConfig.model_validate(cfg.model_dump())  # re-validate overrides
                for planner in self.planners:
                    out.append(
                        RunSpec(
                            experiment=self.name,
                            planner=planner,
                            config=cfg.model_dump(),
                            ticks=self.ticks,
                            metrics=MetricsConfig(
                                warmup_ticks=self.warmup_ticks,
                                window=self.window,
                                deadlock_ticks=self.deadlock_ticks,
                            ),
                        )
                    )
        return out


@dataclass(frozen=True, slots=True)
class RunSpec:
    """One simulation run; plain data so it can be sent to worker processes."""

    experiment: str
    planner: str
    config: dict[str, Any]
    ticks: int
    metrics: MetricsConfig


@dataclass(slots=True)
class RunResult:
    row: dict[str, Any]
    series: list[dict[str, Any]]


@dataclass(slots=True)
class ExperimentResult:
    runs: pd.DataFrame
    timeseries: pd.DataFrame
    out_dir: Path | None


def run_single(spec: RunSpec) -> RunResult:
    cfg = SimConfig.model_validate(spec.config)
    sim = Simulation(cfg, planner=make_planner(spec.planner, cfg))
    collector = MetricsCollector(sim, spec.metrics)
    t0 = time.perf_counter()
    for _ in range(spec.ticks):
        collector.observe(sim.step())
    wall = time.perf_counter() - t0

    keys = {
        "experiment": spec.experiment,
        "planner": spec.planner,
        "num_robots": cfg.num_robots,
        "seed": cfg.seed,
        "task_rate": cfg.task_rate,
    }
    row = {
        **keys,
        "ticks": spec.ticks,
        **collector.result().as_dict(),
        **_planner_stats(sim.planner),
        "wall_seconds": wall,
    }
    series = [{**keys, **w} for w in collector.timeseries()]
    return RunResult(row=row, series=series)


def _planner_stats(planner: object) -> dict[str, float]:
    """Coordinated-planner health: how often CBS fell back / a plan failed."""
    stats = getattr(planner, "stats", None)
    calls = getattr(stats, "calls", 0)
    if not calls:
        return {"fallback_rate": 0.0, "failure_rate": 0.0}
    return {
        "fallback_rate": getattr(stats, "fallbacks", 0) / calls,
        "failure_rate": getattr(stats, "failures", 0) / calls,
    }


def run_experiment(
    spec: ExperimentSpec,
    out_dir: str | Path | None = "results",
    workers: int | None = None,
    progress: Callable[[int, int, RunResult], None] | None = None,
) -> ExperimentResult:
    """Run every run of ``spec``; ``workers=1`` runs in-process (easier debugging)."""
    runs = spec.runs()
    workers = workers or max(1, min(len(runs), (os.cpu_count() or 2) - 2))
    results: list[RunResult] = []

    if workers == 1:
        for i, run in enumerate(runs, 1):
            res = run_single(run)
            results.append(res)
            if progress:
                progress(i, len(runs), res)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = [pool.submit(run_single, run) for run in runs]
            for i, fut in enumerate(as_completed(futures), 1):
                res = fut.result()
                results.append(res)
                if progress:
                    progress(i, len(runs), res)

    order = ["num_robots", "seed", "planner"]
    runs_df = pd.DataFrame([r.row for r in results]).sort_values(order, ignore_index=True)
    ts_df = pd.DataFrame([w for r in results for w in r.series])
    if not ts_df.empty:
        ts_df = ts_df.sort_values([*order, "t_end"], ignore_index=True)

    target: Path | None = None
    if out_dir is not None:
        target = Path(out_dir) / spec.name
        target.mkdir(parents=True, exist_ok=True)
        runs_df.to_parquet(target / "runs.parquet", index=False)
        ts_df.to_parquet(target / "timeseries.parquet", index=False)
        (target / "spec.yaml").write_text(
            yaml.safe_dump(spec.model_dump(mode="json"), sort_keys=False), encoding="utf-8"
        )
    return ExperimentResult(runs=runs_df, timeseries=ts_df, out_dir=target)
