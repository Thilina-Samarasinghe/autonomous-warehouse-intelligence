"""Slotting optimisation service used by the dashboard's Optimise view."""

from __future__ import annotations

import math
import time
from typing import Any

from pydantic import BaseModel, Field

from awi.api.session import SessionError, layout_payload
from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.layout import generate_layout
from awi.core.orders import TaskGenerator
from awi.core.rng import make_rng
from awi.experiments.registry import PLANNERS, make_assigner, make_planner
from awi.experiments.runner import deep_merge
from awi.metrics.collector import MetricsCollector, MetricsConfig
from awi.optimization.ga import GAConfig, genetic_slotting
from awi.optimization.slotting import SlottingConfig, SlottingProblem
from awi.planning.dijkstra import DistanceMaps

STRATEGIES = ("random", "closest", "ga")


class OptimizeRequest(BaseModel):
    config: dict[str, Any] = Field(default_factory=dict, description="SimConfig overrides")
    congestion_weight: float = Field(0.5, ge=0, le=10)
    generations: int = Field(150, ge=1, le=2000)
    population: int = Field(60, ge=4, le=500)
    validate_ticks: int = Field(0, ge=0, le=10_000, description="0 = skip simulation check")
    planner: str = "prioritized"


def _clean(v: float) -> float | None:
    return v if math.isfinite(v) else None


def optimize_slotting(req: OptimizeRequest) -> dict[str, Any]:
    """Run the GA, score random / closest / GA slotting on the proxy, and (if
    ``validate_ticks``) measure each in a real simulation on the same seed."""
    if req.planner not in PLANNERS:
        raise SessionError(f"unknown planner {req.planner!r}")
    try:
        cfg = SimConfig.model_validate(deep_merge(SimConfig().model_dump(), req.config))
    except ValueError as exc:
        raise SessionError(str(exc)) from None
    layout = generate_layout(cfg.layout)
    maps = DistanceMaps.for_layout(layout)
    problem = SlottingProblem(layout, maps, cfg.zipf_s, req.congestion_weight)

    t0 = time.perf_counter()
    ga = genetic_slotting(
        problem,
        GAConfig(population=req.population, generations=req.generations),
        make_rng(cfg.seed, "slotting"),
    )
    ga_seconds = time.perf_counter() - t0
    # "random" is exactly the slotting the simulator draws for this seed.
    rand = TaskGenerator(layout, make_rng(cfg.seed, "tasks"), rate=1.0, zipf_s=cfg.zipf_s)
    assignments = {
        "random": rand.random_assignment,
        "closest": problem.closest_assignment(),
        "ga": ga.best,
    }
    strategies: dict[str, Any] = {}
    for name, assignment in assignments.items():
        score = problem.score(assignment)
        strategies[name] = {
            "fitness": score.fitness,
            "travel": score.travel,
            "concentration": score.concentration,
            "demand": problem.shelf_probabilities(assignment).tolist(),
        }

    if req.validate_ticks:
        warmup = min(300, req.validate_ticks // 5)
        for name in STRATEGIES:
            slot = SlottingConfig(
                strategy=name,
                congestion_weight=req.congestion_weight,
                generations=req.generations,
                population=req.population,
            )
            run_cfg = cfg.model_copy(update={"slotting": slot})
            sim = Simulation(
                run_cfg,
                planner=make_planner(req.planner, run_cfg),
                assigner=make_assigner("hungarian", run_cfg),
            )
            metrics = MetricsCollector(sim, MetricsConfig(warmup_ticks=warmup))
            for _ in range(req.validate_ticks):
                metrics.observe(sim.step())
            r = metrics.result()
            strategies[name]["simulation"] = {
                "throughput_per_1k": _clean(r.throughput_per_1k),
                "delivery_time_mean": _clean(r.delivery_time_mean),
                "distance_per_delivery": _clean(r.distance_per_delivery),
                "conflicts_total": r.conflicts_total,
            }

    return {
        "layout": layout_payload(layout),
        "shelves": [[r, c] for r, c in layout.shelves],
        "n_aisles": problem.n_aisles,
        "congestion_weight": req.congestion_weight,
        "strategies": strategies,
        "history": {"best": ga.history_best, "mean": ga.history_mean},
        "evaluations": ga.evaluations,
        "ga_seconds": ga_seconds,
        "planner": req.planner,
        "validate_ticks": req.validate_ticks,
    }
