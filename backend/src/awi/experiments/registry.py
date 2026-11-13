"""Named strategy factories, so experiment specs can refer to planners by string."""

from __future__ import annotations

from collections.abc import Callable

from awi.core.config import SimConfig
from awi.planning.base import PathPlanner
from awi.planning.independent import IndependentAStarPlanner
from awi.planning.naive import BFSPlanner

PlannerFactory = Callable[[SimConfig], PathPlanner]

PLANNERS: dict[str, PlannerFactory] = {
    "independent_astar": lambda cfg: IndependentAStarPlanner(seed=cfg.seed),
    "independent_bfs": lambda cfg: BFSPlanner(seed=cfg.seed),
}


def register_planner(name: str, factory: PlannerFactory) -> None:
    if name in PLANNERS:
        raise ValueError(f"planner {name!r} already registered")
    PLANNERS[name] = factory


def make_planner(name: str, cfg: SimConfig) -> PathPlanner:
    try:
        factory = PLANNERS[name]
    except KeyError:
        raise ValueError(f"unknown planner {name!r}; choose from {sorted(PLANNERS)}") from None
    return factory(cfg)
