"""Independent, reactive planning behaviour shared by the single-agent planners.

* Every robot plans its own optimal path as if the warehouse were empty.
* When a robot has been blocked for a while (``PathRequest.stuck``), it first looks
  for a **detour** around the robots currently in its way. If no useful detour
  exists, it **yields** with probability ``yield_prob``: it steps to a random free
  neighbour, then replans from there.

Random yielding breaks symmetric deadlocks. Example: a robot standing on a drop
station wants to leave while queued robots occupy every exit, all waiting for that
same station. Without it, independent fleets gridlock permanently. The random stream
is seeded, so runs stay reproducible.

What this behaviour *cannot* solve is a corridor packed from both ends, where no
robot has a free cell to retreat into. That needs coordinated planning (phase 4).
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections.abc import Collection, Sequence
from dataclasses import dataclass

from awi.core.rng import make_rng
from awi.core.types import Cell, RobotId
from awi.core.world import World
from awi.planning.base import PathRequest


@dataclass(slots=True)
class PlannerStats:
    calls: int = 0
    paths: int = 0
    expanded: int = 0
    seconds: float = 0.0


class ReactivePlanner(ABC):
    rng_stream = "planner.reactive"

    def __init__(self, seed: int = 0, yield_prob: float = 0.5) -> None:
        if not 0.0 <= yield_prob <= 1.0:
            raise ValueError("yield_prob must be in [0, 1]")
        self._rng = make_rng(seed, self.rng_stream)
        self.yield_prob = yield_prob
        self.stats = PlannerStats()

    @abstractmethod
    def shortest_path(
        self, world: World, start: Cell, goal: Cell, blocked: Collection[Cell] = ()
    ) -> list[Cell] | None:
        """Optimal single-agent path (excluding ``start``) or ``None``."""

    def plan(self, world: World, requests: Sequence[PathRequest]) -> dict[RobotId, list[Cell]]:
        t0 = time.perf_counter()
        grid = world.layout.grid
        occupied = world.occupancy()
        paths: dict[RobotId, list[Cell]] = {}
        for req in requests:
            path: list[Cell] | None = None
            if req.stuck:
                blocked = {c for c, rid in occupied.items() if rid != req.robot_id}
                path = self.shortest_path(world, req.start, req.goal, blocked)
                if path and path[0] in occupied:
                    path = None  # only "detour" is into an occupied goal: useless
                if path is None:
                    free = [nb for nb in grid.neighbors(req.start) if nb not in occupied]
                    if free and self._rng.random() < self.yield_prob:
                        step = free[int(self._rng.integers(len(free)))]
                        rest = self.shortest_path(world, step, req.goal) or []
                        paths[req.robot_id] = [step, *rest]
                        continue
            if path is None:
                path = self.shortest_path(world, req.start, req.goal)
            paths[req.robot_id] = path or []
        self.stats.calls += 1
        self.stats.paths += len(requests)
        self.stats.seconds += time.perf_counter() - t0
        return paths
