"""Reference planner: per-robot BFS shortest path that ignores other robots.

This is the *independent, reactive* behaviour of a naive fleet:

* Every robot plans its own shortest path as if the warehouse were empty.
* When a robot has been blocked for a while (``PathRequest.stuck``), it either
  **yields** (steps to a random free neighbour, then replans) with probability
  ``yield_prob``, or tries a **detour** around the robots currently in its way.

Random yielding is what breaks symmetric deadlocks. Example: a robot standing
on a drop station wants to leave while queued robots occupy every exit and wait
for that same station. Without it, independent fleets can gridlock permanently.
The random stream is seeded, so runs stay reproducible.

Phase 2 replaces BFS with A* on cached Dijkstra heuristics, keeping this behaviour.
"""

from __future__ import annotations

from collections.abc import Sequence

from awi.core.rng import make_rng
from awi.core.types import Cell, RobotId
from awi.core.world import World
from awi.planning.base import PathRequest


class BFSPlanner:
    def __init__(self, seed: int = 0, yield_prob: float = 0.5) -> None:
        if not 0.0 <= yield_prob <= 1.0:
            raise ValueError("yield_prob must be in [0, 1]")
        self._rng = make_rng(seed, "planner.bfs")
        self.yield_prob = yield_prob

    def plan(self, world: World, requests: Sequence[PathRequest]) -> dict[RobotId, list[Cell]]:
        grid = world.layout.grid
        occupied = world.occupancy()
        paths: dict[RobotId, list[Cell]] = {}
        for req in requests:
            path: list[Cell] | None = None
            if req.stuck:
                blocked = {c for c, rid in occupied.items() if rid != req.robot_id}
                path = grid.bfs_path(req.start, req.goal, blocked)
                if path and path[0] in occupied:
                    path = None  # only "detour" is into an occupied goal: useless
                if path is None:
                    free = [nb for nb in grid.neighbors(req.start) if nb not in occupied]
                    if free and self._rng.random() < self.yield_prob:
                        step = free[int(self._rng.integers(len(free)))]
                        paths[req.robot_id] = [step, *(grid.bfs_path(step, req.goal) or [])]
                        continue
            if path is None:
                path = grid.bfs_path(req.start, req.goal)
            paths[req.robot_id] = path or []
        return paths
