"""Reference planner: reactive behaviour on plain breadth-first search.

Kept as a simple, obviously correct baseline to cross-check the A* planner.
"""

from __future__ import annotations

from collections.abc import Collection

from awi.core.types import Cell
from awi.core.world import World
from awi.planning.reactive import ReactivePlanner


class BFSPlanner(ReactivePlanner):
    rng_stream = "planner.bfs"

    def shortest_path(
        self, world: World, start: Cell, goal: Cell, blocked: Collection[Cell] = ()
    ) -> list[Cell] | None:
        return world.layout.grid.bfs_path(start, goal, blocked)
