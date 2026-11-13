"""Independent planning: per-robot A* with exact Dijkstra heuristics.

This is the *baseline* side of the main experiment. Every robot finds its own
optimal path and ignores the others; conflicts are only handled reactively (wait,
detour, yield) by :class:`~awi.planning.reactive.ReactivePlanner`.
"""

from __future__ import annotations

from collections.abc import Collection

from awi.core.grid import CellType
from awi.core.types import Cell
from awi.core.world import World
from awi.planning.astar import astar
from awi.planning.dijkstra import DistanceMaps, make_cell_costs
from awi.planning.reactive import ReactivePlanner


class IndependentAStarPlanner(ReactivePlanner):
    rng_stream = "planner.astar"

    def __init__(self, seed: int = 0, yield_prob: float = 0.5, pick_cell_cost: float = 1.0) -> None:
        """
        Args:
            pick_cell_cost: cost of entering a pick-aisle cell. Values above 1 steer
                through-traffic onto the main and cross aisles.
        """
        super().__init__(seed=seed, yield_prob=yield_prob)
        if pick_cell_cost <= 0:
            raise ValueError("pick_cell_cost must be positive")
        self.pick_cell_cost = pick_cell_cost
        self._maps: DistanceMaps | None = None

    def distance_maps(self, world: World) -> DistanceMaps:
        maps = self._maps
        if maps is None or maps.grid is not world.layout.grid:
            if self.pick_cell_cost == 1.0:
                maps = world.distances
            else:
                costs = make_cell_costs(world.layout.grid, {CellType.PICK: self.pick_cell_cost})
                maps = DistanceMaps.for_layout(world.layout, costs)
            self._maps = maps
        return maps

    def shortest_path(
        self, world: World, start: Cell, goal: Cell, blocked: Collection[Cell] = ()
    ) -> list[Cell] | None:
        maps = self.distance_maps(world)
        result = astar(
            world.layout.grid,
            start,
            goal,
            heuristic=maps.to(goal),
            cell_cost=None if maps.is_uniform else maps.cell_cost,
            blocked=blocked,
        )
        self.stats.expanded += result.expanded
        return result.path
