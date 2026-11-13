"""Exact cost-to-go maps via (reverse) Dijkstra on a NetworkX graph of the grid.

For every goal cell (pick point, drop station, charger...) we compute the cheapest
cost from *every* cell to that goal once, then cache it as a NumPy array. These maps
serve as:

* the A* heuristic, which is exact on the static map, so A* expands almost only
  cells on an optimal path;
* true travel costs for task assignment (Hungarian / CP-SAT, phase 5);
* the fitness proxy of the layout genetic algorithm (phase 10).

Costs are *per cell entered*: moving into cell ``c`` costs ``cell_cost[c]``. The
default cost is 1 everywhere (so maps hold step counts). A higher cost for pick-aisle
cells makes robots prefer the main and cross aisles for through-traffic.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

import networkx as nx
import numpy as np
from numpy.typing import NDArray

from awi.core.grid import CellType, Grid
from awi.core.layout import Layout
from awi.core.types import Cell


def make_cell_costs(
    grid: Grid, overrides: Mapping[CellType, float] | None = None
) -> NDArray[np.float64]:
    """Cost of entering each cell: 1.0 by default, ``overrides`` per cell type."""
    costs = np.ones(grid.cells.shape, dtype=np.float64)
    for cell_type, cost in (overrides or {}).items():
        if cost <= 0:
            raise ValueError("cell costs must be positive")
        costs[grid.cells == cell_type] = cost
    costs[grid.cells == CellType.SHELF] = np.inf
    return costs


def grid_to_graph(grid: Grid, cell_cost: NDArray[np.float64] | None = None) -> nx.DiGraph:
    """Directed graph of passable cells; edge ``u -> v`` weighs ``cell_cost[v]``."""
    graph = nx.DiGraph()
    graph.add_nodes_from(grid.passable_cells())
    for u in graph.nodes:
        for v in grid.neighbors(u):
            graph.add_edge(u, v, weight=1.0 if cell_cost is None else float(cell_cost[v]))
    return graph


class DistanceMaps:
    """Lazily computed, cached cost-to-go maps for one grid and cost model."""

    def __init__(self, grid: Grid, cell_cost: NDArray[np.float64] | None = None) -> None:
        if cell_cost is None:
            cell_cost = make_cell_costs(grid)
        if cell_cost.shape != grid.cells.shape:
            raise ValueError("cell_cost must match the grid shape")
        passable = grid.cells != CellType.SHELF
        if not np.all(np.isfinite(cell_cost[passable]) & (cell_cost[passable] > 0)):
            raise ValueError("passable cells need finite positive costs")
        self.grid = grid
        self.cell_cost: NDArray[np.float64] = cell_cost.copy()
        self.cell_cost.flags.writeable = False
        self.is_uniform = bool(np.all(cell_cost[passable] == 1.0))
        # Reverse the graph: Dijkstra *from* the goal on reversed edges gives the
        # cost *to* the goal from every cell.
        self._reverse = grid_to_graph(grid, self.cell_cost).reverse(copy=True)
        self._maps: dict[Cell, NDArray[np.float64]] = {}

    @classmethod
    def for_layout(
        cls, layout: Layout, cell_cost: NDArray[np.float64] | None = None
    ) -> DistanceMaps:
        """Maps with every pick point, drop station and charger precomputed."""
        maps = cls(layout.grid, cell_cost)
        maps.precompute(
            [*sorted(set(layout.pick_point.values())), *layout.drop_stations, *layout.chargers]
        )
        return maps

    def precompute(self, goals: Iterable[Cell]) -> None:
        for goal in goals:
            self.to(goal)

    def to(self, goal: Cell) -> NDArray[np.float64]:
        """Cost from every cell to ``goal`` (``inf`` if unreachable or a shelf)."""
        cached = self._maps.get(goal)
        if cached is not None:
            return cached
        dist = np.full(self.grid.cells.shape, np.inf, dtype=np.float64)
        if goal in self._reverse:
            lengths = nx.single_source_dijkstra_path_length(self._reverse, goal, weight="weight")
            rows, cols = zip(*lengths.keys(), strict=True)
            dist[list(rows), list(cols)] = list(lengths.values())
        dist.flags.writeable = False
        self._maps[goal] = dist
        return dist

    def distance(self, start: Cell, goal: Cell) -> float:
        return float(self.to(goal)[start])

    @property
    def cached_goals(self) -> int:
        return len(self._maps)
