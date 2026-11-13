"""Single-agent A* search on the static grid."""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Callable, Collection
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from awi.core.grid import Grid
from awi.core.types import Cell, manhattan


@dataclass(frozen=True, slots=True)
class SearchResult:
    path: list[Cell] | None
    """Cells after ``start`` up to and including ``goal``; ``None`` if unreachable."""
    cost: float
    expanded: int
    """Nodes popped and expanded: the search-effort metric."""


def astar(
    grid: Grid,
    start: Cell,
    goal: Cell,
    *,
    heuristic: NDArray[np.float64] | None = None,
    cell_cost: NDArray[np.float64] | None = None,
    blocked: Collection[Cell] = (),
) -> SearchResult:
    """A* from ``start`` to ``goal``.

    Args:
        heuristic: cost-to-go map for ``goal`` (e.g. ``DistanceMaps.to(goal)``). It
            must be admissible. Defaults to Manhattan distance scaled by the cheapest
            cell cost. ``inf`` entries are pruned as dead ends.
        cell_cost: cost of entering each cell (default 1 everywhere).
        blocked: cells to avoid (the goal itself is always allowed).

    Ties on ``f`` prefer the node closer to the goal (lower ``h``), then insertion
    order. This keeps the search deterministic and, with an exact heuristic, linear in
    the path length.
    """
    if not (grid.is_passable(start) and grid.is_passable(goal)):
        return SearchResult(None, math.inf, 0)
    if start == goal:
        return SearchResult([], 0.0, 0)

    h = _make_heuristic(goal, heuristic, cell_cost)
    counter = itertools.count()
    h0 = h(start)
    if math.isinf(h0):
        return SearchResult(None, math.inf, 0)
    open_heap: list[tuple[float, float, int, Cell]] = [(h0, h0, next(counter), start)]
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    expanded = 0

    while open_heap:
        _, _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            return SearchResult(_reconstruct(parent, goal), g[goal], expanded)
        closed.add(cur)
        expanded += 1
        g_cur = g[cur]
        for nb in grid.neighbors(cur):
            if nb in closed or (nb in blocked and nb != goal):
                continue
            ng = g_cur + (1.0 if cell_cost is None else float(cell_cost[nb]))
            if ng < g.get(nb, math.inf):
                h_nb = h(nb)
                if math.isinf(h_nb):
                    continue
                g[nb] = ng
                parent[nb] = cur
                heapq.heappush(open_heap, (ng + h_nb, h_nb, next(counter), nb))
    return SearchResult(None, math.inf, expanded)


def _make_heuristic(
    goal: Cell, heuristic: NDArray[np.float64] | None, cell_cost: NDArray[np.float64] | None
) -> Callable[[Cell], float]:
    if heuristic is not None:
        h_map = heuristic
        return lambda c: float(h_map[c])
    scale = 1.0 if cell_cost is None else float(np.min(cell_cost))
    return lambda c: scale * manhattan(c, goal)


def _reconstruct(parent: dict[Cell, Cell], goal: Cell) -> list[Cell]:
    """Walk parents back from ``goal``; the result excludes the start cell."""
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    path.pop()
    path.reverse()
    return path
