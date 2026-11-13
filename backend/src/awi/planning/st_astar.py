"""Windowed Space-Time A*: the low-level search of CBS and Prioritized Planning.

The search runs over states ``(cell, t)`` with five actions: four moves plus wait.
Planning is *windowed*: only the first ``horizon`` ticks are checked against other
robots. A path either

* reaches the goal at time ``t`` and can **stay there** until the horizon, because
  no constraint forbids the goal cell later (cost ``t``), or
* is cut at ``t == horizon``, with the static cost-to-go estimate as the remainder
  (cost ``horizon + h(cell)``).

The heuristic ``h`` is the exact static distance (Dijkstra map), so ``f = t + h`` is
admissible, and the first terminal node popped is optimal under the constraints.

Constraint inputs (all optional):

* ``vertex``: ``(cell, t)`` pairs the robot must not occupy.
* ``moves``: ``(from, to, t)`` moves the robot must not make (arriving at ``t``).
* ``blocked_from``: ``cell -> t0``, the cell is unusable for every ``t >= t0``.
  Used for static robots (``t0 = 0``) and for robots parked at the end of their
  path in Prioritized Planning.
"""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Container, Mapping
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from awi.core.grid import Grid
from awi.core.types import Cell

_EMPTY: frozenset[tuple[Cell, int]] = frozenset()
_EMPTY_MOVES: frozenset[tuple[Cell, Cell, int]] = frozenset()
_NO_BLOCKS: Mapping[Cell, int] = {}

type _State = tuple[Cell, int]


@dataclass(frozen=True, slots=True)
class STPath:
    cells: list[Cell]
    """Positions for ``t = 0 .. len-1`` (``cells[0]`` is the start)."""
    cost: float
    expanded: int


def space_time_astar(
    grid: Grid,
    start: Cell,
    goal: Cell,
    heuristic: NDArray[np.float64],
    horizon: int,
    *,
    vertex: Container[tuple[Cell, int]] = _EMPTY,
    moves: Container[tuple[Cell, Cell, int]] = _EMPTY_MOVES,
    blocked_from: Mapping[Cell, int] = _NO_BLOCKS,
    absorbing_goal: bool = False,
    avoid: Mapping[tuple[Cell, int], int] | None = None,
    max_expansions: int = 50_000,
) -> STPath | None:
    """Optimal windowed path under constraints, or ``None`` if none exists in budget.

    With ``absorbing_goal`` the goal may only be entered when the robot can stay
    there until the horizon. Task robots need this because the engine starts
    picking or dropping as soon as a robot stands on its goal, so passing through
    the goal is not possible.

    ``avoid`` is a *conflict-avoidance table* of other robots' planned positions,
    ``(cell, t) -> count``. Among equally short paths, the one with the fewest
    soft collisions wins. This tie-break greatly reduces the number of CBS nodes.
    """
    if horizon < 1:
        raise ValueError("horizon must be >= 1")

    def free(cell: Cell, t: int) -> bool:
        b = blocked_from.get(cell)
        return (b is None or t < b) and (cell, t) not in vertex

    def can_stay(cell: Cell, t: int) -> bool:
        b = blocked_from.get(cell)
        if b is not None and b <= horizon:
            return False
        return all((cell, k) not in vertex for k in range(t + 1, horizon + 1))

    h0 = float(heuristic[start])
    if math.isinf(h0):
        return None
    counter = itertools.count()
    # (f, soft conflicts, h, -t, tie, cell, t): shortest first, then fewest soft
    # conflicts, then deepest, then insertion order.
    heap: list[tuple[float, int, float, int, int, Cell, int]] = [
        (h0, 0, h0, 0, next(counter), start, 0)
    ]
    parent: dict[_State, _State] = {}
    best_soft: dict[_State, int] = {(start, 0): 0}
    closed: set[_State] = set()
    expanded = 0

    while heap:
        f, soft, _, _, _, cell, t = heapq.heappop(heap)
        state = (cell, t)
        if state in closed:
            continue
        at_goal = cell == goal and can_stay(cell, t)
        if at_goal or t == horizon:
            return STPath(_reconstruct(parent, state), float(t) if at_goal else f, expanded)
        closed.add(state)
        expanded += 1
        if expanded > max_expansions:
            return None
        nt = t + 1
        for nb in (*grid.neighbors(cell), cell):
            nxt = (nb, nt)
            if nxt in closed or not free(nb, nt):
                continue
            if nb != cell and (cell, nb, nt) in moves:
                continue
            if absorbing_goal and nb == goal and not can_stay(nb, nt):
                continue
            h_nb = float(heuristic[nb])
            if math.isinf(h_nb):
                continue
            n_soft = soft + (avoid.get(nxt, 0) if avoid else 0)
            if best_soft.get(nxt, n_soft + 1) <= n_soft:
                continue  # same g (= t) already reached with no more soft conflicts
            best_soft[nxt] = n_soft
            parent[nxt] = state
            heapq.heappush(heap, (nt + h_nb, n_soft, h_nb, -nt, next(counter), nb, nt))
    return None


def _reconstruct(parent: dict[_State, _State], end: _State) -> list[Cell]:
    states = [end]
    while states[-1] in parent:
        states.append(parent[states[-1]])
    states.reverse()
    return [cell for cell, _ in states]
