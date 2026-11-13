"""Coordinated (multi-agent) planners: Prioritized Planning and windowed CBS.

Both planners work in the *lifelong, rolling-horizon* setting:

1. On every replan they solve a windowed MAPF instance for the **whole fleet**
   (``horizon`` ticks, see :mod:`awi.planning.mapf`).
2. They hand robots only the first ``commit_steps`` moves. All task robots ask
   again at the same tick, so the fleet always executes one consistent joint plan.

If the solution is conflict-free (``stats.failures == 0``), the engine never has to
block a move. All budgets count search nodes, not wall time, so every run is exactly
reproducible.
"""

from __future__ import annotations

import heapq
import itertools
import time
from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass, field

from awi.core.grid import Grid
from awi.core.types import Cell, RobotId
from awi.core.world import World
from awi.planning.base import PathRequest
from awi.planning.dijkstra import DistanceMaps
from awi.planning.mapf import (
    Agent,
    AgentKind,
    Conflict,
    Instance,
    build_instance,
    count_conflicts_of,
    find_conflicts,
    occupancy_table,
    pad,
    to_engine_paths,
)
from awi.planning.st_astar import STPath, space_time_astar

type JointPaths = dict[RobotId, list[Cell]]


@dataclass(slots=True)
class CoordinatedStats:
    calls: int = 0
    agents: int = 0
    low_level_calls: int = 0
    expanded: int = 0
    high_level_nodes: int = 0
    fallbacks: int = 0
    failures: int = 0
    """Replans whose returned joint plan still had a robot without a feasible path."""
    seconds: float = 0.0


@dataclass(slots=True)
class _Constraints:
    vertex: frozenset[tuple[Cell, int]] = frozenset()
    moves: frozenset[tuple[Cell, Cell, int]] = frozenset()


class CoordinatedPlanner(ABC):
    def __init__(
        self, horizon: int = 10, commit_steps: int = 4, max_expansions: int = 20_000
    ) -> None:
        if horizon < 1 or not 1 <= commit_steps <= horizon:
            raise ValueError("need horizon >= 1 and 1 <= commit_steps <= horizon")
        self.horizon = horizon
        self.commit_steps = commit_steps
        self.max_expansions = max_expansions
        self.stats = CoordinatedStats()
        self.last_instance: Instance | None = None
        self.last_solution: JointPaths | None = None

    def plan(self, world: World, requests: Sequence[PathRequest]) -> dict[RobotId, list[Cell]]:
        t0 = time.perf_counter()
        instance = build_instance(world)
        if not any(a.kind is AgentKind.TASK for a in instance.agents):
            return {}
        solution = self.solve(world.layout.grid, world.distances, instance)
        self.last_instance, self.last_solution = instance, solution
        self.stats.calls += 1
        self.stats.agents += len(instance.agents)
        self.stats.seconds += time.perf_counter() - t0
        return to_engine_paths(instance, solution, self.commit_steps)

    @abstractmethod
    def solve(self, grid: Grid, maps: DistanceMaps, instance: Instance) -> JointPaths:
        """Joint paths (``t = 0..T``) for every agent of ``instance``."""

    def _low_level(
        self,
        grid: Grid,
        maps: DistanceMaps,
        agent: Agent,
        *,
        vertex: frozenset[tuple[Cell, int]] | set[tuple[Cell, int]] = frozenset(),
        moves: frozenset[tuple[Cell, Cell, int]] | set[tuple[Cell, Cell, int]] = frozenset(),
        blocked_from: dict[Cell, int],
        avoid: dict[tuple[Cell, int], int] | None = None,
    ) -> STPath | None:
        self.stats.low_level_calls += 1
        res = space_time_astar(
            grid,
            agent.start,
            agent.goal,
            maps.to(agent.goal),
            self.horizon,
            vertex=vertex,
            moves=moves,
            blocked_from=blocked_from,
            absorbing_goal=agent.kind is AgentKind.TASK,
            avoid=avoid,
            max_expansions=self.max_expansions,
        )
        if res is not None:
            self.stats.expanded += res.expanded
        return res


# ---------------------------------------------------------------- prioritized


class PrioritizedPlanner(CoordinatedPlanner):
    """Plan robots one at a time; each reserves its space-time path.

    The default order is task robots first, the longest remaining distance first,
    then idle robots. If some robot finds no path, the failed robots are moved to
    the front and planning restarts (up to ``restarts`` times). The attempt with
    the fewest failures is kept.
    """

    def __init__(
        self,
        horizon: int = 10,
        commit_steps: int = 4,
        restarts: int = 8,
        max_expansions: int = 20_000,
    ) -> None:
        super().__init__(horizon, commit_steps, max_expansions)
        self.restarts = restarts

    def solve(self, grid: Grid, maps: DistanceMaps, instance: Instance) -> JointPaths:
        order = sorted(
            instance.agents,
            key=lambda a: (
                a.kind is AgentKind.IDLE,
                -maps.distance(a.start, a.goal),
                a.id,
            ),
        )
        best: tuple[int, JointPaths] | None = None
        for _ in range(self.restarts + 1):
            paths, failed = self._plan_in_order(grid, maps, instance, order)
            if not failed:
                return paths
            if best is None or len(failed) < best[0]:
                best = (len(failed), paths)
            failed_ids = {a.id for a in failed}
            order = [*failed, *(a for a in order if a.id not in failed_ids)]
        self.stats.failures += 1
        assert best is not None
        return best[1]

    def _plan_in_order(
        self, grid: Grid, maps: DistanceMaps, instance: Instance, order: Sequence[Agent]
    ) -> tuple[JointPaths, list[Agent]]:
        vertex: set[tuple[Cell, int]] = set()
        moves: set[tuple[Cell, Cell, int]] = set()
        blocked_from: dict[Cell, int] = dict.fromkeys(instance.static, 0)
        paths: JointPaths = {}
        failed: list[Agent] = []
        for agent in order:
            res = self._low_level(
                grid, maps, agent, vertex=vertex, moves=moves, blocked_from=blocked_from
            )
            if res is None:
                failed.append(agent)
                path = [agent.start]
            else:
                path = res.cells
            paths[agent.id] = path
            for t, c in enumerate(path):
                vertex.add((c, t))
                if t and path[t - 1] != c:
                    moves.add((c, path[t - 1], t))  # forbid the reverse move (swap)
            end, t_end = path[-1], len(path) - 1
            blocked_from[end] = min(blocked_from.get(end, t_end), t_end)
        return paths, failed


# ------------------------------------------------------------------------ CBS


@dataclass(order=True, slots=True)
class _Node:
    cost: float
    n_conflicts: int
    tie: int
    constraints: dict[RobotId, _Constraints] = field(compare=False)
    paths: JointPaths = field(compare=False)
    costs: dict[RobotId, float] = field(compare=False)


class CBSPlanner(CoordinatedPlanner):
    """Conflict-Based Search (Sharon et al., 2015) on the windowed instance.

    * High level: best-first search over a constraint tree, ordered by the sum of
      windowed path costs, with fewer conflicts breaking ties. Each node resolves
      its earliest conflict by branching into two children, each adding one
      constraint to one of the two robots.
    * Low level: Space-Time A* under the robot's constraints.

    CBS is optimal for the windowed sum of costs, but it can blow up exponentially.
    After ``max_nodes`` high-level expansions it falls back to Prioritized Planning
    (counted in ``stats.fallbacks``).
    """

    def __init__(
        self,
        horizon: int = 10,
        commit_steps: int = 4,
        max_nodes: int = 100,
        max_expansions: int = 20_000,
        fallback: PrioritizedPlanner | None = None,
    ) -> None:
        super().__init__(horizon, commit_steps, max_expansions)
        self.max_nodes = max_nodes
        self.fallback = fallback or PrioritizedPlanner(horizon, commit_steps)
        self.last_solved_optimally = False

    def solve(self, grid: Grid, maps: DistanceMaps, instance: Instance) -> JointPaths:
        horizon = self.horizon
        blocked_from = dict.fromkeys(instance.static, 0)
        agents = {a.id: a for a in instance.agents}

        # Root: individually optimal paths, each tie-broken against those before it.
        paths: JointPaths = {}
        costs: dict[RobotId, float] = {}
        for agent in instance.agents:
            res = self._low_level(
                grid, maps, agent, blocked_from=blocked_from, avoid=occupancy_table(paths, agent.id)
            )
            if res is None:
                return self._fall_back(grid, maps, instance)
            paths[agent.id], costs[agent.id] = pad(res.cells, horizon), res.cost

        tie = itertools.count()
        heap = [
            _Node(
                cost=sum(costs.values()),
                n_conflicts=sum(count_conflicts_of(rid, paths) for rid in paths) // 2,
                tie=next(tie),
                constraints={},
                paths=paths,
                costs=costs,
            )
        ]
        expanded = 0
        while heap and expanded < self.max_nodes:
            node = heapq.heappop(heap)
            conflicts = find_conflicts(node.paths, horizon, first_only=True)
            if not conflicts:
                self.stats.high_level_nodes += expanded
                self.last_solved_optimally = True
                return node.paths
            expanded += 1
            children: list[_Node] = []
            for rid, extra in _branches(conflicts[0], agents, horizon):
                old = node.constraints.get(rid, _Constraints())
                new = _Constraints(old.vertex | extra.vertex, old.moves | extra.moves)
                res = self._low_level(
                    grid,
                    maps,
                    agents[rid],
                    vertex=new.vertex,
                    moves=new.moves,
                    blocked_from=blocked_from,
                    avoid=occupancy_table(node.paths, rid),
                )
                if res is None:
                    continue
                child_paths = {**node.paths, rid: pad(res.cells, horizon)}
                child_costs = {**node.costs, rid: res.cost}
                n_conf = (
                    node.n_conflicts
                    - count_conflicts_of(rid, node.paths)
                    + count_conflicts_of(rid, child_paths)
                )
                cost = node.cost - node.costs[rid] + res.cost
                if cost == node.cost and n_conf < node.n_conflicts:
                    # Bypass: an equally cheap path with fewer conflicts. Adopt it in
                    # place of branching (it satisfies the parent's constraints too).
                    node.paths, node.costs, node.n_conflicts = child_paths, child_costs, n_conf
                    children = [node]
                    break
                children.append(
                    _Node(
                        cost=cost,
                        n_conflicts=n_conf,
                        tie=next(tie),
                        constraints={**node.constraints, rid: new},
                        paths=child_paths,
                        costs=child_costs,
                    )
                )
            for child in children:
                heapq.heappush(heap, child)
        self.stats.high_level_nodes += expanded
        return self._fall_back(grid, maps, instance)

    def _fall_back(self, grid: Grid, maps: DistanceMaps, instance: Instance) -> JointPaths:
        self.stats.fallbacks += 1
        self.last_solved_optimally = False
        before = self.fallback.stats.failures
        paths = self.fallback.solve(grid, maps, instance)
        self.stats.failures += self.fallback.stats.failures - before
        return paths


def _branches(
    c: Conflict, agents: dict[RobotId, Agent], horizon: int
) -> list[tuple[RobotId, _Constraints]]:
    """The two child constraints that resolve conflict ``c``.

    Target reasoning (Li et al., 2019): if one robot of a vertex conflict is a task
    robot standing on *its own goal*, the absorbing-goal rule means it stays there
    until the horizon. The other robot is then forbidden from that cell for the
    whole rest of the window (``t..horizon``) instead of one tick at a time. This
    avoids an exponential chain of one-tick branches when robots share a drop
    station or pick point. The split stays sound and complete.
    """
    if c.to is not None:
        # a moves cell -> to and b moves to -> cell, both arriving at t.
        return [
            (c.a, _Constraints(moves=frozenset({(c.cell, c.to, c.t)}))),
            (c.b, _Constraints(moves=frozenset({(c.to, c.cell, c.t)}))),
        ]
    out: list[tuple[RobotId, _Constraints]] = []
    for x, y in ((c.a, c.b), (c.b, c.a)):
        other = agents[y]
        if other.kind is AgentKind.TASK and other.goal == c.cell:
            v = frozenset((c.cell, k) for k in range(c.t, horizon + 1))
        else:
            v = frozenset({(c.cell, c.t)})
        out.append((x, _Constraints(vertex=v)))
    return out
