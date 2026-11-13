"""Order batching as a pickup-and-delivery vehicle routing problem (OR-Tools).

Each free robot is a vehicle that starts at its own position. Each candidate task
is a pickup node and a drop node, which must be served by the same robot, pickup
first. A robot carries at most ``capacity`` items and takes at most ``capacity``
tasks per trip. Routes are *open*: they end at a dummy node reached at zero cost.
Serving a task is optional, but skipping one costs a penalty that grows with the
task's age, so old orders are not starved.

The routing solver minimises total travel distance plus penalties. A robot that
picks two items close together and drops both at the same station saves a full
return trip. That is the benefit of batching over one-task-per-trip assignment.

The search is deterministic: a fixed first-solution strategy, greedy-descent local
search and a *solution* limit (not a time limit).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from awi.assignment.costs import candidate_tasks
from awi.core.robot import Robot
from awi.core.route import Route, Stop, StopKind
from awi.core.task import Task
from awi.core.types import Cell, RobotId
from awi.core.world import World

_UNREACHABLE = 10**6


class BatchingAssigner:
    def __init__(
        self,
        capacity: int = 2,
        age_weight: float = 0.2,
        skip_penalty: int = 1_000,
        solution_limit: int = 100,
    ) -> None:
        if capacity < 1 or age_weight < 0 or skip_penalty <= 0:
            raise ValueError("invalid batching parameters")
        self.capacity = capacity
        self.age_weight = age_weight
        self.skip_penalty = skip_penalty
        self.solution_limit = solution_limit

    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> dict[RobotId, Route]:
        if not robots or not tasks:
            return {}
        robots = sorted(robots, key=lambda r: r.id)
        cands = candidate_tasks(tasks, len(robots) * self.capacity, factor=2)
        problem = _Problem.build(world, robots, cands)
        routing, manager = self._model(world, problem, cands)

        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
        )
        params.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT
        )
        params.solution_limit = self.solution_limit
        solution = routing.SolveWithParameters(params)
        if solution is None:
            return {}

        routes: dict[RobotId, Route] = {}
        for v, robot in enumerate(robots):
            index = routing.Start(v)
            stops: list[Stop] = []
            while not routing.IsEnd(index):
                stop = problem.stop_of.get(manager.IndexToNode(index))
                if stop is not None:
                    stops.append(stop)
                index = solution.Value(routing.NextVar(index))
            if stops:
                routes[robot.id] = tuple(stops)
        return routes

    def _model(
        self, world: World, problem: _Problem, cands: Sequence[Task]
    ) -> tuple[pywrapcp.RoutingModel, pywrapcp.RoutingIndexManager]:
        n_r, dist, stop_of = problem.n_robots, problem.dist, problem.stop_of
        manager = pywrapcp.RoutingIndexManager(len(dist), n_r, list(range(1, n_r + 1)), [0] * n_r)
        routing = pywrapcp.RoutingModel(manager)

        def transit(i: int, j: int) -> int:
            return int(dist[manager.IndexToNode(i), manager.IndexToNode(j)])

        def load(i: int) -> int:
            stop = stop_of.get(manager.IndexToNode(i))
            return 0 if stop is None else (1 if stop.kind is StopKind.PICK else -1)

        def pickups(i: int) -> int:
            stop = stop_of.get(manager.IndexToNode(i))
            return int(stop is not None and stop.kind is StopKind.PICK)

        transit_cb = routing.RegisterTransitCallback(transit)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_cb)
        routing.AddDimension(transit_cb, 0, 10**7, True, "distance")
        distance_dim = routing.GetDimensionOrDie("distance")
        routing.AddDimensionWithVehicleCapacity(
            routing.RegisterUnaryTransitCallback(load), 0, [self.capacity] * n_r, True, "load"
        )
        routing.AddDimension(
            routing.RegisterUnaryTransitCallback(pickups), 0, self.capacity, True, "tasks"
        )

        solver = routing.solver()
        for k, task in enumerate(cands):
            p = manager.NodeToIndex(n_r + 1 + 2 * k)
            d = manager.NodeToIndex(n_r + 2 + 2 * k)
            routing.AddPickupAndDelivery(p, d)
            solver.Add(routing.VehicleVar(p) == routing.VehicleVar(d))
            solver.Add(distance_dim.CumulVar(p) <= distance_dim.CumulVar(d))
            penalty = self.skip_penalty + int(self.age_weight * (world.t - task.created_t))
            routing.AddDisjunction([p], penalty)
            routing.AddDisjunction([d], penalty)
        return routing, manager


@dataclass(frozen=True, slots=True)
class _Problem:
    """Node layout: 0 = dummy end, 1..R = robot starts, then (pickup, drop) per task."""

    n_robots: int
    dist: NDArray[np.int64]
    stop_of: dict[int, Stop]

    @classmethod
    def build(cls, world: World, robots: Sequence[Robot], cands: Sequence[Task]) -> _Problem:
        cells: list[Cell] = [robots[0].pos, *(r.pos for r in robots)]  # [0] is a placeholder
        stop_of: dict[int, Stop] = {}
        for task in cands:
            for kind, cell in ((StopKind.PICK, task.pickup), (StopKind.DROP, task.dropoff)):
                stop_of[len(cells)] = Stop(kind, task.id, cell)
                cells.append(cell)
        n_r, n = len(robots), len(cells)
        dist = np.zeros((n, n), dtype=np.int64)
        for j in range(n_r + 1, n):
            to_j = world.distances.to(cells[j])
            for i in range(1, n):
                d = to_j[cells[i]]
                dist[i, j] = _UNREACHABLE if np.isinf(d) else int(d)
        dist[:, 1 : n_r + 1] = _UNREACHABLE  # never drive *into* a robot start
        dist[0, :] = 0  # the dummy end node has no outgoing travel
        return cls(n_robots=n_r, dist=dist, stop_of=stop_of)
