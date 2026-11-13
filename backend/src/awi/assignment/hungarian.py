"""Optimal one-to-one assignment with the Hungarian algorithm (SciPy).

``scipy.optimize.linear_sum_assignment`` solves the rectangular assignment problem
in O(n^3). With R robots and T tasks it matches ``min(R, T)`` pairs and minimises
the total cost from :mod:`awi.assignment.costs`.
"""

from __future__ import annotations

from collections.abc import Sequence

from scipy.optimize import linear_sum_assignment

from awi.assignment.costs import UNREACHABLE, candidate_tasks, cost_matrix
from awi.core.robot import Robot
from awi.core.task import Task
from awi.core.types import RobotId, TaskId
from awi.core.world import World


class HungarianAssigner:
    def __init__(self, age_weight: float = 0.2) -> None:
        if age_weight < 0:
            raise ValueError("age_weight must be >= 0")
        self.age_weight = age_weight

    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> dict[RobotId, TaskId]:
        if not robots or not tasks:
            return {}
        robots = sorted(robots, key=lambda r: r.id)
        cands = candidate_tasks(tasks, len(robots))
        cost = cost_matrix(world, robots, cands, self.age_weight)
        rows, cols = linear_sum_assignment(cost)
        return {
            robots[i].id: cands[j].id
            for i, j in zip(rows.tolist(), cols.tolist(), strict=True)
            if cost[i, j] < UNREACHABLE / 2
        }
