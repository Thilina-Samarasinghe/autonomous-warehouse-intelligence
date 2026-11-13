"""Assignment cost model shared by the optimisation-based assigners.

``cost[r, j] = travel(robot r -> pickup of task j) - age_weight * age(task j)``

* Travel is the true shortest-path distance (cached Dijkstra maps), not Manhattan.
* Pairs the robot cannot afford on its battery (route plus the trip to a charger,
  with a safety margin) are marked unreachable.
* Tasks whose pick point is currently blocked are skipped until it reopens.
* The age term rewards serving old tasks. When there are more tasks than robots,
  this stops far-away tasks from starving. Matching every robot shifts each
  column by a constant, so the term only decides *which* tasks are served.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

from awi.core.robot import Robot
from awi.core.route import single_task_route
from awi.core.task import Task
from awi.core.world import World

UNREACHABLE = 1e9


def candidate_tasks(tasks: Sequence[Task], n_robots: int, factor: int = 3) -> list[Task]:
    """The oldest ``max(factor * n_robots, 30)`` tasks; keeps the problems small."""
    ordered = sorted(tasks, key=lambda t: (t.created_t, t.id))
    return ordered[: max(factor * n_robots, 30)]


def cost_matrix(
    world: World, robots: Sequence[Robot], tasks: Sequence[Task], age_weight: float
) -> NDArray[np.float64]:
    battery = world.battery
    blocked = world.blocked_cells()
    cost = np.empty((len(robots), len(tasks)), dtype=np.float64)
    for j, task in enumerate(tasks):
        if task.pickup in blocked:  # pick point behind a blockage or a person
            cost[:, j] = UNREACHABLE
            continue
        to_pickup = world.distances.to(task.pickup)
        age = world.t - task.created_t
        route = single_task_route(task)
        for i, robot in enumerate(robots):
            d = float(to_pickup[robot.pos])
            if np.isinf(d) or (battery is not None and not battery.can_afford(robot, route)):
                cost[i, j] = UNREACHABLE
            else:
                cost[i, j] = d - age_weight * age
    return cost
