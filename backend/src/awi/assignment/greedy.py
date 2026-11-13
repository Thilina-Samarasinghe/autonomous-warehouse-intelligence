"""Greedy baseline: oldest task first, nearest idle robot (Manhattan distance)."""

from __future__ import annotations

from collections.abc import Sequence

from awi.core.robot import Robot
from awi.core.route import single_task_route
from awi.core.task import Task
from awi.core.types import RobotId, TaskId, manhattan
from awi.core.world import World


class GreedyNearestAssigner:
    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> dict[RobotId, TaskId]:
        free = sorted(robots, key=lambda r: r.id)
        result: dict[RobotId, TaskId] = {}
        for task in sorted(tasks, key=lambda t: (t.created_t, t.id)):
            if not free:
                break
            battery = world.battery
            able = [
                r for r in free if battery is None or battery.can_afford(r, single_task_route(task))
            ]
            if not able:
                continue
            best = min(able, key=lambda r: (manhattan(r.pos, task.pickup), r.id))
            result[best.id] = task.id
            free.remove(best)
        return result
