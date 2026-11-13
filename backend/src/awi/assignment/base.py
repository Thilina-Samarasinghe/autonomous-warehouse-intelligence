"""Task-assigner interface."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Protocol

from awi.core.robot import Robot
from awi.core.task import Task
from awi.core.types import RobotId, TaskId
from awi.core.world import World


class TaskAssigner(Protocol):
    """Matches idle robots to pending tasks.

    Must return a one-to-one mapping drawn from the given robots and tasks; leaving
    robots or tasks unmatched is allowed.
    """

    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> Mapping[RobotId, TaskId]: ...
