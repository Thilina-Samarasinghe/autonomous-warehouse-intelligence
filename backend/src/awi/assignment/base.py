"""Task-assigner interface."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Protocol

from awi.core.robot import Robot
from awi.core.route import Route
from awi.core.task import Task
from awi.core.types import RobotId, TaskId
from awi.core.world import World

type Assignment = TaskId | Route
"""A single task (picked then dropped) or an explicit multi-task route."""


class TaskAssigner(Protocol):
    """Matches free robots to pending tasks.

    Returns ``robot -> task id`` or ``robot -> route`` (batching). Each task may be
    used by at most one robot. Leaving robots or tasks unmatched is allowed.
    """

    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> Mapping[RobotId, Assignment]: ...
