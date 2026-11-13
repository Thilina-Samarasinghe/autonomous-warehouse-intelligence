"""Mutable world state shared by the engine, planners and assigners."""

from __future__ import annotations

from dataclasses import dataclass, field

from awi.core.layout import Layout
from awi.core.robot import FREE_STATES, Robot
from awi.core.task import Task
from awi.core.types import Cell, RobotId, TaskId
from awi.planning.dijkstra import DistanceMaps


@dataclass(slots=True)
class World:
    layout: Layout
    robots: dict[RobotId, Robot]
    distances: DistanceMaps
    """Uniform-cost Dijkstra maps shared by planners, assigners and metrics."""
    tasks: dict[TaskId, Task] = field(default_factory=dict)
    pending: list[TaskId] = field(default_factory=list)
    """Unassigned task ids, oldest first."""
    t: int = 0

    def occupancy(self) -> dict[Cell, RobotId]:
        return {r.pos: r.id for r in self.robots.values()}

    def idle_robots(self) -> list[Robot]:
        """Robots without a task (idle or parking): available for assignment."""
        return [r for r in self.robots.values() if r.state in FREE_STATES]

    def pending_tasks(self) -> list[Task]:
        return [self.tasks[i] for i in self.pending]
