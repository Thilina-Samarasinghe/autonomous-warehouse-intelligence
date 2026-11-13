"""Mutable world state shared by the engine, planners and assigners."""

from __future__ import annotations

from dataclasses import dataclass, field

from awi.core.layout import Layout
from awi.core.robot import FREE_STATES, Robot
from awi.core.task import Task
from awi.core.types import Cell, RobotId, TaskId
from awi.dynamics.obstacles import Dynamics
from awi.energy.battery import BatteryModel
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
    battery: BatteryModel | None = None
    """Energy model, or ``None`` when battery management is off."""
    dynamics: Dynamics | None = None
    """Humans and blockages, or ``None`` when there are no dynamic obstacles."""

    def blocked_cells(self) -> set[Cell]:
        """Cells robots may not enter right now (humans and blockages)."""
        return self.dynamics.blocked_cells() if self.dynamics is not None else set()

    def occupancy(self) -> dict[Cell, RobotId]:
        return {r.pos: r.id for r in self.robots.values()}

    def idle_robots(self) -> list[Robot]:
        """Robots without a task (idle or parking) that are not waiting to charge."""
        return [r for r in self.robots.values() if r.state in FREE_STATES and not r.needs_charge]

    def pending_tasks(self) -> list[Task]:
        return [self.tasks[i] for i in self.pending]
