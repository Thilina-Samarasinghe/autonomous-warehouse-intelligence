"""Delivery tasks: pick an item at a shelf, bring it to a drop station."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from awi.core.types import Cell, RobotId, TaskId


class TaskStatus(StrEnum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    PICKED = "picked"
    DELIVERED = "delivered"


@dataclass(slots=True)
class Task:
    id: TaskId
    shelf: Cell
    pickup: Cell
    dropoff: Cell
    created_t: int
    status: TaskStatus = TaskStatus.PENDING
    robot_id: RobotId | None = None
    assigned_t: int | None = None
    picked_t: int | None = None
    delivered_t: int | None = None

    @property
    def delivery_time(self) -> int | None:
        """Ticks from creation to delivery (None while not delivered)."""
        return None if self.delivered_t is None else self.delivered_t - self.created_t

    @property
    def wait_time(self) -> int | None:
        """Ticks the task waited before a robot was assigned."""
        return None if self.assigned_t is None else self.assigned_t - self.created_t
