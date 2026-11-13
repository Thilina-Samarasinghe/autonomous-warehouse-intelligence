"""Robot agent state."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import StrEnum

from awi.core.types import Cell, RobotId, TaskId


class RobotState(StrEnum):
    IDLE = "idle"
    TO_PICKUP = "to_pickup"
    PICKING = "picking"
    TO_DROP = "to_drop"
    DROPPING = "dropping"
    TO_CHARGER = "to_charger"  # used from phase 6 (battery management)
    CHARGING = "charging"
    TO_PARK = "to_park"
    """No task: clearing a station or pick point. Can be re-tasked at any time."""


MOVING_STATES: frozenset[RobotState] = frozenset(
    {RobotState.TO_PICKUP, RobotState.TO_DROP, RobotState.TO_CHARGER, RobotState.TO_PARK}
)
FREE_STATES: frozenset[RobotState] = frozenset({RobotState.IDLE, RobotState.TO_PARK})
"""States without a task: these robots can be assigned work."""
PATH_STATES: frozenset[RobotState] = MOVING_STATES | {RobotState.IDLE}
"""States in which a robot may follow a planned path. Idle robots can be told to
move aside by coordinated planners."""


@dataclass(slots=True)
class Robot:
    id: RobotId
    pos: Cell
    state: RobotState = RobotState.IDLE
    task_id: TaskId | None = None
    goal: Cell | None = None
    path: deque[Cell] = field(default_factory=deque)
    """Planned future cells, one per tick (a repeated cell is a planned wait)."""
    needs_replan: bool = False
    service_left: int = 0
    stuck_ticks: int = 0
    carrying: bool = False
    battery: float = 100.0

    # cumulative counters
    distance: int = 0
    idle_ticks: int = 0
    busy_ticks: int = 0
    wait_ticks: int = 0
    """Ticks spent not moving while on the way to a goal (planned or blocked)."""
    blocked_ticks: int = 0
    """Ticks where the intended move was rejected because of another robot."""
    deliveries: int = 0

    @property
    def is_moving_state(self) -> bool:
        return self.state in MOVING_STATES
