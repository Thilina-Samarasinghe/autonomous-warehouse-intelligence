"""Path-planner interface."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol

from awi.core.types import Cell, RobotId
from awi.core.world import World


@dataclass(frozen=True, slots=True)
class PathRequest:
    robot_id: RobotId
    start: Cell
    goal: Cell
    stuck: bool = False
    """True when the robot has been blocked by other robots and needs a detour."""


class PathPlanner(Protocol):
    """Computes paths for the robots that requested one this tick.

    A path lists the robot's *future* cells, one per tick, excluding its current
    cell; consecutive cells must be equal (wait) or 4-adjacent. A coordinated
    planner may also return new paths for robots that did not request one.
    Missing or empty entries mean "no path found"; the robot waits and asks again.
    """

    def plan(
        self, world: World, requests: Sequence[PathRequest]
    ) -> Mapping[RobotId, Sequence[Cell]]: ...
