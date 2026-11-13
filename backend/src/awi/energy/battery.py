"""Battery model: energy drain, route energy estimates and feasibility checks.

All levels are in energy units (``capacity`` = full). Thresholds are fractions of
``capacity``:

* ``low``: a robot without a task goes to charge.
* ``critical``: a robot hands back unpicked work (it goes back to the queue) and
  goes to charge right away, *keeping its cargo*. It resumes the delivery after
  charging. The same emergency charge starts whenever the battery is barely
  enough to reach the nearest charger, because traffic can make real trips much
  longer than the shortest-path estimate.
* ``resume``: charging stops at this level.

Drain per tick is ``idle_cost`` for every robot that isn't charging, plus
``move_cost + loaded_extra x items carried`` for each move.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from pydantic import BaseModel, ConfigDict, Field, model_validator

from awi.core.route import Stop, StopKind
from awi.core.types import Cell

if TYPE_CHECKING:
    from awi.core.robot import Robot
    from awi.planning.dijkstra import DistanceMaps


class BatteryConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    enabled: bool = False
    capacity: float = Field(100.0, gt=0)
    initial_min: float = Field(0.5, ge=0, le=1, description="Initial charge range (fraction)")
    initial_max: float = Field(1.0, ge=0, le=1)
    move_cost: float = Field(0.1, ge=0)
    loaded_extra: float = Field(0.05, ge=0, description="Extra cost per move per item carried")
    idle_cost: float = Field(0.01, ge=0, description="Drain per tick when not charging")
    charge_rate: float = Field(1.0, gt=0, description="Energy gained per tick on a charger")
    low: float = Field(0.25, gt=0, lt=1)
    critical: float = Field(0.10, ge=0, lt=1)
    resume: float = Field(0.90, gt=0, le=1)
    margin: float = Field(1.3, ge=1, description="Safety factor on route energy estimates")

    @model_validator(mode="after")
    def _ordered(self) -> BatteryConfig:
        if not self.critical < self.low < self.resume:
            raise ValueError("need critical < low < resume")
        if self.initial_min > self.initial_max:
            raise ValueError("initial_min must be <= initial_max")
        return self


class BatteryModel:
    def __init__(
        self,
        cfg: BatteryConfig,
        chargers: Sequence[Cell],
        distances: DistanceMaps,
        pick_ticks: int,
        drop_ticks: int,
    ) -> None:
        if not chargers:
            raise ValueError("battery management needs at least one charger")
        self.cfg = cfg
        self.chargers = tuple(chargers)
        self.distances = distances
        self.pick_ticks = pick_ticks
        self.drop_ticks = drop_ticks

    @property
    def capacity(self) -> float:
        return self.cfg.capacity

    def tick_cost(self, moved: bool, items: int) -> float:
        cfg = self.cfg
        move = cfg.move_cost + cfg.loaded_extra * items if moved else 0.0
        return cfg.idle_cost + move

    def nearest_charger(self, cell: Cell) -> tuple[Cell, float]:
        return min(
            ((c, self.distances.distance(cell, c)) for c in self.chargers),
            key=lambda item: (item[1], item[0]),
        )

    def route_energy(self, start: Cell, route: Sequence[Stop], items: int = 0) -> float:
        """Estimated energy for ``route`` plus the trip to the nearest charger
        afterwards, times the safety margin."""
        cfg = self.cfg
        energy, ticks, pos, load = 0.0, 0.0, start, items
        for stop in route:
            d = self.distances.distance(pos, stop.cell)
            energy += d * (cfg.move_cost + cfg.loaded_extra * load)
            if stop.kind is StopKind.PICK:
                ticks += d + self.pick_ticks
                load += 1
            else:
                ticks += d + self.drop_ticks
                load -= 1
            pos = stop.cell
        _, d_charge = self.nearest_charger(pos)
        energy += d_charge * cfg.move_cost
        ticks += d_charge
        return (energy + ticks * cfg.idle_cost) * cfg.margin

    def reach_charger_energy(self, robot: Robot) -> float:
        """Energy needed to drive to the nearest charger now (with margin)."""
        cfg = self.cfg
        _, d = self.nearest_charger(robot.pos)
        per_move = cfg.move_cost + cfg.loaded_extra * len(robot.cargo)
        return d * (per_move + cfg.idle_cost) * cfg.margin

    def is_emergency(self, robot: Robot) -> bool:
        """Below critical, or only just able to reach a charger."""
        return (
            robot.battery < self.cfg.critical * self.cfg.capacity
            or robot.battery < self.reach_charger_energy(robot) + 1.0
        )

    def can_afford(self, robot: Robot, route: Sequence[Stop]) -> bool:
        """True if the robot finishes ``route`` and reaches a charger above critical."""
        reserve = self.cfg.critical * self.cfg.capacity
        return robot.battery - reserve >= self.route_energy(robot.pos, route, len(robot.cargo))

    def fraction(self, robot: Robot) -> float:
        return robot.battery / self.cfg.capacity
