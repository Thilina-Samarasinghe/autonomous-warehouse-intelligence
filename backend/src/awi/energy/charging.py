"""Charger dispatch: a shared, urgency-ordered queue over a limited set of chargers.

Every tick, robots that need charging (lowest battery first) are matched to the
nearest *free* charger. A charger is free when no robot is charging on it or
already heading to it. Robots in an emergency may be mid-route (carrying cargo);
they keep their route and resume it after charging. Robots that find no free
charger keep doing what they were doing, still flagged, so the queue is implicit
and the most urgent robot is always served first. Robots busy picking or dropping
are dispatched once their service ends.
"""

from __future__ import annotations

from dataclasses import dataclass

from awi.core.robot import Robot, RobotState
from awi.core.types import Cell
from awi.core.world import World
from awi.energy.battery import BatteryModel

_NOT_DISPATCHABLE = frozenset(
    {
        RobotState.TO_CHARGER,
        RobotState.CHARGING,
        RobotState.STRANDED,
        RobotState.PICKING,
        RobotState.DROPPING,
    }
)


@dataclass(frozen=True, slots=True)
class Dispatch:
    assignments: list[tuple[Robot, Cell]]
    """Robot -> charger it should drive to now."""
    preempted: list[Robot]
    """Robots that must leave their charger early for an emergency."""


def dispatch_chargers(world: World, chargers: tuple[Cell, ...], battery: BatteryModel) -> Dispatch:
    """Match needy robots (lowest battery first) to the nearest free charger.

    *Preemption*: if an **emergency** robot finds no free charger, it takes the
    charger of the fullest robot that is charging above the ``low`` threshold.
    That robot stops charging and resumes its work. Since a charging robot passes
    ``low`` within a few dozen ticks, an emergency robot never waits long.
    """
    robots = world.robots.values()
    taken = {r.goal for r in robots if r.state is RobotState.TO_CHARGER} | {
        r.pos for r in robots if r.state is RobotState.CHARGING
    }
    free = [c for c in chargers if c not in taken]
    needy = sorted(
        (r for r in robots if r.needs_charge and r.state not in _NOT_DISPATCHABLE),
        key=lambda r: (r.battery, r.id),
    )
    low = battery.cfg.low * battery.capacity
    preemptable = sorted(
        (r for r in robots if r.state is RobotState.CHARGING and r.battery >= low),
        key=lambda r: (-r.battery, r.id),
    )
    out: list[tuple[Robot, Cell]] = []
    preempted: list[Robot] = []
    for robot in needy:
        if free:
            charger = min(free, key=lambda c: (world.distances.distance(robot.pos, c), c))
            free.remove(charger)
        elif preemptable and battery.is_emergency(robot):
            victim = preemptable.pop(0)
            preempted.append(victim)
            charger = victim.pos
        else:
            continue
        out.append((robot, charger))
    return Dispatch(out, preempted)
