import math

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from pydantic import ValidationError

from awi.assignment.hungarian import HungarianAssigner
from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.robot import RobotState
from awi.core.route import single_task_route
from awi.core.task import Task, TaskStatus
from awi.energy.battery import BatteryConfig
from awi.energy.charging import dispatch_chargers
from awi.metrics.collector import MetricsCollector
from awi.planning.coordinated import PrioritizedPlanner
from tests.conftest import SMALL_LAYOUT


def _cfg(**battery: object) -> dict[str, object]:
    return {"enabled": True, "initial_min": 1.0, "initial_max": 1.0, **battery}


def _sim(robots: int = 2, battery: dict[str, object] | None = None, **kw: object) -> Simulation:
    data = {
        "num_robots": robots,
        "layout": SMALL_LAYOUT.model_dump(),
        "battery": battery if battery is not None else _cfg(),
        **kw,
    }
    return Simulation(SimConfig.model_validate(data))


def _task(sim: Simulation, shelf_index: int = 0) -> Task:
    layout = sim.layout
    shelf = layout.shelves[shelf_index]
    task = Task(
        id=len(sim.world.tasks),
        shelf=shelf,
        pickup=layout.pick_point[shelf],
        dropoff=layout.drop_stations[0],
        created_t=sim.world.t,
    )
    sim.world.tasks[task.id] = task
    sim.world.pending.append(task.id)
    return task


# ------------------------------------------------------------------ config


def test_disabled_by_default_and_identical_behaviour() -> None:
    cfg = SimConfig(num_robots=4, layout=SMALL_LAYOUT, seed=2)
    sim = Simulation(cfg)
    assert sim.world.battery is None
    sim.run(200)
    assert all(r.battery == 100.0 and r.energy_used == 0 for r in sim.world.robots.values())


def test_config_validation() -> None:
    with pytest.raises(ValidationError, match="critical < low < resume"):
        BatteryConfig(low=0.05, critical=0.1)
    with pytest.raises(ValidationError, match="initial_min"):
        BatteryConfig(initial_min=0.9, initial_max=0.5)


def test_initial_charge_is_seeded() -> None:
    def levels(seed: int) -> list[float]:
        s = _sim(robots=5, battery={"enabled": True}, seed=seed)
        return [r.battery for r in s.world.robots.values()]

    assert levels(1) == levels(1) != levels(2)
    assert all(50.0 <= b <= 100.0 for b in levels(3))


# ------------------------------------------------------------------- drain


def test_drain_matches_model() -> None:
    sim = _sim(robots=1, max_tasks=0, battery=_cfg(move_cost=0.5, idle_cost=0.1, loaded_extra=0.2))
    robot = sim.world.robots[0]
    task = _task(sim, 3)
    start = robot.battery
    sim.run(80)
    assert task.status is TaskStatus.DELIVERED
    used = start - robot.battery
    assert used == pytest.approx(robot.energy_used)
    # 0.1 per tick for 80 ticks + 0.5 per move + 0.2 per loaded move
    assert used >= 80 * 0.1 + robot.distance * 0.5
    assert used <= 80 * 0.1 + robot.distance * (0.5 + 0.2)


def test_route_energy_and_affordability() -> None:
    sim = _sim(robots=1, max_tasks=0)
    battery, robot = sim.world.battery, sim.world.robots[0]
    assert battery is not None
    route = single_task_route(_task(sim, 0))
    need = battery.route_energy(robot.pos, route)
    assert need > 0
    robot.battery = battery.cfg.critical * battery.capacity + need + 0.01
    assert battery.can_afford(robot, route)
    robot.battery -= 0.02
    assert not battery.can_afford(robot, route)


# ---------------------------------------------------------------- policies


def test_low_robot_charges_then_returns_to_work() -> None:
    sim = _sim(robots=1, max_tasks=0)
    robot = sim.world.robots[0]
    robot.battery = 20.0  # below low (25%)
    states = []
    for _ in range(200):
        sim.step()
        states.append(robot.state)
    assert RobotState.TO_CHARGER in states and RobotState.CHARGING in states
    assert robot.charge_sessions == 1
    assert not robot.needs_charge
    # Charged to 90%, minus the little it used driving off the charger to park.
    assert 85.0 <= robot.battery < 90.0
    assert robot.pos not in sim.layout.chargers  # parked off the charger afterwards


def test_unaffordable_task_is_not_assigned() -> None:
    # Expensive moves: a robot just above "low" cannot afford a long trip.
    sim = _sim(robots=1, max_tasks=0, battery=_cfg(move_cost=1.0))
    robot = sim.world.robots[0]
    robot.battery = 26.0  # above low (25%), so it is offered to the assigner
    task = _task(sim, len(sim.layout.shelves) - 1)
    battery = sim.world.battery
    assert battery is not None
    assert not battery.can_afford(robot, single_task_route(task))
    sim.step()
    assert task.status is TaskStatus.PENDING
    assert robot.needs_charge or robot.state is RobotState.TO_CHARGER


def test_engine_rejects_unaffordable_route_from_any_assigner() -> None:
    class Reckless:
        def assign(self, world, robots, tasks):  # type: ignore[no-untyped-def]
            return {robots[0].id: tasks[0].id}

    cfg = SimConfig.model_validate(
        {
            "num_robots": 1,
            "layout": SMALL_LAYOUT.model_dump(),
            "max_tasks": 0,
            "battery": _cfg(move_cost=1.0),
        }
    )
    sim = Simulation(cfg, assigner=Reckless())
    sim.world.robots[0].battery = 26.0
    task = _task(sim, len(sim.layout.shelves) - 1)
    rep = sim.step()
    assert rep.energy_rejections == 1
    assert task.status is TaskStatus.PENDING


def test_optimising_assigners_skip_unaffordable_pairs() -> None:
    sim = _sim(robots=2, max_tasks=0)
    poor, rich = sim.world.robots[0], sim.world.robots[1]
    poor.battery, rich.battery = 11.0, 100.0
    task = _task(sim, 5)
    result = HungarianAssigner().assign(sim.world, [poor, rich], [task])
    assert result == {rich.id: task.id}


def test_critical_robot_releases_unpicked_task() -> None:
    sim = _sim(robots=1, max_tasks=0)
    robot = sim.world.robots[0]
    task = _task(sim, 10)
    sim.step()
    assert robot.state is RobotState.TO_PICKUP
    robot.battery = 5.0  # below critical
    rep = sim.step()
    assert rep.aborted_tasks == [task.id]
    assert task.status is TaskStatus.PENDING and task.robot_id is None
    assert task.id in sim.world.pending
    assert robot.needs_charge


def test_emergency_charge_keeps_cargo_and_resumes_delivery() -> None:
    sim = _sim(robots=1, max_tasks=0, park_idle=False)
    robot = sim.world.robots[0]
    task = _task(sim, 0)
    while task.status is not TaskStatus.PICKED:
        sim.step()
    robot.battery = 9.0  # critical, with cargo aboard
    charged_with_cargo = False
    for _ in range(300):
        sim.step()
        if robot.state is RobotState.CHARGING and task.id in robot.cargo:
            charged_with_cargo = True
        if task.status is TaskStatus.DELIVERED:
            break
    assert charged_with_cargo
    assert task.status is TaskStatus.DELIVERED
    assert robot.charge_sessions == 1 and robot.battery > 80


def test_empty_battery_strands_robot() -> None:
    sim = _sim(robots=1, max_tasks=0, battery=_cfg(idle_cost=1.0))
    robot = sim.world.robots[0]
    robot.battery = 2.0
    robot.needs_charge = False
    reports = sim.run(5)
    assert robot.state is RobotState.STRANDED and robot.battery == 0.0
    assert sum(len(r.stranded) for r in reports) == 1
    pos = robot.pos
    sim.run(5)
    assert robot.pos == pos  # stranded robots never move


def test_charger_queue_serves_most_urgent_first() -> None:
    layout = SMALL_LAYOUT.model_copy(update={"num_chargers": 1})
    sim = Simulation(
        SimConfig.model_validate(
            {"num_robots": 3, "layout": layout.model_dump(), "max_tasks": 0, "battery": _cfg()}
        )
    )
    robots = sorted(sim.world.robots.values(), key=lambda r: r.id)
    for r, level in zip(robots, (22.0, 12.0, 18.0), strict=True):
        r.battery, r.needs_charge = level, True
    assert sim.world.battery is not None
    (first,) = dispatch_chargers(sim.world, sim.layout.chargers, sim.world.battery).assignments
    assert first[0] is robots[1]  # lowest battery
    sim.step()
    assert robots[1].state is RobotState.TO_CHARGER
    assert robots[0].state is not RobotState.TO_CHARGER  # the only charger is taken


# ------------------------------------------------------------ fleet-level


def test_metrics_report_energy() -> None:
    cfg = SimConfig.model_validate(
        {
            "num_robots": 6,
            "task_rate": 0.15,
            "layout": SMALL_LAYOUT.model_dump(),
            "battery": {"enabled": True, "initial_min": 0.3, "initial_max": 0.6},
        }
    )
    sim = Simulation(cfg, planner=PrioritizedPlanner())
    m = MetricsCollector(sim)
    for _ in range(1500):
        m.observe(sim.step())
    r = m.result()
    assert r.charge_sessions > 0 and 0 < r.charging_frac < 1
    assert r.energy_per_delivery > 0
    assert r.stranded_robots == 0
    assert r.min_battery_frac > 0
    off = MetricsCollector(Simulation(SimConfig(num_robots=2, layout=SMALL_LAYOUT))).result()
    assert math.isnan(off.energy_per_delivery) and off.charge_sessions == 0


@settings(max_examples=10, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(seed=st.integers(0, 10_000), robots=st.integers(2, 10), rate=st.floats(0.05, 0.4))
def test_policy_never_strands_a_robot(seed: int, robots: int, rate: float) -> None:
    cfg = SimConfig.model_validate(
        {
            "seed": seed,
            "num_robots": robots,
            "task_rate": rate,
            "layout": SMALL_LAYOUT.model_dump(),
            "battery": {"enabled": True, "initial_min": 0.2, "initial_max": 0.7},
        }
    )
    sim = Simulation(cfg, planner=PrioritizedPlanner(), assigner=HungarianAssigner())
    for _ in range(1200):
        rep = sim.step()
        assert not rep.stranded
    assert all(r.battery > 0 for r in sim.world.robots.values())


def test_emergency_preempts_a_charged_robot() -> None:
    layout = SMALL_LAYOUT.model_copy(update={"num_chargers": 1})
    sim = Simulation(
        SimConfig.model_validate(
            {"num_robots": 2, "layout": layout.model_dump(), "max_tasks": 0, "battery": _cfg()}
        )
    )
    full, dying = sim.world.robots[0], sim.world.robots[1]
    full.pos, full.state, full.battery = sim.layout.chargers[0], RobotState.CHARGING, 60.0
    dying.battery, dying.needs_charge = 5.0, True
    rep = sim.step()
    assert rep.preempted == [full.id]
    assert full.state is not RobotState.CHARGING and not full.needs_charge
    assert dying.state is RobotState.TO_CHARGER and dying.goal == sim.layout.chargers[0]


def test_no_preemption_for_merely_low_robots() -> None:
    layout = SMALL_LAYOUT.model_copy(update={"num_chargers": 1})
    sim = Simulation(
        SimConfig.model_validate(
            {"num_robots": 2, "layout": layout.model_dump(), "max_tasks": 0, "battery": _cfg()}
        )
    )
    full, low = sim.world.robots[0], sim.world.robots[1]
    full.pos, full.state, full.battery = sim.layout.chargers[0], RobotState.CHARGING, 60.0
    low.battery = 20.0  # low, but not an emergency
    rep = sim.step()
    assert rep.preempted == []
    assert full.state is RobotState.CHARGING
