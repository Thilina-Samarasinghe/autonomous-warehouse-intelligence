import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from awi.assignment.hungarian import HungarianAssigner
from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.grid import CellType
from awi.core.layout import generate_layout
from awi.core.render import render_ascii
from awi.core.robot import RobotState
from awi.core.task import Task
from awi.core.types import manhattan
from awi.dynamics.obstacles import WALKABLE, Dynamics, DynamicsConfig, _stays_connected
from awi.metrics.collector import MetricsCollector
from awi.planning.coordinated import PrioritizedPlanner
from awi.planning.mapf import build_instance
from tests.conftest import SMALL_LAYOUT


def _sim(dynamics: dict[str, object], robots: int = 4, **kw: object) -> Simulation:
    data = {
        "num_robots": robots,
        "layout": SMALL_LAYOUT.model_dump(),
        "dynamics": {"enabled": True, **dynamics},
        **kw,
    }
    return Simulation(SimConfig.model_validate(data))


def test_disabled_by_default() -> None:
    assert Simulation(SimConfig(num_robots=2, layout=SMALL_LAYOUT)).world.dynamics is None


def test_humans_spawn_on_free_aisle_cells_and_walk() -> None:
    sim = _sim({"humans": 5, "human_move_prob": 1.0})
    dyn = sim.world.dynamics
    assert dyn is not None and len(dyn.humans) == 5
    robot_cells = {r.pos for r in sim.world.robots.values()}
    grid = sim.layout.grid
    for _ in range(100):
        before = [h.pos for h in dyn.humans]
        sim.step()
        after = [h.pos for h in dyn.humans]
        assert len(set(after)) == len(after)
        for b, a in zip(before, after, strict=True):
            assert manhattan(a, b) <= 1
            assert grid.cell_type(a) in WALKABLE
        robot_cells = {r.pos for r in sim.world.robots.values()}
        assert not robot_cells & set(after)  # humans and robots never share a cell
    assert any(a != b for a, b in zip(before, after, strict=True))


def test_dynamics_are_deterministic() -> None:
    def trace(seed: int) -> list[tuple[int, int]]:
        sim = _sim({"humans": 3, "blockage_rate": 0.05}, seed=seed)
        sim.run(150)
        assert sim.world.dynamics is not None
        return [h.pos for h in sim.world.dynamics.humans] + sorted(sim.world.dynamics.blockages)

    assert trace(4) == trace(4)
    assert trace(4) != trace(5)


def test_blockages_expire_and_respect_limits() -> None:
    sim = _sim(
        {"blockage_rate": 1.0, "blockage_length": 2, "blockage_duration": 30, "max_blockages": 2}
    )
    dyn = sim.world.dynamics
    assert dyn is not None
    started = 0
    for _ in range(200):
        rep = sim.step()
        started += rep.blockages_started
        assert len(dyn.blockages) <= 2 * 2
        assert _stays_connected(sim.layout.grid, set(dyn.blockages))
    assert started > 2  # blockages come and go
    quiet = _sim({"blockage_rate": 0.0})
    quiet.run(100)
    assert quiet.world.dynamics is not None and not quiet.world.dynamics.blockages


def test_connectivity_guard() -> None:
    grid = generate_layout(SMALL_LAYOUT).grid
    assert _stays_connected(grid, set())
    # Closing a whole column of the open top zone cuts nothing off (rows below
    # still connect), but sealing every neighbour of a drop station does.
    drop = (0, generate_layout(SMALL_LAYOUT).drop_stations[0][1])
    seal = set(grid.neighbors(drop))
    assert not _stays_connected(grid, seal)


def test_manual_blockage_skips_robots_and_non_walkable_cells() -> None:
    sim = Simulation(SimConfig(num_robots=2, layout=SMALL_LAYOUT))
    robot_cell = sim.world.robots[0].pos
    drop = sim.layout.drop_stations[0]
    free = next(
        c
        for c in sim.layout.grid.cells_of_type(CellType.PICK)
        if c not in {r.pos for r in sim.world.robots.values()}
    )
    added = sim.add_blockage([robot_cell, drop, free], duration=10)
    assert added == [free]
    assert sim.world.blocked_cells() == {free}
    sim.run(11)
    assert sim.world.blocked_cells() == set()


def _hard_obstacles(sim: Simulation) -> set[tuple[int, int]]:
    dyn = sim.world.dynamics
    assert dyn is not None
    return {h.pos for h in dyn.humans} | set(dyn.blockages)


def test_robots_never_enter_obstacles_or_safety_zones() -> None:
    sim = _sim({"humans": 6, "blockage_rate": 0.05}, robots=6, task_rate=0.2)
    dyn = sim.world.dynamics
    assert dyn is not None
    for _ in range(400):
        before = {r.id: r.pos for r in sim.world.robots.values()}
        sim.step()
        zone = dyn.safety_zone() | set(dyn.blockages)  # humans already moved this tick
        for r in sim.world.robots.values():
            assert r.pos not in _hard_obstacles(sim)
            if r.pos != before[r.id]:
                assert r.pos not in zone  # never steps *into* a keep-out cell


def test_new_blockage_invalidates_paths() -> None:
    sim = Simulation(SimConfig(num_robots=1, layout=SMALL_LAYOUT, max_tasks=0))
    robot = sim.world.robots[0]
    shelf = max(
        sim.layout.shelves,
        key=lambda sh: sim.world.distances.distance(robot.pos, sim.layout.pick_point[sh]),
    )
    task = Task(0, shelf, sim.layout.pick_point[shelf], sim.layout.drop_stations[0], 0)
    sim.world.tasks[0] = task
    sim.world.pending.append(0)
    sim.step()
    assert robot.state is RobotState.TO_PICKUP and len(robot.path) > 3
    cell = robot.path[2]
    sim.add_blockage([cell], duration=50)
    rep = sim.step()
    assert rep.obstacle_replans == 1
    sim.run(5)
    assert cell not in robot.path  # replanned around it


def test_coordinated_instance_treats_obstacles_as_static() -> None:
    sim = _sim({"humans": 3, "blockage_rate": 1.0, "blockage_length": 2}, robots=3)
    sim.step()
    inst = build_instance(sim.world)
    assert sim.world.blocked_cells() <= inst.static


def test_assigners_skip_blocked_pickups() -> None:
    sim = Simulation(SimConfig(num_robots=1, layout=SMALL_LAYOUT, max_tasks=0))
    shelves = sim.layout.shelves
    blocked_task = Task(
        0, shelves[0], sim.layout.pick_point[shelves[0]], sim.layout.drop_stations[0], 0
    )
    sim.add_blockage([blocked_task.pickup], duration=100)
    assert HungarianAssigner().assign(sim.world, sim.world.idle_robots(), [blocked_task]) == {}


def test_render_shows_humans_and_blockages() -> None:
    sim = _sim({"humans": 2, "blockage_rate": 1.0, "blockage_length": 1}, robots=1)
    sim.step()
    text = render_ascii(sim.world, legend=False)
    assert text.count("H") == 2 and "X" in text


def test_metrics_and_dynamics_config() -> None:
    sim = _sim({"humans": 6, "blockage_rate": 0.02}, robots=6, task_rate=0.2)
    m = MetricsCollector(sim)
    for _ in range(600):
        m.observe(sim.step())
    r = m.result()
    assert r.obstacle_replans > 0 and r.blockage_events > 0
    with pytest.raises(ValueError):
        DynamicsConfig(humans=-1)
    dyn = Dynamics(
        DynamicsConfig(humans=50), generate_layout(SMALL_LAYOUT), np.random.default_rng(0)
    )
    dyn.spawn_humans(set())
    assert len(dyn.humans) <= len(generate_layout(SMALL_LAYOUT).grid.passable_cells())


@settings(max_examples=8, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(seed=st.integers(0, 10_000), humans=st.integers(0, 6), robots=st.integers(2, 8))
def test_coordinated_fleet_stays_safe_and_productive(seed: int, humans: int, robots: int) -> None:
    cfg = SimConfig.model_validate(
        {
            "seed": seed,
            "num_robots": robots,
            "task_rate": 0.1,
            "layout": SMALL_LAYOUT.model_dump(),
            "dynamics": {"enabled": True, "humans": humans, "blockage_rate": 0.02},
        }
    )
    sim = Simulation(cfg, planner=PrioritizedPlanner())
    for _ in range(400):
        sim.step()
        positions = [r.pos for r in sim.world.robots.values()]
        assert len(set(positions)) == len(positions)
        assert not set(positions) & _hard_obstacles(sim)
    assert sim.summary().tasks_delivered > 0


def test_safety_zone_shape() -> None:
    sim = _sim({"humans": 1, "human_move_prob": 0.0, "safety_radius": 1}, robots=1)
    dyn = sim.world.dynamics
    assert dyn is not None
    (human,) = dyn.humans
    zone = dyn.safety_zone()
    assert human.pos in zone
    assert all(manhattan(c, human.pos) <= 1 for c in zone)
    assert {*sim.layout.grid.neighbors(human.pos), human.pos} == zone
    none = _sim({"humans": 1, "safety_radius": 0}, robots=1).world.dynamics
    assert none is not None and none.safety_zone() == {none.humans[0].pos}
