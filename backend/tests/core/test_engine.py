import itertools

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from awi.assignment.greedy import GreedyNearestAssigner
from awi.core.config import SimConfig
from awi.core.engine import Simulation, SimulationError, resolve_moves
from awi.core.grid import Grid
from awi.core.layout import Layout
from awi.core.render import render_ascii, robot_glyph
from awi.core.robot import FREE_STATES, RobotState
from awi.core.task import TaskStatus
from awi.core.types import manhattan
from tests.conftest import SMALL_LAYOUT

# ----------------------------------------------------------------- resolve_moves


def test_free_moves_all_succeed() -> None:
    final, blocked = resolve_moves({0: (0, 0), 1: (5, 5)}, {0: (0, 1), 1: (5, 6)})
    assert final == {0: (0, 1), 1: (5, 6)}
    assert blocked == set()


def test_head_on_swap_is_rejected() -> None:
    final, blocked = resolve_moves({0: (0, 0), 1: (0, 1)}, {0: (0, 1), 1: (0, 0)})
    assert final == {0: (0, 0), 1: (0, 1)}
    assert blocked == {0, 1}


def test_vertex_conflict_lowest_id_wins() -> None:
    final, blocked = resolve_moves({2: (0, 0), 1: (0, 2)}, {2: (0, 1), 1: (0, 1)})
    assert final == {1: (0, 1), 2: (0, 0)}
    assert blocked == {2}


def test_stationary_robot_keeps_its_cell() -> None:
    final, blocked = resolve_moves({0: (0, 0), 1: (0, 1)}, {0: (0, 1), 1: (0, 1)})
    assert final == {0: (0, 0), 1: (0, 1)}
    assert blocked == {0}


def test_following_is_allowed() -> None:
    final, blocked = resolve_moves({0: (0, 0), 1: (0, 1)}, {0: (0, 1), 1: (0, 2)})
    assert final == {0: (0, 1), 1: (0, 2)}
    assert blocked == set()


def test_blocking_cascades_down_a_chain() -> None:
    # 0 -> 1's cell -> 2's cell, but 2 stays: everyone behind waits.
    current = {0: (0, 0), 1: (0, 1), 2: (0, 2)}
    intent = {0: (0, 1), 1: (0, 2), 2: (0, 2)}
    final, blocked = resolve_moves(current, intent)
    assert final == current
    assert blocked == {0, 1}


def test_rotation_cycle_moves() -> None:
    current = {0: (0, 0), 1: (0, 1), 2: (1, 1), 3: (1, 0)}
    intent = {0: (0, 1), 1: (1, 1), 2: (1, 0), 3: (0, 0)}
    final, blocked = resolve_moves(current, intent)
    assert final == intent
    assert blocked == set()


# ------------------------------------------------------------------- lifecycle

TINY_MAP = """
D......
.......
.+++++.
.#####.
.......
"""


def test_single_robot_delivery_timing() -> None:
    """delivered_t == created_t + d(spawn->pickup) + d(pickup->drop) + pick + drop - 1."""
    layout = Layout.from_grid(Grid.from_ascii(TINY_MAP))
    cfg = SimConfig(
        num_robots=1,
        task_rate=5.0,
        max_tasks=1,
        pick_ticks=2,
        drop_ticks=1,
        seed=4,
        park_idle=False,
    )
    sim = Simulation(cfg, layout=layout)
    start = sim.world.robots[0].pos
    sim.run(60)

    task = sim.world.tasks[0]
    grid = layout.grid
    d1 = int(grid.bfs_distances(start)[task.pickup])
    d2 = int(grid.bfs_distances(task.pickup)[task.dropoff])
    assert task.status is TaskStatus.DELIVERED
    assert task.assigned_t == task.created_t == 0
    assert task.delivered_t == d1 + d2 + cfg.pick_ticks + cfg.drop_ticks - 1
    robot = sim.world.robots[0]
    assert robot.state is RobotState.IDLE
    assert robot.pos == task.dropoff
    assert robot.distance == d1 + d2
    assert robot.deliveries == 1
    assert robot.blocked_ticks == 0


def test_idle_robot_parks_off_the_drop_station() -> None:
    layout = Layout.from_grid(Grid.from_ascii(TINY_MAP))
    sim = Simulation(SimConfig(num_robots=1, task_rate=5.0, max_tasks=1, seed=4), layout=layout)
    sim.run(80)
    robot, task = sim.world.robots[0], sim.world.tasks[0]
    assert task.status is TaskStatus.DELIVERED
    assert robot.state is RobotState.IDLE
    assert robot.pos != task.dropoff
    assert robot.pos in layout.parking_cells
    assert all(manhattan(robot.pos, d) >= 2 for d in layout.drop_stations)


def test_parking_robot_can_be_reassigned() -> None:
    layout = Layout.from_grid(Grid.from_ascii(TINY_MAP))
    sim = Simulation(SimConfig(num_robots=1, task_rate=0.5, max_tasks=6, seed=1), layout=layout)
    states: set[RobotState] = set()
    for _ in range(400):
        sim.step()
        states.add(sim.world.robots[0].state)
    assert RobotState.TO_PARK in states
    assert all(t.status is TaskStatus.DELIVERED for t in sim.world.tasks.values())


def test_zero_service_time() -> None:
    layout = Layout.from_grid(Grid.from_ascii(TINY_MAP))
    cfg = SimConfig(num_robots=1, task_rate=5.0, max_tasks=1, pick_ticks=0, drop_ticks=0, seed=4)
    sim = Simulation(cfg, layout=layout)
    start = sim.world.robots[0].pos
    sim.run(40)
    task = sim.world.tasks[0]
    d1 = int(layout.grid.bfs_distances(start)[task.pickup])
    d2 = int(layout.grid.bfs_distances(task.pickup)[task.dropoff])
    assert task.delivered_t == d1 + d2 - 1


def test_simulation_is_deterministic() -> None:
    def fingerprint(seed: int) -> tuple[object, ...]:
        sim = Simulation(SimConfig(seed=seed, num_robots=8, task_rate=0.3, layout=SMALL_LAYOUT))
        reports = sim.run(300)
        return (
            sim.summary(),
            tuple(r.pos for r in sim.world.robots.values()),
            tuple(len(rep.blocked) for rep in reports),
        )

    assert fingerprint(11) == fingerprint(11)
    assert fingerprint(11) != fingerprint(12)


def test_default_fleet_delivers_steadily() -> None:
    sim = Simulation(SimConfig(num_robots=10, seed=1))
    sim.run(1500)
    s = sim.summary()
    assert s.tasks_delivered > 0.8 * s.tasks_generated
    assert s.tasks_generated == s.tasks_delivered + s.tasks_pending + s.tasks_in_progress


def test_too_many_robots_rejected() -> None:
    with pytest.raises(ValueError, match="at most"):
        Simulation(SimConfig(num_robots=200, layout=SMALL_LAYOUT))


def test_invalid_planner_output_raises() -> None:
    class TeleportPlanner:
        def plan(self, world, requests):  # type: ignore[no-untyped-def]
            return {req.robot_id: [req.goal] for req in requests}

    sim = Simulation(SimConfig(num_robots=1, task_rate=5.0), planner=TeleportPlanner())
    with pytest.raises(SimulationError, match="invalid path"):
        sim.run(5)


def test_invalid_assignment_raises() -> None:
    class DoubleBooker:
        def assign(self, world, robots, tasks):  # type: ignore[no-untyped-def]
            return {r.id: tasks[0].id for r in robots}

    sim = Simulation(SimConfig(num_robots=2, task_rate=5.0), assigner=DoubleBooker())
    with pytest.raises(SimulationError, match="several robots"):
        sim.run(2)


def test_greedy_assigner_prefers_nearest_robot() -> None:
    sim = Simulation(SimConfig(num_robots=3, task_rate=5.0, max_tasks=1, seed=2))
    world = sim.world
    (task,) = sim.task_generator.generate(0)
    result = GreedyNearestAssigner().assign(world, world.idle_robots(), [task])
    nearest = min(world.robots.values(), key=lambda r: (manhattan(r.pos, task.pickup), r.id))
    assert result == {nearest.id: task.id}


def test_render_shows_every_robot() -> None:
    sim = Simulation(SimConfig(num_robots=5, layout=SMALL_LAYOUT))
    sim.run(10)
    text = render_ascii(sim.world, legend=False)
    grid_rows = text.splitlines()
    for r in sim.world.robots.values():
        assert grid_rows[r.pos[0]][r.pos[1]] == robot_glyph(r.id)


# ------------------------------------------------------------ property test


@settings(max_examples=40, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    seed=st.integers(0, 10_000),
    robots=st.integers(1, 14),
    rate=st.floats(0.05, 0.8),
    pick=st.integers(0, 3),
)
def test_physical_and_bookkeeping_invariants(
    seed: int, robots: int, rate: float, pick: int
) -> None:
    cfg = SimConfig(
        seed=seed, num_robots=robots, task_rate=rate, pick_ticks=pick, layout=SMALL_LAYOUT
    )
    sim = Simulation(cfg)
    grid = sim.layout.grid
    world = sim.world
    for _ in range(150):
        before = {r.id: r.pos for r in world.robots.values()}
        sim.step()
        after = {r.id: r.pos for r in world.robots.values()}

        # No two robots share a cell, all on the floor, at most one step each.
        assert len(set(after.values())) == len(after)
        assert all(grid.is_passable(c) for c in after.values())
        assert all(manhattan(before[i], after[i]) <= 1 for i in after)
        # No swaps.
        for a, b in itertools.combinations(after, 2):
            assert not (after[a] == before[b] and after[b] == before[a] and after[a] != before[a])

        # Bookkeeping.
        tasks = world.tasks
        assert set(world.pending) == {i for i, t in tasks.items() if t.status is TaskStatus.PENDING}
        for r in world.robots.values():
            assert r.carrying == (r.state in (RobotState.TO_DROP, RobotState.DROPPING))
            if r.state in FREE_STATES:
                assert r.task_id is None
            else:
                assert r.task_id is not None and tasks[r.task_id].robot_id == r.id
        for t in tasks.values():
            stamps = [t.created_t, t.assigned_t, t.picked_t, t.delivered_t]
            known = [s for s in stamps if s is not None]
            assert known == sorted(known)
