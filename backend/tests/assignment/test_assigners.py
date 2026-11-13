import itertools

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from awi.assignment.batching_vrp import BatchingAssigner
from awi.assignment.costs import cost_matrix
from awi.assignment.hungarian import HungarianAssigner
from awi.assignment.ortools_cpsat import CPSATAssigner
from awi.core.config import SimConfig
from awi.core.engine import Simulation, SimulationError
from awi.core.robot import RobotState
from awi.core.route import Stop, StopKind, route_tasks, single_task_route, validate_route
from awi.core.task import Task, TaskStatus
from awi.experiments.registry import make_assigner
from tests.conftest import SMALL_LAYOUT


def _sim(robots: int = 4, **kw: object) -> Simulation:
    cfg = SimConfig.model_validate(
        {"num_robots": robots, "layout": SMALL_LAYOUT.model_dump(), "max_tasks": 0, **kw}
    )
    return Simulation(cfg)


def _add_tasks(sim: Simulation, picks: list[int], created: list[int] | None = None) -> list[Task]:
    """Add pending tasks for the given shelf indices (drop station 0)."""
    layout = sim.layout
    tasks = []
    for k, si in enumerate(picks):
        shelf = layout.shelves[si]
        task = Task(
            id=len(sim.world.tasks),
            shelf=shelf,
            pickup=layout.pick_point[shelf],
            dropoff=layout.drop_stations[0],
            created_t=(created or [0] * len(picks))[k],
        )
        sim.world.tasks[task.id] = task
        sim.world.pending.append(task.id)
        tasks.append(task)
    return tasks


def _objective(sim: Simulation, assignment: dict[int, int], age_weight: float) -> float:
    robots = sorted(sim.world.robots.values(), key=lambda r: r.id)
    tasks = sim.world.pending_tasks()
    cost = cost_matrix(sim.world, robots, tasks, age_weight)
    ri = {r.id: i for i, r in enumerate(robots)}
    tj = {t.id: j for j, t in enumerate(tasks)}
    return float(sum(cost[ri[r], tj[t]] for r, t in assignment.items()))


# -------------------------------------------------------------------- routes


def test_route_validation() -> None:
    sim = _sim()
    a, b = _add_tasks(sim, [0, 5])
    tasks = sim.world.tasks
    ok = (
        Stop(StopKind.PICK, a.id, a.pickup),
        Stop(StopKind.PICK, b.id, b.pickup),
        Stop(StopKind.DROP, b.id, b.dropoff),
        Stop(StopKind.DROP, a.id, a.dropoff),
    )
    validate_route(ok, tasks, capacity=2)
    assert route_tasks(ok) == [a.id, b.id]
    with pytest.raises(ValueError, match="capacity"):
        validate_route(ok, tasks, capacity=1)
    with pytest.raises(ValueError, match="bad drop"):
        validate_route(tuple(reversed(ok)), tasks, capacity=2)
    with pytest.raises(ValueError, match="never drops"):
        validate_route(ok[:2], tasks, capacity=2)
    with pytest.raises(ValueError, match="bad pick"):
        validate_route((Stop(StopKind.PICK, a.id, b.pickup),), tasks, capacity=2)
    with pytest.raises(ValueError, match="empty"):
        validate_route((), tasks, capacity=2)
    assert single_task_route(a) == (
        Stop(StopKind.PICK, a.id, a.pickup),
        Stop(StopKind.DROP, a.id, a.dropoff),
    )


def test_engine_executes_batched_route() -> None:
    class OneBatch:
        def assign(self, world, robots, tasks):  # type: ignore[no-untyped-def]
            if len(tasks) < 2:
                return {}
            a, b = tasks[0], tasks[1]
            return {
                robots[0].id: (
                    Stop(StopKind.PICK, a.id, a.pickup),
                    Stop(StopKind.PICK, b.id, b.pickup),
                    Stop(StopKind.DROP, a.id, a.dropoff),
                    Stop(StopKind.DROP, b.id, b.dropoff),
                )
            }

    cfg = SimConfig(num_robots=1, layout=SMALL_LAYOUT, max_tasks=0, capacity=2, park_idle=False)
    sim = Simulation(cfg, assigner=OneBatch())
    a, b = _add_tasks(sim, [0, 9])
    robot = sim.world.robots[0]
    saw_carry_to_pickup = False
    for _ in range(200):
        sim.step()
        if robot.state is RobotState.TO_PICKUP and robot.carrying:
            saw_carry_to_pickup = True
    assert saw_carry_to_pickup  # heading to the 2nd pick with the 1st item aboard
    assert a.status is TaskStatus.DELIVERED and b.status is TaskStatus.DELIVERED
    assert a.picked_t is not None and b.picked_t is not None and a.picked_t < b.picked_t
    assert robot.deliveries == 2 and not robot.cargo and robot.state is RobotState.IDLE


def test_engine_rejects_route_over_capacity() -> None:
    class Greedy2:
        def assign(self, world, robots, tasks):  # type: ignore[no-untyped-def]
            a, b = tasks[0], tasks[1]
            return {
                robots[0].id: (
                    Stop(StopKind.PICK, a.id, a.pickup),
                    Stop(StopKind.PICK, b.id, b.pickup),
                    Stop(StopKind.DROP, a.id, a.dropoff),
                    Stop(StopKind.DROP, b.id, b.dropoff),
                )
            }

    sim = Simulation(SimConfig(num_robots=1, layout=SMALL_LAYOUT, max_tasks=0), assigner=Greedy2())
    _add_tasks(sim, [0, 1])
    with pytest.raises(SimulationError, match="capacity"):
        sim.step()


# ------------------------------------------------------- Hungarian & CP-SAT


@settings(max_examples=40, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    robots=st.integers(1, 5),
    picks=st.lists(st.integers(0, 31), min_size=1, max_size=6),
    seed=st.integers(0, 1000),
)
def test_hungarian_is_optimal_and_matches_cpsat(robots: int, picks: list[int], seed: int) -> None:
    sim = _sim(robots=robots, seed=seed)
    sim.world.t = 50
    rng = np.random.default_rng(seed)
    _add_tasks(sim, picks, created=rng.integers(0, 50, len(picks)).tolist())
    world, tasks = sim.world, sim.world.pending_tasks()
    free = sorted(world.idle_robots(), key=lambda r: r.id)

    hung = HungarianAssigner(age_weight=0.2).assign(world, free, tasks)
    cpsat = CPSATAssigner(age_weight=0.2).assign(world, free, tasks)
    assert len(hung) == len(cpsat) == min(robots, len(tasks))
    assert len(set(hung.values())) == len(hung)

    # Brute force over every injective matching of the right size.
    k = min(robots, len(tasks))
    best = min(
        _objective(sim, dict(zip(rs, ts, strict=True)), 0.2)
        for rs in itertools.combinations([r.id for r in free], k)
        for ts in itertools.permutations([t.id for t in tasks], k)
    )
    assert _objective(sim, hung, 0.2) == pytest.approx(best)
    assert _objective(sim, cpsat, 0.2) == pytest.approx(best, abs=0.01 * k)


def test_age_weight_prefers_old_tasks() -> None:
    sim = _sim(robots=1)
    sim.world.t = 500
    robot = sim.world.robots[0]
    near, far = _add_tasks(sim, [0, 31], created=[499, 0])
    d_near = sim.world.distances.distance(robot.pos, near.pickup)
    d_far = sim.world.distances.distance(robot.pos, far.pickup)
    if d_near > d_far:
        near, far = far, near
    tasks = sim.world.pending_tasks()
    assert HungarianAssigner(age_weight=0.0).assign(sim.world, [robot], tasks)[0] == near.id
    old = max(tasks, key=lambda t: -t.created_t)
    assert HungarianAssigner(age_weight=1.0).assign(sim.world, [robot], tasks)[0] == old.id


def test_assigners_handle_empty_inputs() -> None:
    sim = _sim()
    robots = sim.world.idle_robots()
    for assigner in (HungarianAssigner(), CPSATAssigner(), BatchingAssigner()):
        assert assigner.assign(sim.world, robots, []) == {}
        assert assigner.assign(sim.world, [], []) == {}
    with pytest.raises(ValueError):
        HungarianAssigner(age_weight=-1)


# ------------------------------------------------------------------ batching


@pytest.mark.parametrize("capacity", [1, 2, 3])
def test_batching_routes_are_valid(capacity: int) -> None:
    sim = _sim(robots=3, capacity=capacity)
    _add_tasks(sim, [0, 1, 2, 8, 9, 15, 20, 21, 30])
    routes = BatchingAssigner(capacity=capacity).assign(
        sim.world, sim.world.idle_robots(), sim.world.pending_tasks()
    )
    assert routes
    used = [tid for r in routes.values() for tid in route_tasks(r)]
    assert len(used) == len(set(used))
    for route in routes.values():
        validate_route(route, sim.world.tasks, capacity)
        assert len(route_tasks(route)) <= capacity


def test_batching_groups_neighbouring_picks() -> None:
    """One robot, two adjacent shelves, same drop: one trip with both items."""
    sim = _sim(robots=1, capacity=2)
    a, b = _add_tasks(sim, [0, 2])
    route = BatchingAssigner(capacity=2).assign(
        sim.world, sim.world.idle_robots(), sim.world.pending_tasks()
    )[0]
    assert sorted(route_tasks(route)) == sorted([a.id, b.id])
    assert [s.kind for s in route] == [StopKind.PICK, StopKind.PICK, StopKind.DROP, StopKind.DROP]


@pytest.mark.parametrize("name", ["greedy", "hungarian", "cpsat", "batching"])
def test_every_assigner_runs_a_full_simulation(name: str) -> None:
    cfg = SimConfig(num_robots=5, task_rate=0.1, layout=SMALL_LAYOUT, seed=3, capacity=2)
    sim = Simulation(cfg, assigner=make_assigner(name, cfg))
    sim.run(400)
    s = sim.summary()
    assert s.tasks_delivered > 0.6 * s.tasks_generated
    assert s.tasks_generated == s.tasks_delivered + s.tasks_pending + s.tasks_in_progress
