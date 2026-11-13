import math

import pytest

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.grid import Grid
from awi.core.layout import Layout
from awi.core.robot import RobotState
from awi.metrics.collector import MetricsCollector, MetricsConfig
from awi.planning.independent import IndependentAStarPlanner
from tests.conftest import SMALL_LAYOUT


def _run(sim: Simulation, ticks: int, **cfg: int) -> MetricsCollector:
    collector = MetricsCollector(sim, MetricsConfig(**cfg))
    for _ in range(ticks):
        collector.observe(sim.step())
    return collector


def test_metrics_match_engine_counters_without_warmup() -> None:
    sim = Simulation(SimConfig(num_robots=6, task_rate=0.2, layout=SMALL_LAYOUT, seed=3))
    m = _run(sim, 600, window=100).result()
    summary = sim.summary()
    robots = sim.world.robots.values()

    assert m.measured_ticks == 600
    assert m.tasks_created == summary.tasks_generated
    assert m.tasks_delivered == summary.tasks_delivered
    assert m.distance_total == summary.total_distance
    assert m.delivery_time_mean == pytest.approx(summary.avg_delivery_time)
    assert m.idle_frac == pytest.approx(summary.total_idle_ticks / (6 * 600))
    assert m.idle_frac + m.busy_frac == pytest.approx(1.0)
    assert m.blocked_frac == pytest.approx(sum(r.blocked_ticks for r in robots) / 3600)
    assert m.throughput_per_1k == pytest.approx(1000 * m.tasks_delivered / 600)
    assert m.delivery_time_p50 <= m.delivery_time_p95 <= m.delivery_time_max
    assert m.conflicts_total == m.vertex_conflicts + m.edge_conflicts
    assert m.plan_calls > 0 and m.plan_ms_total > 0


def test_warmup_is_excluded_exactly() -> None:
    cfg = SimConfig(num_robots=5, task_rate=0.2, layout=SMALL_LAYOUT, seed=8)
    full = Simulation(cfg)
    reports = full.run(400)
    warm = _run(Simulation(cfg), 400, warmup_ticks=150).result()

    measured = reports[150:]
    assert warm.measured_ticks == 250
    assert warm.vertex_conflicts == sum(r.vertex_conflicts for r in measured)
    assert warm.edge_conflicts == sum(r.edge_conflicts for r in measured)
    assert warm.distance_total == sum(r.moved for r in measured)
    assert warm.idle_frac + warm.busy_frac == pytest.approx(1.0)
    created_late = [t for t in full.world.tasks.values() if t.created_t >= 150]
    assert warm.tasks_created == len(created_late)


def test_timeseries_windows_cover_all_ticks() -> None:
    sim = Simulation(SimConfig(num_robots=4, layout=SMALL_LAYOUT))
    collector = _run(sim, 250, window=100)
    series = collector.timeseries()
    assert [w["ticks"] for w in series] == [100, 100, 50]
    assert [w["t_end"] for w in series] == [99, 199, 249]
    assert sum(w["deliveries"] for w in series) == collector.result().tasks_delivered


def test_deadlock_detection() -> None:
    layout = Layout.from_grid(Grid.from_ascii("D.....\n#+####"))
    sim = Simulation(
        SimConfig(num_robots=2, max_tasks=0),
        planner=IndependentAStarPlanner(yield_prob=0.0),
        layout=layout,
    )
    a, b = sim.world.robots[0], sim.world.robots[1]
    a.pos, b.pos = (0, 1), (0, 4)
    for robot, goal in ((a, (0, 4)), (b, (0, 1))):
        robot.state, robot.goal, robot.needs_replan = RobotState.TO_PICKUP, goal, True

    m = _run(sim, 60, deadlock_ticks=10).result()
    assert m.deadlock_events == 2  # both robots cross the threshold once
    assert m.max_concurrent_deadlocked == 2
    assert m.deadlocked_robot_ticks > 0
    assert m.edge_conflicts > 40
    assert math.isnan(m.delivery_time_mean)  # no tasks at all


def test_invalid_config() -> None:
    with pytest.raises(ValueError):
        MetricsConfig(window=0)
