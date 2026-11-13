"""Per-run metric collection.

Usage::

    sim = Simulation(cfg, planner=...)
    metrics = MetricsCollector(sim, MetricsConfig(warmup_ticks=300))
    for _ in range(ticks):
        metrics.observe(sim.step())
    row = metrics.result()           # RunMetrics: flat KPIs for one run
    series = metrics.timeseries()    # one dict per window

Everything is measured *after the warm-up*: ticks ``t < warmup_ticks`` are ignored,
and so are tasks created during warm-up, so start-up transients don't skew the
steady-state KPIs.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from awi.core.conflicts import robots_in_cycles
from awi.core.engine import Simulation, TickReport
from awi.core.robot import FREE_STATES, MOVING_STATES
from awi.core.types import RobotId


@dataclass(frozen=True, slots=True)
class MetricsConfig:
    warmup_ticks: int = 0
    window: int = 100
    """Time-series bucket size in ticks."""
    deadlock_ticks: int = 20
    """A robot in a wait-for cycle for this many consecutive ticks is deadlocked."""

    def __post_init__(self) -> None:
        if self.warmup_ticks < 0 or self.window < 1 or self.deadlock_ticks < 1:
            raise ValueError("invalid metrics configuration")


@dataclass(frozen=True, slots=True)
class RunMetrics:
    measured_ticks: int
    # tasks
    tasks_created: int
    tasks_delivered: int
    completion_rate: float
    throughput_per_1k: float
    delivery_time_mean: float
    delivery_time_p50: float
    delivery_time_p95: float
    delivery_time_max: float
    assign_wait_mean: float
    # motion
    distance_total: int
    distance_per_delivery: float
    # robot time fractions (of robot-ticks)
    idle_frac: float
    busy_frac: float
    wait_frac: float
    blocked_frac: float
    # conflicts / deadlocks
    vertex_conflicts: int
    edge_conflicts: int
    conflicts_total: int
    conflicts_per_1k: float
    deadlock_events: int
    deadlocked_robot_ticks: int
    max_concurrent_deadlocked: int
    # planner effort
    plan_calls: int
    plan_ms_total: float
    plan_ms_mean: float
    plan_ms_max: float
    # energy (zero / NaN when battery management is off)
    charging_frac: float = 0.0
    charge_sessions: int = 0
    energy_per_delivery: float = math.nan
    min_battery_frac: float = math.nan
    aborted_tasks: int = 0
    energy_rejections: int = 0
    stranded_robots: int = 0
    preemptions: int = 0
    # dynamic obstacles
    obstacle_blocks: int = 0
    obstacle_replans: int = 0
    blockage_events: int = 0

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class _Counters:
    charging: int = 0
    sessions: int = 0
    energy: float = 0.0
    distance: int = 0
    idle: int = 0
    busy: int = 0
    wait: int = 0
    blocked: int = 0


@dataclass(slots=True)
class _Window:
    deliveries: int = 0
    conflicts: int = 0
    blocked: int = 0
    moves: int = 0
    plan_ms: float = 0.0
    ticks: int = 0


@dataclass(slots=True)
class MetricsCollector:
    sim: Simulation
    config: MetricsConfig = field(default_factory=MetricsConfig)

    _base: _Counters = field(default_factory=_Counters)
    """Robot counters at the end of warm-up (zero when there is none)."""
    _measured: int = 0
    _vertex: int = 0
    _edge: int = 0
    _plan_calls: int = 0
    _plan_seconds: float = 0.0
    _plan_max: float = 0.0
    _aborted: int = 0
    _rejections: int = 0
    _stranded: int = 0
    _preempted: int = 0
    _obstacle_blocks: int = 0
    _obstacle_replans: int = 0
    _blockages: int = 0
    _min_battery: float = math.inf
    _in_cycle_for: dict[RobotId, int] = field(default_factory=dict)
    _deadlock_events: int = 0
    _deadlocked_robot_ticks: int = 0
    _max_deadlocked: int = 0
    _window: _Window = field(default_factory=_Window)
    _series: list[dict[str, Any]] = field(default_factory=list)

    def observe(self, report: TickReport) -> None:
        """Record one tick. Call right after ``sim.step()``."""
        cfg = self.config
        if report.t < cfg.warmup_ticks:
            if report.t == cfg.warmup_ticks - 1:
                self._base = self._robot_counters()  # exact state at end of warm-up
            return

        self._measured += 1
        self._vertex += report.vertex_conflicts
        self._edge += report.edge_conflicts
        self._plan_calls += report.plan_calls
        self._plan_seconds += report.plan_seconds
        self._plan_max = max(self._plan_max, report.plan_seconds)
        self._track_deadlocks(report)
        self._aborted += len(report.aborted_tasks)
        self._rejections += report.energy_rejections
        self._stranded += len(report.stranded)
        self._preempted += len(report.preempted)
        self._obstacle_blocks += report.obstacle_blocks
        self._obstacle_replans += report.obstacle_replans
        self._blockages += report.blockages_started
        battery = self.sim.world.battery
        if battery is not None:
            low = min(r.battery for r in self.sim.world.robots.values()) / battery.capacity
            self._min_battery = min(self._min_battery, low)

        w = self._window
        w.ticks += 1
        w.deliveries += len(report.delivered)
        w.conflicts += report.vertex_conflicts + report.edge_conflicts
        w.blocked += len(report.blocked)
        w.moves += report.moved
        w.plan_ms += report.plan_seconds * 1e3
        if w.ticks == cfg.window:
            self._flush_window(report.t)

    # ----------------------------------------------------------------- results
    def result(self) -> RunMetrics:
        cfg = self.config
        world = self.sim.world
        measured = self._measured
        n = len(world.robots)
        robot_ticks = max(1, measured * n)

        tasks = [t for t in world.tasks.values() if t.created_t >= cfg.warmup_ticks]
        delivered = [t for t in tasks if t.delivered_t is not None]
        times = np.array([t.delivery_time for t in delivered], dtype=np.float64)
        waits = np.array([t.wait_time for t in tasks if t.wait_time is not None], dtype=np.float64)

        cur = self._robot_counters()
        base = self._base
        dist = cur.distance - base.distance
        conflicts = self._vertex + self._edge
        nan = math.nan

        def pct(q: float) -> float:
            return float(np.percentile(times, q)) if times.size else nan

        return RunMetrics(
            measured_ticks=measured,
            tasks_created=len(tasks),
            tasks_delivered=len(delivered),
            completion_rate=len(delivered) / len(tasks) if tasks else nan,
            throughput_per_1k=1e3 * len(delivered) / measured if measured else nan,
            delivery_time_mean=float(times.mean()) if times.size else nan,
            delivery_time_p50=pct(50),
            delivery_time_p95=pct(95),
            delivery_time_max=float(times.max()) if times.size else nan,
            assign_wait_mean=float(waits.mean()) if waits.size else nan,
            distance_total=dist,
            distance_per_delivery=dist / len(delivered) if delivered else nan,
            idle_frac=(cur.idle - base.idle) / robot_ticks,
            busy_frac=(cur.busy - base.busy) / robot_ticks,
            wait_frac=(cur.wait - base.wait) / robot_ticks,
            blocked_frac=(cur.blocked - base.blocked) / robot_ticks,
            vertex_conflicts=self._vertex,
            edge_conflicts=self._edge,
            conflicts_total=conflicts,
            conflicts_per_1k=1e3 * conflicts / measured if measured else nan,
            deadlock_events=self._deadlock_events,
            deadlocked_robot_ticks=self._deadlocked_robot_ticks,
            max_concurrent_deadlocked=self._max_deadlocked,
            plan_calls=self._plan_calls,
            plan_ms_total=self._plan_seconds * 1e3,
            plan_ms_mean=1e3 * self._plan_seconds / self._plan_calls if self._plan_calls else 0.0,
            plan_ms_max=self._plan_max * 1e3,
            charging_frac=(cur.charging - base.charging) / robot_ticks,
            charge_sessions=cur.sessions - base.sessions,
            energy_per_delivery=(
                (cur.energy - base.energy) / len(delivered)
                if delivered and world.battery is not None
                else nan
            ),
            min_battery_frac=self._min_battery if math.isfinite(self._min_battery) else nan,
            aborted_tasks=self._aborted,
            energy_rejections=self._rejections,
            stranded_robots=self._stranded,
            preemptions=self._preempted,
            obstacle_blocks=self._obstacle_blocks,
            obstacle_replans=self._obstacle_replans,
            blockage_events=self._blockages,
        )

    def timeseries(self) -> list[dict[str, Any]]:
        """Windowed time series, including a final partial window."""
        if self._window.ticks:
            self._flush_window(self.sim.world.t - 1)
        return list(self._series)

    # ---------------------------------------------------------------- internals
    def _track_deadlocks(self, report: TickReport) -> None:
        in_cycle = robots_in_cycles(report.waits_for)
        threshold = self.config.deadlock_ticks
        streaks: dict[RobotId, int] = {}
        for rid in in_cycle:
            streak = self._in_cycle_for.get(rid, 0) + 1
            streaks[rid] = streak
            if streak == threshold:
                self._deadlock_events += 1
        self._in_cycle_for = streaks
        now = sum(1 for s in streaks.values() if s >= threshold)
        self._deadlocked_robot_ticks += now
        self._max_deadlocked = max(self._max_deadlocked, now)

    def _flush_window(self, t_end: int) -> None:
        w = self._window
        world = self.sim.world
        robots = world.robots.values()
        self._series.append(
            {
                "t_end": t_end,
                "ticks": w.ticks,
                "deliveries": w.deliveries,
                "throughput_per_1k": 1e3 * w.deliveries / w.ticks,
                "conflicts": w.conflicts,
                "blocked": w.blocked,
                "moves": w.moves,
                "plan_ms": w.plan_ms,
                "pending": len(world.pending),
                "idle_robots": sum(r.state in FREE_STATES for r in robots),
                "moving_robots": sum(r.state in MOVING_STATES for r in robots),
                "deadlocked_robots": sum(
                    s >= self.config.deadlock_ticks for s in self._in_cycle_for.values()
                ),
            }
        )
        self._window = _Window()

    def _robot_counters(self) -> _Counters:
        c = _Counters()
        for r in self.sim.world.robots.values():
            c.distance += r.distance
            c.idle += r.idle_ticks
            c.busy += r.busy_ticks
            c.wait += r.wait_ticks
            c.blocked += r.blocked_ticks
            c.charging += r.charging_ticks
            c.sessions += r.charge_sessions
            c.energy += r.energy_used
        return c
