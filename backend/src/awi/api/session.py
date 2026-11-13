"""Live simulation sessions for the API and the dashboard.

A :class:`Session` wraps one :class:`~awi.core.engine.Simulation` plus a metrics
collector. While running, an asyncio task advances it at ``speed`` ticks per second
and publishes frames to every subscriber at up to ``MAX_FPS`` frames per second.
The simulation steps run in a worker thread (``asyncio.to_thread``) under a lock,
so a slow planning tick never blocks the server, and REST reads always see a
consistent state. Subscribers get bounded queues: a slow viewer skips old frames
instead of slowing the simulation down.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import threading
import time
import uuid
from typing import Any

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.grid import CellType
from awi.core.layout import Layout
from awi.core.types import Cell
from awi.experiments.registry import ASSIGNERS, PLANNERS, make_assigner, make_planner
from awi.experiments.runner import deep_merge
from awi.metrics.collector import MetricsCollector

MAX_FPS = 20
PATH_PREVIEW = 12  # planned cells sent per robot
KPI_FIELDS = (
    "throughput_per_1k",
    "delivery_time_mean",
    "delivery_time_p95",
    "conflicts_total",
    "blocked_frac",
    "idle_frac",
    "charging_frac",
    "deadlock_events",
    "obstacle_blocks",
    "plan_ms_mean",
    "stranded_robots",
)


def layout_payload(layout: Layout) -> dict[str, Any]:
    """Static map as JSON: cell types, stations, chargers, parking cells."""
    grid = layout.grid
    return {
        "width": grid.width,
        "height": grid.height,
        "cells": grid.cells.tolist(),
        "cell_types": {t.name.lower(): int(t) for t in CellType},
        "drop_stations": [_cell(c) for c in layout.drop_stations],
        "chargers": [_cell(c) for c in layout.chargers],
        "parking": [_cell(c) for c in layout.parking_cells],
    }


class SessionError(ValueError):
    """Invalid session request (unknown planner, bad config...)."""


def _clean(value: Any) -> Any:
    """JSON-safe: NaN / inf become ``None``."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _cell(c: Cell) -> list[int]:
    return [c[0], c[1]]


class Session:
    def __init__(
        self,
        planner: str,
        assigner: str,
        config: dict[str, Any] | None = None,
        name: str | None = None,
        speed: float = 10.0,
    ) -> None:
        if planner not in PLANNERS:
            raise SessionError(f"unknown planner {planner!r}; choose from {sorted(PLANNERS)}")
        if assigner not in ASSIGNERS:
            raise SessionError(f"unknown assigner {assigner!r}; choose from {sorted(ASSIGNERS)}")
        try:
            self.config = SimConfig.model_validate(
                deep_merge(SimConfig().model_dump(), config or {})
            )
        except ValueError as exc:
            raise SessionError(str(exc)) from None
        self.id = uuid.uuid4().hex[:12]
        self.name = name or f"{planner}+{assigner}"
        self.planner_name = planner
        self.assigner_name = assigner
        self.speed = speed
        self.running = False
        self.last_active = time.monotonic()
        self._lock = threading.Lock()
        self._task: asyncio.Task[None] | None = None
        self._subscribers: set[asyncio.Queue[dict[str, Any]]] = set()
        self._build()

    def _build(self) -> None:
        cfg = self.config
        self.sim = Simulation(
            cfg,
            planner=make_planner(self.planner_name, cfg),
            assigner=make_assigner(self.assigner_name, cfg),
        )
        self.metrics = MetricsCollector(self.sim)

    # ------------------------------------------------------------ simulation
    @property
    def watched(self) -> bool:
        return bool(self._subscribers)

    def touch(self) -> None:
        self.last_active = time.monotonic()

    def step(self, ticks: int = 1) -> None:
        self.touch()
        with self._lock:
            for _ in range(ticks):
                self.metrics.observe(self.sim.step())

    def reset(self) -> None:
        with self._lock:
            self._build()

    def add_blockage(self, cells: list[Cell], duration: int) -> list[Cell]:
        grid = self.sim.layout.grid
        valid = [c for c in cells if grid.in_bounds(c)]
        with self._lock:
            return self.sim.add_blockage(valid, duration)

    # -------------------------------------------------------------- payloads
    def status(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "planner": self.planner_name,
            "assigner": self.assigner_name,
            "t": self.sim.world.t,
            "running": self.running,
            "speed": self.speed,
            "num_robots": self.config.num_robots,
            "config": self.config.model_dump(),
        }

    def layout(self) -> dict[str, Any]:
        return layout_payload(self.sim.layout)

    def frame(self) -> dict[str, Any]:
        with self._lock:
            world = self.sim.world
            battery = world.battery
            robots = [
                {
                    "id": r.id,
                    "pos": _cell(r.pos),
                    "state": r.state.value,
                    "goal": _cell(r.goal) if r.goal else None,
                    "battery": round(100 * r.battery / battery.capacity, 1) if battery else None,
                    "carrying": len(r.cargo),
                    "path": [_cell(c) for c in list(r.path)[:PATH_PREVIEW]],
                }
                for r in sorted(world.robots.values(), key=lambda r: r.id)
            ]
            dyn = world.dynamics
            tasks = world.tasks.values()
            delivered = sum(t.delivered_t is not None for t in tasks)
            kpis = self.metrics.result().as_dict()
            return {
                "type": "frame",
                "session": self.id,
                "t": world.t,
                "running": self.running,
                "speed": self.speed,
                "robots": robots,
                "humans": [_cell(h.pos) for h in dyn.humans] if dyn else [],
                "blocked": [_cell(c) for c in sorted(dyn.blockages)] if dyn else [],
                "tasks": {
                    "generated": len(world.tasks),
                    "pending": len(world.pending),
                    "delivered": delivered,
                    "in_progress": len(world.tasks) - len(world.pending) - delivered,
                },
                "kpis": {k: _clean(kpis[k]) for k in KPI_FIELDS},
            }

    def full_metrics(self) -> dict[str, Any]:
        with self._lock:
            row = {k: _clean(v) for k, v in self.metrics.result().as_dict().items()}
            series = [{k: _clean(v) for k, v in w.items()} for w in self.metrics.timeseries()]
        return {"metrics": row, "timeseries": series}

    # ------------------------------------------------------------- streaming
    def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=2)
        self._subscribers.add(queue)
        self.touch()
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._subscribers.discard(queue)
        self.touch()

    def publish(self, message: dict[str, Any]) -> None:
        for queue in list(self._subscribers):
            if queue.full():
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()  # drop the stale frame
            queue.put_nowait(message)

    async def start(self, speed: float | None = None) -> None:
        if speed is not None:
            self.speed = speed
        if self.running:
            return
        self.running = True
        self._task = asyncio.create_task(self._run())

    async def pause(self) -> None:
        self.running = False
        task, self._task = self._task, None
        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        self.publish(self.frame())

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        while self.running:
            started = loop.time()
            interval = max(1.0 / MAX_FPS, 1.0 / self.speed)
            ticks = max(1, round(self.speed * interval))
            await asyncio.to_thread(self.step, ticks)
            self.publish(self.frame())
            await asyncio.sleep(max(0.0, interval - (loop.time() - started)))


class SessionManager:
    def __init__(self, max_sessions: int = 8) -> None:
        self.max_sessions = max_sessions
        self._sessions: dict[str, Session] = {}

    def create(self, **kwargs: Any) -> Session:
        """Create a session. At capacity, the least recently used session that
        nobody is watching and that isn't running is evicted (browsers often
        close without deleting their sessions)."""
        if len(self._sessions) >= self.max_sessions:
            idle = [s for s in self._sessions.values() if not s.watched and not s.running]
            if not idle:
                raise SessionError(f"at most {self.max_sessions} active sessions; delete one first")
            victim = min(idle, key=lambda s: s.last_active)
            del self._sessions[victim.id]
        session = Session(**kwargs)
        self._sessions[session.id] = session
        return session

    def get(self, session_id: str) -> Session | None:
        return self._sessions.get(session_id)

    def all(self) -> list[Session]:
        return list(self._sessions.values())

    async def delete(self, session_id: str) -> bool:
        session = self._sessions.pop(session_id, None)
        if session is None:
            return False
        await session.pause()
        return True

    async def close(self) -> None:
        for sid in list(self._sessions):
            await self.delete(sid)
