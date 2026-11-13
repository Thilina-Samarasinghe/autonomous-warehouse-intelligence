"""Robot routes: ordered pick and drop stops.

A plain task is the route ``(pick t, drop t)``. A *batch* lets one robot carry
several items per trip, e.g. ``(pick a, pick b, drop b, drop a)``. Routes are built
by task assigners (the OR-Tools batching assigner in particular) and executed stop
by stop by the engine.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum

from awi.core.task import Task
from awi.core.types import Cell, TaskId


class StopKind(StrEnum):
    PICK = "pick"
    DROP = "drop"


@dataclass(frozen=True, slots=True)
class Stop:
    kind: StopKind
    task_id: TaskId
    cell: Cell


type Route = tuple[Stop, ...]


def single_task_route(task: Task) -> Route:
    return (Stop(StopKind.PICK, task.id, task.pickup), Stop(StopKind.DROP, task.id, task.dropoff))


def route_tasks(route: Sequence[Stop]) -> list[TaskId]:
    """Task ids in order of their pick stop."""
    return [s.task_id for s in route if s.kind is StopKind.PICK]


def validate_route(route: Sequence[Stop], tasks: Mapping[TaskId, Task], capacity: int) -> None:
    """Raise ``ValueError`` unless every task is picked once, then dropped once, at
    its own pick and drop cells, never carrying more than ``capacity`` items."""
    if not route:
        raise ValueError("empty route")
    picked: set[TaskId] = set()
    dropped: set[TaskId] = set()
    load = 0
    for stop in route:
        task = tasks.get(stop.task_id)
        if task is None:
            raise ValueError(f"unknown task {stop.task_id}")
        if stop.kind is StopKind.PICK:
            if stop.task_id in picked or stop.cell != task.pickup:
                raise ValueError(f"bad pick stop for task {stop.task_id}")
            picked.add(stop.task_id)
            load += 1
            if load > capacity:
                raise ValueError(f"route exceeds capacity {capacity}")
        else:
            if stop.task_id not in picked or stop.task_id in dropped or stop.cell != task.dropoff:
                raise ValueError(f"bad drop stop for task {stop.task_id}")
            dropped.add(stop.task_id)
            load -= 1
    if picked != dropped:
        raise ValueError("route picks tasks it never drops")
