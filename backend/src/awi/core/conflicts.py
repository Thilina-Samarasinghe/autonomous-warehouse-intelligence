"""Physical conflict accounting: would-be collisions and wait-for deadlocks.

Definitions (standard multi-agent pathfinding terms):

* **Vertex conflict**: two robots intend to occupy the same cell at ``t+1``. A
  robot that stays put "intends" its own cell, so moving into a parked robot counts.
* **Edge conflict**: two robots intend to swap cells between ``t`` and ``t+1``.

The engine never lets these happen: it rejects the moves and the robots wait. The
*count* of conflicting intents is what a fleet without central arbitration would
have collided on, and it is the "collisions" metric of the experiments. A robot that
keeps pushing into a blocked cell is counted on every tick, so the unit is
conflict-ticks.

A **wait-for graph** links every blocked robot to the robot occupying (or winning)
the cell it wanted. Each blocked robot has exactly one successor. A cycle in this
graph is a deadlock: no robot in it can move until one of them changes its plan.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Mapping

from awi.core.types import Cell, RobotId


def count_conflicts(
    current: Mapping[RobotId, Cell], intent: Mapping[RobotId, Cell]
) -> tuple[int, int]:
    """Return ``(vertex_conflicts, edge_conflicts)`` as counts of robot pairs."""
    claims = Counter(intent.values())
    vertex = sum(k * (k - 1) // 2 for k in claims.values() if k > 1)
    at_cell = {c: rid for rid, c in current.items()}
    edge = 0
    for rid, dest in intent.items():
        if dest == current[rid]:
            continue
        other = at_cell.get(dest)
        if other is not None and other > rid and intent[other] == current[rid]:
            edge += 1
    return vertex, edge


def wait_for_graph(
    current: Mapping[RobotId, Cell],
    intent: Mapping[RobotId, Cell],
    final: Mapping[RobotId, Cell],
    blocked: set[RobotId],
) -> dict[RobotId, RobotId]:
    """Map each blocked robot to the robot holding the cell it wanted."""
    final_at = {c: rid for rid, c in final.items()}
    current_at = {c: rid for rid, c in current.items()}
    graph: dict[RobotId, RobotId] = {}
    for rid in blocked:
        target = intent[rid]
        holder = final_at.get(target, current_at.get(target))
        if holder is not None and holder != rid:
            graph[rid] = holder
    return graph


def robots_in_cycles(graph: Mapping[RobotId, RobotId]) -> set[RobotId]:
    """Robots on a cycle of a functional graph (each node has at most one successor)."""
    on_cycle: set[RobotId] = set()
    done: set[RobotId] = set()
    for start in graph:
        if start in done:
            continue
        order: dict[RobotId, int] = {}
        node: RobotId | None = start
        while node is not None and node not in done and node not in order:
            order[node] = len(order)
            node = graph.get(node)
        if node is not None and node in order:  # closed a new cycle
            cycle_start = order[node]
            on_cycle.update(n for n, i in order.items() if i >= cycle_start)
        done.update(order)
    return on_cycle
