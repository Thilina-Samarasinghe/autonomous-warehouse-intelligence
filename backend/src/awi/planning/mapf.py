"""Shared multi-agent pathfinding (MAPF) model for the coordinated planners.

Each replan builds a snapshot of the fleet:

* **Task agents**: robots heading to a goal (pickup / drop / charger).
* **Idle agents**: idle robots. Their goal is their own cell, so they stay put
  unless moving aside is cheaper for the fleet. This lets coordinated planners clear
  idle robots out of aisles.
* **Static cells**: robots that are picking or dropping, or that are already on
  their goal. They are obstacles for the whole window.

Paths are lists of positions for ``t = 0..T``. After ``T`` a robot stays on its last
cell ("stay at goal" semantics), which is how conflicts beyond a path's end are found.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum

from awi.core.robot import MOVING_STATES, RobotState
from awi.core.types import Cell, RobotId
from awi.core.world import World


class AgentKind(StrEnum):
    TASK = "task"
    IDLE = "idle"


@dataclass(frozen=True, slots=True)
class Agent:
    id: RobotId
    start: Cell
    goal: Cell
    kind: AgentKind


@dataclass(frozen=True, slots=True)
class Conflict:
    a: RobotId
    b: RobotId
    t: int
    """Time of the vertex conflict, or arrival time of the swap."""
    cell: Cell
    """Vertex: the shared cell. Edge: the cell ``a`` moves *from* (b moves into it)."""
    to: Cell | None = None
    """Edge conflicts only: the cell ``a`` moves *to*."""

    @property
    def is_edge(self) -> bool:
        return self.to is not None


@dataclass(frozen=True, slots=True)
class Instance:
    agents: tuple[Agent, ...]
    static: frozenset[Cell]


def build_instance(world: World) -> Instance:
    agents: list[Agent] = []
    static: set[Cell] = set()
    for rid in sorted(world.robots):
        r = world.robots[rid]
        if r.state in MOVING_STATES and r.goal is not None and r.pos != r.goal:
            agents.append(Agent(rid, r.pos, r.goal, AgentKind.TASK))
        elif r.state is RobotState.IDLE:
            agents.append(Agent(rid, r.pos, r.pos, AgentKind.IDLE))
        else:
            static.add(r.pos)
    return Instance(tuple(agents), frozenset(static))


def pos_at(path: Sequence[Cell], t: int) -> Cell:
    return path[t] if t < len(path) else path[-1]


def pad(path: Sequence[Cell], horizon: int) -> list[Cell]:
    """Path cut or extended (staying at its end) to exactly ``horizon + 1`` cells."""
    out = list(path[: horizon + 1])
    out.extend([out[-1]] * (horizon + 1 - len(out)))
    return out


def pad_paths(paths: Mapping[RobotId, Sequence[Cell]], horizon: int) -> dict[RobotId, list[Cell]]:
    return {rid: pad(p, horizon) for rid, p in paths.items()}


def find_conflicts(
    paths: Mapping[RobotId, Sequence[Cell]], horizon: int, *, first_only: bool = False
) -> list[Conflict]:
    """Vertex and edge conflicts within ``0..horizon``, earliest first."""
    padded = pad_paths(paths, horizon)
    ids = sorted(padded)
    out: list[Conflict] = []
    for t in range(horizon + 1):
        at: dict[Cell, RobotId] = {}
        for rid in ids:
            c = padded[rid][t]
            other = at.get(c)
            if other is not None:
                out.append(Conflict(other, rid, t, c))
                if first_only:
                    return out
            else:
                at[c] = rid
        if t == 0:
            continue
        prev_at = {padded[rid][t - 1]: rid for rid in ids}
        for rid in ids:
            u, v = padded[rid][t - 1], padded[rid][t]
            if u == v:
                continue
            other = prev_at.get(v)
            if other is not None and other > rid and padded[other][t] == u:
                out.append(Conflict(rid, other, t, u, v))
                if first_only:
                    return out
    return out


def count_conflicts_of(rid: RobotId, padded: Mapping[RobotId, Sequence[Cell]]) -> int:
    """Conflicts between robot ``rid`` and every other path (all equally padded)."""
    mine = padded[rid]
    n = 0
    for other, theirs in padded.items():
        if other == rid:
            continue
        for t in range(len(mine)):
            if mine[t] == theirs[t] or (
                t and mine[t] == theirs[t - 1] and mine[t - 1] == theirs[t]
            ):
                n += 1
    return n


def occupancy_table(
    padded: Mapping[RobotId, Sequence[Cell]], exclude: RobotId
) -> dict[tuple[Cell, int], int]:
    """Conflict-avoidance table ``(cell, t) -> robots there``, excluding one robot."""
    table: dict[tuple[Cell, int], int] = {}
    for rid, path in padded.items():
        if rid != exclude:
            for t, c in enumerate(path):
                table[(c, t)] = table.get((c, t), 0) + 1
    return table


def static_violations(
    paths: Mapping[RobotId, Sequence[Cell]], static: frozenset[Cell], horizon: int
) -> list[tuple[RobotId, int]]:
    """``(robot, t)`` pairs where a planned robot enters a static robot's cell."""
    return [
        (rid, t)
        for rid, path in paths.items()
        for t in range(1, horizon + 1)
        if pos_at(path, t) in static
    ]


def to_engine_paths(
    instance: Instance, paths: Mapping[RobotId, Sequence[Cell]], commit_steps: int
) -> dict[RobotId, list[Cell]]:
    """Convert joint MAPF paths into engine paths (future cells only).

    Every agent gets its first ``commit_steps`` positions, even idle robots that
    only wait, so a stale path from an earlier plan can never survive a replan.
    Task agents stop at their goal, where the engine starts their service.
    """
    out: dict[RobotId, list[Cell]] = {}
    for agent in instance.agents:
        path = paths[agent.id]
        steps = [pos_at(path, t) for t in range(1, commit_steps + 1)]
        if agent.kind is AgentKind.TASK and agent.goal in steps:
            steps = steps[: steps.index(agent.goal) + 1]
        out[agent.id] = steps
    return out
