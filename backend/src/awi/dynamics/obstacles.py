"""Dynamic obstacles: people/forklifts walking the aisles, and temporary blockages.

* **Humans** (pickers, forklift drivers) start on random aisle cells. Each tick a
  human moves with probability ``human_move_prob``, keeping its direction with
  probability ``human_momentum`` (otherwise it turns randomly). Humans walk on
  floor and pick-aisle cells, never onto stations or chargers, and never into a
  cell occupied by a robot, another human or a blockage. Robots only know where
  humans are *now*, not where they will go.
* **Safety zone**: robots may not *enter* any cell within ``safety_radius``
  (Manhattan distance) of a person, in line with AMR safety practice (speed and
  separation monitoring). A robot already inside the zone (because the person
  walked up to it) is never pushed through the person: it may stay, and
  coordinated planners move it *out* of the zone. So people always keep free
  cells around them. Without the zone, robots queue tightly around a person in a
  narrow aisle, the person can never move again, and the robots wait forever.
* **Blockages** (a spill, a pallet, maintenance) appear at random with rate
  ``blockage_rate`` per tick: ``blockage_length`` consecutive aisle cells close for
  ``blockage_duration`` ticks. A blockage is only placed if every remaining open
  cell stays reachable, so goals never become permanently unreachable. Blockages
  can also be added by hand (e.g. from the dashboard).

Everything is driven by the dedicated ``"dynamics"`` random stream, so runs are
reproducible and identical across planners.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Collection, Iterable
from dataclasses import dataclass, field

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from awi.core.grid import DIRECTIONS, CellType, Grid
from awi.core.layout import Layout
from awi.core.types import Cell

WALKABLE = (CellType.FLOOR, CellType.PICK)


class DynamicsConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    enabled: bool = False
    humans: int = Field(0, ge=0, le=50)
    human_move_prob: float = Field(0.6, ge=0, le=1)
    human_momentum: float = Field(0.7, ge=0, le=1)
    safety_radius: int = Field(1, ge=0, le=3, description="Robot keep-out distance (cells)")
    blockage_rate: float = Field(0.0, ge=0, le=1, description="New blockages per tick")
    blockage_length: int = Field(3, ge=1, le=20)
    blockage_duration: int = Field(150, ge=1)
    max_blockages: int = Field(2, ge=0, description="Concurrent random blockages")


@dataclass(slots=True)
class Human:
    pos: Cell
    heading: Cell = (0, 1)


@dataclass(slots=True)
class DynamicsEvents:
    blockages_started: list[list[Cell]] = field(default_factory=list)
    blockages_cleared: list[Cell] = field(default_factory=list)


class Dynamics:
    def __init__(self, cfg: DynamicsConfig, layout: Layout, rng: np.random.Generator) -> None:
        self.cfg = cfg
        self.layout = layout
        self.rng = rng
        self.humans: list[Human] = []
        self.blockages: dict[Cell, int] = {}
        """Blocked cell -> tick at which it reopens."""
        self._random_groups: list[tuple[Cell, ...]] = []
        grid = layout.grid
        self._walkable = [c for c in grid.passable_cells() if grid.cell_type(c) in WALKABLE]
        self._walkable_set = frozenset(self._walkable)

    # ---------------------------------------------------------------- queries
    def blocked_cells(self) -> set[Cell]:
        """Every cell a robot may not enter right now: people, their safety zone and
        blockages."""
        return self.safety_zone() | set(self.blockages)

    def safety_zone(self) -> set[Cell]:
        radius = self.cfg.safety_radius
        grid = self.layout.grid
        zone: set[Cell] = set()
        for h in self.humans:
            r0, c0 = h.pos
            for dr in range(-radius, radius + 1):
                span = radius - abs(dr)
                for dc in range(-span, span + 1):
                    cell = (r0 + dr, c0 + dc)
                    if grid.is_passable(cell):
                        zone.add(cell)
        return zone

    # ------------------------------------------------------------------ setup
    def spawn_humans(self, occupied: Collection[Cell]) -> None:
        free = [c for c in self._walkable if c not in occupied]
        n = min(self.cfg.humans, len(free))
        for i in sorted(self.rng.choice(len(free), size=n, replace=False).tolist()):
            heading = DIRECTIONS[int(self.rng.integers(len(DIRECTIONS)))]
            self.humans.append(Human(free[i], heading))

    # ------------------------------------------------------------------- tick
    def step(self, t: int, robot_cells: Collection[Cell]) -> DynamicsEvents:
        events = DynamicsEvents()
        self._expire(t, events)
        if self.cfg.blockage_rate and self.rng.random() < self.cfg.blockage_rate:
            active = sum(1 for g in self._random_groups if g[0] in self.blockages)
            if active < self.cfg.max_blockages:
                cells = self._random_blockage(robot_cells)
                if cells:
                    self.add_blockage(cells, t + self.cfg.blockage_duration)
                    self._random_groups.append(tuple(cells))
                    events.blockages_started.append(cells)
        self._move_humans(robot_cells)
        return events

    def add_blockage(self, cells: Iterable[Cell], until: int) -> list[Cell]:
        """Block ``cells`` until tick ``until`` (only walkable cells are accepted)."""
        added = []
        for c in cells:
            if c in self._walkable_set:
                self.blockages[c] = max(until, self.blockages.get(c, until))
                added.append(c)
        return added

    def clear_blockage(self, cell: Cell) -> bool:
        return self.blockages.pop(cell, None) is not None

    # -------------------------------------------------------------- internals
    def _expire(self, t: int, events: DynamicsEvents) -> None:
        for c in [c for c, until in self.blockages.items() if until <= t]:
            del self.blockages[c]
            events.blockages_cleared.append(c)
        self._random_groups = [g for g in self._random_groups if g[0] in self.blockages]

    def _move_humans(self, robot_cells: Collection[Cell]) -> None:
        cfg = self.cfg
        taken = set(robot_cells) | set(self.blockages) | {h.pos for h in self.humans}
        for h in self.humans:
            if self.rng.random() >= cfg.human_move_prob:
                continue
            options = [
                d
                for d in DIRECTIONS
                if (nb := (h.pos[0] + d[0], h.pos[1] + d[1])) in self._walkable_set
                and nb not in taken
            ]
            if not options:
                continue
            if h.heading in options and self.rng.random() < cfg.human_momentum:
                d = h.heading
            else:
                d = options[int(self.rng.integers(len(options)))]
            taken.discard(h.pos)
            h.pos, h.heading = (h.pos[0] + d[0], h.pos[1] + d[1]), d
            taken.add(h.pos)

    def _random_blockage(self, robot_cells: Collection[Cell]) -> list[Cell] | None:
        grid = self.layout.grid
        avoid = set(robot_cells) | self.blocked_cells() | {h.pos for h in self.humans}
        for _ in range(20):  # a few attempts to find a valid segment
            start = self._walkable[int(self.rng.integers(len(self._walkable)))]
            d = (0, 1) if self.rng.random() < 0.5 else (1, 0)
            cells = [
                (start[0] + k * d[0], start[1] + k * d[1]) for k in range(self.cfg.blockage_length)
            ]
            closed = set(self.blockages) | set(cells)
            if all(c in self._walkable_set and c not in avoid for c in cells) and _stays_connected(
                grid, closed
            ):
                return cells
        return None


def _stays_connected(grid: Grid, closed: set[Cell]) -> bool:
    """True if all open passable cells are still mutually reachable."""
    open_cells = [c for c in grid.passable_cells() if c not in closed]
    if not open_cells:
        return False
    seen = {open_cells[0]}
    queue = deque([open_cells[0]])
    while queue:
        cur = queue.popleft()
        for nb in grid.neighbors(cur):
            if nb not in seen and nb not in closed:
                seen.add(nb)
                queue.append(nb)
    return len(seen) == len(open_cells)
