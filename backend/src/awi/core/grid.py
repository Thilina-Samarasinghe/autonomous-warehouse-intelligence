"""Static 4-connected occupancy grid of the warehouse floor."""

from __future__ import annotations

from collections import deque
from collections.abc import Collection, Iterator
from enum import IntEnum

import numpy as np
from numpy.typing import NDArray

from awi.core.types import Cell


class CellType(IntEnum):
    FLOOR = 0
    SHELF = 1  # impassable storage rack
    PICK = 2  # aisle cell from which a robot picks from an adjacent shelf
    DROP = 3  # delivery / packing station
    CHARGER = 4  # charging pad


CELL_CHARS: dict[CellType, str] = {
    CellType.FLOOR: ".",
    CellType.SHELF: "#",
    CellType.PICK: "+",
    CellType.DROP: "D",
    CellType.CHARGER: "C",
}
_CHAR_TO_TYPE = {ch: t for t, ch in CELL_CHARS.items()}

# Fixed neighbour order (up, down, left, right) keeps every search deterministic.
DIRECTIONS: tuple[Cell, ...] = ((-1, 0), (1, 0), (0, -1), (0, 1))


class Grid:
    """Immutable warehouse grid. Every cell except ``SHELF`` is traversable."""

    __slots__ = ("_cells", "_neighbors")

    def __init__(self, cells: NDArray[np.uint8]) -> None:
        if cells.ndim != 2 or cells.size == 0:
            raise ValueError("grid must be a non-empty 2D array")
        valid = {int(t) for t in CellType}
        if not set(np.unique(cells).tolist()) <= valid:
            raise ValueError("grid contains unknown cell types")
        self._cells: NDArray[np.uint8] = cells.astype(np.uint8, copy=True)
        self._cells.flags.writeable = False
        self._neighbors: dict[Cell, tuple[Cell, ...]] = {
            c: tuple(self._iter_neighbors(c)) for c in self.passable_cells()
        }

    # ------------------------------------------------------------- construction
    @classmethod
    def from_ascii(cls, text: str) -> Grid:
        lines = [ln.strip() for ln in text.strip().splitlines() if ln.strip()]
        if len({len(ln) for ln in lines}) != 1:
            raise ValueError("all rows must have the same width")
        try:
            data = [[int(_CHAR_TO_TYPE[ch]) for ch in ln] for ln in lines]
        except KeyError as exc:
            raise ValueError(f"unknown grid character {exc}") from None
        return cls(np.array(data, dtype=np.uint8))

    def to_ascii(self) -> str:
        return "\n".join(
            "".join(CELL_CHARS[CellType(v)] for v in row) for row in self._cells.tolist()
        )

    # ------------------------------------------------------------------ queries
    @property
    def cells(self) -> NDArray[np.uint8]:
        """Read-only view of the raw cell-type array (shape ``height x width``)."""
        return self._cells

    @property
    def height(self) -> int:
        return int(self._cells.shape[0])

    @property
    def width(self) -> int:
        return int(self._cells.shape[1])

    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.height and 0 <= c[1] < self.width

    def cell_type(self, c: Cell) -> CellType:
        return CellType(int(self._cells[c]))

    def is_passable(self, c: Cell) -> bool:
        return self.in_bounds(c) and int(self._cells[c]) != CellType.SHELF

    def neighbors(self, c: Cell) -> tuple[Cell, ...]:
        """Passable 4-neighbours of a passable cell (empty for shelves / out of bounds)."""
        return self._neighbors.get(c, ())

    def cells_of_type(self, t: CellType) -> list[Cell]:
        """All cells of a type in row-major order."""
        return [(int(r), int(c)) for r, c in np.argwhere(self._cells == t)]

    def passable_cells(self) -> list[Cell]:
        return [(int(r), int(c)) for r, c in np.argwhere(self._cells != CellType.SHELF)]

    # ----------------------------------------------------------------- searches
    def bfs_distances(self, source: Cell) -> NDArray[np.int32]:
        """Shortest step counts from ``source`` to every cell (-1 = unreachable)."""
        dist = np.full(self._cells.shape, -1, dtype=np.int32)
        if not self.is_passable(source):
            return dist
        dist[source] = 0
        queue: deque[Cell] = deque([source])
        while queue:
            cur = queue.popleft()
            d = int(dist[cur]) + 1
            for nb in self._neighbors[cur]:
                if dist[nb] < 0:
                    dist[nb] = d
                    queue.append(nb)
        return dist

    def bfs_path(
        self, start: Cell, goal: Cell, blocked: Collection[Cell] = ()
    ) -> list[Cell] | None:
        """Shortest path from ``start`` to ``goal``, excluding ``start``.

        Cells in ``blocked`` are avoided, except ``goal`` itself. Returns ``[]`` when
        ``start == goal`` and ``None`` when the goal is unreachable.
        """
        if not (self.is_passable(start) and self.is_passable(goal)):
            return None
        if start == goal:
            return []
        parent: dict[Cell, Cell] = {start: start}
        queue: deque[Cell] = deque([start])
        while queue:
            cur = queue.popleft()
            for nb in self._neighbors[cur]:
                if nb in parent or (nb in blocked and nb != goal):
                    continue
                parent[nb] = cur
                if nb == goal:
                    path = [nb]
                    while (prev := parent[path[-1]]) != start:
                        path.append(prev)
                    path.reverse()
                    return path
                queue.append(nb)
        return None

    def is_connected(self) -> bool:
        """True if every passable cell is reachable from every other one."""
        cells = self.passable_cells()
        if not cells:
            return False
        return bool(np.count_nonzero(self.bfs_distances(cells[0]) >= 0) == len(cells))

    # ---------------------------------------------------------------- internals
    def _iter_neighbors(self, c: Cell) -> Iterator[Cell]:
        for dr, dc in DIRECTIONS:
            nb = (c[0] + dr, c[1] + dc)
            if self.is_passable(nb):
                yield nb
