"""Procedural warehouse layout generator.

Layout (example: 2 block rows, 2 segments of length 4)::

    D......D......D     <- top zone: drop stations on row 0
    ...............
    ...............
    .++++.++++.++++     <- aisle: pick points above upper shelves
    .####.####.####     <- double-sided rack (block 0)
    .####.####.####
    .++++.++++.++++     <- shared aisle between blocks
    .####.####.####     <- block 1
    .####.####.####
    .++++.++++.++++
    ...............     <- bottom zone: robot spawn / parking
    ..C....C....C..     <- chargers on the last row

Vertical cross-aisles separate rack segments, and the outer columns are aisles,
so every pick point can be reached from both sides.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, model_validator

from awi.core.grid import CellType, Grid
from awi.core.types import Cell


class LayoutConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    shelf_block_rows: int = Field(4, ge=1, le=20, description="Rows of double-sided racks")
    shelf_segments: int = Field(4, ge=1, le=20, description="Rack segments per block row")
    shelf_length: int = Field(6, ge=1, le=40, description="Shelf cells per segment")
    top_zone_rows: int = Field(3, ge=1, le=10, description="Open rows holding drop stations")
    bottom_zone_rows: int = Field(2, ge=1, le=10, description="Open rows holding chargers")
    num_drop_stations: int = Field(3, ge=1)
    num_chargers: int = Field(4, ge=1)

    @property
    def width(self) -> int:
        return 1 + self.shelf_segments * (self.shelf_length + 1)

    @property
    def height(self) -> int:
        return self.top_zone_rows + 1 + 3 * self.shelf_block_rows + self.bottom_zone_rows

    @model_validator(mode="after")
    def _check_station_counts(self) -> LayoutConfig:
        if self.num_drop_stations > self.width or self.num_chargers > self.width:
            raise ValueError(f"at most {self.width} drop stations / chargers fit this layout")
        return self


@dataclass(frozen=True, slots=True)
class Layout:
    grid: Grid
    shelves: tuple[Cell, ...]
    pick_point: Mapping[Cell, Cell]
    """Shelf cell -> aisle cell the robot stands on to pick from it."""
    drop_stations: tuple[Cell, ...]
    chargers: tuple[Cell, ...]
    spawn_cells: tuple[Cell, ...]
    """Plain floor cells in the bottom zone, used for initial robot placement."""

    @classmethod
    def from_grid(cls, grid: Grid) -> Layout:
        """Derive a layout from an arbitrary grid (useful for hand-made test maps).

        Each shelf is served by its first adjacent PICK cell (up/down/left/right
        order). Shelves without one are ignored.
        """
        pick_point: dict[Cell, Cell] = {}
        for shelf in grid.cells_of_type(CellType.SHELF):
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nb = (shelf[0] + dr, shelf[1] + dc)
                if grid.in_bounds(nb) and grid.cell_type(nb) is CellType.PICK:
                    pick_point[shelf] = nb
                    break
        return cls(
            grid=grid,
            shelves=tuple(pick_point),
            pick_point=pick_point,
            drop_stations=tuple(grid.cells_of_type(CellType.DROP)),
            chargers=tuple(grid.cells_of_type(CellType.CHARGER)),
            spawn_cells=tuple(grid.cells_of_type(CellType.FLOOR)),
        )


def _spread(n: int, width: int) -> list[int]:
    """``n`` distinct column indices spread evenly across ``width`` columns."""
    cols = np.linspace(0, width - 1, n + 2)[1:-1] if n < width else np.arange(width)
    return sorted({int(c) for c in np.rint(cols)})


def generate_layout(cfg: LayoutConfig | None = None) -> Layout:
    cfg = cfg or LayoutConfig()
    h, w, seg_len = cfg.height, cfg.width, cfg.shelf_length
    cells = np.full((h, w), CellType.FLOOR, dtype=np.uint8)

    shelves: list[Cell] = []
    pick_point: dict[Cell, Cell] = {}
    for b in range(cfg.shelf_block_rows):
        upper = cfg.top_zone_rows + 1 + 3 * b
        lower = upper + 1
        for s in range(cfg.shelf_segments):
            start = 1 + s * (seg_len + 1)
            for col in range(start, start + seg_len):
                for row, access in ((upper, upper - 1), (lower, lower + 1)):
                    cells[row, col] = CellType.SHELF
                    cells[access, col] = CellType.PICK
                    shelves.append((row, col))
                    pick_point[(row, col)] = (access, col)

    drops = [(0, c) for c in _spread(cfg.num_drop_stations, w)]
    chargers = [(h - 1, c) for c in _spread(cfg.num_chargers, w)]
    for r, c in drops:
        cells[r, c] = CellType.DROP
    for r, c in chargers:
        cells[r, c] = CellType.CHARGER

    grid = Grid(cells)
    if not grid.is_connected():  # pragma: no cover - guaranteed by construction
        raise RuntimeError("generated layout is not connected")

    bottom_start = h - cfg.bottom_zone_rows
    spawn = tuple(
        (r, c)
        for r in range(bottom_start, h)
        for c in range(w)
        if grid.cell_type((r, c)) is CellType.FLOOR
    )
    shelves.sort()
    return Layout(
        grid=grid,
        shelves=tuple(shelves),
        pick_point=pick_point,
        drop_stations=tuple(drops),
        chargers=tuple(chargers),
        spawn_cells=spawn,
    )
