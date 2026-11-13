"""Primitive type aliases shared across the simulator."""

from __future__ import annotations

type Cell = tuple[int, int]
"""A grid coordinate as ``(row, col)``; row 0 is the top of the warehouse."""

type RobotId = int
type TaskId = int


def manhattan(a: Cell, b: Cell) -> int:
    """Manhattan (L1) distance between two cells."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
