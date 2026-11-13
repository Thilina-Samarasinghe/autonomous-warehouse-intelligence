"""ASCII renderer for debugging and the CLI.

Legend: ``#`` shelf · ``+`` pick point · ``D`` drop station · ``C`` charger ·
``.`` floor · robots shown as ``0-9a-z`` then ``A-Z`` (cycling).
"""

from __future__ import annotations

from awi.core.grid import CELL_CHARS, CellType
from awi.core.world import World

ROBOT_GLYPHS = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"


def robot_glyph(robot_id: int) -> str:
    return ROBOT_GLYPHS[robot_id % len(ROBOT_GLYPHS)]


def render_ascii(world: World, *, legend: bool = True) -> str:
    grid = world.layout.grid
    rows = [[CELL_CHARS[CellType(v)] for v in row] for row in grid.cells.tolist()]
    for r in world.robots.values():
        rows[r.pos[0]][r.pos[1]] = robot_glyph(r.id)
    lines = ["".join(row) for row in rows]
    if legend:
        lines.append(
            f"t={world.t}  pending={len(world.pending)}  "
            f"delivered={sum(r.deliveries for r in world.robots.values())}"
        )
        for r in sorted(world.robots.values(), key=lambda r: r.id):
            cargo = " [cargo]" if r.carrying else ""
            lines.append(f"  {robot_glyph(r.id)}: {r.state.value:<10} at {r.pos}{cargo}")
    return "\n".join(lines)
