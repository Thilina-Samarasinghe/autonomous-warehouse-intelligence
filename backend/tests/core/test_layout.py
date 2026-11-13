import pytest
from pydantic import ValidationError

from awi.core.grid import CellType, Grid
from awi.core.layout import Layout, LayoutConfig, generate_layout
from awi.core.types import manhattan


def test_default_layout_dimensions_and_counts() -> None:
    layout = generate_layout()
    cfg = LayoutConfig()
    assert (layout.grid.height, layout.grid.width) == (cfg.height, cfg.width) == (18, 29)
    assert len(layout.shelves) == 4 * 4 * 6 * 2
    assert len(layout.drop_stations) == cfg.num_drop_stations
    assert len(layout.chargers) == cfg.num_chargers


@pytest.mark.parametrize(
    "cfg",
    [
        LayoutConfig(),
        LayoutConfig(
            shelf_block_rows=1,
            shelf_segments=1,
            shelf_length=1,
            num_drop_stations=2,
            num_chargers=2,
        ),
        LayoutConfig(shelf_block_rows=6, shelf_segments=5, shelf_length=8, num_drop_stations=5),
    ],
)
def test_layout_is_valid(cfg: LayoutConfig) -> None:
    layout = generate_layout(cfg)
    grid = layout.grid
    assert grid.is_connected()
    assert set(layout.pick_point) == set(layout.shelves)
    for shelf, pick in layout.pick_point.items():
        assert grid.cell_type(shelf) is CellType.SHELF
        assert grid.cell_type(pick) is CellType.PICK
        assert manhattan(shelf, pick) == 1
    assert all(r == 0 and grid.cell_type((r, c)) is CellType.DROP for r, c in layout.drop_stations)
    assert all(
        r == grid.height - 1 and grid.cell_type((r, c)) is CellType.CHARGER
        for r, c in layout.chargers
    )
    assert layout.spawn_cells
    assert all(grid.cell_type(c) is CellType.FLOOR for c in layout.spawn_cells)


def test_every_pick_point_reachable_from_every_drop() -> None:
    layout = generate_layout()
    for drop in layout.drop_stations:
        dist = layout.grid.bfs_distances(drop)
        assert all(dist[p] > 0 for p in layout.pick_point.values())


def test_config_rejects_too_many_stations() -> None:
    with pytest.raises(ValidationError):
        LayoutConfig(shelf_segments=1, shelf_length=1, num_drop_stations=10)


def test_config_is_frozen_and_strict() -> None:
    with pytest.raises(ValidationError):
        LayoutConfig(unknown_field=1)  # type: ignore[call-arg]


def test_layout_from_grid() -> None:
    grid = Grid.from_ascii(
        """
        D.....
        .++++.
        .####.
        ..C...
        """
    )
    layout = Layout.from_grid(grid)
    assert len(layout.shelves) == 4
    assert layout.pick_point[(2, 1)] == (1, 1)
    assert layout.drop_stations == ((0, 0),)
    assert layout.chargers == ((3, 2),)
