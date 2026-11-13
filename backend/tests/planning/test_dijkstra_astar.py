import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from awi.core.grid import CellType, Grid
from awi.core.layout import generate_layout
from awi.core.types import Cell, manhattan
from awi.planning.astar import astar
from awi.planning.dijkstra import DistanceMaps, grid_to_graph, make_cell_costs

MAP = """
D.......
.######.
.++++++.
........
"""


def _valid(grid: Grid, start: Cell, path: list[Cell], goal: Cell) -> bool:
    prev = start
    for c in path:
        if not grid.is_passable(c) or manhattan(prev, c) != 1:
            return False
        prev = c
    return prev == goal


# ------------------------------------------------------------------- Dijkstra


def test_uniform_maps_equal_bfs_distances() -> None:
    layout = generate_layout()
    maps = DistanceMaps.for_layout(layout)
    assert maps.is_uniform
    for goal in [*layout.drop_stations, *list(layout.pick_point.values())[:10]]:
        bfs = layout.grid.bfs_distances(goal).astype(np.float64)
        bfs[bfs < 0] = np.inf
        assert np.array_equal(maps.to(goal), bfs)


def test_for_layout_precomputes_all_goals() -> None:
    layout = generate_layout()
    maps = DistanceMaps.for_layout(layout)
    expected = (
        len(set(layout.pick_point.values())) + len(layout.drop_stations) + len(layout.chargers)
    )
    assert maps.cached_goals == expected
    maps.to(layout.spawn_cells[0])  # lazily adds new goals
    assert maps.cached_goals == expected + 1


def test_maps_are_cached_and_read_only() -> None:
    maps = DistanceMaps(Grid.from_ascii(MAP))
    a = maps.to((0, 0))
    assert maps.to((0, 0)) is a
    with pytest.raises(ValueError):
        a[0, 0] = 5


def test_weighted_costs_are_per_entered_cell() -> None:
    grid = Grid.from_ascii(MAP)
    costs = make_cell_costs(grid, {CellType.PICK: 3.0})
    maps = DistanceMaps(grid, costs)
    assert not maps.is_uniform
    # (3,1) -> (2,1) enters one pick cell.
    assert maps.distance((3, 1), (2, 1)) == 3.0
    # Directed costs: leaving a pick cell into floor costs 1.
    assert maps.distance((2, 1), (3, 1)) == 1.0
    assert math.isinf(maps.distance((1, 1), (0, 0)))  # shelf


def test_invalid_costs_rejected() -> None:
    grid = Grid.from_ascii(MAP)
    with pytest.raises(ValueError):
        make_cell_costs(grid, {CellType.PICK: 0.0})
    with pytest.raises(ValueError):
        DistanceMaps(grid, np.ones((2, 2)))
    bad = make_cell_costs(grid)
    bad[0, 0] = -1
    with pytest.raises(ValueError):
        DistanceMaps(grid, bad)


def test_graph_structure() -> None:
    grid = Grid.from_ascii(MAP)
    g = grid_to_graph(grid)
    assert g.number_of_nodes() == len(grid.passable_cells())
    assert all(manhattan(u, v) == 1 for u, v in g.edges)
    assert g.has_edge((0, 0), (0, 1)) and g.has_edge((0, 1), (0, 0))


# ------------------------------------------------------------------------ A*


def test_astar_basic_cases() -> None:
    grid = Grid.from_ascii(MAP)
    assert astar(grid, (0, 0), (0, 0)).path == []
    assert astar(grid, (0, 0), (1, 1)).path is None  # shelf goal
    res = astar(grid, (0, 3), (2, 3))
    assert res.path is not None and _valid(grid, (0, 3), res.path, (2, 3))
    assert res.cost == len(res.path) == 8  # left around the rack (right is 10)


def test_astar_respects_blocked_cells_but_allows_goal() -> None:
    grid = Grid.from_ascii(MAP)
    res = astar(grid, (0, 1), (2, 1), blocked={(1, 0)})
    assert res.path is not None and (1, 0) not in res.path
    assert len(res.path) == 14  # forced the long way round
    assert astar(grid, (0, 0), (0, 1), blocked={(0, 1)}).path == [(0, 1)]
    walled = {(0, 1), (1, 0)}
    assert astar(grid, (0, 0), (3, 3), blocked=walled).path is None


def test_exact_heuristic_expands_only_the_path() -> None:
    layout = generate_layout()
    maps = DistanceMaps.for_layout(layout)
    start, goal = layout.spawn_cells[0], layout.drop_stations[-1]
    with_h = astar(layout.grid, start, goal, heuristic=maps.to(goal))
    manh = astar(layout.grid, start, goal)
    assert with_h.path is not None and manh.path is not None
    assert len(with_h.path) == len(manh.path)
    assert with_h.expanded == len(with_h.path)  # every expanded node is on the path
    assert with_h.expanded <= manh.expanded


def test_weighted_astar_avoids_expensive_cells() -> None:
    grid = Grid.from_ascii(MAP)
    costs = make_cell_costs(grid, {CellType.PICK: 10.0})
    maps = DistanceMaps(grid, costs)
    res = astar(grid, (2, 0), (2, 7), heuristic=maps.to((2, 7)), cell_cost=costs)
    assert res.path is not None
    # Goes along row 3 instead of through the pick aisle (row 2).
    assert [c for c in res.path[:-1] if grid.cell_type(c) is CellType.PICK] == []
    assert res.cost == maps.distance((2, 0), (2, 7))


@st.composite
def random_grid(draw: st.DrawFn) -> Grid:
    h, w = draw(st.integers(2, 9)), draw(st.integers(2, 9))
    cells = draw(st.lists(st.booleans(), min_size=h * w, max_size=h * w))
    arr = np.array(
        [CellType.SHELF if wall else CellType.FLOOR for wall in cells], dtype=np.uint8
    ).reshape(h, w)
    arr[0, 0] = arr[h - 1, w - 1] = CellType.FLOOR
    return Grid(arr)


@settings(max_examples=150, deadline=None)
@given(grid=random_grid(), use_maps=st.booleans())
def test_astar_is_optimal_vs_bfs(grid: Grid, use_maps: bool) -> None:
    start, goal = (0, 0), (grid.height - 1, grid.width - 1)
    heuristic = DistanceMaps(grid).to(goal) if use_maps else None
    res = astar(grid, start, goal, heuristic=heuristic)
    bfs = grid.bfs_path(start, goal)
    if bfs is None:
        assert res.path is None
    else:
        assert res.path is not None
        assert len(res.path) == len(bfs) == res.cost
        assert _valid(grid, start, res.path, goal)
