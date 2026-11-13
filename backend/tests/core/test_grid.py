import numpy as np
import pytest

from awi.core.grid import CellType, Grid

MAP = """
D....
.###.
.+++.
.....
"""


def test_ascii_roundtrip() -> None:
    g = Grid.from_ascii(MAP)
    assert g.to_ascii() == MAP.strip()
    assert (g.height, g.width) == (4, 5)
    assert g.cell_type((0, 0)) is CellType.DROP
    assert g.cell_type((1, 1)) is CellType.SHELF


@pytest.mark.parametrize("bad", ["..\n...", "..x\n..."])
def test_from_ascii_rejects_bad_input(bad: str) -> None:
    with pytest.raises(ValueError):
        Grid.from_ascii(bad)


def test_rejects_unknown_cell_values() -> None:
    with pytest.raises(ValueError):
        Grid(np.array([[0, 9]], dtype=np.uint8))


def test_cells_are_read_only() -> None:
    g = Grid.from_ascii(MAP)
    with pytest.raises(ValueError):
        g.cells[0, 0] = 1


def test_neighbors_skip_shelves_and_bounds() -> None:
    g = Grid.from_ascii(MAP)
    assert g.neighbors((0, 0)) == ((1, 0), (0, 1))
    assert g.neighbors((0, 2)) == ((0, 1), (0, 3))  # (1, 2) is a shelf
    assert g.neighbors((1, 2)) == ()  # shelves have no neighbours


def test_bfs_path_goes_around_shelves() -> None:
    g = Grid.from_ascii(MAP)
    path = g.bfs_path((0, 2), (2, 2))
    assert path is not None
    assert len(path) == 6  # around the 3-wide rack
    assert path[-1] == (2, 2)
    assert (1, 1) not in path and (1, 2) not in path


def test_bfs_path_edge_cases() -> None:
    g = Grid.from_ascii(MAP)
    assert g.bfs_path((0, 0), (0, 0)) == []
    assert g.bfs_path((0, 0), (1, 1)) is None  # goal is a shelf
    # Wall off the right side completely: unreachable.
    assert g.bfs_path((0, 0), (0, 4), blocked={(0, 3), (1, 4), (2, 3), (3, 3), (2, 4)}) is None


def test_bfs_path_blocked_allows_goal() -> None:
    g = Grid.from_ascii(MAP)
    assert g.bfs_path((0, 0), (0, 1), blocked={(0, 1)}) == [(0, 1)]


def test_bfs_distances() -> None:
    g = Grid.from_ascii(MAP)
    d = g.bfs_distances((0, 0))
    assert d[0, 0] == 0
    assert d[3, 4] == 7
    assert d[1, 1] == -1  # shelf


def test_connectivity() -> None:
    assert Grid.from_ascii(MAP).is_connected()
    assert not Grid.from_ascii("..#..\n..#..").is_connected()
