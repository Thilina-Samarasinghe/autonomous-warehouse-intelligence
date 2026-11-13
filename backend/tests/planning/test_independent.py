import pytest

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.grid import Grid
from awi.core.layout import Layout
from awi.planning.base import PathRequest
from awi.planning.independent import IndependentAStarPlanner
from awi.planning.naive import BFSPlanner
from tests.conftest import SMALL_LAYOUT


def test_engine_defaults_to_astar() -> None:
    sim = Simulation(SimConfig(num_robots=2, layout=SMALL_LAYOUT))
    assert isinstance(sim.planner, IndependentAStarPlanner)


def test_astar_and_bfs_paths_have_equal_length() -> None:
    sim = Simulation(SimConfig(num_robots=1, layout=SMALL_LAYOUT))
    world = sim.world
    start = world.robots[0].pos
    astar_p, bfs_p = IndependentAStarPlanner(), BFSPlanner()
    for goal in [*world.layout.drop_stations, *world.layout.pick_point.values()]:
        req = [PathRequest(0, start, goal)]
        a = astar_p.plan(world, req)[0]
        b = bfs_p.plan(world, req)[0]
        assert len(a) == len(b) == world.distances.distance(start, goal)
    assert astar_p.stats.calls > 0 and astar_p.stats.expanded > 0


def test_stuck_robot_detours_around_blocker() -> None:
    grid = Grid.from_ascii(
        """
        D......
        .......
        .+++++.
        .#####.
        .......
        """
    )
    layout = Layout.from_grid(grid)
    sim = Simulation(SimConfig(num_robots=2), layout=layout)
    a, b = sim.world.robots[0], sim.world.robots[1]
    a.pos, b.pos = (1, 0), (1, 1)  # b blocks a's direct route along row 1
    planner = IndependentAStarPlanner(yield_prob=0.0)
    direct = planner.plan(sim.world, [PathRequest(0, a.pos, (1, 3))])[0]
    assert direct[0] == (1, 1)
    detour = planner.plan(sim.world, [PathRequest(0, a.pos, (1, 3), stuck=True)])[0]
    assert (1, 1) not in detour and detour[-1] == (1, 3)


def test_stuck_robot_without_detour_yields() -> None:
    # Corridor: b sits between a and a's goal, so no detour exists.
    grid = Grid.from_ascii("D...\n#+##")
    layout = Layout.from_grid(grid)
    sim = Simulation(SimConfig(num_robots=2), layout=layout)
    a, b = sim.world.robots[0], sim.world.robots[1]
    a.pos, b.pos = (0, 1), (0, 2)
    planner = IndependentAStarPlanner(yield_prob=1.0)
    path = planner.plan(sim.world, [PathRequest(0, a.pos, (0, 3), stuck=True)])[0]
    assert path[0] in {(0, 0), (1, 1)}  # steps aside into a free cell
    assert path[-1] == (0, 3)
    never = IndependentAStarPlanner(yield_prob=0.0)
    path = never.plan(sim.world, [PathRequest(0, a.pos, (0, 3), stuck=True)])[0]
    assert path[0] == (0, 2)  # no yield: keeps pushing toward b


def test_pick_cost_builds_weighted_maps() -> None:
    sim = Simulation(SimConfig(num_robots=1, layout=SMALL_LAYOUT))
    planner = IndependentAStarPlanner(pick_cell_cost=2.0)
    maps = planner.distance_maps(sim.world)
    assert not maps.is_uniform and maps is not sim.world.distances
    assert planner.distance_maps(sim.world) is maps  # cached
    assert IndependentAStarPlanner().distance_maps(sim.world) is sim.world.distances


@pytest.mark.parametrize("kwargs", [{"yield_prob": 1.5}, {"pick_cell_cost": 0.0}])
def test_invalid_parameters(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        IndependentAStarPlanner(**kwargs)
