import itertools

import numpy as np
import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.grid import CellType, Grid
from awi.core.types import Cell, manhattan
from awi.metrics.collector import MetricsCollector
from awi.planning.coordinated import CBSPlanner, PrioritizedPlanner
from awi.planning.dijkstra import DistanceMaps
from awi.planning.mapf import (
    Agent,
    AgentKind,
    Instance,
    count_conflicts_of,
    find_conflicts,
    pad,
    pad_paths,
    static_violations,
    to_engine_paths,
)
from awi.planning.st_astar import space_time_astar
from tests.conftest import SMALL_LAYOUT

CORRIDOR = Grid.from_ascii(
    """
    .......
    ###.###
    """
)  # a 1-wide corridor with one side pocket at (1, 3)


def _valid(grid: Grid, path: list[Cell]) -> bool:
    return all(grid.is_passable(c) for c in path) and all(
        manhattan(a, b) <= 1 for a, b in itertools.pairwise(path)
    )


# ------------------------------------------------------------- space-time A*


def test_st_astar_without_constraints_is_shortest() -> None:
    maps = DistanceMaps(CORRIDOR)
    res = space_time_astar(CORRIDOR, (0, 0), (0, 6), maps.to((0, 6)), horizon=10)
    assert res is not None
    assert res.cells == [(0, c) for c in range(7)]
    assert res.cost == 6


def test_st_astar_waits_for_vertex_constraint() -> None:
    maps = DistanceMaps(CORRIDOR)
    res = space_time_astar(
        CORRIDOR, (0, 0), (0, 2), maps.to((0, 2)), horizon=10, vertex={((0, 1), 1)}
    )
    assert res is not None
    assert res.cells[1] == (0, 0)  # forced to wait one tick
    assert res.cost == 3


def test_st_astar_respects_move_constraint() -> None:
    maps = DistanceMaps(CORRIDOR)
    res = space_time_astar(
        CORRIDOR, (0, 0), (0, 1), maps.to((0, 1)), horizon=10, moves={((0, 0), (0, 1), 1)}
    )
    assert res is not None and res.cells[1] == (0, 0) and res.cost == 2


def test_st_astar_cannot_stay_on_constrained_goal() -> None:
    maps = DistanceMaps(CORRIDOR)
    res = space_time_astar(
        CORRIDOR, (0, 0), (0, 1), maps.to((0, 1)), horizon=6, vertex={((0, 1), 4)}
    )
    assert res is not None
    assert res.cells[-1] == (0, 1)
    assert len(res.cells) - 1 == 5  # arrives after the constraint, at t=5


def test_absorbing_goal_forbids_passing_through() -> None:
    grid = Grid.from_ascii("....")
    maps = DistanceMaps(grid)
    # The goal (0,1) is blocked at t=3, so the robot cannot settle there before t=3.
    common = {"horizon": 6, "vertex": {((0, 1), 3)}}
    loose = space_time_astar(grid, (0, 0), (0, 1), maps.to((0, 1)), **common)  # type: ignore[arg-type]
    strict = space_time_astar(
        grid,
        (0, 0),
        (0, 1),
        maps.to((0, 1)),
        absorbing_goal=True,
        **common,  # type: ignore[arg-type]
    )
    assert loose is not None and strict is not None
    first_visit = strict.cells.index((0, 1))
    assert first_visit == len(strict.cells) - 1  # enters the goal only once, at the end
    assert first_visit >= 4


def test_st_astar_window_cut_and_blocked_from() -> None:
    maps = DistanceMaps(CORRIDOR)
    res = space_time_astar(CORRIDOR, (0, 0), (0, 6), maps.to((0, 6)), horizon=3)
    assert res is not None and len(res.cells) == 4 and res.cost == 3 + 3
    walled = space_time_astar(
        CORRIDOR, (0, 0), (0, 6), maps.to((0, 6)), horizon=10, blocked_from={(0, 3): 0}
    )
    assert walled is not None and (0, 3) not in walled.cells  # cut at the horizon
    assert (
        space_time_astar(
            CORRIDOR,
            (0, 0),
            (0, 6),
            maps.to((0, 6)),
            horizon=10,
            blocked_from={(0, 1): 0, (0, 0): 1},
        )
        is None
    )  # trapped
    with pytest.raises(ValueError):
        space_time_astar(CORRIDOR, (0, 0), (0, 6), maps.to((0, 6)), horizon=0)


def test_avoid_table_breaks_ties() -> None:
    grid = Grid.from_ascii("...\n...")
    maps = DistanceMaps(grid)
    base = space_time_astar(grid, (0, 0), (1, 1), maps.to((1, 1)), horizon=5)
    assert base is not None
    via = base.cells[1]
    other = (1, 0) if via == (0, 1) else (0, 1)
    res = space_time_astar(grid, (0, 0), (1, 1), maps.to((1, 1)), horizon=5, avoid={(via, 1): 1})
    assert res is not None and res.cells[1] == other and res.cost == base.cost


# ------------------------------------------------------------- MAPF helpers


def test_find_conflicts_vertex_edge_and_stay_at_end() -> None:
    paths = {
        0: [(0, 0), (0, 1)],  # stays at (0,1) afterwards
        1: [(0, 3), (0, 2), (0, 1)],  # arrives where 0 waits -> vertex at t=2
        2: [(5, 0), (5, 1)],
        3: [(5, 1), (5, 0)],  # swap with 2 -> edge at t=1
    }
    conflicts = find_conflicts(paths, horizon=4)
    kinds = {(c.is_edge, c.t) for c in conflicts}
    assert (True, 1) in kinds and (False, 2) in kinds
    assert find_conflicts(paths, horizon=4, first_only=True)[0].t == 1
    padded = pad_paths(paths, 4)
    assert count_conflicts_of(1, padded) >= 1 and count_conflicts_of(2, padded) == 1


def test_pad_and_engine_conversion() -> None:
    assert pad([(0, 0), (0, 1)], 3) == [(0, 0), (0, 1), (0, 1), (0, 1)]
    inst = Instance(
        (Agent(0, (0, 0), (0, 2), AgentKind.TASK), Agent(1, (3, 3), (3, 3), AgentKind.IDLE)),
        frozenset(),
    )
    out = to_engine_paths(inst, {0: [(0, 0), (0, 1), (0, 2)], 1: [(3, 3)]}, commit_steps=4)
    assert out[0] == [(0, 1), (0, 2)]  # stops at the goal
    assert out[1] == [(3, 3)] * 4  # idle robots always get a (waiting) path


# ----------------------------------------------------------- joint planners


@st.composite
def instances(draw: st.DrawFn) -> tuple[Grid, DistanceMaps, Instance]:
    h, w = draw(st.integers(3, 6)), draw(st.integers(3, 7))
    walls = draw(st.lists(st.booleans(), min_size=h * w, max_size=h * w))
    arr = np.array(
        [CellType.SHELF if wall and draw(st.booleans()) else CellType.FLOOR for wall in walls],
        dtype=np.uint8,
    ).reshape(h, w)
    grid = Grid(arr)
    free = grid.passable_cells()
    n = draw(st.integers(2, min(6, len(free) // 2)))
    starts = draw(st.lists(st.sampled_from(free), min_size=n, max_size=n, unique=True))
    goals = draw(st.lists(st.sampled_from(free), min_size=n, max_size=n, unique=True))
    maps = DistanceMaps(grid)
    agents = []
    for i, (s, g) in enumerate(zip(starts, goals, strict=True)):
        if s == g or np.isinf(maps.distance(s, g)):
            agents.append(Agent(i, s, s, AgentKind.IDLE))
        else:
            agents.append(Agent(i, s, g, AgentKind.TASK))
    return grid, maps, Instance(tuple(agents), frozenset())


def _check_solution(grid: Grid, inst: Instance, paths: dict[int, list[Cell]], horizon: int) -> None:
    assert set(paths) == {a.id for a in inst.agents}
    for a in inst.agents:
        assert paths[a.id][0] == a.start
        assert _valid(grid, paths[a.id])
    assert find_conflicts(paths, horizon) == []
    assert static_violations(paths, inst.static, horizon) == []


@settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(problem=instances())
def test_prioritized_solutions_are_conflict_free(
    problem: tuple[Grid, DistanceMaps, Instance],
) -> None:
    grid, maps, inst = problem
    pp = PrioritizedPlanner(horizon=8)
    paths = pp.solve(grid, maps, inst)
    if pp.stats.failures == 0:
        _check_solution(grid, inst, paths, pp.horizon)


@settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(problem=instances())
def test_cbs_is_conflict_free_and_no_worse_than_pp(
    problem: tuple[Grid, DistanceMaps, Instance],
) -> None:
    grid, maps, inst = problem
    cbs = CBSPlanner(horizon=8, max_nodes=300)
    pp = PrioritizedPlanner(horizon=8)
    cbs_paths = cbs.solve(grid, maps, inst)
    pp_paths = pp.solve(grid, maps, inst)
    if cbs.stats.failures == 0:
        _check_solution(grid, inst, cbs_paths, cbs.horizon)
    if cbs.last_solved_optimally and pp.stats.failures == 0:
        # CBS minimises the windowed sum of costs, so a PP solution can't beat it.
        def soc(paths: dict[int, list[Cell]]) -> float:
            """Windowed sum of costs: settle time at the goal, else horizon + cost-to-go."""
            total = 0.0
            for a in inst.agents:
                q = pad(paths[a.id], cbs.horizon)
                if q[-1] == a.goal:
                    settle = len(q)
                    while settle > 0 and q[settle - 1] == a.goal:
                        settle -= 1
                    total += settle
                else:
                    total += cbs.horizon + maps.distance(q[-1], a.goal)
            return total

        assert soc(cbs_paths) <= soc(pp_paths) + 1e-9


def test_cbs_solves_corridor_swap_using_the_pocket() -> None:
    maps = DistanceMaps(CORRIDOR)
    inst = Instance(
        (
            Agent(0, (0, 2), (0, 5), AgentKind.TASK),
            Agent(1, (0, 4), (0, 1), AgentKind.TASK),
        ),
        frozenset(),
    )
    cbs = CBSPlanner(horizon=12)
    paths = cbs.solve(CORRIDOR, maps, inst)
    assert cbs.last_solved_optimally and cbs.stats.fallbacks == 0
    _check_solution(CORRIDOR, inst, paths, 12)
    assert any((1, 3) in p for p in paths.values())  # one robot ducks into the pocket


def test_shared_goal_uses_target_reasoning() -> None:
    """Two robots want the same drop cell: one must wait, CBS must not blow up."""
    grid = Grid.from_ascii(".....\n.....")
    maps = DistanceMaps(grid)
    inst = Instance(
        (Agent(0, (0, 0), (0, 2), AgentKind.TASK), Agent(1, (1, 4), (0, 2), AgentKind.TASK)),
        frozenset(),
    )
    cbs = CBSPlanner(horizon=10, max_nodes=50)
    paths = cbs.solve(grid, maps, inst)
    assert cbs.last_solved_optimally
    assert find_conflicts(paths, 10) == []
    assert sum(p[-1] == (0, 2) for p in paths.values()) == 1


def test_idle_robot_is_moved_aside() -> None:
    maps = DistanceMaps(CORRIDOR)
    inst = Instance(
        (Agent(0, (0, 0), (0, 6), AgentKind.TASK), Agent(1, (0, 3), (0, 3), AgentKind.IDLE)),
        frozenset(),
    )
    for planner in (CBSPlanner(horizon=10), PrioritizedPlanner(horizon=10)):
        paths = planner.solve(CORRIDOR, maps, inst)
        assert find_conflicts(paths, 10) == []
        assert pad(paths[0], 10)[-1] == (0, 6)  # the task robot gets through
        assert (1, 3) in paths[1]  # the idle robot steps into the pocket


def test_static_robots_are_respected() -> None:
    maps = DistanceMaps(CORRIDOR)
    inst = Instance((Agent(0, (0, 0), (0, 6), AgentKind.TASK),), frozenset({(0, 3)}))
    for planner in (CBSPlanner(horizon=10), PrioritizedPlanner(horizon=10)):
        paths = planner.solve(CORRIDOR, maps, inst)
        assert (0, 3) not in paths[0]


def test_invalid_parameters() -> None:
    with pytest.raises(ValueError):
        CBSPlanner(horizon=4, commit_steps=5)
    with pytest.raises(ValueError):
        PrioritizedPlanner(horizon=0)


# ---------------------------------------------------- engine integration


@pytest.mark.parametrize("planner_cls", [PrioritizedPlanner, CBSPlanner])
def test_coordinated_fleet_never_conflicts(planner_cls: type) -> None:
    sim = Simulation(
        SimConfig(num_robots=8, task_rate=0.15, layout=SMALL_LAYOUT, seed=5),
        planner=planner_cls(),
    )
    m = MetricsCollector(sim)
    for _ in range(400):
        m.observe(sim.step())
    r = m.result()
    assert sim.planner.stats.failures == 0  # type: ignore[attr-defined]
    assert r.conflicts_total == 0
    assert r.blocked_frac == 0
    assert r.deadlock_events == 0
    assert r.tasks_delivered > 20


@settings(max_examples=12, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(seed=st.integers(0, 10_000), robots=st.integers(2, 10), rate=st.floats(0.05, 0.4))
def test_prioritized_fleet_property(seed: int, robots: int, rate: float) -> None:
    planner = PrioritizedPlanner()
    sim = Simulation(
        SimConfig(seed=seed, num_robots=robots, task_rate=rate, layout=SMALL_LAYOUT),
        planner=planner,
    )
    m = MetricsCollector(sim)
    for _ in range(200):
        m.observe(sim.step())
    if planner.stats.failures == 0:
        assert m.result().conflicts_total == 0
