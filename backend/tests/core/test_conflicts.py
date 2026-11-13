from awi.core.config import SimConfig
from awi.core.conflicts import count_conflicts, robots_in_cycles, wait_for_graph
from awi.core.engine import Simulation, resolve_moves
from awi.core.grid import Grid
from awi.core.layout import Layout
from awi.core.robot import RobotState
from awi.planning.independent import IndependentAStarPlanner


def test_count_vertex_and_edge_conflicts() -> None:
    current = {0: (0, 0), 1: (0, 1), 2: (0, 3), 3: (1, 2), 4: (5, 5)}
    intent = {0: (0, 1), 1: (0, 0), 2: (0, 2), 3: (0, 2), 4: (5, 5)}
    # 0<->1 swap (edge); 2 and 3 both into (0,2) (vertex).
    assert count_conflicts(current, intent) == (1, 1)


def test_moving_into_parked_robot_is_a_vertex_conflict() -> None:
    assert count_conflicts({0: (0, 0), 1: (0, 1)}, {0: (0, 1), 1: (0, 1)}) == (1, 0)


def test_three_way_claim_counts_pairs() -> None:
    current = {0: (0, 0), 1: (0, 2), 2: (1, 1)}
    intent = {0: (0, 1), 1: (0, 1), 2: (0, 1)}
    assert count_conflicts(current, intent) == (3, 0)


def test_no_conflicts_for_following_and_rotation() -> None:
    current = {0: (0, 0), 1: (0, 1), 2: (1, 1), 3: (1, 0)}
    intent = {0: (0, 1), 1: (1, 1), 2: (1, 0), 3: (0, 0)}
    assert count_conflicts(current, intent) == (0, 0)


def test_wait_for_graph_and_cycles() -> None:
    current = {0: (0, 0), 1: (0, 1), 2: (0, 2), 3: (0, 3)}
    intent = {0: (0, 1), 1: (0, 0), 2: (0, 1), 3: (0, 2)}  # 0<->1 swap, 2 & 3 queue behind
    final, blocked = resolve_moves(current, intent)
    graph = wait_for_graph(current, intent, final, blocked)
    assert graph == {0: 1, 1: 0, 2: 1, 3: 2}
    assert robots_in_cycles(graph) == {0, 1}


def test_cycle_detection_handles_chains_and_multiple_cycles() -> None:
    assert robots_in_cycles({}) == set()
    assert robots_in_cycles({1: 2, 2: 3}) == set()  # chain ends at a free robot
    assert robots_in_cycles({1: 2, 2: 3, 3: 1, 4: 1, 5: 6, 6: 5}) == {1, 2, 3, 5, 6}


def test_engine_reports_conflicts_and_deadlock_cycle() -> None:
    """Two robots meet head-on in a 1-wide corridor and can never pass."""
    layout = Layout.from_grid(Grid.from_ascii("D.....\n#+####"))
    cfg = SimConfig(num_robots=2, max_tasks=0)
    sim = Simulation(cfg, planner=IndependentAStarPlanner(yield_prob=0.0), layout=layout)
    a, b = sim.world.robots[0], sim.world.robots[1]
    a.pos, b.pos = (0, 1), (0, 4)
    for robot, goal in ((a, (0, 4)), (b, (0, 1))):
        robot.state, robot.goal, robot.needs_replan = RobotState.TO_PICKUP, goal, True

    reports = sim.run(10)
    last = reports[-1]
    assert last.edge_conflicts == 1
    assert last.waits_for == {0: 1, 1: 0}
    assert robots_in_cycles(last.waits_for) == {0, 1}
    assert all(r.plan_seconds >= 0 for r in reports)
    assert sum(r.plan_calls for r in reports) >= 1
