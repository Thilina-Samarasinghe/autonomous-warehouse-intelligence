"""The same assignment problem as an integer program, solved with OR-Tools CP-SAT.

Boolean ``x[r, j]`` means robot r takes task j. The constraints are: at most one
task per robot and at most one robot per task. The objective first maximises the
number of matches, then minimises the scaled integer cost (the same as Hungarian).

CP-SAT is overkill for a plain assignment, which Hungarian solves exactly. It is
here as a cross-check (both must reach the same optimum) and as the extension point
for side constraints Hungarian cannot express (robot/zone eligibility, battery
reserves, task priorities as hard rules). One search worker and a fixed seed keep
it deterministic.
"""

from __future__ import annotations

from collections.abc import Sequence

from ortools.sat.python import cp_model

from awi.assignment.costs import UNREACHABLE, candidate_tasks, cost_matrix
from awi.core.robot import Robot
from awi.core.task import Task
from awi.core.types import RobotId, TaskId
from awi.core.world import World

SCALE = 100  # costs are rounded to 1/100 tick
MATCH_REWARD = 10**9  # dominates any total cost: maximise the number of matches first


class CPSATAssigner:
    def __init__(self, age_weight: float = 0.2, time_limit_s: float = 2.0) -> None:
        if age_weight < 0:
            raise ValueError("age_weight must be >= 0")
        self.age_weight = age_weight
        self.time_limit_s = time_limit_s
        self.last_status: str | None = None

    def assign(
        self, world: World, robots: Sequence[Robot], tasks: Sequence[Task]
    ) -> dict[RobotId, TaskId]:
        if not robots or not tasks:
            return {}
        robots = sorted(robots, key=lambda r: r.id)
        cands = candidate_tasks(tasks, len(robots))
        cost = cost_matrix(world, robots, cands, self.age_weight)
        feasible = [
            (i, j)
            for i in range(len(robots))
            for j in range(len(cands))
            if cost[i, j] < UNREACHABLE / 2
        ]
        if not feasible:
            return {}

        model = cp_model.CpModel()
        x = {(i, j): model.new_bool_var(f"x_{i}_{j}") for i, j in feasible}
        for i in range(len(robots)):
            model.add_at_most_one(v for (ri, _), v in x.items() if ri == i)
        for j in range(len(cands)):
            model.add_at_most_one(v for (_, tj), v in x.items() if tj == j)
        # Lexicographic objective: first the most matches, then the lowest cost. This
        # is what Hungarian does when unreachable pairs carry a huge cost.
        model.minimize(
            sum((round(cost[i, j] * SCALE) - MATCH_REWARD) * v for (i, j), v in x.items())
        )

        solver = cp_model.CpSolver()
        solver.parameters.num_search_workers = 1
        solver.parameters.random_seed = 0
        solver.parameters.max_time_in_seconds = self.time_limit_s
        status = solver.solve(model)
        self.last_status = solver.status_name(status)
        if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            return {}
        return {robots[i].id: cands[j].id for (i, j), v in x.items() if solver.boolean_value(v)}
