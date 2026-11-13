"""Discrete-time simulation engine.

Each call to :meth:`Simulation.step` processes tick ``t`` in a fixed order:

1. **Orders**: new tasks arrive (seeded Poisson stream).
2. **Assignment**: the task assigner matches idle robots to pending tasks.
3. **Planning**: robots that need a path (new goal, exhausted path or stuck)
   request one from the path planner.
4. **Movement**: every robot proposes its next cell. The engine is the physical
   arbiter: proposals that would put two robots in one cell (vertex conflict) or
   make two robots swap cells (edge conflict) are rejected and the robot waits.
   So robots never physically collide; every rejected move is recorded as a
   *blocked* event. Coordinated planners should produce none.
5. **Service**: robots that reached their goal pick or drop, and finished tasks
   are completed.
6. **Accounting**: idle and busy counters are updated, then ``t`` advances.

Event timestamps use the index ``t`` of the tick in which they happened.
"""

from __future__ import annotations

import time
from collections import defaultdict, deque
from collections.abc import Sequence
from dataclasses import dataclass, field

from awi.assignment.base import TaskAssigner
from awi.assignment.greedy import GreedyNearestAssigner
from awi.core.config import SimConfig
from awi.core.conflicts import count_conflicts, wait_for_graph
from awi.core.grid import CellType
from awi.core.layout import Layout, generate_layout
from awi.core.orders import TaskGenerator
from awi.core.rng import make_rng
from awi.core.robot import FREE_STATES, MOVING_STATES, PATH_STATES, Robot, RobotState
from awi.core.route import Route, StopKind, route_tasks, single_task_route, validate_route
from awi.core.task import TaskStatus
from awi.core.types import Cell, RobotId, TaskId, manhattan
from awi.core.world import World
from awi.energy.battery import BatteryModel
from awi.energy.charging import dispatch_chargers
from awi.planning.base import PathPlanner, PathRequest
from awi.planning.dijkstra import DistanceMaps
from awi.planning.independent import IndependentAStarPlanner


class SimulationError(RuntimeError):
    """A strategy returned invalid output (bad path, illegal assignment...)."""


@dataclass(slots=True)
class TickReport:
    t: int
    new_tasks: list[TaskId] = field(default_factory=list)
    assigned: list[tuple[RobotId, TaskId]] = field(default_factory=list)
    picked: list[TaskId] = field(default_factory=list)
    delivered: list[TaskId] = field(default_factory=list)
    moved: int = 0
    blocked: list[RobotId] = field(default_factory=list)
    replanned: list[RobotId] = field(default_factory=list)
    vertex_conflicts: int = 0
    edge_conflicts: int = 0
    waits_for: dict[RobotId, RobotId] = field(default_factory=dict)
    """Wait-for graph: blocked robot -> robot holding the cell it wanted."""
    plan_calls: int = 0
    plan_seconds: float = 0.0
    moved_robots: list[RobotId] = field(default_factory=list)
    charges_started: list[RobotId] = field(default_factory=list)
    aborted_tasks: list[TaskId] = field(default_factory=list)
    """Unpicked tasks handed back to the queue by robots at critical battery."""
    energy_rejections: int = 0
    """Assignments refused because the robot could not afford the route."""
    stranded: list[RobotId] = field(default_factory=list)
    preempted: list[RobotId] = field(default_factory=list)
    """Robots that left a charger early to make room for an emergency."""


@dataclass(slots=True)
class SimSummary:
    ticks: int
    tasks_generated: int
    tasks_delivered: int
    tasks_pending: int
    tasks_in_progress: int
    avg_delivery_time: float | None
    total_distance: int
    total_wait_ticks: int
    total_blocked_ticks: int
    total_idle_ticks: int


class Simulation:
    def __init__(
        self,
        config: SimConfig | None = None,
        planner: PathPlanner | None = None,
        assigner: TaskAssigner | None = None,
        layout: Layout | None = None,
    ) -> None:
        self.config = config or SimConfig()
        self.layout = layout or generate_layout(self.config.layout)
        self.planner: PathPlanner = planner or IndependentAStarPlanner(seed=self.config.seed)
        self.assigner: TaskAssigner = assigner or GreedyNearestAssigner()
        self.task_generator = TaskGenerator(
            self.layout,
            make_rng(self.config.seed, "tasks"),
            rate=self.config.task_rate,
            zipf_s=self.config.zipf_s,
            max_tasks=self.config.max_tasks,
        )
        self.world = World(
            layout=self.layout,
            robots=self._spawn_robots(),
            distances=DistanceMaps.for_layout(self.layout),
        )
        bcfg = self.config.battery
        if bcfg.enabled:
            self.world.battery = BatteryModel(
                bcfg,
                self.layout.chargers,
                self.world.distances,
                self.config.pick_ticks,
                self.config.drop_ticks,
            )
            rng = make_rng(self.config.seed, "battery")
            for rid in sorted(self.world.robots):
                frac = rng.uniform(bcfg.initial_min, bcfg.initial_max)
                self.world.robots[rid].battery = frac * bcfg.capacity

    # ------------------------------------------------------------------ public
    @property
    def t(self) -> int:
        return self.world.t

    def run(self, ticks: int) -> list[TickReport]:
        return [self.step() for _ in range(ticks)]

    def step(self) -> TickReport:
        report = TickReport(t=self.world.t)
        self._generate_tasks(report)
        if self.world.battery is not None:
            self._battery_policy(report)
        self._assign_tasks(report)
        if self.world.battery is not None:
            self._flag_unaffordable_robots()
            dispatch = dispatch_chargers(self.world, self.layout.chargers, self.world.battery)
            for victim in dispatch.preempted:
                self._leave_charger(victim)
                report.preempted.append(victim.id)
            for robot, charger in dispatch.assignments:
                self._set_goal(robot, RobotState.TO_CHARGER, charger)
                report.charges_started.append(robot.id)
        if self.config.park_idle:
            self._park_idle_robots()
        self._plan_paths(report)
        self._move_robots(report)
        if self.world.battery is not None:
            self._drain_batteries(report)
        self._service_robots(report)
        for r in self.world.robots.values():
            if r.state in FREE_STATES:
                r.idle_ticks += 1
            else:
                r.busy_ticks += 1
        self.world.t += 1
        return report

    def summary(self) -> SimSummary:
        tasks = self.world.tasks.values()
        times = [t.delivery_time for t in tasks if t.delivery_time is not None]
        robots = self.world.robots.values()
        return SimSummary(
            ticks=self.world.t,
            tasks_generated=len(self.world.tasks),
            tasks_delivered=len(times),
            tasks_pending=len(self.world.pending),
            tasks_in_progress=sum(
                t.status in (TaskStatus.ASSIGNED, TaskStatus.PICKED) for t in tasks
            ),
            avg_delivery_time=sum(times) / len(times) if times else None,
            total_distance=sum(r.distance for r in robots),
            total_wait_ticks=sum(r.wait_ticks for r in robots),
            total_blocked_ticks=sum(r.blocked_ticks for r in robots),
            total_idle_ticks=sum(r.idle_ticks for r in robots),
        )

    # ------------------------------------------------------------ tick phases
    def _generate_tasks(self, report: TickReport) -> None:
        for task in self.task_generator.generate(self.world.t):
            self.world.tasks[task.id] = task
            self.world.pending.append(task.id)
            report.new_tasks.append(task.id)

    def _assign_tasks(self, report: TickReport) -> None:
        idle = sorted(self.world.idle_robots(), key=lambda r: r.id)
        if not idle or not self.world.pending:
            return
        tasks = self.world.pending_tasks()
        assignment = self.assigner.assign(self.world, idle, tasks)

        idle_ids = {r.id for r in idle}
        pending_ids = set(self.world.pending)
        routes: dict[RobotId, Route] = {}
        for rid, value in assignment.items():
            if isinstance(value, int):
                if value not in pending_ids:
                    raise SimulationError(f"illegal assignment robot={rid} task={value}")
                routes[rid] = single_task_route(self.world.tasks[value])
            else:
                routes[rid] = tuple(value)
        used = [tid for route in routes.values() for tid in route_tasks(route)]
        if len(used) != len(set(used)):
            raise SimulationError("assigner gave one task to several robots")
        accepted = dict(routes)
        for rid, route in sorted(routes.items()):
            if rid not in idle_ids or not set(route_tasks(route)) <= pending_ids:
                raise SimulationError(f"illegal assignment robot={rid} route={route}")
            try:
                validate_route(route, self.world.tasks, self.config.capacity)
            except ValueError as exc:
                raise SimulationError(f"invalid route for robot {rid}: {exc}") from None
            robot = self.world.robots[rid]
            battery = self.world.battery
            if battery is not None and not battery.can_afford(robot, route):
                report.energy_rejections += 1
                robot.needs_charge = True
                del accepted[rid]
                continue
            for tid in route_tasks(route):
                task = self.world.tasks[tid]
                task.status, task.robot_id, task.assigned_t = TaskStatus.ASSIGNED, rid, self.world.t
                report.assigned.append((rid, tid))
            robot.stops = deque(route)
            self._head_to_next_stop(robot)
        assigned = {tid for route in accepted.values() for tid in route_tasks(route)}
        self.world.pending = [tid for tid in self.world.pending if tid not in assigned]

    def _park_idle_robots(self) -> None:
        """Send unassigned robots off stations and pick points to the nearest free
        parking cell, so they don't block the next robot's goal."""
        grid = self.layout.grid
        taken = {r.pos for r in self.world.robots.values()} | {
            r.goal for r in self.world.robots.values() if r.state is RobotState.TO_PARK
        }
        for r in self._robots():
            if r.state is not RobotState.IDLE or grid.cell_type(r.pos) is CellType.FLOOR:
                continue
            free = [c for c in self.layout.parking_cells if c not in taken]
            if not free:
                continue
            spot = min(free, key=lambda c: (manhattan(r.pos, c), c))
            taken.add(spot)
            self._set_goal(r, RobotState.TO_PARK, spot)

    def _plan_paths(self, report: TickReport) -> None:
        threshold = self.config.stuck_replan_ticks
        requests: list[PathRequest] = []
        for r in self._robots():
            if r.state not in MOVING_STATES or r.goal is None or r.pos == r.goal:
                continue
            stuck = r.stuck_ticks >= threshold
            if r.needs_replan or not r.path or stuck:
                requests.append(PathRequest(r.id, r.pos, r.goal, stuck=stuck))
        if not requests:
            return

        t0 = time.perf_counter()
        paths = self.planner.plan(self.world, requests)
        report.plan_seconds += time.perf_counter() - t0
        report.plan_calls += 1
        requested = {req.robot_id for req in requests}
        for rid, path in sorted(paths.items()):
            robot = self.world.robots.get(rid)
            if robot is None or robot.state not in PATH_STATES:
                raise SimulationError(f"planner returned a path for busy robot {rid}")
            if self.config.validate_paths:
                self._validate_path(robot, path)
            robot.path = deque(path)
            report.replanned.append(rid)
        for rid in requested:
            robot = self.world.robots[rid]
            robot.needs_replan = not robot.path  # no path found: ask again next tick
            if robot.stuck_ticks >= threshold:
                robot.stuck_ticks = 0

    def _move_robots(self, report: TickReport) -> None:
        robots = self._robots()
        current = {r.id: r.pos for r in robots}
        intent = {r.id: (r.path[0] if r.state in PATH_STATES and r.path else r.pos) for r in robots}
        report.vertex_conflicts, report.edge_conflicts = count_conflicts(current, intent)
        final, blocked = resolve_moves(current, intent)
        report.waits_for = wait_for_graph(current, intent, final, blocked)

        for r in robots:
            nxt = final[r.id]
            if r.id in blocked:
                r.wait_ticks += 1
                r.blocked_ticks += 1
                r.stuck_ticks += 1
                report.blocked.append(r.id)
                continue
            if r.state in PATH_STATES and r.path:
                r.path.popleft()
                if nxt == r.pos and r.state in MOVING_STATES:  # planned wait
                    r.wait_ticks += 1
            if nxt != r.pos:
                r.pos = nxt
                r.distance += 1
                r.stuck_ticks = 0
                report.moved += 1
                report.moved_robots.append(r.id)

    def _service_robots(self, report: TickReport) -> None:
        cfg = self.config
        for r in self._robots():
            # Robots already picking/dropping make progress first...
            if r.state in (RobotState.PICKING, RobotState.DROPPING):
                r.service_left -= 1
                if r.service_left <= 0:
                    self._finish_service(r, report)
            # ...then robots that just arrived start their service.
            elif r.state is RobotState.TO_PICKUP and r.pos == r.goal:
                r.state, r.service_left, r.goal = RobotState.PICKING, cfg.pick_ticks, None
                r.path.clear()
                if r.service_left <= 0:
                    self._finish_service(r, report)
            elif r.state is RobotState.CHARGING:
                self._charge(r)
            elif r.state is RobotState.TO_CHARGER and r.pos == r.goal:
                r.state, r.goal = RobotState.CHARGING, None
                r.path.clear()
                r.charge_sessions += 1
            elif r.state is RobotState.TO_PARK and r.pos == r.goal:
                r.state, r.goal = RobotState.IDLE, None
                r.path.clear()
            elif r.state is RobotState.TO_DROP and r.pos == r.goal:
                r.state, r.service_left, r.goal = RobotState.DROPPING, cfg.drop_ticks, None
                r.path.clear()
                if r.service_left <= 0:
                    self._finish_service(r, report)

    # ---------------------------------------------------------------- helpers
    def _finish_service(self, r: Robot, report: TickReport) -> None:
        stop = r.stops.popleft()
        task = self.world.tasks[stop.task_id]
        t = self.world.t
        if stop.kind is StopKind.PICK:
            task.status, task.picked_t = TaskStatus.PICKED, t
            r.cargo.add(task.id)
            report.picked.append(task.id)
        else:
            task.status, task.delivered_t = TaskStatus.DELIVERED, t
            r.cargo.discard(task.id)
            r.deliveries += 1
            report.delivered.append(task.id)
        self._head_to_next_stop(r)

    # --------------------------------------------------------------- battery
    def _battery_policy(self, report: TickReport) -> None:
        """Flag free robots below ``low``; at ``critical``, hand back unpicked work."""
        battery = self.world.battery
        assert battery is not None
        cfg = battery.cfg
        for r in self._robots():
            if r.state in (RobotState.CHARGING, RobotState.TO_CHARGER, RobotState.STRANDED):
                continue
            if battery.is_emergency(r):
                r.needs_charge = True  # cargo stays aboard; resumed after charging
                if r.state in (RobotState.PICKING, RobotState.DROPPING):
                    continue  # finish the current service first
                if any(s.task_id not in r.cargo for s in r.stops):
                    report.aborted_tasks.extend(self._release_unpicked(r))
                    self._head_to_next_stop(r)
            elif r.state in FREE_STATES and battery.fraction(r) < cfg.low:
                r.needs_charge = True

    def _flag_unaffordable_robots(self) -> None:
        """A free robot left without work while orders wait, and unable to afford any
        of the oldest ones, goes to charge instead of idling forever."""
        battery = self.world.battery
        assert battery is not None
        if not self.world.pending:
            return
        oldest = self.world.pending_tasks()[:30]
        for r in self.world.idle_robots():
            if not any(battery.can_afford(r, single_task_route(t)) for t in oldest):
                r.needs_charge = True

    def _release_unpicked(self, r: Robot) -> list[TaskId]:
        """Drop stops of tasks not yet picked and return those tasks to the queue."""
        released = sorted({s.task_id for s in r.stops if s.task_id not in r.cargo})
        r.stops = deque(s for s in r.stops if s.task_id in r.cargo)
        for tid in released:
            task = self.world.tasks[tid]
            task.status, task.robot_id, task.assigned_t = TaskStatus.PENDING, None, None
        self.world.pending = sorted(
            [*self.world.pending, *released],
            key=lambda tid: (self.world.tasks[tid].created_t, tid),
        )
        return released

    def _drain_batteries(self, report: TickReport) -> None:
        battery = self.world.battery
        assert battery is not None
        moved = set(report.moved_robots)
        for r in self._robots():
            if r.state in (RobotState.CHARGING, RobotState.STRANDED):
                continue
            cost = battery.tick_cost(r.id in moved, len(r.cargo))
            r.energy_used += cost
            r.battery -= cost
            if r.battery <= 0:
                r.battery = 0.0
                self._release_unpicked(r)
                r.state, r.goal = RobotState.STRANDED, None
                r.path.clear()
                report.stranded.append(r.id)

    def _charge(self, r: Robot) -> None:
        battery = self.world.battery
        assert battery is not None
        cfg = battery.cfg
        r.battery = min(cfg.capacity, r.battery + cfg.charge_rate)
        r.charging_ticks += 1
        if r.battery >= cfg.resume * cfg.capacity:
            self._leave_charger(r)

    def _leave_charger(self, r: Robot) -> None:
        r.needs_charge = False
        if r.stops:  # emergency charge mid-route: resume the delivery
            self._head_to_next_stop(r)
        else:
            r.state = RobotState.IDLE

    def _head_to_next_stop(self, r: Robot) -> None:
        """Set state and goal from the route's next stop (or go idle when done)."""
        if not r.stops:
            r.state, r.goal = RobotState.IDLE, None
            r.path.clear()
            return
        stop = r.stops[0]
        state = RobotState.TO_PICKUP if stop.kind is StopKind.PICK else RobotState.TO_DROP
        self._set_goal(r, state, stop.cell)

    @staticmethod
    def _set_goal(r: Robot, state: RobotState, goal: Cell) -> None:
        r.state, r.goal = state, goal
        r.path.clear()
        r.needs_replan = True
        r.stuck_ticks = 0

    def _validate_path(self, robot: Robot, path: Sequence[Cell]) -> None:
        grid = self.layout.grid
        prev = robot.pos
        for cell in path:
            if not grid.is_passable(cell) or manhattan(prev, cell) > 1:
                raise SimulationError(f"invalid path step {prev}->{cell} for robot {robot.id}")
            prev = cell

    def _robots(self) -> list[Robot]:
        return [self.world.robots[i] for i in sorted(self.world.robots)]

    def _spawn_robots(self) -> dict[RobotId, Robot]:
        n = self.config.num_robots
        grid = self.layout.grid
        primary = list(self.layout.spawn_cells)
        taken = set(primary)
        extra = [c for c in grid.cells_of_type(CellType.FLOOR) if c not in taken]
        if n > len(primary) + len(extra):
            raise ValueError(f"layout has room for at most {len(primary) + len(extra)} robots")
        rng = make_rng(self.config.seed, "spawn")
        if n <= len(primary):
            cells = [primary[i] for i in sorted(rng.choice(len(primary), n, replace=False))]
        else:
            cells = primary + extra[: n - len(primary)]
        return {i: Robot(id=i, pos=c) for i, c in enumerate(cells)}


def resolve_moves(
    current: dict[RobotId, Cell], intent: dict[RobotId, Cell]
) -> tuple[dict[RobotId, Cell], set[RobotId]]:
    """Resolve simultaneous moves so no vertex or edge (swap) conflict remains.

    Rejected robots stay in place. When several robots target one cell, a robot
    already standing there wins, otherwise the lowest id. Iterates to a fixpoint,
    since a rejected robot may in turn block others. Returns the final positions
    and the set of blocked robots.
    """
    final = dict(intent)
    blocked: set[RobotId] = set()
    at_cell = {c: rid for rid, c in current.items()}

    def reject(rid: RobotId) -> None:
        final[rid] = current[rid]
        blocked.add(rid)

    changed = True
    while changed:
        changed = False
        claims: defaultdict[Cell, list[RobotId]] = defaultdict(list)
        for rid in sorted(final):
            claims[final[rid]].append(rid)
        for cell, rids in claims.items():
            if len(rids) < 2:
                continue
            stayer = at_cell.get(cell)
            winner = stayer if stayer in rids else rids[0]
            for rid in rids:
                if rid != winner:
                    reject(rid)
                    changed = True
        for rid in sorted(final):
            dest = final[rid]
            other = at_cell.get(dest)
            if other is not None and other != rid and final[other] == current[rid]:
                reject(rid)
                reject(other)
                changed = True
    return final, blocked
