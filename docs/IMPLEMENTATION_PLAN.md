# Autonomous Warehouse Intelligence — Implementation Plan

> **Status (v1.0.0):** all phases are complete. This is the original plan, kept for reference. Changes from it: Docker Compose was dropped in favour of the one-command scripts in `scripts/`, and the results are in [RESULTS.md](RESULTS.md) and [REPORT.md](REPORT.md).

A virtual warehouse where 10–30 autonomous robots get tasks, pick up packages, move between shelves, deliver orders and recharge, all without colliding or blocking each other. The main experiment compares **independent pathfinding** with **coordinated planning**.

---

## 1. Tech stack decision

| Layer | Choice | Why (and what was rejected) |
|---|---|---|
| Language (core) | **Python 3.12** | Already installed. Strong algorithm and optimisation libraries. |
| Package manager | **uv** (`pip install uv`) | Fast, gives a lockfile, handles workspaces. Better than plain pip/venv. |
| Simulation | **Custom discrete-time grid engine** | Mesa was rejected. MAPF needs full control of the tick order, reservation tables and replanning, and Mesa's scheduler gets in the way while adding overhead. A custom engine is roughly 400 lines and fully deterministic. |
| Numerics | **NumPy** | Grid representation and fast distance maps. |
| Graph | **NetworkX** | Graph of the warehouse layout, Dijkstra, and validating the layout (is every cell reachable?). The hot-path A* is written by hand on NumPy for speed. |
| Assignment | **SciPy `linear_sum_assignment`** | A fast, battle-tested Hungarian algorithm. |
| Optimisation | **OR-Tools** | Order batching as a capacitated VRP, plus a CP-SAT task-assignment variant to compare against Hungarian. |
| Genetic algorithm | **Custom GA on NumPy** (DEAP is optional) | Optimises the warehouse layout and SKU slotting. Writing it by hand keeps the logic easy to explain. |
| Config / models | **Pydantic v2** | Typed scenario configs plus the API schemas, from a single source of truth. |
| API | **FastAPI + WebSocket** | REST to control and configure runs, a WebSocket to stream simulation state at 10–30 fps. |
| Frontend | **React + Vite + TypeScript + HTML Canvas (or PixiJS)** + **Plotly.js** | **Streamlit was rejected for the live view.** Its rerun-the-script model can't draw 30 robots smoothly in real time. React + Canvas gives a fluid animation, and Plotly keeps the charts the spec asked for. Node 20 is already installed. |
| Analysis | **Jupyter + Plotly + pandas**, results stored as **Parquet / DuckDB** | Batch experiments get stored and analysed reproducibly. |
| Statistics | **SciPy.stats** | Mann–Whitney U and 95% confidence intervals for the comparison experiment. |
| Quality | **pytest + Hypothesis**, **ruff**, **mypy**, **pre-commit**, GitHub Actions | Property tests check the key invariant: *coordinated plans never contain a conflict*. |
| Packaging | **Docker Compose** (api + web) | One-command demo. |

> If you want a pure-Python project with less frontend work, swap React for **Streamlit + Plotly** and stream frames by polling. You lose smooth animation but keep everything else. The plan below assumes React because you asked for a "perfect" project.

---

## 2. Architecture

```
┌──────────────────────── backend (Python) ────────────────────────┐
│                                                                  │
│  core/            sim engine: World, Robot, Task, tick loop      │
│  planning/        A*, Space-Time A*, Dijkstra maps, CBS, PP      │
│  assignment/      Greedy, Hungarian, OR-Tools CP-SAT / VRP       │
│  energy/          battery model, charger scheduling              │
│  optimization/    Genetic Algorithm (layout + slotting)          │
│  metrics/         collectors, KPIs, event log                    │
│  experiments/     headless batch runner, seeds, stats            │
│  api/             FastAPI REST + WebSocket stream                │
└──────────────────────────────────────────────────────────────────┘
                 │ WebSocket (JSON frames)   │ REST
┌──────────────── frontend (React + TS) ────────────────────────────┐
│  Live warehouse canvas · Controls · KPI cards · Plotly charts     │
│  Comparison view (independent vs coordinated, side by side)       │
└───────────────────────────────────────────────────────────────────┘
```

**Key design rule:** the engine must not know about the API or the UI. Every planner and assigner sits behind a `Protocol` interface, so an experiment just swaps strategies:

```python
class PathPlanner(Protocol):
    def plan(self, world: WorldView, requests: list[PathRequest], t: int) -> dict[RobotId, Path]: ...

class TaskAssigner(Protocol):
    def assign(self, world: WorldView, idle: list[Robot], tasks: list[Task]) -> dict[RobotId, TaskId]: ...
```

### Folder layout

```
autonomous-warehouse-intelligence/
├─ backend/
│  ├─ pyproject.toml
│  ├─ src/awi/
│  │  ├─ core/        (grid.py, world.py, robot.py, task.py, engine.py, rng.py)
│  │  ├─ planning/    (astar.py, dijkstra.py, st_astar.py, reservation.py,
│  │  │                cbs.py, prioritized.py, independent.py, conflicts.py)
│  │  ├─ assignment/  (greedy.py, hungarian.py, ortools_cpsat.py, batching_vrp.py)
│  │  ├─ energy/      (battery.py, charging.py)
│  │  ├─ dynamics/    (obstacles.py  – humans/forklifts, blocked aisles)
│  │  ├─ optimization/(ga_layout.py, ga_slotting.py)
│  │  ├─ metrics/     (collector.py, kpis.py)
│  │  ├─ experiments/ (runner.py, scenarios/*.yaml, stats.py)
│  │  └─ api/         (main.py, routes.py, ws.py, schemas.py, session.py)
│  └─ tests/
├─ frontend/          (Vite + React + TS)
├─ notebooks/         (analysis, report figures)
├─ docs/              (this plan, algorithm notes, results)
├─ docker-compose.yml
└─ README.md
```

---

## 3. Simulation model

- **World:** a 2D grid (for example 40×25, scalable). Cell types are `FLOOR`, `SHELF` (blocked), `PICK_POINT` (the floor cell next to a shelf), `DROP_STATION`, `CHARGER`, `AISLE` (optionally one-way).
- **Time:** discrete ticks. Each tick a robot does one of `UP/DOWN/LEFT/RIGHT/WAIT`. Optionally, rotation can cost one tick (more realistic).
- **Robot state machine:** `IDLE → TO_PICKUP → PICKING → TO_DROP → DROPPING → IDLE`, with `TO_CHARGER → CHARGING` as an interrupt from any state between tasks.
- **Tasks:** created by a seeded order generator (Poisson arrivals, demand follows Zipf/ABC so a few SKUs are hot). Each task records `created_t`, `assigned_t`, `picked_t`, `delivered_t`.
- **Conflicts (MAPF definitions):**
  - *Vertex conflict:* two robots in the same cell at time t.
  - *Edge (swap) conflict:* two robots swap cells between t and t+1.
- **Determinism:** one seed drives the task stream, obstacles and the GA. Given the same seed, both strategies see **the exact same task stream**, which is essential for a fair comparison.

---

## 4. Algorithms: where each one is used

| Algorithm | Role in the system |
|---|---|
| **Dijkstra** | Runs *backwards* from every pick point, drop station and charger to build true-distance maps over the static grid. These maps are (a) the A* heuristic (perfect on a static map) and (b) the cost matrix for task assignment. Built with NetworkX once at startup, then cached as NumPy arrays. |
| **A\*** | Single-agent shortest path on the static grid. It is the **independent** baseline planner. |
| **Space-Time A\*** | A* over (cell, t) states with a reservation table. Used inside CBS and Prioritized Planning. |
| **CBS (Conflict-Based Search)** | The **coordinated** planner. High level: a constraint tree branching on the first conflict. Low level: Space-Time A*. Plain CBS is exponential, so for 20–30 robots in a *lifelong* setting use **windowed CBS (rolling horizon)**: resolve conflicts only within the next *w* = 10–20 ticks and replan every *k* = 5 ticks or whenever an event happens. Add a **time budget with a fallback to Prioritized Planning**, so the sim never freezes. Stretch goal: ECBS (bounded-suboptimal) for 30+ robots. |
| **Prioritized Planning** | The fast coordinated fallback and a third comparison arm. Robots plan in priority order and reserve their paths. |
| **Hungarian** | Assigns idle robots to pending tasks every assignment tick. Cost = distance(robot → pickup) + λ·task_age penalty + battery-risk penalty (infeasible if the battery can't reach pickup → drop → nearest charger). |
| **OR-Tools** | (1) **Order batching:** a CVRP that groups multi-item orders into one robot trip with capacity *c*. (2) A **CP-SAT** assignment variant to benchmark against Hungarian. |
| **Genetic Algorithm** | **Slotting/layout optimisation:** a chromosome is a permutation of SKUs to shelf locations (plus optional station positions). Fitness = demand-weighted travel distance (fast proxy via the Dijkstra maps), and the top candidates are then checked with short simulations. Uses order crossover (OX), swap mutation, tournament selection and elitism. The result is a before/after comparison of the optimised layout. |

### Lifelong MAPF loop (coordinated mode)

```
each tick:
  1. dynamics.update()                 # humans/forklifts move, aisles block/unblock
  2. if assignment_tick: assigner.assign(idle robots, open tasks)
  3. battery.check()                   # low battery → insert charge goal
  4. if replan_needed (every k ticks / new goal / blocked path / conflict):
        planner.plan(all active robots, horizon w)   # windowed CBS, budget → PP fallback
  5. execute one step per robot; verify no conflict (assert in coordinated mode)
  6. metrics.record(tick)
```

### Independent mode (the baseline)

Each robot runs plain A* while ignoring other robots and only reacts locally: if its next cell is occupied, it waits, and if it has been stuck for N ticks it replans with that cell treated as an obstacle. This is how real naive fleets behave, and it produces **conflicts, near-collisions, blocking and deadlocks** that we can measure. A "physics-off" variant can also record raw path collisions when robots are allowed to overlap, for a pure measurement of conflict count.

---

## 5. Battery management

- Drain per move (higher when loaded), a small idle drain, and a charge rate per tick at a charger.
- **Feasibility check before assignment:** `battery ≥ cost(robot→pickup→drop→nearest_charger) × safety_margin`.
- Threshold policy: below 25% → go to charge once the current task is done; below 10% → abort the task and go to charge now (the task is requeued).
- Chargers are limited resources with a queue, and the assignment takes waiting time into account.
- KPIs: % of time charging, number of stranded robots (must be 0), energy per delivery.

## 6. Dynamic obstacles

- **Human pickers / forklifts:** agents that follow random walks or scripted routes. Robots can't see their future positions, only the current one plus a safety radius.
- **Temporary blockages:** an aisle closes for X ticks (a spill or maintenance).
- Response: invalidate any path that crosses a newly blocked cell, then do a local replan (independent mode) or a windowed re-solve (coordinated mode).

## 7. Metrics (the comparison experiment)

Collected per run, and per robot where it makes sense:

| Metric | Definition |
|---|---|
| Avg / P95 delivery time | `delivered_t − created_t` |
| Throughput | deliveries per 1,000 ticks |
| Collisions / conflicts | vertex + edge conflicts (physics-off) or near-collision events (physics-on) |
| Distance travelled | total cells moved, per robot and per fleet |
| Idle time | ticks spent `IDLE` with no task, *plus* ticks spent `WAIT` while blocked (reported separately) |
| Deadlocks | groups of robots that are mutually blocked for more than N ticks |
| Planner CPU time | ms per replan (shows the cost of coordination) |
| Energy | battery used per delivery, % of time charging |

**Experiment protocol:** 10 robots, independent vs coordinated, **30 seeds** each, same task streams, 5,000 ticks. Report mean ± 95% CI, Mann–Whitney U p-values, box plots, and a heatmap of congestion and conflict hotspots. Then run a **scaling sweep** at 10 / 20 / 30 robots to show where independent planning breaks down. Add **assignment ablations** (greedy vs Hungarian vs CP-SAT) and **layout ablations** (baseline vs GA-optimised).

---

## 8. Dashboard (frontend)

1. **Live view:** a canvas showing the grid, shelves, stations and chargers. Robots are coloured by state, with a battery ring, the planned path drawn ahead of each robot and a fading trail behind. Blocked cells and human agents are shown too.
2. **Controls:** start, pause, step and speed. Choose robot count, planner, assigner and seed. Inject an obstacle by clicking a cell.
3. **KPI cards:** throughput, average delivery time, conflicts, idle %, fleet battery.
4. **Charts (Plotly):** throughput over time, delivery-time histogram, battery levels, per-robot utilisation, congestion heatmap.
5. **Comparison view:** independent vs coordinated running side by side with the same seed, synchronised, plus experiment results loaded from the batch runner.
6. **Optimisation view:** GA fitness curve per generation, and the layout heatmap before and after.

---

## 9. Phased roadmap

Each phase ends with something working and tested.

| Phase | Deliverable | Est. |
|---|---|---|
| **0. Setup** | Monorepo, uv, ruff/mypy/pytest, pre-commit, CI, Vite app skeleton, README | 0.5 day |
| **1. Core engine** | Grid + layout generator, robot state machine, task generator, tick loop, seeded RNG, ASCII renderer for debugging | 2 days |
| **2. Single-agent planning** | Dijkstra distance maps (NetworkX → NumPy cache), A*, robots completing tasks end-to-end in independent mode with local wait rules | 1.5 days |
| **3. Metrics + headless runner** | Metrics collector, conflict detector, deadlock detector, batch runner → Parquet | 1 day |
| **4. Coordinated planning** | Space-Time A*, reservation table, Prioritized Planning, CBS, windowed lifelong CBS with time budget + fallback; Hypothesis property test that plans are conflict-free | 4 days |
| **5. Task assignment** | Greedy baseline, Hungarian (SciPy), OR-Tools CP-SAT, CVRP order batching | 1.5 days |
| **6. Battery** | Energy model, feasibility-aware assignment, charger queueing | 1 day |
| **7. Dynamic obstacles** | Human/forklift agents, aisle blockages, replanning triggers | 1 day |
| **8. API** | FastAPI sessions, REST control, WebSocket frame streaming, Pydantic schemas | 1.5 days |
| **9. Frontend** | Live canvas, controls, KPI cards, Plotly charts, comparison view | 3–4 days |
| **10. GA optimisation** | Slotting/layout GA, proxy fitness, sim validation, optimisation view | 1.5 days |
| **11. Experiments + report** | 30-seed comparison, scaling sweep, ablations, notebook figures, results in `docs/RESULTS.md` | 1.5 days |
| **12. Polish** | Docker Compose, demo scenarios, architecture diagram, demo GIF, README | 1 day |

**Total: about 3–4 weeks of focused work.** Phases 1–4 + 9 are the critical path. Ship them first for a demoable MVP of **independent vs CBS with a live view**.

---

## 10. Quality bar ("perfect project" checklist)

- [ ] Coordinated mode: **zero conflicts**, enforced by a runtime assertion and property tests
- [ ] Fully deterministic runs (seed → identical results), covered by a test
- [ ] Unit tests for each algorithm on small hand-made grids with known optimal answers (CBS optimality on 2–3 agent puzzles)
- [ ] Performance target: 30 robots, ≥20 ticks/s with windowed CBS on a laptop; planner budget ≤50 ms per replan
- [ ] Typed (mypy strict on `core/` and `planning/`), linted, CI green
- [ ] Scenario files (YAML) for reproducible demos
- [ ] Results doc with charts and statistical tests, plus an honest discussion of trade-offs (CBS CPU cost vs fewer conflicts)
- [ ] One command to run: `docker compose up`

## 11. Risks & mitigations

| Risk | Mitigation |
|---|---|
| CBS blows up with 30 robots | Windowed horizon, time budget, PP fallback, ECBS as a stretch goal |
| Deadlocks in lifelong mode | Make sure drop stations and chargers can be left (no dead-end parking), give idle robots "park" goals off the main aisles, detect deadlocks and handle them with randomised priority |
| Unfair comparison | Same seed and task stream, same assigner, same battery model; only the planner changes |
| UI lag | Send diffs or compact frames over WS, throttle to 20 fps, render on canvas (not DOM/SVG) |
| GA fitness is too slow | Use the Dijkstra-map proxy fitness and run the simulator only on the top-k candidates |
