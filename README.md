# Autonomous Warehouse Intelligence

A virtual warehouse where **10–30 autonomous robots** get tasks, pick up packages, move between shelves, deliver orders and recharge, all without colliding or blocking each other.

The main experiment compares **independent pathfinding (A\*)** with **coordinated planning (Conflict-Based Search)**. It measures delivery time, collisions, distance travelled and idle time.

| Area | Techniques |
|---|---|
| Pathfinding | Dijkstra distance maps, A\*, Space-Time A\* |
| Multi-agent coordination | CBS (windowed / lifelong), Prioritized Planning |
| Task assignment | Hungarian algorithm, OR-Tools CP-SAT, VRP order batching |
| Optimisation | Genetic algorithm for warehouse layout and SKU slotting |
| Fleet management | Battery model, charger scheduling, dynamic obstacle avoidance |
| Platform | Custom Python simulator · FastAPI + WebSocket · React + Canvas + Plotly |

See [`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md) for the architecture and roadmap.

## Repository layout

```
backend/    Python package `awi`: simulator, planners, optimisers, FastAPI service
frontend/   React + TypeScript dashboard (Vite)
docs/       Plan, algorithm notes, results
notebooks/  Experiment analysis
```

## Requirements

- Python 3.12
- [uv](https://docs.astral.sh/uv/) (`pip install uv`)
- Node.js 20.19+

## Getting started

```bash
# Backend
cd backend
uv sync                      # create .venv and install dependencies
uv run awi-api               # API on http://127.0.0.1:8000  (docs at /docs)

# Frontend (second terminal)
cd frontend
npm install
npm run dev                  # dashboard on http://localhost:5173 (proxies /api to the backend)
```

Run a headless simulation from the command line:

```bash
cd backend
uv run awi-sim --robots 10 --ticks 2000                      # summary only
uv run awi-sim --robots 4 --ticks 100 --render-every 25      # ASCII map frames
uv run awi-sim --planner cbs --robots 10 --ticks 1000         # coordinated planning (also: prioritized)
uv run awi-sim --planner prioritized --assigner batching --capacity 2 --ticks 1000  # OR-Tools order batching
uv run awi-sim --config scenarios/stress_30.yaml --ticks 3000
```

Run batch experiments (parallel; results land in `backend/results/<name>/*.parquet`):

```bash
cd backend
uv run awi-exp run scenarios/experiments/smoke.yaml                  # seconds
uv run awi-exp run scenarios/experiments/baseline_sweep.yaml --workers 6
uv run awi-exp run scenarios/experiments/comparison.yaml --workers 6   # main experiment (~30 min)
uv run awi-exp run scenarios/experiments/assignment.yaml --workers 6   # assignment strategies
uv run awi-exp report results/baseline_sweep                         # tables + paired stats
```

See [`docs/RESULTS.md`](docs/RESULTS.md) for findings so far.

Optional analysis tools (pandas, DuckDB, Plotly, JupyterLab):

```bash
cd backend && uv sync --extra analysis
```

## Development

```bash
# Backend checks
cd backend
uv run pytest                # tests
uv run ruff check . && uv run ruff format --check .
uv run mypy                  # strict type checking

# Frontend checks
cd frontend
npm run lint && npm run build

# Git hooks (run once, from the repo root)
uv run --project backend pre-commit install
```

CI (GitHub Actions) runs the same checks on every push and pull request.

## Roadmap

- [x] Phase 0: project setup, tooling, CI
- [x] Phase 1: core simulation engine
- [x] Phase 2: Dijkstra + A\* (independent planning)
- [x] Phase 3: metrics + headless experiment runner
- [x] Phase 4: Space-Time A\*, Prioritized Planning, CBS
- [x] Phase 5: task assignment (Hungarian, CP-SAT, VRP batching)
- [ ] Phase 6: battery management
- [ ] Phase 7: dynamic obstacles
- [ ] Phase 8: FastAPI REST + WebSocket streaming
- [ ] Phase 9: live dashboard
- [ ] Phase 10: genetic algorithm layout optimisation
- [ ] Phase 11: experiments + results report
- [ ] Phase 12: polish, Docker, demo
