"""Command-line runner: ``uv run awi-sim --robots 10 --ticks 500``."""

from __future__ import annotations

import argparse
import time
from collections.abc import Sequence

from awi.core.config import SimConfig
from awi.core.engine import Simulation
from awi.core.render import render_ascii
from awi.experiments.registry import make_assigner
from awi.planning.base import PathPlanner
from awi.planning.coordinated import CBSPlanner, PrioritizedPlanner
from awi.planning.independent import IndependentAStarPlanner
from awi.planning.naive import BFSPlanner


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="awi-sim", description="Run a headless warehouse simulation")
    p.add_argument("--config", help="YAML scenario file (CLI flags override it)")
    p.add_argument("--robots", type=int, help="number of robots")
    p.add_argument("--ticks", type=int, default=500, help="ticks to simulate")
    p.add_argument("--seed", type=int, help="random seed")
    p.add_argument("--rate", type=float, help="expected new tasks per tick")
    p.add_argument(
        "--planner",
        choices=["astar", "bfs", "prioritized", "cbs"],
        default="astar",
        help="path planner (default: astar)",
    )
    p.add_argument(
        "--pick-cost",
        type=float,
        default=1.0,
        help="A* cost of entering pick-aisle cells (>1 discourages through-traffic)",
    )
    p.add_argument(
        "--assigner",
        choices=["greedy", "hungarian", "cpsat", "batching"],
        default="greedy",
        help="task assigner (default: greedy)",
    )
    p.add_argument("--capacity", type=int, help="items per robot trip (batching)")
    p.add_argument(
        "--render-every", type=int, default=0, metavar="N", help="print the map every N ticks"
    )
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = SimConfig.from_yaml(args.config) if args.config else SimConfig()
    overrides = {
        k: v
        for k, v in {
            "num_robots": args.robots,
            "seed": args.seed,
            "task_rate": args.rate,
            "capacity": args.capacity,
        }.items()
        if v is not None
    }
    cfg = SimConfig.model_validate(cfg.model_dump() | overrides)

    planner: PathPlanner
    if args.planner == "bfs":
        planner = BFSPlanner(seed=cfg.seed)
    elif args.planner == "prioritized":
        planner = PrioritizedPlanner()
    elif args.planner == "cbs":
        planner = CBSPlanner()
    else:
        planner = IndependentAStarPlanner(seed=cfg.seed, pick_cell_cost=args.pick_cost)
    sim = Simulation(cfg, planner=planner, assigner=make_assigner(args.assigner, cfg))
    start = time.perf_counter()
    for _ in range(args.ticks):
        sim.step()
        if args.render_every and sim.t % args.render_every == 0:
            print(render_ascii(sim.world), end="\n\n")
    elapsed = time.perf_counter() - start

    s = sim.summary()
    avg = f"{s.avg_delivery_time:.1f}" if s.avg_delivery_time is not None else "n/a"
    print(
        f"planner={args.planner} assigner={args.assigner} "
        f"robots={cfg.num_robots} seed={cfg.seed} "
        f"ticks={s.ticks} ({s.ticks / elapsed:,.0f}/s)"
    )
    print(
        f"tasks: generated={s.tasks_generated} delivered={s.tasks_delivered} "
        f"in_progress={s.tasks_in_progress} pending={s.tasks_pending}"
    )
    print(
        f"avg delivery time={avg} ticks  distance={s.total_distance}  "
        f"waits={s.total_wait_ticks}  blocked={s.total_blocked_ticks}  idle={s.total_idle_ticks}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
