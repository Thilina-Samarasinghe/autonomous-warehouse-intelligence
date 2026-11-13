"""Report figures generated from experiment Parquet files (``awi-exp figures``).

Style follows the project's data-viz rules: a validated colour-blind-safe
categorical order (blue, orange, aqua, yellow), each entity keeps its colour in
every figure, thin marks, hairline grid, no dual axes, a legend for two or more
series, and text in neutral ink (never in series colours). Requires the
``report`` extra (matplotlib).
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from awi.experiments.stats import mean_ci

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SLOTS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]

PLANNER_COLOR = {"independent_astar": SLOTS[0], "prioritized": SLOTS[1], "cbs": SLOTS[2]}
PLANNER_LABEL = {
    "independent_astar": "Independent A*",
    "independent_bfs": "Independent BFS",
    "prioritized": "Prioritized Planning",
    "cbs": "CBS",
}


def _style() -> Any:
    import matplotlib as mpl  # noqa: PLC0415

    mpl.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "savefig.facecolor": SURFACE,
            "font.family": ["Segoe UI", "DejaVu Sans", "sans-serif"],
            "font.size": 10,
            "text.color": INK,
            "axes.labelcolor": INK_2,
            "axes.edgecolor": AXIS,
            "axes.linewidth": 1,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "axes.axisbelow": True,
            "grid.color": GRID,
            "grid.linewidth": 0.8,
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "xtick.labelcolor": INK_2,
            "ytick.labelcolor": INK_2,
            "legend.frameon": False,
            "legend.labelcolor": INK_2,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.titlelocation": "left",
            "axes.titlecolor": INK,
        }
    )
    return plt


def _ci(values: Sequence[float] | np.ndarray) -> tuple[float, float]:
    m, lo, hi = mean_ci(np.asarray(values, dtype=float))
    return m, (hi - lo) / 2 if np.isfinite(hi) else 0.0


def _load(results: Path, name: str) -> pd.DataFrame | None:
    path = results / name / "runs.parquet"
    return pd.read_parquet(path) if path.exists() else None


def _grouped_bars(
    ax: Any,
    groups: list[str],
    series: list[tuple[str, str, list[tuple[float, float]]]],
    fmt: str = "{:.0f}",
) -> None:
    """Grouped columns with 95% CI whiskers; value labels at the caps."""
    n = len(series)
    width = min(0.8 / n, 0.3)
    x = np.arange(len(groups))
    for i, (label, color, stats) in enumerate(series):
        pos = x + (i - (n - 1) / 2) * width
        means = [m for m, _ in stats]
        errs = [e for _, e in stats]
        ax.bar(pos, means, width, color=color, edgecolor=SURFACE, linewidth=2, label=label)
        ax.errorbar(pos, means, yerr=errs, fmt="none", ecolor=INK_2, elinewidth=1, capsize=3)
        for p, m, e in zip(pos, means, errs, strict=True):
            ax.annotate(
                fmt.format(m),
                (p, m + e),
                xytext=(0, 3),
                textcoords="offset points",
                ha="center",
                va="bottom",
                fontsize=8,
                color=INK_2,
            )
    ax.set_xticks(x, groups)
    ax.grid(axis="x", visible=False)
    top = max((m + e) for _, _, stats in series for m, e in stats)
    ax.set_ylim(0, top * 1.28 if top > 0 else 1)
    if n > 1:
        ax.legend(loc="upper left", ncol=n, fontsize=8, handlelength=1.2, columnspacing=1.2)


def make_figures(results: Path, out: Path) -> list[Path]:
    """Render every figure whose experiment results exist; return written paths."""
    plt = _style()
    out.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    def save(fig: Any, name: str) -> None:
        path = out / name
        fig.savefig(path, dpi=160, bbox_inches="tight")
        plt.close(fig)
        written.append(path)

    cmp = _load(results, "comparison")
    if cmp is not None:
        _fig_scaling(plt, cmp, save)
        _fig_collisions_delivery(plt, cmp, save)
        ts_path = results / "comparison" / "timeseries.parquet"
        if ts_path.exists():
            _fig_gridlock(plt, cmp, pd.read_parquet(ts_path), save)
    asg = _load(results, "assignment")
    if asg is not None:
        _fig_assignment(plt, asg, save)
    for name, fname, variants, labels, title in VARIANT_FIGURES:
        df = _load(results, name)
        if df is not None:
            _fig_variants(
                plt,
                df,
                name=name,
                fname=fname,
                variants=variants,
                labels=labels,
                title=title,
                save=save,
            )
    return written


PLANNERS = ["independent_astar", "prioritized", "cbs"]


def _fig_scaling(plt: Any, cmp: pd.DataFrame, save: Any) -> None:
    from matplotlib.lines import Line2D  # noqa: PLC0415

    robots = sorted(cmp.num_robots.unique())
    dodge = {"independent_astar": 0.0, "prioritized": -0.35, "cbs": 0.35}
    fig, ax = plt.subplots(figsize=(6.4, 3.8))
    ends: dict[str, float] = {}
    present = [p for p in PLANNERS if p in set(cmp.planner)]
    for p in present:
        stats = [
            _ci(cmp[(cmp.planner == p) & (cmp.num_robots == n)].throughput_per_1k) for n in robots
        ]
        m = [s[0] for s in stats]
        xs = [n + dodge[p] for n in robots]
        ax.errorbar(
            xs,
            m,
            yerr=[s[1] for s in stats],
            color=PLANNER_COLOR[p],
            lw=2,
            marker="o",
            ms=6,
            mec=SURFACE,
            mew=2,
            capsize=3,
        )
        ends[p] = m[-1]
    if "independent_astar" in ends:
        ax.annotate(
            f"{ends['independent_astar']:.0f}",
            (robots[-1], ends["independent_astar"]),
            xytext=(10, 0),
            textcoords="offset points",
            va="center",
            fontsize=9,
            color=INK_2,
        )
    if "prioritized" in ends and "cbs" in ends:
        ax.annotate(
            f"PP {ends['prioritized']:.0f} · CBS {ends['cbs']:.0f}",
            (robots[-1], ends["cbs"]),
            xytext=(-8, 12),
            textcoords="offset points",
            ha="right",
            fontsize=9,
            color=INK_2,
        )
    handles = [
        Line2D([], [], color=PLANNER_COLOR[p], lw=2, marker="o", ms=6, mec=SURFACE) for p in present
    ]
    ax.legend(handles, [PLANNER_LABEL[p] for p in present], loc="upper left")
    ax.set_xticks(robots)
    ax.set_xlabel("Robots")
    ax.set_ylabel("Deliveries per 1,000 ticks")
    ax.set_title("Throughput vs fleet size (mean, 95% CI, 10 seeds)")
    ax.set_ylim(0, max(ends.values()) * 1.15)
    save(fig, "fig1_throughput_scaling.png")


def _fig_collisions_delivery(plt: Any, cmp: pd.DataFrame, save: Any) -> None:
    """Two panels side by side (never a dual axis)."""
    robots = sorted(cmp.num_robots.unique())
    groups = [f"{n} robots" for n in robots]

    def series(col: str) -> list[tuple[str, str, list[tuple[float, float]]]]:
        return [
            (
                PLANNER_LABEL[p],
                PLANNER_COLOR[p],
                [
                    _ci(cmp[(cmp.planner == p) & (cmp.num_robots == n)][col].dropna())
                    for n in robots
                ],
            )
            for p in PLANNERS
            if p in set(cmp.planner)
        ]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 3.6))
    _grouped_bars(a1, groups, series("conflicts_total"), fmt="{:,.0f}")
    a1.set_title("Would-be collisions per run")
    _grouped_bars(a2, groups, series("delivery_time_mean"))
    a2.set_title("Average delivery time (ticks)")
    save(fig, "fig2_collisions_delivery.png")


def _fig_gridlock(plt: Any, cmp: pd.DataFrame, ts: pd.DataFrame, save: Any) -> None:
    ind = cmp[(cmp.planner == "independent_astar") & (cmp.num_robots == 30)]
    if ind.empty or "cbs" not in set(cmp.planner):
        return
    seed = int(ind.sort_values("throughput_per_1k").iloc[0].seed)
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for p in ("independent_astar", "cbs"):
        s = ts[(ts.planner == p) & (ts.num_robots == 30) & (ts.seed == seed)].sort_values("t_end")
        ax.plot(s.t_end, s.throughput_per_1k, color=PLANNER_COLOR[p], lw=2, label=PLANNER_LABEL[p])
    ax.set_xlabel("Tick")
    ax.set_ylabel("Deliveries per 1,000 ticks")
    ax.set_title(f"30 robots, seed {seed}: independent planning gridlocks")
    ax.set_ylim(bottom=0)
    ax.legend(loc="center right")
    save(fig, "fig3_gridlock_timeseries.png")


def _fig_assignment(plt: Any, asg: pd.DataFrame, save: Any) -> None:
    order = ["greedy", "hungarian", "cpsat", "batching"]
    labels = {
        "greedy": "Greedy",
        "hungarian": "Hungarian",
        "cpsat": "CP-SAT",
        "batching": "Batching (2/trip)",
    }
    robots = sorted(asg.num_robots.unique())
    groups = [f"{n} robots" for n in robots]

    def series(col: str) -> list[tuple[str, str, list[tuple[float, float]]]]:
        return [
            (
                labels[a],
                SLOTS[i],
                [
                    _ci(asg[(asg.assigner == a) & (asg.num_robots == n)][col].dropna())
                    for n in robots
                ],
            )
            for i, a in enumerate(order)
        ]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(9.6, 3.6))
    _grouped_bars(a1, groups, series("throughput_per_1k"))
    a1.set_title("Throughput per 1,000 ticks")
    _grouped_bars(a2, groups, series("delivery_time_mean"))
    a2.set_title("Average delivery time (ticks)")
    save(fig, "fig4_assignment.png")


VARIANT_FIGURES: list[tuple[str, str, list[str], dict[str, str], str]] = [
    (
        "battery",
        "fig5_battery.png",
        ["no_battery", "battery_4_chargers", "battery_2_chargers", "battery_conservative"],
        {
            "no_battery": "No battery",
            "battery_4_chargers": "4 chargers",
            "battery_2_chargers": "2 chargers",
            "battery_conservative": "Conservative",
        },
        "Energy management, 20 robots (prioritized planning): throughput per 1,000 ticks",
    ),
    (
        "dynamics",
        "fig6_dynamics.png",
        ["static", "humans_6", "humans_12", "blockages", "humans_6_blockages"],
        {
            "static": "None",
            "humans_6": "6 people",
            "humans_12": "12 people",
            "blockages": "Blockages",
            "humans_6_blockages": "6 people\n+ blockages",
        },
        "Dynamic obstacles, 20 robots: throughput per 1,000 ticks",
    ),
    (
        "slotting",
        "fig7_slotting.png",
        ["random", "closest", "ga_light", "ga_balanced"],
        {
            "random": "Random",
            "closest": "Closest",
            "ga_light": "GA λ=0.05",
            "ga_balanced": "GA λ=0.5",
        },
        "SKU slotting, 30 robots: throughput per 1,000 ticks",
    ),
]


def _fig_variants(
    plt: Any,
    df: pd.DataFrame,
    *,
    name: str,
    fname: str,
    variants: list[str],
    labels: dict[str, str],
    title: str,
    save: Any,
) -> None:
    n = 30 if name == "slotting" else 20
    df = df[df.num_robots == n]
    if df.empty or not set(variants) <= set(df.variant):
        return
    planners = [p for p in ("independent_astar", "prioritized") if p in set(df.planner)]
    fig, ax = plt.subplots(figsize=(7.6, 3.6))
    _grouped_bars(
        ax,
        [labels[v] for v in variants],
        [
            (
                PLANNER_LABEL[p],
                PLANNER_COLOR[p],
                [
                    _ci(df[(df.planner == p) & (df.variant == v)].throughput_per_1k)
                    for v in variants
                ],
            )
            for p in planners
        ],
    )
    ax.set_title(title)
    save(fig, fname)
