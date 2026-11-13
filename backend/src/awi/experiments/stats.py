"""Statistics for comparing planners across seeds.

* :func:`summarize`: mean, std and a t-based 95% confidence interval per group.
* :func:`compare`: planner A vs planner B on one metric, per robot count.
  Runs are *paired by seed* (same task stream), so the main test is the Wilcoxon
  signed-rank test on paired differences. Mann-Whitney U (unpaired) and Cliff's
  delta (effect size, -1..1) are reported as well.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pandas as pd
from scipy import stats

DEFAULT_METRICS: tuple[str, ...] = (
    "throughput_per_1k",
    "delivery_time_mean",
    "delivery_time_p95",
    "conflicts_total",
    "distance_per_delivery",
    "idle_frac",
    "blocked_frac",
    "deadlock_events",
    "plan_ms_mean",
)


def mean_ci(
    values: Sequence[float] | np.ndarray, confidence: float = 0.95
) -> tuple[float, float, float]:
    """Return ``(mean, low, high)`` of a t-distribution confidence interval."""
    x = np.asarray(values, dtype=np.float64)
    x = x[~np.isnan(x)]
    if x.size == 0:
        return (np.nan, np.nan, np.nan)
    m = float(x.mean())
    if x.size < 2 or np.all(x == x[0]):
        return (m, m, m)
    half = float(stats.sem(x) * stats.t.ppf((1 + confidence) / 2, x.size - 1))
    return (m, m - half, m + half)


def cliffs_delta(a: Sequence[float] | np.ndarray, b: Sequence[float] | np.ndarray) -> float:
    """P(a > b) - P(a < b), in [-1, 1]. |d| < .147 negligible, < .33 small, < .474 medium."""
    x, y = np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)
    if x.size == 0 or y.size == 0:
        return np.nan
    diff = x[:, None] - y[None, :]
    return float((np.sum(diff > 0) - np.sum(diff < 0)) / diff.size)


def summarize(
    runs: pd.DataFrame,
    metrics: Sequence[str] = DEFAULT_METRICS,
    by: Sequence[str] = ("num_robots", "planner"),
) -> pd.DataFrame:
    rows = []
    for keys, group in runs.groupby(list(by), sort=True):
        key_vals = keys if isinstance(keys, tuple) else (keys,)
        for metric in metrics:
            m, lo, hi = mean_ci(group[metric].to_numpy())
            rows.append(
                {
                    **dict(zip(by, key_vals, strict=True)),
                    "metric": metric,
                    "n": int(group[metric].notna().sum()),
                    "mean": m,
                    "std": float(group[metric].std(ddof=1)) if len(group) > 1 else 0.0,
                    "ci95_low": lo,
                    "ci95_high": hi,
                }
            )
    return pd.DataFrame(rows)


def compare(
    runs: pd.DataFrame,
    a: str,
    b: str,
    metrics: Sequence[str] = DEFAULT_METRICS,
    *,
    by: str = "num_robots",
    pair_on: str = "seed",
) -> pd.DataFrame:
    """Compare planner ``a`` against ``b``; ``diff`` and ``rel_change`` are a - b."""
    rows = []
    for key, group in runs.groupby(by, sort=True):
        ga = group[group["planner"] == a].set_index(pair_on)
        gb = group[group["planner"] == b].set_index(pair_on)
        common = ga.index.intersection(gb.index)
        for metric in metrics:
            xa = ga.loc[common, metric].to_numpy(dtype=np.float64)
            xb = gb.loc[common, metric].to_numpy(dtype=np.float64)
            ok = ~(np.isnan(xa) | np.isnan(xb))
            xa, xb = xa[ok], xb[ok]
            mean_a = float(xa.mean()) if xa.size else np.nan
            mean_b = float(xb.mean()) if xb.size else np.nan
            rows.append(
                {
                    by: key,
                    "metric": metric,
                    "n_pairs": int(xa.size),
                    f"mean_{a}": mean_a,
                    f"mean_{b}": mean_b,
                    "diff": mean_a - mean_b,
                    "rel_change": (mean_a - mean_b) / abs(mean_b) if mean_b else np.nan,
                    "wilcoxon_p": _wilcoxon_p(xa, xb),
                    "mannwhitney_p": _mannwhitney_p(xa, xb),
                    "cliffs_delta": cliffs_delta(xa, xb),
                }
            )
    return pd.DataFrame(rows)


def _wilcoxon_p(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 2:
        return np.nan
    if np.allclose(a, b):
        return 1.0
    return float(stats.wilcoxon(a, b).pvalue)


def _mannwhitney_p(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 2 or b.size < 2:
        return np.nan
    if np.allclose(np.concatenate([a, b]), a[0]):
        return 1.0
    return float(stats.mannwhitneyu(a, b, alternative="two-sided").pvalue)
