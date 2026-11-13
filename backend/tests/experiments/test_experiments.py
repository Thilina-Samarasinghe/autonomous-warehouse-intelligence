from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from pydantic import ValidationError

from awi.experiments.cli import main as exp_main
from awi.experiments.cli import strategy_column
from awi.experiments.registry import make_planner, register_planner
from awi.experiments.runner import ExperimentSpec, run_experiment, run_single
from awi.experiments.stats import cliffs_delta, compare, mean_ci, summarize
from tests.conftest import SMALL_LAYOUT

SPECS = Path(__file__).resolve().parents[2] / "scenarios" / "experiments"


def _spec(**kw: object) -> ExperimentSpec:
    base = {
        "name": "t",
        "planners": ["independent_astar", "independent_bfs"],
        "robots": [3, 5],
        "seeds": 2,
        "ticks": 120,
        "warmup_ticks": 20,
        "base": {"task_rate": 0.1, "layout": SMALL_LAYOUT.model_dump()},
    }
    return ExperimentSpec.model_validate(base | kw)


# --------------------------------------------------------------------- spec


def test_spec_expands_cross_product_with_shared_seeds() -> None:
    runs = _spec(task_rate_per_robot=0.02).runs()
    assert len(runs) == 2 * 2 * 2
    by_key = {(r.config["num_robots"], r.config["seed"], r.planner): r for r in runs}
    for n in (3, 5):
        for seed in (0, 1):
            a = by_key[(n, seed, "independent_astar")].config
            b = by_key[(n, seed, "independent_bfs")].config
            assert a == b  # identical scenario per planner
            assert a["task_rate"] == pytest.approx(0.02 * n)


def test_spec_validation() -> None:
    with pytest.raises(ValidationError, match="unknown planners"):
        _spec(planners=["nope"])
    with pytest.raises(ValidationError):
        _spec(name="bad name!")
    assert _spec(seeds=[4, 7]).seed_list == [4, 7]


def test_bundled_specs_are_valid() -> None:
    specs = sorted(SPECS.glob("*.yaml"))
    assert specs
    for path in specs:
        assert ExperimentSpec.from_yaml(path).runs()


def test_registry() -> None:
    with pytest.raises(ValueError, match="unknown planner"):
        make_planner("missing", _spec().base)
    with pytest.raises(ValueError, match="already registered"):
        register_planner("independent_astar", lambda cfg: make_planner("independent_bfs", cfg))


# --------------------------------------------------------------------- runs


def test_run_single_is_deterministic() -> None:
    run = _spec().runs()[0]
    a, b = run_single(run), run_single(run)
    a.row.pop("wall_seconds"), b.row.pop("wall_seconds")
    for row in (a.row, b.row):
        row.pop("plan_ms_total"), row.pop("plan_ms_mean"), row.pop("plan_ms_max")
    assert a.row == b.row
    assert {"planner", "num_robots", "seed", "throughput_per_1k"} <= set(a.row)
    assert a.series and all(w["planner"] == run.planner for w in a.series)


@pytest.mark.parametrize("workers", [1, 2])
def test_run_experiment_writes_parquet(tmp_path: Path, workers: int) -> None:
    spec = _spec()
    res = run_experiment(spec, tmp_path, workers=workers)
    assert res.out_dir == tmp_path / "t"
    runs = pd.read_parquet(tmp_path / "t" / "runs.parquet")
    ts = pd.read_parquet(tmp_path / "t" / "timeseries.parquet")
    assert len(runs) == len(spec.runs()) == len(res.runs)
    assert set(ts["planner"]) == set(spec.planners)
    assert ExperimentSpec.from_yaml(tmp_path / "t" / "spec.yaml") == spec


def test_cli_run_and_report(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    spec_file = tmp_path / "spec.yaml"
    spec_file.write_text((SPECS / "smoke.yaml").read_text(encoding="utf-8"), encoding="utf-8")
    out = tmp_path / "res"
    assert (
        exp_main(["run", str(spec_file), "--workers", "1", "--out", str(out), "--seeds", "2"]) == 0
    )
    text = capsys.readouterr().out
    assert "experiment smoke: 4 runs" in text
    assert "Paired comparison" in text
    assert exp_main(["report", str(out / "smoke")]) == 0
    assert "Mean per" in capsys.readouterr().out
    assert exp_main(["report", str(tmp_path)]) == 1


# -------------------------------------------------------------------- stats


def test_mean_ci() -> None:
    m, lo, hi = mean_ci([1.0, 2.0, 3.0, 4.0])
    assert m == 2.5 and lo < m < hi
    assert mean_ci([5.0, 5.0]) == (5.0, 5.0, 5.0)
    assert all(np.isnan(v) for v in mean_ci([]))


def test_cliffs_delta() -> None:
    assert cliffs_delta([3, 4], [1, 2]) == 1.0
    assert cliffs_delta([1, 2], [3, 4]) == -1.0
    assert cliffs_delta([1, 2], [1, 2]) == 0.0


def test_summarize_and_compare() -> None:
    rng = np.random.default_rng(0)
    rows = []
    for seed in range(12):
        base = rng.normal(100, 5)
        rows.append({"planner": "a", "seed": seed, "num_robots": 10, "x": base + 10})
        rows.append({"planner": "b", "seed": seed, "num_robots": 10, "x": base})
    df = pd.DataFrame(rows)
    s = summarize(df, ["x"])
    assert set(s["planner"]) == {"a", "b"} and (s["n"] == 12).all()
    c = compare(df, "a", "b", ["x"]).iloc[0]
    assert c["diff"] == pytest.approx(10.0)
    assert c["wilcoxon_p"] < 0.01  # paired test sees the consistent shift
    same = compare(df.assign(x=1.0), "a", "b", ["x"]).iloc[0]
    assert same["wilcoxon_p"] == 1.0 and same["mannwhitney_p"] == 1.0


def test_assigner_dimension_and_strategy_column() -> None:
    spec = _spec(planners=["independent_astar"], assigners=["greedy", "hungarian"], robots=[3])
    assert len(spec.runs()) == 2 * 2
    res = run_experiment(spec, None, workers=1)
    assert set(res.runs["assigner"]) == {"greedy", "hungarian"}
    assert strategy_column(res.runs.copy()) == "assigner"
    both = res.runs.copy()
    both.loc[0, "planner"] = "x"
    assert strategy_column(both) == "strategy"
    cmp = compare(res.runs, "hungarian", "greedy", ["throughput_per_1k"], group_col="assigner")
    assert cmp.iloc[0]["n_pairs"] == 2
    with pytest.raises(ValidationError, match="unknown assigners"):
        _spec(assigners=["nope"])
