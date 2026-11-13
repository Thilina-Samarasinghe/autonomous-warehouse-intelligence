from pathlib import Path

import pandas as pd
import pytest

from awi.experiments.runner import ExperimentSpec, run_experiment
from awi.experiments.verify import verify_results
from tests.conftest import SMALL_LAYOUT


def _run(tmp_path: Path, name: str = "t") -> Path:
    spec = ExperimentSpec.model_validate(
        {
            "name": name,
            "planners": ["independent_astar", "prioritized"],
            "robots": [3],
            "seeds": 2,
            "ticks": 120,
            "warmup_ticks": 20,
            "base": {"task_rate": 0.1, "layout": SMALL_LAYOUT.model_dump()},
        }
    )
    res = run_experiment(spec, tmp_path, workers=1)
    assert res.out_dir is not None
    return res.out_dir


def test_verify_reproduces_and_detects_tampering(tmp_path: Path) -> None:
    out = _run(tmp_path)
    report = verify_results(out, sample=4)
    assert report.ok and report.checked == 4 and not report.mismatches

    runs = pd.read_parquet(out / "runs.parquet")
    runs.loc[:, "distance_total"] += 1  # pretend the recorded results were edited
    runs.to_parquet(out / "runs.parquet", index=False)
    tampered = verify_results(out, sample=2)
    assert not tampered.ok
    assert {m["column"] for m in tampered.mismatches} == {"distance_total"}


def test_figures_render_from_partial_results(tmp_path: Path) -> None:
    pytest.importorskip("matplotlib")
    from awi.experiments.figures import make_figures  # noqa: PLC0415

    _run(tmp_path, name="comparison")
    written = make_figures(tmp_path, tmp_path / "figs")
    names = {p.name for p in written}
    assert {"fig1_throughput_scaling.png", "fig2_collisions_delivery.png"} <= names
    assert "fig3_gridlock_timeseries.png" not in names  # needs 30-robot CBS runs
    assert all(p.stat().st_size > 5_000 for p in written)
