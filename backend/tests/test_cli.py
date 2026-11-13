from pathlib import Path

import pytest

from awi.cli import main

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def test_cli_runs_and_reports(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--robots", "3", "--ticks", "50", "--seed", "1", "--render-every", "50"]) == 0
    out = capsys.readouterr().out
    assert "planner=astar assigner=greedy robots=3 seed=1 ticks=50" in out
    assert "t=50" in out  # rendered frame legend
    assert "delivered=" in out


@pytest.mark.parametrize("scenario", sorted(SCENARIOS.glob("*.yaml")), ids=lambda p: p.stem)
def test_bundled_scenarios_load(scenario: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--config", str(scenario), "--ticks", "20"]) == 0
    assert "tasks:" in capsys.readouterr().out
