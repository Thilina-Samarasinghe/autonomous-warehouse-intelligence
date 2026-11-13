from collections import Counter
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from awi.core.config import SimConfig
from awi.core.layout import generate_layout
from awi.core.orders import TaskGenerator
from awi.core.rng import make_rng


def _run(gen: TaskGenerator, ticks: int) -> list[tuple[int, tuple[int, int], tuple[int, int]]]:
    return [
        (task.created_t, task.shelf, task.dropoff) for t in range(ticks) for task in gen.generate(t)
    ]


def test_rng_streams_are_reproducible_and_independent() -> None:
    a = make_rng(42, "tasks").random(5)
    assert np.array_equal(a, make_rng(42, "tasks").random(5))
    assert not np.array_equal(a, make_rng(42, "spawn").random(5))
    assert not np.array_equal(a, make_rng(43, "tasks").random(5))


def test_generator_is_deterministic() -> None:
    layout = generate_layout()
    a = _run(TaskGenerator(layout, make_rng(7, "tasks"), rate=0.5), 300)
    b = _run(TaskGenerator(layout, make_rng(7, "tasks"), rate=0.5), 300)
    c = _run(TaskGenerator(layout, make_rng(8, "tasks"), rate=0.5), 300)
    assert a == b
    assert a != c


def test_arrival_rate_matches_poisson_mean() -> None:
    layout = generate_layout()
    gen = TaskGenerator(layout, make_rng(1, "tasks"), rate=0.4)
    n = len(_run(gen, 5000))
    assert 0.9 * 2000 < n < 1.1 * 2000


def test_tasks_are_well_formed() -> None:
    layout = generate_layout()
    gen = TaskGenerator(layout, make_rng(3, "tasks"), rate=2.0)
    tasks = [task for t in range(50) for task in gen.generate(t)]
    assert [t.id for t in tasks] == list(range(len(tasks)))
    for task in tasks:
        assert task.pickup == layout.pick_point[task.shelf]
        assert task.dropoff in layout.drop_stations


def test_zipf_skew_concentrates_demand() -> None:
    layout = generate_layout()
    gen = TaskGenerator(layout, make_rng(5, "tasks"), rate=5.0, zipf_s=1.2)
    counts = Counter(shelf for _, shelf, _ in _run(gen, 2000))
    hottest = layout.shelves[int(np.argmax(gen.shelf_probabilities))]
    assert counts.most_common(1)[0][0] == hottest
    assert np.isclose(gen.shelf_probabilities.sum(), 1.0)


def test_uniform_demand_when_zipf_zero() -> None:
    gen = TaskGenerator(generate_layout(), make_rng(0, "tasks"), rate=1.0, zipf_s=0.0)
    assert np.allclose(gen.shelf_probabilities, gen.shelf_probabilities[0])


def test_max_tasks_caps_generation() -> None:
    gen = TaskGenerator(generate_layout(), make_rng(0, "tasks"), rate=10.0, max_tasks=25)
    assert len(_run(gen, 100)) == 25
    assert gen.generated == 25


def test_invalid_rate() -> None:
    with pytest.raises(ValueError):
        TaskGenerator(generate_layout(), make_rng(0, "tasks"), rate=0)


def test_sim_config_yaml(tmp_path: Path) -> None:
    p = tmp_path / "scenario.yaml"
    p.write_text("seed: 9\nnum_robots: 12\nlayout:\n  shelf_segments: 3\n", encoding="utf-8")
    cfg = SimConfig.from_yaml(p)
    assert (cfg.seed, cfg.num_robots, cfg.layout.shelf_segments) == (9, 12, 3)
    p.write_text("robots: 3\n", encoding="utf-8")
    with pytest.raises(ValidationError):
        SimConfig.from_yaml(p)
