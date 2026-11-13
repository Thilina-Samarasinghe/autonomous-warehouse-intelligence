import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from pydantic import ValidationError

from awi.core.config import SimConfig
from awi.core.engine import Simulation, plan_slotting
from awi.core.layout import generate_layout
from awi.optimization.ga import GAConfig, genetic_slotting, mutate, order_crossover
from awi.optimization.slotting import SlottingConfig, SlottingProblem, pick_aisles, zipf_weights
from awi.planning.dijkstra import DistanceMaps
from tests.conftest import SMALL_LAYOUT


@pytest.fixture(scope="module")
def problem() -> SlottingProblem:
    layout = generate_layout()
    return SlottingProblem(
        layout, DistanceMaps.for_layout(layout), zipf_s=1.0, congestion_weight=0.5
    )


def _is_perm(x: np.ndarray, n: int) -> bool:
    return sorted(x.tolist()) == list(range(n))


# ---------------------------------------------------------------- operators


@settings(max_examples=100, deadline=None)
@given(n=st.integers(2, 60), seed=st.integers(0, 10_000))
def test_crossover_and_mutation_keep_permutations(n: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    a, b = rng.permutation(n).astype(np.int64), rng.permutation(n).astype(np.int64)
    child = order_crossover(a, b, rng)
    assert _is_perm(child, n)
    assert _is_perm(mutate(child, rng), n)


def test_crossover_keeps_a_slice_of_the_first_parent() -> None:
    rng = np.random.default_rng(1)
    a = np.arange(10, dtype=np.int64)
    b = a[::-1].copy()
    child = order_crossover(a, b, rng)
    same = child == a
    # OX1 copies one contiguous block from parent A.
    idx = np.flatnonzero(same)
    assert idx.size >= 1


# ------------------------------------------------------------------ problem


def test_zipf_weights_and_aisles(problem: SlottingProblem) -> None:
    w = zipf_weights(5, 1.0)
    assert np.isclose(w.sum(), 1.0) and np.all(np.diff(w) < 0)
    assert np.allclose(zipf_weights(4, 0.0), 0.25)
    aisle, n = pick_aisles(problem.layout)
    assert n == 20 and len(aisle) == problem.n
    assert set(aisle.tolist()) == set(range(n))


def test_closest_is_travel_optimal(problem: SlottingProblem) -> None:
    rng = np.random.default_rng(0)
    best = problem.score(problem.closest_assignment()).travel
    for _ in range(50):
        assert problem.score(problem.random_assignment(rng)).travel >= best - 1e-9


def test_concentration_bounds(problem: SlottingProblem) -> None:
    rng = np.random.default_rng(3)
    for _ in range(20):
        c = problem.score(problem.random_assignment(rng)).concentration
        assert 1.0 - 1e-9 <= c <= problem.n_aisles
    probs = problem.shelf_probabilities(problem.closest_assignment())
    assert np.isclose(probs.sum(), 1.0)
    assert np.allclose(np.sort(probs)[::-1], problem.weights)


# ------------------------------------------------------------------------ GA


def test_ga_improves_and_never_regresses(problem: SlottingProblem) -> None:
    rng = np.random.default_rng(0)
    res = genetic_slotting(problem, GAConfig(population=30, generations=60), rng)
    assert _is_perm(res.best, problem.n)
    assert res.best_fitness == pytest.approx(problem.fitness(res.best))
    assert all(b2 <= b1 for b1, b2 in zip(res.history_best, res.history_best[1:], strict=False))
    assert res.best_fitness < problem.fitness(problem.closest_assignment())  # beats its seed
    random_fit = np.mean([problem.fitness(problem.random_assignment(rng)) for _ in range(20)])
    assert res.best_fitness < 0.8 * random_fit
    assert res.evaluations == 30 * 61


def test_ga_matches_greedy_without_congestion_penalty() -> None:
    layout = generate_layout()
    p = SlottingProblem(layout, DistanceMaps.for_layout(layout), congestion_weight=0.0)
    res = genetic_slotting(p, GAConfig(population=20, generations=20), np.random.default_rng(0))
    assert res.best_fitness == pytest.approx(p.score(p.closest_assignment()).travel)


def test_ga_is_deterministic(problem: SlottingProblem) -> None:
    a = genetic_slotting(problem, GAConfig(population=20, generations=15), np.random.default_rng(7))
    b = genetic_slotting(problem, GAConfig(population=20, generations=15), np.random.default_rng(7))
    assert np.array_equal(a.best, b.best) and a.history_best == b.history_best


def test_ga_config_validation() -> None:
    with pytest.raises(ValueError):
        GAConfig(population=2)
    with pytest.raises(ValueError):
        GAConfig(population=10, elite=10)
    with pytest.raises(ValidationError):
        SlottingConfig(strategy="magic")  # type: ignore[arg-type]


# --------------------------------------------------------------- simulation


def test_slotting_changes_shelves_but_not_arrivals() -> None:
    def tasks(strategy: str) -> list[tuple[int, tuple[int, int], tuple[int, int]]]:
        cfg = SimConfig.model_validate(
            {
                "num_robots": 4,
                "layout": SMALL_LAYOUT.model_dump(),
                "seed": 5,
                "slotting": {"strategy": strategy, "generations": 10, "population": 10},
            }
        )
        sim = Simulation(cfg)
        sim.run(150)
        return [(t.created_t, t.shelf, t.dropoff) for t in sim.world.tasks.values()]

    random, closest, ga = tasks("random"), tasks("closest"), tasks("ga")
    # Same arrival ticks and drop stations: only the shelf differs.
    for a, b in ((random, closest), (random, ga)):
        assert [(t, d) for t, _, d in a] == [(t, d) for t, _, d in b]
    assert [s for _, s, _ in random] != [s for _, s, _ in closest]


def test_plan_slotting_strategies() -> None:
    layout = generate_layout(SMALL_LAYOUT)
    maps = DistanceMaps.for_layout(layout)
    base = SimConfig(layout=SMALL_LAYOUT)
    assert plan_slotting(base, layout, maps) is None
    closest = plan_slotting(
        base.model_copy(update={"slotting": SlottingConfig(strategy="closest")}), layout, maps
    )
    assert closest is not None and _is_perm(closest, len(layout.shelves))
    sim = Simulation(base.model_copy(update={"slotting": SlottingConfig(strategy="closest")}))
    probs = sim.task_generator.shelf_probabilities
    hottest = int(np.argmax(probs))
    assert hottest == int(closest[0])
