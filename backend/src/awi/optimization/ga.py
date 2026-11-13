"""Permutation genetic algorithm for SKU slotting.

* **Chromosome:** ``assignment[rank] = shelf`` (a permutation of shelf indices).
* **Initial population:** the greedy *closest* slotting plus random permutations,
  so the GA starts from a strong travel-only solution and can trade a little
  travel for much less aisle congestion.
* **Selection:** tournament of size ``tournament``.
* **Crossover:** order crossover OX1 (Davis, 1985), which keeps a slice of one
  parent and fills the rest in the other parent's order. Children are always
  valid permutations.
* **Mutation:** swap two genes, or reverse a segment (inversion).
* **Elitism:** the best ``elite`` individuals survive unchanged, so the best
  fitness never gets worse.

Everything is driven by one seeded generator, so results are reproducible.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from awi.optimization.slotting import SlottingProblem


@dataclass(frozen=True, slots=True)
class GAConfig:
    population: int = 60
    generations: int = 150
    tournament: int = 3
    crossover_rate: float = 0.9
    mutation_rate: float = 0.3
    elite: int = 2

    def __post_init__(self) -> None:
        if self.population < 4 or self.generations < 1:
            raise ValueError("population >= 4 and generations >= 1 required")
        if not 1 <= self.elite < self.population or self.tournament < 1:
            raise ValueError("invalid elite / tournament size")


@dataclass(slots=True)
class GAResult:
    best: NDArray[np.int64]
    best_fitness: float
    history_best: list[float] = field(default_factory=list)
    history_mean: list[float] = field(default_factory=list)
    evaluations: int = 0


def order_crossover(
    a: NDArray[np.int64], b: NDArray[np.int64], rng: np.random.Generator
) -> NDArray[np.int64]:
    """OX1: copy ``a[i:j]``, then fill the remaining slots with ``b``'s genes in
    order, starting after ``j``."""
    n = len(a)
    i, j = sorted(rng.choice(n + 1, size=2, replace=False).tolist())
    child = np.full(n, -1, dtype=np.int64)
    child[i:j] = a[i:j]
    taken = np.zeros(n, dtype=bool)
    taken[a[i:j]] = True
    order = np.concatenate([b[j:], b[:j]])
    fill = order[~taken[order]]
    slots = np.concatenate([np.arange(j, n), np.arange(0, i)])
    child[slots] = fill
    return child


def mutate(x: NDArray[np.int64], rng: np.random.Generator) -> NDArray[np.int64]:
    y = x.copy()
    i, j = sorted(rng.choice(len(y), size=2, replace=False).tolist())
    if rng.random() < 0.5:
        y[i], y[j] = y[j], y[i]
    else:
        y[i : j + 1] = y[i : j + 1][::-1]
    return y


def genetic_slotting(problem: SlottingProblem, cfg: GAConfig, rng: np.random.Generator) -> GAResult:
    pop = [problem.closest_assignment()]
    pop += [problem.random_assignment(rng) for _ in range(cfg.population - 1)]
    fit = np.array([problem.fitness(x) for x in pop])
    result = GAResult(best=pop[int(np.argmin(fit))].copy(), best_fitness=float(fit.min()))
    result.evaluations = len(pop)

    def pick() -> NDArray[np.int64]:
        idx = rng.choice(len(pop), size=cfg.tournament, replace=False)
        return pop[int(idx[np.argmin(fit[idx])])]

    for _ in range(cfg.generations):
        order = np.argsort(fit, kind="stable")
        children = [pop[int(k)].copy() for k in order[: cfg.elite]]
        while len(children) < cfg.population:
            a, b = pick(), pick()
            child = order_crossover(a, b, rng) if rng.random() < cfg.crossover_rate else a.copy()
            if rng.random() < cfg.mutation_rate:
                child = mutate(child, rng)
            children.append(child)
        pop = children
        fit = np.array([problem.fitness(x) for x in pop])
        result.evaluations += len(pop)
        best = int(np.argmin(fit))
        if fit[best] < result.best_fitness:
            result.best, result.best_fitness = pop[best].copy(), float(fit[best])
        result.history_best.append(result.best_fitness)
        result.history_mean.append(float(fit.mean()))
    return result
