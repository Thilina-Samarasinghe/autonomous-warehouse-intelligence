"""Seeded order stream: Poisson arrivals with Zipf-distributed SKU popularity."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from awi.core.layout import Layout
from awi.core.task import Task


class TaskGenerator:
    """Generates tasks tick by tick.

    * Arrivals per tick ~ Poisson(``rate``).
    * Shelf popularity follows a Zipf law with exponent ``zipf_s``: the k-th most
      popular shelf is picked with probability proportional to ``1 / k**s``
      (``s = 0`` gives uniform demand). Which shelf is "hot" is a random
      permutation drawn from the generator's own stream, unless a planned
      ``assignment`` (rank -> shelf index, see :mod:`awi.optimization.slotting`)
      is given. The permutation is drawn either way, so the arrival times and
      drop stations stay identical across slotting strategies (paired
      comparisons).
    * Drop stations are chosen uniformly.
    """

    def __init__(
        self,
        layout: Layout,
        rng: np.random.Generator,
        rate: float,
        *,
        zipf_s: float = 1.0,
        max_tasks: int | None = None,
        assignment: NDArray[np.int64] | None = None,
    ) -> None:
        if rate <= 0:
            raise ValueError("rate must be positive")
        if not layout.shelves or not layout.drop_stations:
            raise ValueError("layout needs at least one shelf and one drop station")
        self._layout = layout
        self._rng = rng
        self._rate = rate
        self._max_tasks = max_tasks
        self._next_id = 0

        n = len(layout.shelves)
        weights = 1.0 / np.arange(1, n + 1, dtype=np.float64) ** zipf_s
        weights /= weights.sum()
        drawn = rng.permutation(n)
        self.random_assignment: NDArray[np.int64] = drawn.astype(np.int64)
        if assignment is not None:
            if sorted(assignment.tolist()) != list(range(n)):
                raise ValueError("assignment must be a permutation of the shelf indices")
            drawn = assignment
        probs = np.empty(n, dtype=np.float64)
        probs[drawn] = weights
        self._probs: NDArray[np.float64] = probs

    @property
    def shelf_probabilities(self) -> NDArray[np.float64]:
        """Demand probability per shelf, aligned with ``layout.shelves``."""
        return self._probs

    @property
    def generated(self) -> int:
        return self._next_id

    def generate(self, t: int) -> list[Task]:
        k = int(self._rng.poisson(self._rate))
        if self._max_tasks is not None:
            k = min(k, self._max_tasks - self._next_id)
        if k <= 0:
            return []
        shelf_idx = self._rng.choice(len(self._probs), size=k, p=self._probs)
        drop_idx = self._rng.integers(len(self._layout.drop_stations), size=k)
        tasks: list[Task] = []
        for si, di in zip(shelf_idx.tolist(), drop_idx.tolist(), strict=True):
            shelf = self._layout.shelves[si]
            tasks.append(
                Task(
                    id=self._next_id,
                    shelf=shelf,
                    pickup=self._layout.pick_point[shelf],
                    dropoff=self._layout.drop_stations[di],
                    created_t=t,
                )
            )
            self._next_id += 1
        return tasks
