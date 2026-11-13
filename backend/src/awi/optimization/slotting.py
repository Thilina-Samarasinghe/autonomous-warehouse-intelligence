"""SKU slotting: which shelf holds which product.

Demand follows a Zipf law over popularity ranks: rank 0 is the hottest SKU. A
*slotting* is a permutation ``assignment`` with ``assignment[rank] = shelf index``
(indices into ``layout.shelves``).

Two forces pull in opposite directions:

* **Travel**: hot SKUs close to the packing stations means short trips.
* **Congestion**: hot SKUs bunched in one pick aisle means robots queue in that
  aisle. The phase 3-4 experiments showed this is exactly what makes fleets
  gridlock.

:class:`SlottingProblem` scores a slotting on both, from the exact Dijkstra
distances:

* ``travel``: demand-weighted mean distance from the pick point to the drop
  stations;
* ``concentration``: normalised Herfindahl index of demand per pick aisle,
  ``A x sum(load_a^2)``. It is 1 when demand is spread perfectly evenly over the
  ``A`` aisles and ``A`` when it sits in a single aisle;
* ``fitness = travel x (1 + congestion_weight x (concentration - 1))``, lower is
  better.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, ConfigDict, Field

from awi.core.grid import CellType
from awi.core.layout import Layout
from awi.core.types import Cell
from awi.planning.dijkstra import DistanceMaps


class SlottingConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    strategy: Literal["random", "closest", "ga"] = Field(
        "random",
        description="random: unplanned (default) · closest: hottest SKUs nearest the "
        "stations · ga: genetic algorithm balancing travel and aisle congestion",
    )
    congestion_weight: float = Field(0.5, ge=0, le=10)
    generations: int = Field(150, ge=1, le=5000)
    population: int = Field(60, ge=4, le=1000)


def zipf_weights(n: int, s: float) -> NDArray[np.float64]:
    """Demand share per popularity rank (rank 0 = hottest), summing to 1."""
    w = 1.0 / np.arange(1, n + 1, dtype=np.float64) ** s
    return w / w.sum()


def pick_aisles(layout: Layout) -> tuple[NDArray[np.int64], int]:
    """Aisle id per shelf: connected runs of pick cells (4-neighbour) form an aisle."""
    grid = layout.grid
    picks = set(grid.cells_of_type(CellType.PICK))
    aisle_of: dict[Cell, int] = {}
    n_aisles = 0
    for start in sorted(picks):
        if start in aisle_of:
            continue
        queue = deque([start])
        aisle_of[start] = n_aisles
        while queue:
            cur = queue.popleft()
            for nb in grid.neighbors(cur):
                if nb in picks and nb not in aisle_of:
                    aisle_of[nb] = n_aisles
                    queue.append(nb)
        n_aisles += 1
    ids = np.array([aisle_of[layout.pick_point[s]] for s in layout.shelves], dtype=np.int64)
    return ids, n_aisles


@dataclass(frozen=True, slots=True)
class Score:
    fitness: float
    travel: float
    concentration: float


class SlottingProblem:
    def __init__(
        self,
        layout: Layout,
        distances: DistanceMaps,
        zipf_s: float = 1.0,
        congestion_weight: float = 0.5,
    ) -> None:
        self.layout = layout
        self.n = len(layout.shelves)
        self.weights = zipf_weights(self.n, zipf_s)
        self.congestion_weight = congestion_weight
        # Mean distance from each shelf's pick point to the drop stations.
        self.shelf_cost = np.array(
            [
                np.mean([distances.distance(layout.pick_point[s], d) for d in layout.drop_stations])
                for s in layout.shelves
            ],
            dtype=np.float64,
        )
        self.aisle, self.n_aisles = pick_aisles(layout)

    # ---------------------------------------------------------------- scoring
    def score(self, assignment: NDArray[np.int64]) -> Score:
        travel = float(self.weights @ self.shelf_cost[assignment])
        loads = np.bincount(self.aisle[assignment], weights=self.weights, minlength=self.n_aisles)
        concentration = float(self.n_aisles * np.sum(loads**2))
        fitness = travel * (1.0 + self.congestion_weight * (concentration - 1.0))
        return Score(fitness, travel, concentration)

    def fitness(self, assignment: NDArray[np.int64]) -> float:
        return self.score(assignment).fitness

    # -------------------------------------------------------------- baselines
    def random_assignment(self, rng: np.random.Generator) -> NDArray[np.int64]:
        return rng.permutation(self.n).astype(np.int64)

    def closest_assignment(self) -> NDArray[np.int64]:
        """Hottest SKU on the cheapest shelf, and so on (optimal for travel alone,
        by the rearrangement inequality)."""
        return np.argsort(self.shelf_cost, kind="stable").astype(np.int64)

    def shelf_probabilities(self, assignment: NDArray[np.int64]) -> NDArray[np.float64]:
        """Demand per shelf (aligned with ``layout.shelves``) for a slotting."""
        probs = np.empty(self.n, dtype=np.float64)
        probs[assignment] = self.weights
        return probs
