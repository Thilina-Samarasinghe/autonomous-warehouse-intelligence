"""Deterministic, independent random streams derived from one experiment seed.

Each subsystem (task arrivals, robot spawning, obstacles, GA...) gets its own named
stream. Adding draws to one subsystem never shifts the random numbers of another,
which keeps comparisons between strategies fair: with the same seed, independent and
coordinated planners see exactly the same task stream.
"""

from __future__ import annotations

import zlib

import numpy as np


def make_rng(seed: int, stream: str) -> np.random.Generator:
    key = zlib.crc32(stream.encode("utf-8"))
    return np.random.default_rng(np.random.SeedSequence(entropy=seed, spawn_key=(key,)))
