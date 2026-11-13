import importlib

import networkx as nx
import numpy as np
import pytest
from fastapi.testclient import TestClient
from ortools.sat.python import cp_model
from scipy.optimize import linear_sum_assignment

import awi
from awi.api.main import app

SUBPACKAGES = [
    "core",
    "planning",
    "assignment",
    "energy",
    "dynamics",
    "optimization",
    "metrics",
    "experiments",
    "api",
]


@pytest.mark.parametrize("name", SUBPACKAGES)
def test_subpackages_import(name: str) -> None:
    importlib.import_module(f"awi.{name}")


def test_health_endpoint() -> None:
    client = TestClient(app)
    resp = client.get("/api/health")
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok", "version": awi.__version__}


def test_scientific_stack_available() -> None:
    rows, cols = linear_sum_assignment(np.array([[4, 1], [2, 3]]))
    assert list(cols[rows.argsort()]) == [1, 0]
    assert nx.shortest_path_length(nx.grid_2d_graph(3, 3), (0, 0), (2, 2)) == 4
    assert cp_model.CpModel() is not None
