from collections.abc import Iterator
from typing import Any

import pytest
from fastapi.testclient import TestClient

from awi.api.main import create_app
from tests.conftest import SMALL_LAYOUT

SMALL = {"num_robots": 4, "layout": SMALL_LAYOUT.model_dump(), "task_rate": 0.15}


@pytest.fixture
def client() -> Iterator[TestClient]:
    with TestClient(create_app(max_sessions=3)) as c:  # runs the lifespan
        yield c


def _create(client: TestClient, **body: Any) -> dict[str, Any]:
    resp = client.post("/api/sessions", json={"config": SMALL, **body})
    assert resp.status_code == 201, resp.text
    data: dict[str, Any] = resp.json()
    return data


def test_health_and_options(client: TestClient) -> None:
    assert client.get("/api/health").json()["status"] == "ok"
    opts = client.get("/api/options").json()
    assert {"cbs", "prioritized", "independent_astar"} <= set(opts["planners"])
    assert {"hungarian", "batching"} <= set(opts["assigners"])
    assert opts["default_config"]["num_robots"] == 10
    assert "properties" in opts["config_schema"]


def test_session_lifecycle(client: TestClient) -> None:
    s = _create(client, planner="prioritized", assigner="hungarian", name="demo")
    sid = s["id"]
    assert s["name"] == "demo" and s["t"] == 0 and not s["running"]
    assert [x["id"] for x in client.get("/api/sessions").json()] == [sid]

    layout = client.get(f"/api/sessions/{sid}/layout").json()
    assert layout["height"] == SMALL_LAYOUT.height and layout["width"] == SMALL_LAYOUT.width
    assert len(layout["cells"]) == layout["height"]
    assert layout["cell_types"]["shelf"] == 1

    frame = client.post(f"/api/sessions/{sid}/step", json={"ticks": 50}).json()
    assert frame["t"] == 50 and len(frame["robots"]) == 4
    robot = frame["robots"][0]
    assert {"id", "pos", "state", "path", "carrying", "battery"} <= set(robot)
    assert frame["tasks"]["generated"] >= frame["tasks"]["delivered"]
    assert set(frame["kpis"]) >= {"throughput_per_1k", "conflicts_total"}

    metrics = client.get(f"/api/sessions/{sid}/metrics").json()
    assert metrics["metrics"]["measured_ticks"] == 50
    assert isinstance(metrics["timeseries"], list)

    reset = client.post(f"/api/sessions/{sid}/control", json={"action": "reset"}).json()
    assert reset["t"] == 0
    assert client.delete(f"/api/sessions/{sid}").status_code == 204
    assert client.get(f"/api/sessions/{sid}").status_code == 404


def test_validation_errors(client: TestClient) -> None:
    bad_planner = client.post("/api/sessions", json={"planner": "teleport"})
    assert bad_planner.status_code == 422 and "unknown planner" in bad_planner.text
    bad_config = client.post("/api/sessions", json={"config": {"num_robots": 0}})
    assert bad_config.status_code == 422
    assert client.post("/api/sessions", json={"config": {"nope": 1}}).status_code == 422
    sid = _create(client)["id"]
    assert client.post(f"/api/sessions/{sid}/step", json={"ticks": 0}).status_code == 422
    assert client.get("/api/sessions/missing/frame").status_code == 404


def test_session_limit_evicts_least_recently_used_idle_session(client: TestClient) -> None:
    first = _create(client)["id"]
    second = _create(client)["id"]
    third = _create(client)["id"]
    client.post(f"/api/sessions/{first}/step", json={"ticks": 1})  # first is now the freshest
    fourth = _create(client)["id"]  # at capacity: evicts the least recently used idle one
    ids = {s["id"] for s in client.get("/api/sessions").json()}
    assert ids == {first, third, fourth}
    assert second not in ids


def test_session_limit_keeps_watched_and_running_sessions(client: TestClient) -> None:
    sessions = [_create(client)["id"] for _ in range(3)]
    for sid in sessions:
        client.post(f"/api/sessions/{sid}/control", json={"action": "start", "speed": 50})
    resp = client.post("/api/sessions", json={"config": SMALL})
    assert resp.status_code == 422 and "at most 3" in resp.text
    for sid in sessions:
        client.post(f"/api/sessions/{sid}/control", json={"action": "pause"})


def test_obstacle_injection(client: TestClient) -> None:
    sid = _create(client)["id"]
    layout = client.get(f"/api/sessions/{sid}/layout").json()
    frame = client.get(f"/api/sessions/{sid}/frame").json()
    robots = {tuple(r["pos"]) for r in frame["robots"]}
    pick = layout["cell_types"]["pick"]
    cell = next(
        [r, c]
        for r, row in enumerate(layout["cells"])
        for c, v in enumerate(row)
        if v == pick and (r, c) not in robots
    )
    drop = layout["drop_stations"][0]
    resp = client.post(
        f"/api/sessions/{sid}/obstacles", json={"cells": [cell, drop, [999, 999]], "duration": 30}
    ).json()
    assert resp["blocked"] == [cell]  # station and out-of-bounds cells are ignored
    assert client.get(f"/api/sessions/{sid}/frame").json()["blocked"] == [cell]


def test_battery_and_dynamics_in_frames(client: TestClient) -> None:
    config = {
        **SMALL,
        "battery": {"enabled": True},
        "dynamics": {"enabled": True, "humans": 2},
    }
    sid = _create(client, config=config)["id"]
    frame = client.post(f"/api/sessions/{sid}/step", json={"ticks": 5}).json()
    assert len(frame["humans"]) == 2
    assert all(0 <= r["battery"] <= 100 for r in frame["robots"])


def test_rest_start_pause(client: TestClient) -> None:
    sid = _create(client)["id"]
    started = client.post(
        f"/api/sessions/{sid}/control", json={"action": "start", "speed": 500}
    ).json()
    assert started["running"] and started["speed"] == 500
    assert client.post(f"/api/sessions/{sid}/step", json={"ticks": 1}).status_code == 409
    paused = client.post(f"/api/sessions/{sid}/control", json={"action": "pause"}).json()
    assert not paused["running"]


def test_websocket_stream_and_commands(client: TestClient) -> None:
    sid = _create(client)["id"]
    with client.websocket_connect(f"/ws/sessions/{sid}") as ws:
        assert ws.receive_json()["type"] == "layout"
        first = ws.receive_json()
        assert first["type"] == "frame" and first["t"] == 0

        ws.send_json({"action": "step", "ticks": 7})
        assert ws.receive_json()["t"] == 7

        ws.send_json({"action": "start", "speed": 400})
        times = [ws.receive_json()["t"] for _ in range(4)]
        assert times == sorted(times) and times[-1] > 7

        ws.send_json({"action": "pause"})
        while ws.receive_json()["running"]:
            pass
        ws.send_json({"action": "bogus"})
        assert ws.receive_json()["type"] == "error"

        ws.send_json({"action": "reset"})
        while (msg := ws.receive_json())["t"] != 0:
            pass
        assert msg["type"] == "frame"


def test_websocket_unknown_session(client: TestClient) -> None:
    with client.websocket_connect("/ws/sessions/nope") as ws:
        msg = ws.receive_json()
        assert msg["type"] == "error"
