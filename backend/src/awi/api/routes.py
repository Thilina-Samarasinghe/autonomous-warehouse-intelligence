"""REST and WebSocket endpoints."""

from __future__ import annotations

import asyncio
import contextlib
from typing import Any

from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect, status
from pydantic import ValidationError

from awi import __version__
from awi.api.optimize import OptimizeRequest, optimize_slotting
from awi.api.schemas import (
    ControlRequest,
    HealthResponse,
    ObstacleRequest,
    ObstacleResponse,
    OptionsResponse,
    SessionCreate,
    SessionStatus,
    StepRequest,
    WsCommand,
)
from awi.api.session import Session, SessionError, SessionManager
from awi.core.config import SimConfig
from awi.experiments.registry import ASSIGNERS, PLANNERS

router = APIRouter()


def _manager(request: Request) -> SessionManager:
    manager: SessionManager = request.app.state.sessions
    return manager


def _session(request: Request, session_id: str) -> Session:
    session = _manager(request).get(session_id)
    if session is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, f"no session {session_id!r}")
    return session


# ------------------------------------------------------------------- meta


@router.get("/api/health", response_model=HealthResponse, tags=["meta"])
def health() -> HealthResponse:
    return HealthResponse(status="ok", version=__version__)


@router.get("/api/options", response_model=OptionsResponse, tags=["meta"])
def options() -> OptionsResponse:
    """Available strategies and the full simulation config schema."""
    return OptionsResponse(
        planners=sorted(PLANNERS),
        assigners=sorted(ASSIGNERS),
        default_config=SimConfig().model_dump(),
        config_schema=SimConfig.model_json_schema(),
    )


# --------------------------------------------------------------- sessions


@router.post(
    "/api/sessions",
    response_model=SessionStatus,
    status_code=status.HTTP_201_CREATED,
    tags=["sessions"],
)
def create_session(body: SessionCreate, request: Request) -> dict[str, Any]:
    try:
        session = _manager(request).create(
            planner=body.planner,
            assigner=body.assigner,
            config=body.config,
            name=body.name,
            speed=body.speed,
        )
    except SessionError as exc:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, str(exc)) from None
    return session.status()


@router.get("/api/sessions", response_model=list[SessionStatus], tags=["sessions"])
def list_sessions(request: Request) -> list[dict[str, Any]]:
    return [s.status() for s in _manager(request).all()]


@router.get("/api/sessions/{session_id}", response_model=SessionStatus, tags=["sessions"])
def get_session(session_id: str, request: Request) -> dict[str, Any]:
    return _session(request, session_id).status()


@router.delete(
    "/api/sessions/{session_id}", status_code=status.HTTP_204_NO_CONTENT, tags=["sessions"]
)
async def delete_session(session_id: str, request: Request) -> None:
    if not await _manager(request).delete(session_id):
        raise HTTPException(status.HTTP_404_NOT_FOUND, f"no session {session_id!r}")


@router.get("/api/sessions/{session_id}/layout", tags=["sessions"])
def get_layout(session_id: str, request: Request) -> dict[str, Any]:
    """Static map: cell types, stations, chargers, parking cells."""
    return _session(request, session_id).layout()


@router.get("/api/sessions/{session_id}/frame", tags=["sessions"])
def get_frame(session_id: str, request: Request) -> dict[str, Any]:
    """Current state: robots (with planned paths), humans, blockages, KPIs."""
    return _session(request, session_id).frame()


@router.get("/api/sessions/{session_id}/metrics", tags=["sessions"])
def get_metrics(session_id: str, request: Request) -> dict[str, Any]:
    """Every KPI of the run so far, plus the windowed time series."""
    return _session(request, session_id).full_metrics()


@router.post("/api/sessions/{session_id}/step", tags=["control"])
def step_session(session_id: str, body: StepRequest, request: Request) -> dict[str, Any]:
    session = _session(request, session_id)
    if session.running:
        raise HTTPException(status.HTTP_409_CONFLICT, "pause the session before stepping")
    session.step(body.ticks)
    return session.frame()


@router.post("/api/sessions/{session_id}/control", response_model=SessionStatus, tags=["control"])
async def control_session(
    session_id: str, body: ControlRequest, request: Request
) -> dict[str, Any]:
    session = _session(request, session_id)
    if body.action == "start":
        await session.start(body.speed)
    elif body.action == "pause":
        await session.pause()
    else:
        await session.pause()
        await asyncio.to_thread(session.reset)
        session.publish(session.frame())
    if body.speed is not None:
        session.speed = body.speed
    return session.status()


@router.post(
    "/api/sessions/{session_id}/obstacles", response_model=ObstacleResponse, tags=["control"]
)
def add_obstacles(session_id: str, body: ObstacleRequest, request: Request) -> ObstacleResponse:
    """Close aisle cells for ``duration`` ticks (stations and robot cells are skipped)."""
    session = _session(request, session_id)
    blocked = session.add_blockage([tuple(c) for c in body.cells], body.duration)  # type: ignore[misc]
    return ObstacleResponse(blocked=blocked)


# ------------------------------------------------------------- optimise


@router.post("/api/optimize/slotting", tags=["optimisation"])
async def optimize(body: OptimizeRequest) -> dict[str, Any]:
    """Run the slotting GA and compare it with random and travel-optimal slotting,
    optionally validated by short simulations (``validate_ticks``)."""
    try:
        return await asyncio.to_thread(optimize_slotting, body)
    except SessionError as exc:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, str(exc)) from None


# -------------------------------------------------------------- websocket


@router.websocket("/ws/sessions/{session_id}")
async def stream_session(websocket: WebSocket, session_id: str) -> None:
    """Live frames. On connect the server sends ``{"type": "layout"}`` and the
    current frame, then a frame per update while running. The client may send
    :class:`~awi.api.schemas.WsCommand` messages (start, pause, reset, step, speed)."""
    manager: SessionManager = websocket.app.state.sessions
    session = manager.get(session_id)
    await websocket.accept()
    if session is None:
        await websocket.send_json({"type": "error", "detail": f"no session {session_id!r}"})
        await websocket.close(code=4404)
        return
    queue = session.subscribe()
    await websocket.send_json({"type": "layout", **session.layout()})
    await websocket.send_json(session.frame())

    async def pump() -> None:
        while True:
            await websocket.send_json(await queue.get())

    sender = asyncio.create_task(pump())
    try:
        while True:
            raw = await websocket.receive_json()
            try:
                cmd = WsCommand.model_validate(raw)
            except ValidationError as exc:
                await websocket.send_json({"type": "error", "detail": exc.errors()})
                continue
            await _apply(session, cmd)
    except WebSocketDisconnect:
        pass
    finally:
        sender.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await sender
        session.unsubscribe(queue)


async def _apply(session: Session, cmd: WsCommand) -> None:
    if cmd.action == "start":
        await session.start(cmd.speed)
    elif cmd.action == "pause":
        await session.pause()
    elif cmd.action == "speed" and cmd.speed is not None:
        session.speed = cmd.speed
        session.publish(session.frame())
    elif cmd.action == "step" and not session.running:
        await asyncio.to_thread(session.step, cmd.ticks)
        session.publish(session.frame())
    elif cmd.action == "reset":
        await session.pause()
        await asyncio.to_thread(session.reset)
        session.publish(session.frame())
