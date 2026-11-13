"""Request / response models of the HTTP and WebSocket API."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    status: str
    version: str


class SessionCreate(BaseModel):
    planner: str = Field("prioritized", description="Path planner name (see /api/options)")
    assigner: str = Field("hungarian", description="Task assigner name (see /api/options)")
    config: dict[str, Any] = Field(
        default_factory=dict, description="SimConfig overrides, deep-merged into the defaults"
    )
    name: str | None = Field(None, max_length=60)
    speed: float = Field(10.0, gt=0, le=2000, description="Ticks per second when running")


class SessionStatus(BaseModel):
    id: str
    name: str
    planner: str
    assigner: str
    t: int
    running: bool
    speed: float
    num_robots: int
    config: dict[str, Any]


class StepRequest(BaseModel):
    ticks: int = Field(1, ge=1, le=10_000)


class ControlRequest(BaseModel):
    action: Literal["start", "pause", "reset"]
    speed: float | None = Field(None, gt=0, le=2000)


class ObstacleRequest(BaseModel):
    cells: list[tuple[int, int]] = Field(min_length=1, max_length=200)
    duration: int = Field(200, ge=1, le=100_000)


class ObstacleResponse(BaseModel):
    blocked: list[tuple[int, int]]


class OptionsResponse(BaseModel):
    planners: list[str]
    assigners: list[str]
    default_config: dict[str, Any]
    config_schema: dict[str, Any]


class WsCommand(BaseModel):
    """Messages a WebSocket client may send."""

    action: Literal["start", "pause", "reset", "step", "speed"]
    ticks: int = Field(1, ge=1, le=10_000)
    speed: float | None = Field(None, gt=0, le=2000)
