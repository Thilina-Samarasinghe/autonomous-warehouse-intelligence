"""Typed simulation configuration (loadable from YAML scenario files)."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field

from awi.core.layout import LayoutConfig
from awi.energy.battery import BatteryConfig


class SimConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    seed: int = Field(0, ge=0)
    num_robots: int = Field(10, ge=1, le=200)
    layout: LayoutConfig = Field(default_factory=LayoutConfig)

    task_rate: float = Field(0.15, gt=0, description="Expected new tasks per tick")
    zipf_s: float = Field(1.0, ge=0, description="SKU popularity skew (0 = uniform)")
    max_tasks: int | None = Field(None, ge=0, description="Stop generating after N tasks")

    pick_ticks: int = Field(2, ge=0, description="Ticks spent picking at a shelf")
    drop_ticks: int = Field(1, ge=0, description="Ticks spent dropping at a station")
    stuck_replan_ticks: int = Field(
        3, ge=1, description="Replan around other robots after being blocked this long"
    )
    capacity: int = Field(1, ge=1, le=10, description="Items a robot can carry per trip")
    park_idle: bool = Field(
        True, description="Idle robots on a station/pick point drive to a parking cell"
    )
    battery: BatteryConfig = Field(default_factory=BatteryConfig)
    validate_paths: bool = Field(True, description="Check planner output every tick")

    @classmethod
    def from_yaml(cls, path: str | Path) -> SimConfig:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.model_validate(data)
