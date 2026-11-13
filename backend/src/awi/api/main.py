"""FastAPI entry point."""

from __future__ import annotations

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from awi import __version__


class HealthResponse(BaseModel):
    status: str
    version: str


def create_app() -> FastAPI:
    app = FastAPI(
        title="Autonomous Warehouse Intelligence API",
        version=__version__,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/api/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        return HealthResponse(status="ok", version=__version__)

    return app


app = create_app()


def run() -> None:
    """Console entry point: `uv run awi-api`."""
    import uvicorn  # noqa: PLC0415

    uvicorn.run("awi.api.main:app", host="127.0.0.1", port=8000, reload=True)
