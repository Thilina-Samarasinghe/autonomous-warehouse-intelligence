"""FastAPI application entry point.

Run with ``uv run awi-api`` (development, auto-reload) and open
http://127.0.0.1:8000/docs for the interactive API reference.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from awi import __version__
from awi.api.routes import router
from awi.api.schemas import HealthResponse
from awi.api.session import SessionManager

__all__ = ["HealthResponse", "app", "create_app", "run"]


def create_app(max_sessions: int = 8) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        app.state.sessions = SessionManager(max_sessions=max_sessions)
        yield
        await app.state.sessions.close()

    app = FastAPI(
        title="Autonomous Warehouse Intelligence API",
        version=__version__,
        description=(
            "Control live warehouse fleet simulations and stream their state. "
            "Create a session, then connect to `/ws/sessions/{id}` for live frames."
        ),
        lifespan=lifespan,
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(router)
    return app


app = create_app()


def run() -> None:
    """Console entry point: ``uv run awi-api``."""
    import uvicorn  # noqa: PLC0415

    uvicorn.run("awi.api.main:app", host="127.0.0.1", port=8000, reload=True)
