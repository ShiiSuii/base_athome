"""Arranque: `.venv/bin/python -m backend [--port 8765]`. Siempre escucha en 127.0.0.1."""
from __future__ import annotations

import argparse

import uvicorn

from .app import create_app
from .settings import DEFAULT_HOST, DEFAULT_PORT


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Base@Home URDF editor (preview local)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("--port debe estar en 1024..65535")
    return args


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    # Host fijo: no se ofrece opcion para exponer el editor en la red.
    uvicorn.run(create_app(), host=DEFAULT_HOST, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
