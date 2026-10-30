"""API FastAPI del editor URDF (solo preview; no publica /robot_description ni toca ROS)."""
from __future__ import annotations

import json

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from . import generator
from .settings import MAX_BODY_BYTES, Settings

MESH_SUFFIXES = {".stl", ".dae"}


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings()
    app = FastAPI(title="Base@Home URDF Editor (preview)", docs_url=None, redoc_url=None)

    async def read_json(request: Request) -> dict:
        body = await request.body()
        if len(body) > MAX_BODY_BYTES:
            raise HTTPException(413, "payload demasiado grande")
        try:
            data = json.loads(body or b"{}")
        except ValueError as exc:
            raise HTTPException(400, f"JSON invalido: {exc}") from exc
        if not isinstance(data, dict):
            raise HTTPException(400, "se esperaba un objeto JSON")
        return data

    def validated(data: dict) -> dict[str, float]:
        try:
            return generator.validate_values(settings, data.get("values", {}))
        except generator.ParamError as exc:
            raise HTTPException(422, {"errors": exc.errors}) from exc

    @app.exception_handler(generator.GenerationError)
    async def _gen_error(_: Request, exc: generator.GenerationError):
        return JSONResponse(status_code=500, content={"detail": str(exc)})

    @app.get("/api/health")
    def health():
        return {"ok": True, "mode": "preview-only", "publishes_ros": False}

    @app.get("/api/params")
    def params():
        return {"source": f"{settings.urdf_rel}/{settings.props_rel}",
                "params": generator.list_params(settings)}

    @app.get("/api/model/urdf")
    def model_urdf():
        return generator.preview(settings, {})

    @app.post("/api/model/preview")
    async def model_preview(request: Request):
        return generator.preview(settings, validated(await read_json(request)))

    @app.post("/api/model/export")
    async def model_export(request: Request):
        data = await read_json(request)
        values = validated(data)
        label = data.get("label")
        if label is not None and not isinstance(label, str):
            raise HTTPException(422, {"errors": ["label debe ser string"]})
        try:
            return generator.export_candidate(settings, values, label)
        except generator.ParamError as exc:
            raise HTTPException(422, {"errors": exc.errors}) from exc

    @app.get("/meshes/{package}/{rel_path:path}")
    def mesh(package: str, rel_path: str):
        pkg_dir = settings.package_dir(package)
        if pkg_dir is None:
            raise HTTPException(404, "paquete no permitido")
        path = (pkg_dir / rel_path).resolve()
        if not path.is_relative_to(pkg_dir) or path.suffix.lower() not in MESH_SUFFIXES or not path.is_file():
            raise HTTPException(404, "mesh no encontrado")
        return FileResponse(path)

    @app.get("/")
    def index():
        return FileResponse(settings.static_dir / "index.html")

    app.mount("/static", StaticFiles(directory=settings.static_dir), name="static")
    return app
