"""Tests del editor URDF (HOME-04). Requieren `xacro` y `check_urdf` de ROS 2 Jazzy."""
from __future__ import annotations

import hashlib
import re
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from backend import generator, xacro_props
from backend.__main__ import parse_args
from backend.app import create_app
from backend.settings import DEFAULT_HOST, TOOL_DIR, Settings

BASE = Settings()
PROPS_REL = "src/hoverboard_base/urdf/xacro/properties.xacro"


def tree_digest(root: Path) -> dict[str, str]:
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob("*")) if p.is_file()}


@pytest.fixture
def settings(tmp_path):
    return replace(BASE, candidates_dir=tmp_path / "candidates")


@pytest.fixture
def client(settings):
    return TestClient(create_app(settings))


@pytest.fixture(autouse=True)
def repo_untouched():
    """Ningun test puede modificar el arbol URDF del repo."""
    before = tree_digest(BASE.urdf_dir)
    yield
    assert tree_digest(BASE.urdf_dir) == before


def joint_signature(urdf_text: str):
    root = ET.fromstring(urdf_text)
    links = sorted(l.get("name") for l in root.findall("link"))
    joints = sorted(
        (j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"),
         ET.tostring(j.find("origin")) if j.find("origin") is not None else b"")
        for j in root.findall("joint"))
    return links, joints


# (a) ---------------------------------------------------------------------------
def test_preview_without_changes_matches_original_xacro(client, tmp_path):
    out = tmp_path / "orig.urdf"
    generator.run_xacro(BASE, BASE.urdf_dir, out)
    reference = out.read_text()

    res = client.get("/api/model/urdf")
    assert res.status_code == 200
    data = res.json()
    assert data["check_urdf"]["ok"], data["check_urdf"]["output"]
    assert data["diff"] == ""
    assert joint_signature(data["urdf"]) == joint_signature(reference)
    # Ademas, el URDF completo es identico salvo comentarios (ruta del archivo fuente).
    strip = lambda t: re.sub(r"<!--.*?-->", "", t, flags=re.S)
    assert strip(data["urdf"]) == strip(reference)

    res2 = client.post("/api/model/preview", json={"values": {}})
    assert res2.status_code == 200
    assert joint_signature(res2.json()["urdf"]) == joint_signature(reference)


def test_preview_applies_change_only_to_copy(client):
    res = client.post("/api/model/preview", json={"values": {"chassis_length": 0.55}})
    assert res.status_code == 200
    data = res.json()
    root = ET.fromstring(data["urdf"])
    sizes = [b.get("size") for b in root.iter("box")]
    assert any(s.startswith("0.55 ") for s in sizes), sizes
    assert '+  <xacro:property name="chassis_length" value="0.55"/>' in data["diff"]
    assert data["check_urdf"]["ok"]


# (b) ---------------------------------------------------------------------------
@pytest.mark.parametrize("value", [5.0, -0.1, 0.0, 1e9])
def test_out_of_range_rejected(client, value):
    res = client.post("/api/model/preview", json={"values": {"chassis_length": value}})
    assert res.status_code == 422
    assert "fuera de rango" in " ".join(res.json()["detail"]["errors"])


@pytest.mark.parametrize("value", ["0.5", "${column_width * 2}", "__import__('os')", True, None, [0.5]])
def test_non_numeric_rejected(client, value):
    res = client.post("/api/model/preview", json={"values": {"chassis_length": value}})
    assert res.status_code == 422
    assert "numerico" in " ".join(res.json()["detail"]["errors"])


@pytest.mark.parametrize("literal", ["NaN", "Infinity", "-Infinity"])
def test_non_finite_rejected(client, literal):
    body = '{"values": {"chassis_length": %s}}' % literal
    res = client.post("/api/model/preview", content=body, headers={"Content-Type": "application/json"})
    assert res.status_code == 422


def test_payload_too_large_rejected(client):
    res = client.post("/api/model/preview", content=b"{" + b" " * 20000 + b"}",
                      headers={"Content-Type": "application/json"})
    assert res.status_code == 413


# (c) ---------------------------------------------------------------------------
@pytest.mark.parametrize("name", ["ground_clearance", "nonexistent", "../x", "head_base_x", "prism_offset_z"])
def test_not_whitelisted_rejected(client, name):
    res = client.post("/api/model/preview", json={"values": {name: 0.1}})
    assert res.status_code == 422
    assert "whitelist" in " ".join(res.json()["detail"]["errors"])


@pytest.mark.parametrize("name", ["track_width", "wheel_radius", "laser_offset_x", "laser_offset_y",
                                  "laser_offset_z", "torso_lift_top_z"])
def test_protected_rejected(client, name):
    res = client.post("/api/model/preview", json={"values": {name: 0.3}})
    assert res.status_code == 422
    assert "protegida" in " ".join(res.json()["detail"]["errors"])


def test_protected_never_in_whitelist():
    schema = BASE.load_schema()
    assert not set(schema["editable"]) & set(schema["protected"])
    for name in ("track_width", "wheel_radius", "laser_offset_x", "laser_offset_y", "laser_offset_z"):
        assert name in schema["protected"]


def test_params_listing(client):
    data = client.get("/api/params").json()
    params = {p["name"]: p for p in data["params"]}
    assert len(params) == 40
    derived = {n for n, p in params.items() if p["kind"] == "derived"}
    assert {"head_base_x", "prism_offset_x", "prism_offset_z"} <= derived
    assert not any(params[n]["editable"] for n in derived)
    assert params["head_base_x"]["raw"] == "${column_width / 2.0 + 0.04}"
    assert params["wheel_radius"]["protected"] and not params["wheel_radius"]["editable"]
    assert params["chassis_length"]["editable"] and params["chassis_length"]["unit"] == "m"


# (d) ---------------------------------------------------------------------------
@pytest.mark.parametrize("label", ["../../evil", "..", "/etc/passwd", "a/b", "x" * 41, "név", "a\x00b", ""])
def test_export_path_traversal_rejected(client, settings, label):
    repo_before = {p for p in BASE.repo_root.rglob("*evil*")}
    res = client.post("/api/model/export", json={"values": {"chassis_length": 0.5}, "label": label})
    assert res.status_code == 422
    assert not settings.candidates_dir.exists() or not any(settings.candidates_dir.iterdir())
    assert {p for p in BASE.repo_root.rglob("*evil*")} == repo_before


def test_export_rejects_symlinked_candidates_dir(tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    link = tmp_path / "candidates"
    link.symlink_to(outside, target_is_directory=True)
    client = TestClient(create_app(replace(BASE, candidates_dir=link)))
    res = client.post("/api/model/export", json={"values": {"chassis_length": 0.5}})
    assert res.status_code == 500
    assert list(outside.iterdir()) == []


def test_export_writes_only_inside_candidates(client, settings):
    res = client.post("/api/model/export", json={"values": {"column_height": 1.4}, "label": "test_1"})
    assert res.status_code == 200, res.text
    files = sorted(settings.candidates_dir.iterdir())
    assert [f.suffix for f in files] == [".diff", ".xacro"]
    for f in files:
        assert f.resolve().parent == settings.candidates_dir.resolve()
        assert re.match(r"properties_\d{8}T\d{12}Z_test_1\.(xacro|diff)$", f.name)
    cand = files[1].read_text()
    assert xacro_props.parse_props(cand)["column_height"].raw == "1.4"


def test_mesh_endpoint_confined(client):
    assert client.get("/meshes/piper_description/meshes/base_link.STL").status_code == 200
    assert client.get("/meshes/piper_description/%2e%2e/%2e%2e/hoverboard_base/package.xml").status_code == 404
    assert client.get("/meshes/piper_description/package.xml").status_code == 404
    assert client.get("/meshes/hoverboard_base/urdf/robot.urdf").status_code == 404


# (e) ---------------------------------------------------------------------------
def test_derived_properties_preserved_byte_for_byte():
    original = BASE.props_path.read_text()
    before = xacro_props.parse_props(original)
    edited = xacro_props.apply_values(original, {"column_width": 0.09, "prism_size_x": 0.3, "head_base_z": 0.15})
    after = xacro_props.parse_props(edited)
    for name, prop in before.items():
        if prop.kind == "derived":
            assert after[name].raw == prop.raw
            s, e = prop.value_span
            s2, e2 = after[name].value_span
            assert edited[s2:e2].encode() == original[s:e].encode()
    # Solo cambian exactamente 3 lineas; el resto (comentarios incluidos) es identico.
    changed = [(a, b) for a, b in zip(original.splitlines(), edited.splitlines()) if a != b]
    assert len(changed) == 3
    assert len(original.splitlines()) == len(edited.splitlines())


def test_derived_cannot_be_overwritten_even_internally():
    with pytest.raises(xacro_props.XacroPropsError):
        xacro_props.apply_values(BASE.props_path.read_text(), {"head_base_x": 0.1})


def test_derived_recomputed_by_xacro_in_preview(client):
    # prism_offset_x = column_width/2 + prism_size_x/2 -> debe reflejarse en el URDF.
    data = client.post("/api/model/preview", json={"values": {"column_width": 0.1, "prism_size_x": 0.3}}).json()
    origins = [j.find("origin").get("xyz") for j in ET.fromstring(data["urdf"]).findall("joint")
               if j.find("origin") is not None]
    assert any(o.startswith("0.2 ") for o in origins), origins  # 0.05 + 0.15


# (f) ---------------------------------------------------------------------------
def test_default_host_is_loopback():
    assert DEFAULT_HOST == "127.0.0.1"
    assert parse_args([]).port == 8765
    with pytest.raises(SystemExit):
        parse_args(["--host", "0.0.0.0"])


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _listening_addrs(port: int) -> set[str]:
    addrs = set()
    for table in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            lines = Path(table).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            local, state = line.split()[1], line.split()[3]
            ip_hex, port_hex = local.split(":")
            if state == "0A" and int(port_hex, 16) == port:
                addrs.add(ip_hex)
    return addrs


@pytest.mark.skipif(not Path("/proc/net/tcp").exists(), reason="requiere /proc (Linux)")
def test_server_binds_only_loopback():
    port = _free_port()
    proc = subprocess.Popen([sys.executable, "-m", "backend", "--port", str(port)], cwd=TOOL_DIR,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.time() + 15
        while time.time() < deadline and not _listening_addrs(port):
            time.sleep(0.2)
        addrs = _listening_addrs(port)
        assert addrs == {"0100007F"}, addrs  # 127.0.0.1 en little-endian hex
        with socket.create_connection(("127.0.0.1", port), timeout=2):
            pass
    finally:
        proc.terminate()
        out, _ = proc.communicate(timeout=10)
    assert f"http://127.0.0.1:{port}" in out
