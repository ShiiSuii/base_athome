#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Studio Web de Edición de Mapas PGM y Zonas Semánticas
# Herramientas para:
#  - Limpieza de obstáculos fantasma (goma/rectángulo libre)
#  - Dibujo de paredes y obstáculos faltantes (lápiz/línea recta)
#  - Catálogo de Opciones y Plantillas (Mesas 76cm, Sofá point_couch, Sillas, WPs)
#  - Delimitación y solidificación de mesas en el mapa PGM para Nav2
#  - Zonas semánticas y áreas prohibidas (Keep-out)
# ==============================================================================

import argparse
import base64
import datetime
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Dict, Any, List, Optional

import cv2
import numpy as np
import yaml

DEFAULT_MAP_YAML = "/home/athome/mapa_base_athome.yaml"
DEFAULT_SEMANTIC_YAML = "/home/athome/base_athome_ws/src/base_athome_navigation/config/semantic_map.yaml"
DEFAULT_WAYPOINTS_YAML = "/home/athome/base_athome_ws/src/base_athome_navigation/config/waypoints.yaml"
BACKUP_DIR = "/home/athome/mapa_backups"


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Servidor HTTP multihilo para atender peticiones web concurrentes sin bloqueo."""
    daemon_threads = True
    allow_reuse_address = True


def reload_nav_map(map_yaml_path: str, run=subprocess.run) -> Dict[str, Any]:
    """Hace que el map_server en marcha use el mapa guardado: solo lee el archivo al arrancar.

    Devuelve {"state": "reloaded" | "nav_off" | "other_map" | "failed", "message": texto para el usuario}.
    """
    wanted = os.path.realpath(map_yaml_path)

    def ros2(args, timeout):
        try:
            return run(["ros2"] + args, capture_output=True, text=True, timeout=timeout).stdout or ""
        except (OSError, subprocess.SubprocessError):
            return ""

    found = re.search(r"String value is: (.+)", ros2(["param", "get", "/map_server", "yaml_filename"], 8))
    if not found:
        return {"state": "nav_off", "message": "Nav2 no está corriendo: leerá este mapa al arrancar."}
    current = found.group(1).strip()
    if os.path.realpath(current) != wanted:
        return {"state": "other_map", "message": f"Nav2 tiene abierto otro mapa: {current}"}
    out = ros2(["service", "call", "/map_server/load_map", "nav2_msgs/srv/LoadMap", f"{{map_url: '{wanted}'}}"], 20)
    if "result=0" in out:
        return {"state": "reloaded", "message": "Nav2 ya usa este mapa."}
    return {"state": "failed", "message": "Nav2 no pudo recargar el mapa: reiniciá navegacion."}


class SemanticMapManager:
    """Gestiona el mapa PGM físico, las transformaciones métricas y los datos semánticos."""

    def __init__(self, map_yaml_path: str = DEFAULT_MAP_YAML, semantic_yaml_path: str = DEFAULT_SEMANTIC_YAML):
        self.map_yaml_path = os.path.abspath(map_yaml_path)
        self.semantic_yaml_path = os.path.abspath(semantic_yaml_path)
        self.waypoints_yaml_path = DEFAULT_WAYPOINTS_YAML

        self.map_info = {
            "resolution": 0.05,
            "origin": [-6.192, -9.153, 0.0],
            "width": 324,
            "height": 582,
            "image_path": "/home/athome/mapa_base_athome.pgm",
        }
        self.raw_png_bytes = b""
        self.zones: Dict[str, Any] = {}
        self.furniture: Dict[str, Any] = {}
        self.keepout_zones: Dict[str, Any] = {}
        self.waypoints: Dict[str, Any] = {}
        self.seats: Dict[str, Any] = {}
        self.objects: Dict[str, Any] = {}

        self.load_map()
        self.load_semantic_data()

    def load_map(self):
        """Carga el mapa PGM y sus metadatos YAML."""
        if not os.path.exists(self.map_yaml_path):
            print(f"⚠ Mapa YAML no encontrado en: {self.map_yaml_path}")
            return

        with open(self.map_yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        self.map_info["resolution"] = float(data.get("resolution", 0.05))
        origin_val = data.get("origin", [-6.192, -9.153, 0.0])
        self.map_info["origin"] = [float(x) for x in origin_val]

        img_rel = data.get("image", "mapa_base_athome.pgm")
        img_abs = os.path.join(os.path.dirname(self.map_yaml_path), img_rel)
        if not os.path.exists(img_abs):
            img_abs = os.path.expanduser(f"~/{img_rel}")

        if os.path.exists(img_abs):
            self.map_info["image_path"] = img_abs
            gray = cv2.imread(img_abs, cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                self.map_info["height"], self.map_info["width"] = gray.shape[:2]
                ret, buf = cv2.imencode(".png", gray)
                if ret:
                    self.raw_png_bytes = buf.tobytes()
                    print(f"✔ Mapa PGM cargado: {self.map_info['width']}x{self.map_info['height']} px | Res: {self.map_info['resolution']} m/px | Origen: {self.map_info['origin']}")
        else:
            print(f"⚠ Imagen PGM no encontrada en: {img_abs}")

    def load_semantic_data(self):
        """Carga zonas, mesas, áreas prohibidas, waypoints y asientos guardados por el usuario."""
        if os.path.exists(self.semantic_yaml_path):
            try:
                with open(self.semantic_yaml_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                self.zones = data.get("zones", {}) or {}
                self.furniture = data.get("furniture", {}) or {}
                self.keepout_zones = data.get("keepout_zones", {}) or {}
                self.waypoints = data.get("waypoints", {}) or {}
                self.seats = data.get("seats", {}) or {}
                self.objects = data.get("objects", {}) or {}
                print(f"✔ Datos semánticos del usuario cargados: {len(self.zones)} zonas, {len(self.furniture)} mesas, {len(self.waypoints)} waypoints, {len(self.seats)} asientos, {len(self.objects)} objetos.")
                return
            except Exception as e:
                print(f"⚠ Error cargando semantic_map.yaml: {e}")

        # Iniciar limpio por defecto: NO precargar elementos intrínsecos ficticios en el mapa del usuario
        self.zones = {}
        self.furniture = {}
        self.keepout_zones = {}
        self.waypoints = {}
        self.seats = {}
        self.objects = {}

    def save_map_image(self, base64_png_data: str) -> bool:
        """Guarda la imagen del mapa PGM editada en disco, creando respaldos de seguridad."""
        try:
            if "," in base64_png_data:
                base64_png_data = base64_png_data.split(",", 1)[1]
            img_bytes = base64.b64decode(base64_png_data)
            nparr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print("❌ [Editor] Error al decodificar la imagen enviada desde el navegador.")
                return False

            result_pgm = np.full_like(img, 205, dtype=np.uint8)
            result_pgm[img > 220] = 254
            result_pgm[img < 40] = 0

            pgm_path = self.map_info.get("image_path") or "/home/athome/mapa_base_athome.pgm"
            os.makedirs(os.path.dirname(os.path.abspath(pgm_path)), exist_ok=True)
            os.makedirs(BACKUP_DIR, exist_ok=True)

            orig_backup = pgm_path + ".orig"
            if os.path.exists(pgm_path) and not os.path.exists(orig_backup):
                shutil.copy2(pgm_path, orig_backup)
                print(f"✔ [Editor] Respaldo original permanente creado en: {orig_backup}")

            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            ts_backup = os.path.join(BACKUP_DIR, f"mapa_base_athome_{ts}.pgm")
            if os.path.exists(pgm_path):
                shutil.copy2(pgm_path, ts_backup)

            cv2.imwrite(pgm_path, result_pgm)
            print(f"✔ [Editor] Mapa PGM guardado exitosamente en: {pgm_path} (Respaldo: {ts_backup})")

            self.load_map()
            return True
        except Exception as e:
            print(f"❌ Error al guardar imagen PGM: {e}")
            return False

    def restore_original_map(self) -> bool:
        """Restaura el mapa PGM a su versión original (.orig)."""
        pgm_path = self.map_info.get("image_path") or "/home/athome/mapa_base_athome.pgm"
        orig_backup = pgm_path + ".orig"
        if os.path.exists(orig_backup):
            shutil.copy2(orig_backup, pgm_path)
            self.load_map()
            print("✔ [Editor] Mapa PGM restaurado exitosamente a la versión original.")
            return True
        print("⚠ [Editor] No se encontró respaldo original (.orig).")
        return False

    def save_data(self, new_data: dict) -> bool:
        """Persiste zonas, mesas, áreas prohibidas, waypoints y asientos en YAML."""
        try:
            self.zones = new_data.get("zones", self.zones)
            self.furniture = new_data.get("furniture", self.furniture)
            self.keepout_zones = new_data.get("keepout_zones", self.keepout_zones)
            self.waypoints = new_data.get("waypoints", self.waypoints)
            self.seats = new_data.get("seats", self.seats)
            self.objects = new_data.get("objects", self.objects)

            sem_payload = {
                "map_info": {
                    "yaml_file": self.map_yaml_path,
                    "image_file": self.map_info["image_path"],
                    "resolution": self.map_info["resolution"],
                    "origin": self.map_info["origin"],
                    "width": self.map_info["width"],
                    "height": self.map_info["height"],
                },
                "zones": self.zones,
                "furniture": self.furniture,
                "keepout_zones": self.keepout_zones,
                "waypoints": self.waypoints,
                "seats": self.seats,
                "objects": self.objects,
            }

            os.makedirs(os.path.dirname(self.semantic_yaml_path), exist_ok=True)
            with open(self.semantic_yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(sem_payload, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

            wp_dict = {}
            for name, wp in self.waypoints.items():
                yaw_deg = float(wp.get("yaw_deg", 0.0))
                yaw_rad = math.radians(yaw_deg)
                qz = math.sin(yaw_rad / 2.0)
                qw = math.cos(yaw_rad / 2.0)
                wp_dict[name] = {
                    "name": name,
                    "x": round(float(wp.get("x", 0.0)), 4),
                    "y": round(float(wp.get("y", 0.0)), 4),
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": round(qz, 4),
                    "qw": round(qw, 4),
                    "yaw_deg": round(yaw_deg, 1),
                    "timestamp": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                }

            wp_path = self.waypoints_yaml_path
            os.makedirs(os.path.dirname(wp_path), exist_ok=True)
            with open(wp_path, "w", encoding="utf-8") as f:
                yaml.dump({"waypoints": wp_dict}, f, default_flow_style=False, sort_keys=False)

            user_wp = os.path.expanduser("~/waypoints_base_athome.yaml")
            with open(user_wp, "w", encoding="utf-8") as f:
                yaml.dump({"waypoints": wp_dict}, f, default_flow_style=False, sort_keys=False)

            print(f"✔ [Editor] Datos semánticos guardados: {len(self.zones)} zonas, {len(self.furniture)} mesas, {len(self.waypoints)} waypoints, {len(self.seats)} asientos, {len(self.objects)} objetos.")
            return True
        except Exception as e:
            print(f"❌ Error guardando datos semánticos: {e}")
            return False

HTML_PAGE = r'''<!DOCTYPE html>
<html lang="es" data-theme="dark">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Base@Home - Studio de Edición de Mapas PGM y Zonas Semánticas</title>
<style>
  :root, [data-theme="dark"] {
    --bg-canvas: #090c15;
    --bg-canvas-pattern: #0e1422;
    --bg-sidebar: #111622;
    --bg-header: #151c2c;
    --bg-surface: #171f30;
    --bg-surface-elevated: #1e283d;
    --bg-surface-active: #243048;
    --bg-input: #0c101a;
    --border-subtle: rgba(255, 255, 255, 0.07);
    --border-default: #232d42;
    --border-strong: #33415e;
    --border-focused: #38bdf8;
    --text-primary: #f1f5f9;
    --text-secondary: #94a3b8;
    --text-muted: #64748b;
    --accent-cyan: #38bdf8;
    --accent-cyan-subtle: rgba(56, 189, 248, 0.15);
    --accent-cyan-glow: rgba(56, 189, 248, 0.25);
    --accent-green: #10b981;
    --accent-green-subtle: rgba(16, 185, 129, 0.15);
    --accent-orange: #f59e0b;
    --accent-orange-subtle: rgba(245, 158, 11, 0.15);
    --accent-red: #f43f5e;
    --accent-red-subtle: rgba(244, 63, 94, 0.15);
    --accent-purple: #a855f7;
    --accent-purple-subtle: rgba(168, 85, 247, 0.15);
    --badge-bg: rgba(56, 189, 248, 0.12);
    --badge-border: rgba(56, 189, 248, 0.28);
    --shadow-sm: 0 1px 3px rgba(0, 0, 0, 0.3);
    --shadow-md: 0 4px 12px rgba(0, 0, 0, 0.4);
    --shadow-lg: 0 12px 32px rgba(0, 0, 0, 0.6);
    --hud-bg: rgba(17, 22, 34, 0.88);
    --hud-border: rgba(255, 255, 255, 0.12);
    --hud-text: #f1f5f9;
    --map-grid-color: rgba(255, 255, 255, 0.035);
  }

  [data-theme="light"] {
    --bg-canvas: #e2e8f0;
    --bg-canvas-pattern: #cbd5e1;
    --bg-sidebar: #ffffff;
    --bg-header: #f8fafc;
    --bg-surface: #f8fafc;
    --bg-surface-elevated: #f1f5f9;
    --bg-surface-active: #e2e8f0;
    --bg-input: #ffffff;
    --border-subtle: rgba(0, 0, 0, 0.06);
    --border-default: #e2e8f0;
    --border-strong: #cbd5e1;
    --border-focused: #0284c7;
    --text-primary: #0f172a;
    --text-secondary: #475569;
    --text-muted: #94a3b8;
    --accent-cyan: #0284c7;
    --accent-cyan-subtle: rgba(2, 132, 199, 0.12);
    --accent-cyan-glow: rgba(2, 132, 199, 0.2);
    --accent-green: #059669;
    --accent-green-subtle: rgba(5, 150, 105, 0.12);
    --accent-orange: #d97706;
    --accent-orange-subtle: rgba(217, 119, 6, 0.12);
    --accent-red: #e11d48;
    --accent-red-subtle: rgba(225, 29, 72, 0.12);
    --accent-purple: #7c3aed;
    --accent-purple-subtle: rgba(124, 58, 237, 0.12);
    --badge-bg: rgba(2, 132, 199, 0.1);
    --badge-border: rgba(2, 132, 199, 0.25);
    --shadow-sm: 0 1px 3px rgba(0, 0, 0, 0.05);
    --shadow-md: 0 4px 12px rgba(0, 0, 0, 0.08);
    --shadow-lg: 0 12px 32px rgba(0, 0, 0, 0.14);
    --hud-bg: rgba(255, 255, 255, 0.92);
    --hud-border: rgba(0, 0, 0, 0.12);
    --hud-text: #0f172a;
    --map-grid-color: rgba(0, 0, 0, 0.04);
  }

  * {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Inter", "Helvetica Neue", sans-serif;
  }

  body {
    background: var(--bg-canvas);
    color: var(--text-primary);
    display: flex;
    height: 100vh;
    overflow: hidden;
    user-select: none;
    transition: background-color 0.25s ease, color 0.25s ease;
  }

  /* Custom Sleek Scrollbars */
  ::-webkit-scrollbar { width: 6px; height: 6px; }
  ::-webkit-scrollbar-track { background: transparent; }
  ::-webkit-scrollbar-thumb { background: var(--border-strong); border-radius: 4px; }
  ::-webkit-scrollbar-thumb:hover { background: var(--text-muted); }

  /* Barra Lateral Principal */
  #sidebar {
    width: 440px;
    min-width: 380px;
    max-width: 480px;
    background: var(--bg-sidebar);
    border-right: 1px solid var(--border-default);
    display: flex;
    flex-direction: column;
    z-index: 10;
    box-shadow: var(--shadow-lg);
    transition: transform 0.25s cubic-bezier(0.16, 1, 0.3, 1), width 0.2s ease, min-width 0.2s ease;
  }
  #sidebar.collapsed {
    transform: translateX(-100%);
    position: absolute;
    height: 100%;
  }

  /* Cabecera Superior */
  .header-bar {
    padding: 14px 18px;
    border-bottom: 1px solid var(--border-default);
    background: var(--bg-header);
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
  }
  .header-title {
    display: flex;
    flex-direction: column;
    gap: 3px;
    min-width: 0;
  }
  .header-title-row {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .header-title h1 {
    font-size: 15px;
    color: var(--text-primary);
    font-weight: 700;
    letter-spacing: -0.2px;
    display: flex;
    align-items: center;
    gap: 8px;
    white-space: nowrap;
  }
  .brand-badge {
    background: linear-gradient(135deg, var(--accent-cyan), var(--accent-green));
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    font-weight: 800;
  }
  .header-title p {
    font-size: 11px;
    color: var(--text-secondary);
    font-family: "JetBrains Mono", "SF Mono", Menlo, Consolas, monospace;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .header-controls {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .header-icon-btn {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    color: var(--text-secondary);
    width: 32px;
    height: 32px;
    border-radius: 6px;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    font-size: 14px;
    transition: all 0.15s ease;
  }
  .header-icon-btn:hover {
    background: var(--bg-surface-elevated);
    color: var(--text-primary);
    border-color: var(--accent-cyan);
    transform: translateY(-1px);
  }
  .header-icon-btn:active {
    transform: scale(0.96);
  }

  .status-badge {
    background: var(--accent-green-subtle);
    color: var(--accent-green);
    border: 1px solid var(--accent-green);
    padding: 3px 8px;
    border-radius: 12px;
    font-size: 11px;
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 5px;
    white-space: nowrap;
    transition: all 0.2s ease;
  }
  .status-badge::before {
    content: "";
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: var(--accent-green);
    box-shadow: 0 0 6px var(--accent-green);
  }
  .status-badge.pending {
    background: var(--accent-orange-subtle);
    color: var(--accent-orange);
    border-color: var(--accent-orange);
  }
  .status-badge.pending::before {
    background: var(--accent-orange);
    box-shadow: 0 0 6px var(--accent-orange);
  }

  /* Selector de Pestañas Segmentado */
  .tab-nav {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    background: var(--bg-surface);
    border-bottom: 1px solid var(--border-default);
    padding: 3px;
    gap: 3px;
  }
  .tab-btn {
    background: transparent;
    border: none;
    border-radius: 6px;
    color: var(--text-secondary);
    padding: 9px 4px;
    font-size: 11px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.18s ease;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 3px;
    position: relative;
  }
  .tab-btn:hover {
    color: var(--text-primary);
    background: var(--bg-surface-elevated);
  }
  .tab-btn.active {
    color: var(--accent-cyan);
    background: var(--bg-sidebar);
    box-shadow: var(--shadow-sm);
    font-weight: 700;
  }
  .tab-badge {
    font-size: 9px;
    padding: 1px 5px;
    border-radius: 10px;
    background: var(--border-default);
    color: var(--text-muted);
    font-family: monospace;
    transition: all 0.15s ease;
  }
  .tab-btn.active .tab-badge {
    background: var(--accent-cyan-subtle);
    color: var(--accent-cyan);
    border: 1px solid var(--badge-border);
  }

  /* Contenedores de Pestañas */
  .tab-content {
    display: none;
    flex: 1;
    flex-direction: column;
    overflow-y: auto;
    padding: 16px;
    gap: 14px;
  }
  .tab-content.active {
    display: flex;
  }

  /* Secciones y Encabezados */
  .section-title {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.8px;
    color: var(--text-muted);
    font-weight: 700;
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-top: 4px;
  }
  .section-badge {
    background: var(--border-default);
    color: var(--text-secondary);
    font-size: 10px;
    padding: 2px 7px;
    border-radius: 10px;
    font-family: monospace;
  }

  /* Cuadrícula de Herramientas Agrupada */
  .tool-group {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .tool-group-label {
    font-size: 11px;
    color: var(--text-muted);
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .tool-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .tool-grid.single {
    grid-template-columns: 1fr;
  }
  .tool-btn {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
    padding: 10px 12px;
    border-radius: 7px;
    font-size: 12px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: space-between;
    transition: all 0.15s ease;
    font-weight: 500;
    position: relative;
    box-shadow: var(--shadow-sm);
  }
  .tool-btn-content {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .tool-btn:hover {
    background: var(--bg-surface-elevated);
    border-color: var(--accent-cyan);
    transform: translateY(-1px);
    box-shadow: var(--shadow-md);
  }
  .tool-btn:active {
    transform: scale(0.98);
  }
  .tool-btn.active {
    background: var(--accent-cyan-subtle);
    border-color: var(--accent-cyan);
    color: var(--accent-cyan);
    font-weight: 700;
    box-shadow: 0 0 0 1px var(--accent-cyan), var(--shadow-sm);
  }
  .tool-btn.danger {
    border-color: rgba(244, 63, 94, 0.3);
  }
  .tool-btn.danger:hover {
    border-color: var(--accent-red);
    color: var(--accent-red);
  }
  .tool-btn.danger.active {
    background: var(--accent-red-subtle);
    border-color: var(--accent-red);
    color: var(--accent-red);
    box-shadow: 0 0 0 1px var(--accent-red), var(--shadow-sm);
  }

  .kbd-badge {
    background: var(--bg-input);
    border: 1px solid var(--border-default);
    color: var(--text-muted);
    font-size: 10px;
    font-family: "JetBrains Mono", monospace;
    font-weight: 700;
    padding: 2px 5px;
    border-radius: 4px;
    box-shadow: inset 0 -1px 0 rgba(0,0,0,0.3);
  }
  .tool-btn.active .kbd-badge {
    border-color: var(--accent-cyan);
    color: var(--accent-cyan);
    background: var(--bg-sidebar);
  }

  /* Control de Pincel */
  .control-box {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    border-radius: 8px;
    padding: 12px 14px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    box-shadow: var(--shadow-sm);
  }
  .control-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 12px;
    color: var(--text-secondary);
  }
  .brush-metric-badge {
    font-family: "JetBrains Mono", monospace;
    font-size: 11px;
    background: var(--accent-cyan-subtle);
    color: var(--accent-cyan);
    border: 1px solid var(--badge-border);
    padding: 2px 7px;
    border-radius: 12px;
    font-weight: 600;
  }
  .slider-row {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .slider-row input[type="range"] {
    flex: 1;
    accent-color: var(--accent-cyan);
    cursor: pointer;
    height: 5px;
    background: var(--border-default);
    border-radius: 4px;
  }
  .preset-row {
    display: flex;
    gap: 5px;
  }
  .preset-btn {
    flex: 1;
    background: var(--bg-sidebar);
    border: 1px solid var(--border-default);
    color: var(--text-secondary);
    font-size: 11px;
    padding: 5px 3px;
    border-radius: 5px;
    cursor: pointer;
    text-align: center;
    font-weight: 500;
    transition: all 0.15s ease;
  }
  .preset-btn:hover {
    color: var(--text-primary);
    border-color: var(--accent-cyan);
    background: var(--bg-surface-elevated);
  }
  .preset-btn.active {
    border-color: var(--accent-cyan);
    color: var(--accent-cyan);
    background: var(--accent-cyan-subtle);
    font-weight: 700;
  }

  /* Historial y Sub-botones */
  .history-row {
    display: flex;
    gap: 8px;
  }
  .sub-btn {
    flex: 1;
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
    padding: 9px 12px;
    border-radius: 6px;
    font-size: 12px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    font-weight: 600;
    transition: all 0.15s ease;
  }
  .sub-btn:hover:not(:disabled) {
    background: var(--bg-surface-elevated);
    border-color: var(--accent-cyan);
    transform: translateY(-1px);
  }
  .sub-btn:disabled {
    opacity: 0.35;
    cursor: not-allowed;
    border-color: var(--border-subtle);
  }
  .sub-btn.danger {
    background: var(--bg-surface);
    border-color: rgba(244, 63, 94, 0.3);
    color: var(--accent-red);
  }
  .sub-btn.danger:hover {
    background: var(--accent-red-subtle);
    border-color: var(--accent-red);
  }

  /* Plantillas y Opciones Rápidas */
  .template-section {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    border-radius: 8px;
    padding: 12px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    box-shadow: var(--shadow-sm);
  }
  .template-title {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.8px;
    color: var(--accent-cyan);
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .template-list {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .template-card {
    background: var(--bg-sidebar);
    border: 1px solid var(--border-default);
    border-radius: 6px;
    padding: 9px 12px;
    color: var(--text-primary);
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: space-between;
    transition: all 0.18s ease;
  }
  .template-card:hover {
    border-color: var(--accent-cyan);
    background: var(--bg-surface-elevated);
    transform: translateX(3px);
    box-shadow: var(--shadow-sm);
  }
  .template-card.active-tpl {
    border-color: var(--accent-green);
    background: var(--accent-green-subtle);
    box-shadow: 0 0 0 1px var(--accent-green);
  }
  .template-info {
    display: flex;
    flex-direction: column;
    gap: 3px;
  }
  .template-name {
    font-size: 13px;
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .template-desc {
    font-size: 11px;
    color: var(--text-secondary);
  }

  .pill-badge {
    font-size: 10px;
    font-weight: 700;
    padding: 2px 7px;
    border-radius: 4px;
    font-family: "JetBrains Mono", monospace;
    white-space: nowrap;
    background: var(--badge-bg);
    color: var(--accent-cyan);
    border: 1px solid var(--badge-border);
  }
  .pill-badge.green { background: var(--accent-green-subtle); color: var(--accent-green); border-color: rgba(16, 185, 129, 0.3); }
  .pill-badge.purple { background: var(--accent-purple-subtle); color: var(--accent-purple); border-color: rgba(168, 85, 247, 0.3); }
  .pill-badge.orange { background: var(--accent-orange-subtle); color: var(--accent-orange); border-color: rgba(245, 158, 11, 0.3); }
  .pill-badge.blue { background: var(--accent-cyan-subtle); color: var(--accent-cyan); border-color: var(--badge-border); }

  /* Búsqueda y Filtros en Listas */
  .list-search-box {
    display: flex;
    align-items: center;
    position: relative;
    margin-bottom: 2px;
  }
  .list-search-input {
    width: 100%;
    background: var(--bg-input);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
    padding: 7px 10px 7px 28px;
    border-radius: 6px;
    font-size: 12px;
    outline: none;
    transition: all 0.15s ease;
  }
  .list-search-input:focus {
    border-color: var(--accent-cyan);
    box-shadow: 0 0 0 2px var(--accent-cyan-subtle);
  }
  .list-search-icon {
    position: absolute;
    left: 9px;
    color: var(--text-muted);
    font-size: 12px;
    pointer-events: none;
  }

  /* Listas de Elementos */
  .item-list {
    display: flex;
    flex-direction: column;
    gap: 6px;
    max-height: 250px;
    overflow-y: auto;
    padding-right: 2px;
  }
  .empty-state {
    padding: 16px 12px;
    text-align: center;
    font-size: 11px;
    color: var(--text-muted);
    border: 1px dashed var(--border-default);
    border-radius: 6px;
    background: var(--bg-surface);
    line-height: 1.6;
  }
  .item-card {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    border-radius: 7px;
    padding: 9px 12px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    transition: all 0.15s ease;
    gap: 8px;
    box-shadow: var(--shadow-sm);
  }
  .item-card:hover {
    border-color: var(--accent-cyan);
    background: var(--bg-surface-elevated);
    transform: translateY(-1px);
    box-shadow: var(--shadow-md);
  }
  .item-info {
    display: flex;
    flex-direction: column;
    gap: 3px;
    min-width: 0;
  }
  .item-title {
    font-size: 12px;
    font-weight: 600;
    color: var(--text-primary);
    display: flex;
    align-items: center;
    gap: 6px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .item-subtitle {
    font-size: 11px;
    color: var(--text-secondary);
    line-height: 1.4;
  }
  .item-actions {
    display: flex;
    align-items: center;
    gap: 3px;
    flex-shrink: 0;
  }
  .icon-btn {
    background: var(--bg-sidebar);
    border: 1px solid var(--border-default);
    color: var(--text-secondary);
    cursor: pointer;
    padding: 5px 7px;
    border-radius: 5px;
    font-size: 12px;
    transition: all 0.15s ease;
    display: flex;
    align-items: center;
    justify-content: center;
  }
  .icon-btn:hover {
    color: var(--text-primary);
    background: var(--bg-surface-active);
    border-color: var(--border-strong);
    transform: scale(1.05);
  }
  .icon-btn.solidify {
    color: var(--accent-orange);
    border-color: rgba(245, 158, 11, 0.3);
  }
  .icon-btn.solidify:hover {
    background: var(--accent-orange-subtle);
    border-color: var(--accent-orange);
  }
  .icon-btn.focus-btn:hover {
    color: var(--accent-cyan);
    border-color: var(--accent-cyan);
  }
  .icon-btn.danger:hover {
    color: var(--accent-red);
    background: var(--accent-red-subtle);
    border-color: var(--accent-red);
  }

  /* Capas de Visualización Plegables */
  .layers-container {
    background: var(--bg-header);
    border-top: 1px solid var(--border-default);
    display: flex;
    flex-direction: column;
  }
  .layers-header {
    padding: 8px 14px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    font-size: 11px;
    font-weight: 700;
    color: var(--text-secondary);
    text-transform: uppercase;
    letter-spacing: 0.8px;
    cursor: pointer;
    user-select: none;
  }
  .layers-header:hover {
    color: var(--text-primary);
  }
  .layer-toggles {
    padding: 6px 14px 10px 14px;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px 10px;
    font-size: 11px;
  }
  .layer-item {
    display: flex;
    align-items: center;
    gap: 7px;
    color: var(--text-secondary);
    cursor: pointer;
    transition: color 0.15s ease;
  }
  .layer-item:hover {
    color: var(--text-primary);
  }
  .layer-swatch {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    display: inline-block;
    flex-shrink: 0;
  }
  .layer-item input[type="checkbox"] {
    accent-color: var(--accent-cyan);
    cursor: pointer;
  }

  /* Footer con Guardado Principal */
  .footer-actions {
    padding: 12px 14px;
    border-top: 1px solid var(--border-default);
    background: var(--bg-sidebar);
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .save-btn {
    background: linear-gradient(135deg, #0284c7 0%, #10b981 100%);
    color: #ffffff;
    border: none;
    padding: 12px 16px;
    border-radius: 7px;
    font-size: 13px;
    font-weight: 700;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    box-shadow: 0 4px 14px rgba(16, 185, 129, 0.3);
    transition: all 0.2s cubic-bezier(0.16, 1, 0.3, 1);
  }
  .save-btn:hover {
    transform: translateY(-1px);
    box-shadow: 0 6px 20px rgba(16, 185, 129, 0.45);
    filter: brightness(1.08);
  }
  .save-btn:active {
    transform: scale(0.98);
  }
  .save-btn.saving {
    opacity: 0.8;
    pointer-events: none;
  }
  .save-btn .btn-shortcut {
    background: rgba(0, 0, 0, 0.25);
    padding: 2px 6px;
    border-radius: 4px;
    font-size: 11px;
    font-family: monospace;
    font-weight: 600;
  }

  /* Área Principal del Mapa */
  #map-container {
    flex: 1;
    position: relative;
    background: var(--bg-canvas);
    background-image:
      radial-gradient(var(--map-grid-color) 1px, transparent 1px),
      radial-gradient(var(--map-grid-color) 1px, transparent 1px);
    background-size: 24px 24px;
    background-position: 0 0, 12px 12px;
    overflow: hidden;
  }
  #mapCanvas {
    position: absolute;
    top: 0;
    left: 0;
    cursor: crosshair;
  }
  #mapCanvas.mode-pan { cursor: grab; }
  #mapCanvas.mode-pan:active { cursor: grabbing; }

  /* HUD Flotante: Coordenadas & Métrica */
  .hud-coordinates {
    position: absolute;
    bottom: 16px;
    left: 16px;
    background: var(--hud-bg);
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    border: 1px solid var(--hud-border);
    border-radius: 8px;
    padding: 8px 14px;
    font-family: "JetBrains Mono", monospace;
    font-size: 11px;
    color: var(--hud-text);
    pointer-events: none;
    display: flex;
    align-items: center;
    gap: 16px;
    box-shadow: var(--shadow-md);
    z-index: 5;
  }
  .hud-coordinates span b {
    color: var(--accent-cyan);
    font-weight: 600;
  }

  /* HUD Flotante: Instrucciones de Herramienta */
  .hud-instructions {
    position: absolute;
    top: 16px;
    left: 16px;
    background: var(--hud-bg);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    border: 1px solid var(--accent-cyan);
    border-radius: 8px;
    padding: 10px 16px;
    font-size: 12px;
    color: var(--hud-text);
    box-shadow: var(--shadow-md);
    max-width: 600px;
    display: flex;
    align-items: center;
    gap: 10px;
    z-index: 5;
    animation: fadeIn 0.2s ease;
  }
  @keyframes fadeIn { from { opacity: 0; transform: translateY(-4px); } to { opacity: 1; transform: translateY(0); } }

  /* Widget Flotante de Zoom & Vista */
  .canvas-view-controls {
    position: absolute;
    bottom: 16px;
    right: 16px;
    display: flex;
    flex-direction: column;
    background: var(--hud-bg);
    backdrop-filter: blur(10px);
    -webkit-backdrop-filter: blur(10px);
    border: 1px solid var(--hud-border);
    border-radius: 8px;
    box-shadow: var(--shadow-md);
    overflow: hidden;
    z-index: 5;
  }
  .canvas-control-btn {
    background: transparent;
    border: none;
    border-bottom: 1px solid var(--hud-border);
    color: var(--text-primary);
    width: 36px;
    height: 34px;
    display: flex;
    align-items: center;
    justify-content: center;
    cursor: pointer;
    font-size: 14px;
    font-weight: 600;
    transition: all 0.15s ease;
  }
  .canvas-control-btn:last-child {
    border-bottom: none;
  }
  .canvas-control-btn:hover {
    background: var(--bg-surface-elevated);
    color: var(--accent-cyan);
  }
  .canvas-control-btn:active {
    transform: scale(0.92);
  }

  /* Botón de apertura de barra lateral cuando está colapsada */
  #sidebar-open-btn {
    position: absolute;
    top: 16px;
    left: 16px;
    z-index: 9;
    display: none;
    background: var(--bg-sidebar);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
    padding: 8px 12px;
    border-radius: 6px;
    box-shadow: var(--shadow-md);
    cursor: pointer;
    font-size: 13px;
    font-weight: 600;
  }
  #sidebar.collapsed ~ #sidebar-open-btn {
    display: flex;
    align-items: center;
    gap: 6px;
  }

  /* Modales */
  .modal-overlay {
    position: fixed;
    top: 0;
    left: 0;
    width: 100vw;
    height: 100vh;
    background: rgba(0, 0, 0, 0.75);
    backdrop-filter: blur(6px);
    -webkit-backdrop-filter: blur(6px);
    display: none;
    align-items: center;
    justify-content: center;
    z-index: 100;
    animation: fadeIn 0.2s ease;
  }
  .modal-overlay.active { display: flex; }
  .modal-card {
    background: var(--bg-sidebar);
    border: 1px solid var(--border-default);
    border-radius: 12px;
    width: 440px;
    max-width: 90vw;
    padding: 24px;
    box-shadow: var(--shadow-modal);
    display: flex;
    flex-direction: column;
    gap: 16px;
  }
  .modal-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .modal-header h3 {
    font-size: 16px;
    color: var(--accent-cyan);
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 700;
  }
  .modal-close-btn {
    background: transparent;
    border: none;
    color: var(--text-muted);
    cursor: pointer;
    font-size: 18px;
    padding: 4px;
    border-radius: 4px;
  }
  .modal-close-btn:hover {
    color: var(--text-primary);
  }

  .vol-metrics-box {
    background: var(--bg-surface);
    border: 1px solid var(--border-default);
    border-radius: 8px;
    padding: 10px 14px;
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
    font-size: 11px;
    text-align: center;
  }
  .vol-metric-item {
    color: var(--text-secondary);
  }
  .vol-metric-item b {
    display: block;
    font-size: 13px;
    color: var(--accent-green);
    margin-top: 3px;
    font-family: "JetBrains Mono", monospace;
  }

  .form-group {
    display: flex;
    flex-direction: column;
    gap: 6px;
    font-size: 12px;
    color: var(--text-secondary);
  }
  .form-group input, .form-group select {
    background: var(--bg-input);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
    padding: 9px 12px;
    border-radius: 6px;
    font-size: 13px;
    outline: none;
    transition: all 0.15s ease;
  }
  .form-group input:focus, .form-group select:focus {
    border-color: var(--accent-cyan);
    box-shadow: 0 0 0 2px var(--accent-cyan-subtle);
  }

  .checkbox-group {
    display: flex;
    align-items: center;
    gap: 10px;
    font-size: 12px;
    color: var(--text-primary);
    cursor: pointer;
    background: var(--bg-surface);
    padding: 10px 12px;
    border-radius: 6px;
    border: 1px solid var(--border-default);
  }
  .checkbox-group input {
    accent-color: var(--accent-cyan);
    width: 16px;
    height: 16px;
    cursor: pointer;
  }

  .modal-actions {
    display: flex;
    gap: 10px;
    margin-top: 6px;
  }
  .modal-btn {
    flex: 1;
    padding: 11px;
    border-radius: 6px;
    font-weight: 600;
    cursor: pointer;
    border: none;
    font-size: 13px;
    transition: all 0.15s ease;
  }
  .modal-btn.primary {
    background: var(--accent-cyan);
    color: #080c13;
  }
  .modal-btn.primary:hover {
    filter: brightness(1.1);
    transform: translateY(-1px);
  }
  .modal-btn.cancel {
    background: var(--bg-surface-elevated);
    border: 1px solid var(--border-default);
    color: var(--text-primary);
  }
  .modal-btn.cancel:hover {
    background: var(--bg-surface-active);
  }

  /* Modal de Atajos de Teclado */
  .shortcuts-table {
    width: 100%;
    border-collapse: collapse;
    font-size: 12px;
  }
  .shortcuts-table td {
    padding: 8px 6px;
    border-bottom: 1px solid var(--border-subtle);
  }
  .shortcuts-table td:first-child {
    font-family: monospace;
    color: var(--accent-cyan);
    white-space: nowrap;
    width: 35%;
  }
  .shortcuts-table td:last-child {
    color: var(--text-secondary);
  }

  /* Notificaciones Toast */
  #toast {
    position: fixed;
    top: 24px;
    right: 24px;
    background: var(--bg-sidebar);
    color: var(--text-primary);
    border: 1px solid var(--accent-green);
    padding: 12px 20px;
    border-radius: 8px;
    font-weight: 600;
    font-size: 13px;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5), 0 0 12px var(--accent-green-subtle);
    opacity: 0;
    transform: translateY(-12px);
    pointer-events: none;
    transition: all 0.25s cubic-bezier(0.16, 1, 0.3, 1);
    z-index: 1000;
    display: flex;
    align-items: center;
    gap: 10px;
  }
  #toast.show {
    opacity: 1;
    transform: translateY(0);
  }
  #toast.warning {
    border-color: var(--accent-orange);
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5), 0 0 12px var(--accent-orange-subtle);
  }
  #toast.error {
    border-color: var(--accent-red);
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5), 0 0 12px var(--accent-red-subtle);
  }

  /* Responsive / Notebook Adjustments */
  @media (max-width: 1200px) {
    #sidebar {
      width: 390px;
      min-width: 360px;
    }
  }
  @media (max-width: 900px) {
    #sidebar {
      width: 100%;
      max-width: 100%;
    }
  }
</style>
</head>
<body>

<div id="sidebar">
  <div class="header-bar">
    <div class="header-title">
      <div class="header-title-row">
        <h1>🤖 <span class="brand-badge">Base@Home</span> Studio</h1>
      </div>
      <p id="map-meta-info">Mapa PGM: 324x582 px (0.05 m/px)</p>
    </div>
    <div class="header-controls">
      <button class="header-icon-btn" id="theme-toggle-btn" title="Alternar Modo Claro / Oscuro" onclick="toggleTheme()">☀️</button>
      <button class="header-icon-btn" title="Atajos de Teclado (?)" onclick="openShortcutsModal()">❓</button>
      <button class="header-icon-btn" title="Colapsar Panel" onclick="toggleSidebar()">◀</button>
      <div class="status-badge" id="save-status-badge">Conectado</div>
    </div>
  </div>

  <div class="tab-nav">
    <button class="tab-btn active" onclick="switchTab('cleaning')">
      <span>🧹 Limpieza</span>
    </button>
    <button class="tab-btn" onclick="switchTab('furniture')">
      <span>🪑 Mesas</span>
      <span class="tab-badge" id="badge-furniture">0</span>
    </button>
    <button class="tab-btn" onclick="switchTab('seats')">
      <span>🛋️ Asientos</span>
      <span class="tab-badge" id="badge-seats">0</span>
    </button>
    <button class="tab-btn" onclick="switchTab('objects')">
      <span>🥫 Objetos</span>
      <span class="tab-badge" id="badge-objects">0</span>
    </button>
    <button class="tab-btn" onclick="switchTab('zones')">
      <span>📍 Zonas/WP</span>
      <span class="tab-badge" id="badge-zones">0</span>
    </button>
  </div>

  <!-- PESTAÑA 1: LIMPIEZA Y EDICIÓN DEL MAPA PGM -->
  <div class="tab-content active" id="tab-cleaning">
    <div class="tool-group">
      <div class="tool-group-label"><span>🖐️ Navegación</span></div>
      <div class="tool-grid single">
        <button class="tool-btn active" id="btn-pan" onclick="setMode('pan')">
          <div class="tool-btn-content"><span>🖐️</span><span>Navegar (Pan)</span></div>
          <span class="kbd-badge">1</span>
        </button>
      </div>
    </div>

    <div class="tool-group">
      <div class="tool-group-label"><span>🧹 Limpieza de Obstáculos (Espacio Libre)</span></div>
      <div class="tool-grid">
        <button class="tool-btn" id="btn-erase" onclick="setMode('erase')">
          <div class="tool-btn-content"><span>🧹</span><span>Goma Libre</span></div>
          <span class="kbd-badge">2</span>
        </button>
        <button class="tool-btn" id="btn-clear_box" onclick="setMode('clear_box')">
          <div class="tool-btn-content"><span>📦</span><span>Limpiar Rect</span></div>
          <span class="kbd-badge">3</span>
        </button>
      </div>
    </div>

    <div class="tool-group">
      <div class="tool-group-label"><span>✏️ Paredes y Obstáculos (Ocupado / Desconocido)</span></div>
      <div class="tool-grid">
        <button class="tool-btn" id="btn-obstacle" onclick="setMode('obstacle')">
          <div class="tool-btn-content"><span>✏️</span><span>Lápiz Pared</span></div>
          <span class="kbd-badge">4</span>
        </button>
        <button class="tool-btn" id="btn-line_wall" onclick="setMode('line_wall')">
          <div class="tool-btn-content"><span>📏</span><span>Línea Pared</span></div>
          <span class="kbd-badge">5</span>
        </button>
      </div>
      <div class="tool-grid single">
        <button class="tool-btn" id="btn-unknown" onclick="setMode('unknown')">
          <div class="tool-btn-content"><span>🌫️</span><span>Pincel Gris (Desconocido)</span></div>
          <span class="kbd-badge">6</span>
        </button>
      </div>
    </div>

    <div class="control-box">
      <div class="control-header">
        <span>Grosor de Pincel</span>
        <span class="brush-metric-badge"><b id="brush-size-val">3 px</b> · <span id="brush-meters-val">15 cm</span></span>
      </div>
      <div class="slider-row">
        <input type="range" id="brush-slider" min="1" max="40" value="3" oninput="updateBrushSize(this.value)">
      </div>
      <div class="preset-row">
        <button class="preset-btn" onclick="updateBrushSize(1)">1px</button>
        <button class="preset-btn active" onclick="updateBrushSize(3)">3px (15cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(6)">6px (30cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(12)">12px (60cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(24)">24px (1.2m)</button>
      </div>
    </div>

    <div class="history-row">
      <button class="sub-btn" id="btn-undo" onclick="undo()" disabled>
        <span>↩️ Deshacer</span> <span class="kbd-badge">Ctrl+Z</span>
      </button>
      <button class="sub-btn" id="btn-redo" onclick="redo()" disabled>
        <span>↪️ Rehacer</span> <span class="kbd-badge">Ctrl+Y</span>
      </button>
    </div>

    <button class="sub-btn danger" style="margin-top: 4px;" onclick="restoreOriginalMap()">
      🔄 Revertir al Mapa Original (.orig)
    </button>
  </div>

  <!-- PESTAÑA 2: MESAS Y MOBILIARIO VOLUMÉTRICO -->
  <div class="tab-content" id="tab-furniture">
    <div class="template-section">
      <div class="template-title">📦 Opciones de Mesas (Dibuja su Volumen 3D):</div>
      <div class="template-list">
        <div class="template-card" onclick="placeVolumeTemplate('furniture', 'Mesa de Centro', 'table', 76, true)">
          <div class="template-info">
            <span class="template-name">☕ Mesa de Centro / Pringles</span>
            <span class="template-desc">Arrastra la caja sobre las patas | Altura 76 cm</span>
          </div>
          <span class="pill-badge orange">76 cm</span>
        </div>
        <div class="template-card" onclick="placeVolumeTemplate('furniture', 'Mesa de Comedor', 'table', 76, true)">
          <div class="template-info">
            <span class="template-name">🍽️ Mesa de Comedor</span>
            <span class="template-desc">Arrastra la caja del volumen | Altura 76 cm</span>
          </div>
          <span class="pill-badge orange">76 cm</span>
        </div>
        <div class="template-card" onclick="placeVolumeTemplate('furniture', 'Escritorio', 'desk', 75, true)">
          <div class="template-info">
            <span class="template-name">💻 Escritorio / Mesada</span>
            <span class="template-desc">Arrastra la caja del volumen | Altura 75 cm</span>
          </div>
          <span class="pill-badge orange">75 cm</span>
        </div>
        <div class="template-card" onclick="placeVolumeTemplate('furniture', 'Estante TV / Repisa', 'shelf', 110, true)">
          <div class="template-info">
            <span class="template-name">📺 Mueble TV / Estante</span>
            <span class="template-desc">Arrastra el volumen | Altura 110 cm</span>
          </div>
          <span class="pill-badge orange">110 cm</span>
        </div>
      </div>
    </div>

    <div class="section-title"><span>Herramientas de Volumen Libre</span></div>
    <div class="tool-grid">
      <button class="tool-btn" id="btn-draw_vol_box" onclick="startFreeVolumeBox('furniture')">
        <div class="tool-btn-content"><span>📦</span><span>Caja 3D (Drag)</span></div>
      </button>
      <button class="tool-btn" id="btn-draw_vol_poly" onclick="startFreeVolumePoly('furniture')">
        <div class="tool-btn-content"><span>📐</span><span>Polígono 3D</span></div>
      </button>
    </div>

    <div class="section-title">
      <span>Mesas en tu Mapa</span>
      <span class="section-badge" id="furniture-count">0</span>
    </div>
    <div class="list-search-box">
      <span class="list-search-icon">🔍</span>
      <input type="text" class="list-search-input" placeholder="Filtrar mesas..." oninput="filterItemList('furniture-list', this.value)">
    </div>
    <div class="item-list" id="furniture-list"></div>
  </div>

  <!-- PESTAÑA 3: ASIENTOS VOLUMÉTRICOS -->
  <div class="tab-content" id="tab-seats">
    <div class="template-section">
      <div class="template-title">🛋️ Opciones de Asientos (Dibuja su Volumen 3D):</div>
      <div class="template-list">
        <div class="template-card" onclick="placeVolumeTemplate('seat', 'Sofá Principal', 'couch', 85, true, 'point_couch', 2)">
          <div class="template-info">
            <span class="template-name">🛋️ Sofá Receptionist (Volumen)</span>
            <span class="template-desc">Arrastra caja | Altura 85 cm | Gesto point_couch</span>
          </div>
          <span class="pill-badge purple">point_couch</span>
        </div>
        <div class="template-card" onclick="placeVolumeTemplate('seat', 'Silla Individual', 'armchair', 90, true, 'point_chair', 1)">
          <div class="template-info">
            <span class="template-name">🪑 Silla Individual (Volumen)</span>
            <span class="template-desc">Arrastra caja | Altura 90 cm | Gesto point_chair</span>
          </div>
          <span class="pill-badge purple">point_chair</span>
        </div>
        <div class="template-card" onclick="placeVolumeTemplate('seat', 'Silla Secundaria', 'armchair', 90, true, 'point_chair', 1)">
          <div class="template-info">
            <span class="template-name">🪑 Silla Secundaria (Volumen)</span>
            <span class="template-desc">Arrastra caja | Altura 90 cm | Gesto point_chair</span>
          </div>
          <span class="pill-badge purple">point_chair</span>
        </div>
      </div>
    </div>

    <div class="section-title"><span>Herramientas de Asiento Libre</span></div>
    <div class="tool-grid">
      <button class="tool-btn" onclick="startFreeVolumeBox('seat')">
        <div class="tool-btn-content"><span>📦</span><span>Asiento Caja (Drag)</span></div>
      </button>
      <button class="tool-btn" onclick="startFreeVolumePoly('seat')">
        <div class="tool-btn-content"><span>📐</span><span>Asiento Polígono</span></div>
      </button>
    </div>

    <div class="section-title">
      <span>Asientos en tu Mapa</span>
      <span class="section-badge" id="seat-count">0</span>
    </div>
    <div class="list-search-box">
      <span class="list-search-icon">🔍</span>
      <input type="text" class="list-search-input" placeholder="Filtrar asientos..." oninput="filterItemList('seats-list', this.value)">
    </div>
    <div class="item-list" id="seats-list"></div>
  </div>

  <!-- PESTAÑA 4: OBJETOS MANIPULABLES -->
  <div class="tab-content" id="tab-objects">
    <div class="template-section">
      <div class="template-title">🥫 Plantillas de Objetos (Clic en Mapa):</div>
      <div class="template-list">
        <div class="template-card" onclick="placeObjectTemplate('pringles', 'can', 'Pringles (Lata)')">
          <div class="template-info">
            <span class="template-name">🥫 Pringles / Papas</span>
            <span class="template-desc">Clic en la mesa | Lata (can) | Approach 0.60m</span>
          </div>
          <span class="pill-badge orange">can</span>
        </div>
        <div class="template-card" onclick="placeObjectTemplate('coke', 'can', 'Coca-Cola (Lata)')">
          <div class="template-info">
            <span class="template-name">🥤 Coca-Cola / Refresco</span>
            <span class="template-desc">Clic en la mesa | Lata (can)</span>
          </div>
          <span class="pill-badge orange">can</span>
        </div>
        <div class="template-card" onclick="placeObjectTemplate('water', 'bottle', 'Agua (Botella)')">
          <div class="template-info">
            <span class="template-name">🍾 Botella de Agua</span>
            <span class="template-desc">Clic en la mesa | Botella (bottle)</span>
          </div>
          <span class="pill-badge blue">bottle</span>
        </div>
        <div class="template-card" onclick="placeObjectTemplate('cup', 'cup', 'Taza / Vaso')">
          <div class="template-info">
            <span class="template-name">☕ Taza / Pocillo</span>
            <span class="template-desc">Clic en la mesa | Taza (cup)</span>
          </div>
          <span class="pill-badge purple">cup</span>
        </div>
      </div>
    </div>

    <div class="section-title"><span>Herramienta de Objeto Libre</span></div>
    <div class="tool-grid single">
      <button class="tool-btn" id="btn-place_object" onclick="startPlaceObject()">
        <div class="tool-btn-content"><span>🎯</span><span>Colocar Objeto (Click)</span></div>
      </button>
    </div>

    <div class="section-title">
      <span>Objetos en tu Mapa</span>
      <span class="section-badge" id="object-count">0</span>
    </div>
    <div class="list-search-box">
      <span class="list-search-icon">🔍</span>
      <input type="text" class="list-search-input" placeholder="Filtrar objetos..." oninput="filterItemList('objects-list', this.value)">
    </div>
    <div class="item-list" id="objects-list"></div>
  </div>

  <!-- PESTAÑA 5: ZONAS Y WAYPOINTS -->
  <div class="tab-content" id="tab-zones">
    <div class="template-section">
      <div class="template-title">⚡ Puntos Oficiales Receptionist (Nav2):</div>
      <div class="template-list">
        <div class="template-card" onclick="placeWaypointTemplate('door', 'entrance')">
          <div class="template-info">
            <span class="template-name">🚪 Puerta / Entrada (door)</span>
            <span class="template-desc">Clic + arrastrar orientación Yaw para esperar invitados</span>
          </div>
          <span class="pill-badge green">door</span>
        </div>
        <div class="template-card" onclick="placeWaypointTemplate('trash_drop', 'outside')">
          <div class="template-info">
            <span class="template-name">🗑️ Punto de la Basura (trash_drop)</span>
            <span class="template-desc">Donde el robot deja la bolsa | Clic y arrastrar Yaw</span>
          </div>
          <span class="pill-badge orange">trash_drop</span>
        </div>
        <div class="template-card" onclick="placeWaypointTemplate('living_room', 'living_room')">
          <div class="template-info">
            <span class="template-name">🛋️ Punto Sala (living_room)</span>
            <span class="template-desc">Clic + arrastrar Yaw para presentar al anfitrión</span>
          </div>
          <span class="pill-badge green">living_room</span>
        </div>
      </div>
    </div>

    <div class="template-section">
      <div class="template-title">⚡ Plantillas de Zonas Semánticas:</div>
      <div class="template-list">
        <div class="template-card" onclick="placeZoneTemplate('Sala de Estar', 'living_room', '#00f090')">
          <div class="template-info">
            <span class="template-name">📐 Delimitar Sala de Estar</span>
            <span class="template-desc">Clics para polígono | Doble clic para terminar</span>
          </div>
          <span class="pill-badge green">living_room</span>
        </div>
        <div class="template-card" onclick="placeZoneTemplate('Entrada', 'entrance', '#00d2ff')">
          <div class="template-info">
            <span class="template-name">📐 Delimitar Entrada</span>
            <span class="template-desc">Clics para polígono de entrada</span>
          </div>
          <span class="pill-badge blue">entrance</span>
        </div>
        <div class="template-card" onclick="placeZoneTemplate('Cocina', 'kitchen', '#ffaa00')">
          <div class="template-info">
            <span class="template-name">🍽️ Delimitar Cocina</span>
            <span class="template-desc">Clics para polígono de cocina</span>
          </div>
          <span class="pill-badge orange">kitchen</span>
        </div>
      </div>
    </div>

    <div class="section-title"><span>Herramientas Libres</span></div>
    <div class="tool-grid">
      <button class="tool-btn" id="btn-waypoint" onclick="setMode('waypoint')">
        <div class="tool-btn-content"><span>📍</span><span>Waypoint Libre</span></div>
      </button>
      <button class="tool-btn" id="btn-zone" onclick="setMode('zone')">
        <div class="tool-btn-content"><span>📐</span><span>Zona Libre</span></div>
      </button>
    </div>
    <div class="tool-grid single">
      <button class="tool-btn danger" id="btn-keepout" onclick="setMode('keepout')">
        <div class="tool-btn-content"><span>🚫</span><span>Zona Prohibida (Keep-Out)</span></div>
      </button>
    </div>

    <div class="section-title">
      <span>Zonas Semánticas</span>
      <span class="section-badge" id="zone-count">0</span>
    </div>
    <div class="list-search-box">
      <span class="list-search-icon">🔍</span>
      <input type="text" class="list-search-input" placeholder="Filtrar zonas..." oninput="filterItemList('zones-list', this.value)">
    </div>
    <div class="item-list" id="zones-list"></div>

    <div class="section-title">
      <span>Waypoints Nav2</span>
      <span class="section-badge" id="wp-count">0</span>
    </div>
    <div class="list-search-box">
      <span class="list-search-icon">🔍</span>
      <input type="text" class="list-search-input" placeholder="Filtrar waypoints..." oninput="filterItemList('waypoints-list', this.value)">
    </div>
    <div class="item-list" id="waypoints-list"></div>
  </div>

  <!-- Capas de Visualización Plegables -->
  <div class="layers-container">
    <div class="layers-header" onclick="toggleLayersPanel()">
      <span>Capas de Visualización</span>
      <span id="layers-chevron">▼</span>
    </div>
    <div class="layer-toggles" id="layers-content">
      <label class="layer-item">
        <input type="checkbox" id="layer-map" checked onchange="draw()">
        <span class="layer-swatch" style="background: #94a3b8;"></span>
        <span>🗺️ Mapa PGM</span>
      </label>
      <label class="layer-item">
        <input type="checkbox" id="layer-furniture" checked onchange="draw()">
        <span class="layer-swatch" style="background: #f59e0b;"></span>
        <span>🪑 Mesas</span>
      </label>
      <label class="layer-item">
        <input type="checkbox" id="layer-seats" checked onchange="draw()">
        <span class="layer-swatch" style="background: #a855f7;"></span>
        <span>🛋️ Asientos</span>
      </label>
      <label class="layer-item">
        <input type="checkbox" id="layer-objects" checked onchange="draw()">
        <span class="layer-swatch" style="background: #10b981;"></span>
        <span>🥫 Objetos</span>
      </label>
      <label class="layer-item">
        <input type="checkbox" id="layer-waypoints" checked onchange="draw()">
        <span class="layer-swatch" style="background: #00f090;"></span>
        <span>📍 Waypoints</span>
      </label>
      <label class="layer-item">
        <input type="checkbox" id="layer-zones" checked onchange="draw()">
        <span class="layer-swatch" style="background: #00d2ff;"></span>
        <span>📐 Zonas</span>
      </label>
      <label class="layer-item" style="grid-column: span 2;">
        <input type="checkbox" id="layer-keepout" checked onchange="draw()">
        <span class="layer-swatch" style="background: #f43f5e;"></span>
        <span>🚫 Zonas Prohibidas</span>
      </label>
    </div>
  </div>

  <!-- Acciones Finales -->
  <div class="footer-actions">
    <button class="save-btn" id="btn-save-all" onclick="saveAll()">
      <span>💾 Guardar Todo (PGM + Semántico)</span>
      <span class="btn-shortcut">Ctrl+S</span>
    </button>
    <button class="sub-btn" onclick="resetView()">
      <span>🎯 Centrar Mapa en Pantalla</span> <span class="kbd-badge">F</span>
    </button>
  </div>
</div>

<button id="sidebar-open-btn" onclick="toggleSidebar()">
  <span>▶</span> <span>Mostrar Panel</span>
</button>

<!-- Área del Mapa -->
<div id="map-container">
  <canvas id="mapCanvas"></canvas>

  <div class="hud-instructions" id="hud-instructions">
    💡 <b>Modo Navegar:</b> Arrastra con el ratón para moverte y usa la rueda para hacer zoom.
  </div>

  <div class="hud-coordinates">
    <span>X: <b id="hud-x">0.00</b>m · Y: <b id="hud-y">0.00</b>m</span>
    <span>Pixel: <b id="hud-px">0</b>, <b id="hud-py">0</b></span>
    <span>Zoom: <b id="zoom-level">100</b>%</span>
  </div>

  <!-- Controles flotantes de Zoom y Ajuste -->
  <div class="canvas-view-controls">
    <button class="canvas-control-btn" title="Acercar (+)" onclick="zoomIn()">＋</button>
    <button class="canvas-control-btn" title="Restablecer 100%" onclick="setZoom100()">1:1</button>
    <button class="canvas-control-btn" title="Alejar (-)" onclick="zoomOut()">－</button>
    <button class="canvas-control-btn" title="Ajustar Mapa (F)" onclick="resetView()">⛶</button>
  </div>
</div>

<!-- Modal Unificado de Registro de Volumen 3D -->
<div class="modal-overlay" id="volume-modal">
  <div class="modal-card">
    <div class="modal-header">
      <h3 id="vol-modal-title">📦 Registrar Volumen 3D</h3>
      <button class="modal-close-btn" onclick="closeVolumeModal()">✕</button>
    </div>
    
    <div class="vol-metrics-box">
      <div class="vol-metric-item">Ancho (X)<b id="vol-metric-w">1.20 m</b></div>
      <div class="vol-metric-item">Largo (Y)<b id="vol-metric-l">0.80 m</b></div>
      <div class="vol-metric-item">Volumen 3D<b id="vol-metric-v">0.73 m³</b></div>
    </div>

    <div class="form-group">
      <label>Nombre del Elemento:</label>
      <input type="text" id="vol-name-input" value="Mesa de Centro">
    </div>

    <div class="form-group">
      <label>Categoría / Tipo:</label>
      <select id="vol-type-input" onchange="onVolTypeChange(this.value)">
        <option value="couch">Sofá (couch - Asiento)</option>
        <option value="armchair">Silla / Sillón (armchair - Asiento)</option>
        <option value="table">Mesa de Centro / Comedor (table)</option>
        <option value="desk">Escritorio / Mesada (desk)</option>
        <option value="shelf">Estante / Repisa (shelf)</option>
        <option value="obstacle">Obstáculo Volumétrico (box)</option>
      </select>
    </div>

    <div class="form-group">
      <label>Altura Total (Z) en cm:</label>
      <input type="number" id="vol-height-input" value="76" min="10" max="260" step="1" oninput="updateModalVolume()">
    </div>

    <div class="form-group" id="vol-gesture-group">
      <label>Gesto del Brazo Piper (para Asientos Receptionist):</label>
      <select id="vol-gesture-input">
        <option value="point_couch">Señalar Sofá (point_couch)</option>
        <option value="point_chair">Señalar Silla (point_chair)</option>
        <option value="wave_hello">Saludar (wave_hello)</option>
        <option value="none">Sin Gesto</option>
      </select>
    </div>

    <label class="checkbox-group">
      <input type="checkbox" id="vol-solidify-check" checked>
      <div>
        <b>⚡ Solidificar volumen en el mapa PGM</b>
        <div style="font-size: 11px; color: var(--text-muted); margin-top: 2px;">Rellena la huella en negro para evitar colisión de Nav2</div>
      </div>
    </label>

    <div class="modal-actions">
      <button class="modal-btn cancel" onclick="closeVolumeModal()">Cancelar</button>
      <button class="modal-btn primary" onclick="confirmSaveVolume()">Guardar Volumen</button>
    </div>
  </div>
</div>

<!-- Modal de Configuración / Edición de Objetos -->
<div class="modal-overlay" id="object-modal">
  <div class="modal-card">
    <div class="modal-header">
      <h3 id="obj-modal-title">🎯 Registrar Objeto Manipulable</h3>
      <button class="modal-close-btn" onclick="closeObjectModal()">✕</button>
    </div>

    <div class="form-group">
      <label>Número Único (visible en el círculo del mapa):</label>
      <input type="number" id="obj-number-input" min="1" step="1" oninput="validateObjectModalNumber()">
      <small id="obj-number-warning" style="color: var(--accent-red); display: none; font-size: 11px;">⚠ Este número ya está en uso</small>
    </div>

    <div class="form-group">
      <label>Nombre del Objeto (ej. pringles, coke, cup):</label>
      <input type="text" id="obj-name-input" value="pringles">
    </div>

    <div class="form-group">
      <label>Clase del Objeto:</label>
      <select id="obj-class-input">
        <option value="can">can (Lata)</option>
        <option value="bottle">bottle (Botella)</option>
        <option value="cup">cup (Vaso / Taza)</option>
        <option value="box">box (Caja)</option>
        <option value="other">other (Otro)</option>
      </select>
    </div>

    <div class="form-group">
      <label>Altura Z del centro del objeto (metros):</label>
      <input type="number" id="obj-z-input" step="0.005" value="0.865">
      <small id="obj-z-desc" style="color: var(--text-muted); font-size: 11px;">Default: altura de la mesa contenedora + 0.10 m</small>
    </div>

    <div class="form-group">
      <label>Mesa contenedora (on):</label>
      <input type="text" id="obj-on-input" readonly style="background: var(--bg-surface); color: var(--accent-green); font-weight: 600;">
    </div>

    <div class="vol-metrics-box">
      <div class="vol-metric-item">Approach X<b id="obj-app-x-disp">0.00 m</b></div>
      <div class="vol-metric-item">Approach Y<b id="obj-app-y-disp">0.00 m</b></div>
      <div class="vol-metric-item">Yaw (hacia obj)<b id="obj-app-yaw-disp">0°</b></div>
    </div>
    <small id="obj-reach-warning" style="color: var(--accent-orange); display: none; font-size: 11px;"></small>

    <div class="modal-actions">
      <button class="modal-btn cancel" onclick="closeObjectModal()">Cancelar</button>
      <button class="modal-btn primary" id="btn-save-object" onclick="confirmSaveObject()">Guardar Objeto</button>
    </div>
  </div>
</div>

<!-- Modal de Atajos de Teclado -->
<div class="modal-overlay" id="shortcuts-modal">
  <div class="modal-card">
    <div class="modal-header">
      <h3>⌨️ Atajos de Teclado</h3>
      <button class="modal-close-btn" onclick="closeShortcutsModal()">✕</button>
    </div>
    <table class="shortcuts-table">
      <tbody>
        <tr><td>1</td><td>Modo Navegar (Pan)</td></tr>
        <tr><td>2</td><td>Goma de Borrar (Espacio Libre)</td></tr>
        <tr><td>3</td><td>Limpiar Rectángulo</td></tr>
        <tr><td>4</td><td>Lápiz de Pared</td></tr>
        <tr><td>5</td><td>Línea Recta de Pared</td></tr>
        <tr><td>6</td><td>Pincel Gris (Desconocido)</td></tr>
        <tr><td>Ctrl + Z</td><td>Deshacer última acción de dibujo</td></tr>
        <tr><td>Ctrl + Y</td><td>Rehacer acción de dibujo</td></tr>
        <tr><td>Ctrl + S</td><td>Guardar todo (PGM + Semántico)</td></tr>
        <tr><td>F</td><td>Ajustar y centrar mapa en pantalla</td></tr>
        <tr><td>R</td><td>Restablecer vista a escala 100%</td></tr>
        <tr><td>Esc</td><td>Cancelar trazado actual o cerrar modal</td></tr>
        <tr><td>Rueda Ratón</td><td>Zoom in / Zoom out en cursor</td></tr>
        <tr><td>Arrastre</td><td>Panoramizar mapa (Modo Pan o botón central)</td></tr>
        <tr><td>Doble Clic</td><td>Cerrar y finalizar polígono en edición</td></tr>
      </tbody>
    </table>
    <div class="modal-actions">
      <button class="modal-btn primary" onclick="closeShortcutsModal()">Entendido</button>
    </div>
  </div>
</div>

<div id="toast">¡Guardado con éxito!</div>

<script>
  // --- Estado del Mapa y Elementos Semánticos ---
  let mapInfo = { resolution: 0.05, origin: [-6.192, -9.153, 0], width: 324, height: 582 };
  let zones = {};
  let furniture = {};
  let keepout_zones = {};
  let waypoints = {};
  let seats = {};
  let objects = {};
  let pendingObject = null;
  let editingObjectId = null;

  let currentMode = 'pan';
  let brushSize = 3; // px
  let mousePixel = { x: 0, y: 0 };
  let isMouseDown = false;
  let dragStartPixel = null;
  let activeTemplate = null;
  let pendingGeometry = null;
  let hasUnsavedChanges = false;

  // Transformación de Vista
  let view = { x: 50, y: 50, zoom: 1.5 };

  // Canvases
  const rawCanvas = document.createElement('canvas');
  const rawCtx = rawCanvas.getContext('2d', { willReadFrequently: true });
  const colorCanvas = document.createElement('canvas');
  const colorCtx = colorCanvas.getContext('2d');
  const displayCanvas = document.getElementById('mapCanvas');
  const ctx = displayCanvas.getContext('2d');
  const container = document.getElementById('map-container');

  let undoStack = [];
  let redoStack = [];
  const MAX_HISTORY = 25;

  function resizeDisplayCanvas() {
    displayCanvas.width = container.clientWidth;
    displayCanvas.height = container.clientHeight;
    draw();
  }
  window.addEventListener('resize', resizeDisplayCanvas);

  function worldToPixel(x, y) {
    const px = (x - mapInfo.origin[0]) / mapInfo.resolution;
    const py = (mapInfo.height - 1) - ((y - mapInfo.origin[1]) / mapInfo.resolution);
    return { x: px, y: py };
  }

  function pixelToWorld(px, py) {
    const x = mapInfo.origin[0] + px * mapInfo.resolution;
    const y = mapInfo.origin[1] + (mapInfo.height - 1 - py) * mapInfo.resolution;
    return { x: x, y: y };
  }

  function screenToPixel(sx, sy) {
    return { x: (sx - view.x) / view.zoom, y: (sy - view.y) / view.zoom };
  }

  function pixelToScreen(px, py) {
    return { x: view.x + px * view.zoom, y: view.y + py * view.zoom };
  }

  function worldToScreen(x, y) {
    const p = worldToPixel(x, y);
    return pixelToScreen(p.x, p.y);
  }

  // --- Temas Claro / Oscuro ---
  function initTheme() {
    const savedTheme = localStorage.getItem('mapeditor_theme') || 'dark';
    setTheme(savedTheme);
  }

  function setTheme(theme) {
    document.documentElement.setAttribute('data-theme', theme);
    localStorage.setItem('mapeditor_theme', theme);
    const btn = document.getElementById('theme-toggle-btn');
    if (btn) btn.textContent = theme === 'dark' ? '☀️' : '🌙';
    if (mapInfo.width && mapInfo.height) {
      updateColorCanvas();
      draw();
    }
  }

  function toggleTheme() {
    const current = document.documentElement.getAttribute('data-theme') || 'dark';
    setTheme(current === 'dark' ? 'light' : 'dark');
  }

  function toggleSidebar() {
    const sidebar = document.getElementById('sidebar');
    sidebar.classList.toggle('collapsed');
    setTimeout(resizeDisplayCanvas, 260);
  }

  function toggleLayersPanel() {
    const content = document.getElementById('layers-content');
    const chevron = document.getElementById('layers-chevron');
    if (content.style.display === 'none') {
      content.style.display = 'grid';
      chevron.textContent = '▼';
    } else {
      content.style.display = 'none';
      chevron.textContent = '▲';
    }
  }

  function openShortcutsModal() {
    document.getElementById('shortcuts-modal').classList.add('active');
  }

  function closeShortcutsModal() {
    document.getElementById('shortcuts-modal').classList.remove('active');
  }

  function setUnsaved(state) {
    hasUnsavedChanges = state;
    const badge = document.getElementById('save-status-badge');
    if (badge) {
      if (state) {
        badge.textContent = '● Pendiente';
        badge.className = 'status-badge pending';
      } else {
        badge.textContent = 'Conectado';
        badge.className = 'status-badge';
      }
    }
  }

  async function loadData() {
    try {
      const res = await fetch('/api/data');
      const data = await res.json();
      mapInfo = data.map_info;
      zones = data.zones || {};
      furniture = data.furniture || {};
      keepout_zones = data.keepout_zones || {};
      waypoints = data.waypoints || {};
      seats = data.seats || {};
      objects = data.objects || {};

      document.getElementById('map-meta-info').textContent =
        `Mapa PGM: ${mapInfo.width}x${mapInfo.height} px (${mapInfo.resolution} m/px)`;

      rawCanvas.width = mapInfo.width;
      rawCanvas.height = mapInfo.height;
      colorCanvas.width = mapInfo.width;
      colorCanvas.height = mapInfo.height;

      const rawImg = new Image();
      rawImg.src = '/api/map_raw.png?t=' + Date.now();
      rawImg.onload = () => {
        rawCtx.drawImage(rawImg, 0, 0);
        updateColorCanvas();
        pushUndo();
        resetView();
        updateSidebar();
      };
    } catch (e) {
      console.error('Error cargando mapa:', e);
      showToast('Error cargando datos del mapa', 'error');
    }
  }

  function updateColorCanvas() {
    const isLight = document.documentElement.getAttribute('data-theme') === 'light';
    const imgData = rawCtx.getImageData(0, 0, mapInfo.width, mapInfo.height);
    const pixels = imgData.data;
    const colorData = colorCtx.createImageData(mapInfo.width, mapInfo.height);
    const out = colorData.data;

    for (let i = 0; i < pixels.length; i += 4) {
      const val = pixels[i];
      if (val > 220) {
        // Espacio libre navegable
        if (isLight) {
          out[i] = 255; out[i+1] = 255; out[i+2] = 255; out[i+3] = 255;
        } else {
          out[i] = 238; out[i+1] = 242; out[i+2] = 248; out[i+3] = 255;
        }
      } else if (val < 40) {
        // Obstáculos / paredes
        if (isLight) {
          out[i] = 15; out[i+1] = 23; out[i+2] = 42; out[i+3] = 255;
        } else {
          out[i] = 9; out[i+1] = 13; out[i+2] = 22; out[i+3] = 255;
        }
      } else {
        // Desconocido
        if (isLight) {
          out[i] = 203; out[i+1] = 213; out[i+2] = 225; out[i+3] = 255;
        } else {
          out[i] = 30; out[i+1] = 38; out[i+2] = 54; out[i+3] = 255;
        }
      }
    }
    colorCtx.putImageData(colorData, 0, 0);
  }

  // --- Historial Deshacer / Rehacer ---
  function pushUndo() {
    if (undoStack.length >= MAX_HISTORY) undoStack.shift();
    undoStack.push(rawCtx.getImageData(0, 0, mapInfo.width, mapInfo.height));
    redoStack = [];
    updateUndoRedoButtons();
    setUnsaved(true);
  }

  function undo() {
    if (undoStack.length <= 1) return;
    redoStack.push(undoStack.pop());
    const prev = undoStack[undoStack.length - 1];
    rawCtx.putImageData(prev, 0, 0);
    updateColorCanvas();
    updateUndoRedoButtons();
    draw();
    setUnsaved(true);
  }

  function redo() {
    if (redoStack.length === 0) return;
    const next = redoStack.pop();
    undoStack.push(next);
    rawCtx.putImageData(next, 0, 0);
    updateColorCanvas();
    updateUndoRedoButtons();
    draw();
    setUnsaved(true);
  }

  function updateUndoRedoButtons() {
    const uBtn = document.getElementById('btn-undo');
    const rBtn = document.getElementById('btn-redo');
    if (uBtn) uBtn.disabled = undoStack.length <= 1;
    if (rBtn) rBtn.disabled = redoStack.length === 0;
  }

  // --- Navegación y Zoom ---
  function resetView() {
    view.zoom = Math.min(
      (displayCanvas.width - 80) / mapInfo.width,
      (displayCanvas.height - 80) / mapInfo.height
    );
    view.x = (displayCanvas.width - mapInfo.width * view.zoom) / 2;
    view.y = (displayCanvas.height - mapInfo.height * view.zoom) / 2;
    document.getElementById('zoom-level').textContent = Math.round(view.zoom * 100);
    draw();
  }

  function setZoom100() {
    zoomAt(displayCanvas.width / 2, displayCanvas.height / 2, 1.0 / view.zoom);
  }

  function zoomIn() {
    zoomAt(displayCanvas.width / 2, displayCanvas.height / 2, 1.25);
  }

  function zoomOut() {
    zoomAt(displayCanvas.width / 2, displayCanvas.height / 2, 0.8);
  }

  function zoomAt(sx, sy, factor) {
    const newZoom = Math.max(0.1, Math.min(25, view.zoom * factor));
    view.x = sx - (sx - view.x) * (newZoom / view.zoom);
    view.y = sy - (sy - view.y) * (newZoom / view.zoom);
    view.zoom = newZoom;
    document.getElementById('zoom-level').textContent = Math.round(view.zoom * 100);
    draw();
  }

  function focusItem(worldX, worldY, zoomFactor = 2.8) {
    const targetZoom = Math.max(view.zoom, zoomFactor);
    view.zoom = targetZoom;
    const p = worldToPixel(worldX, worldY);
    view.x = displayCanvas.width / 2 - p.x * view.zoom;
    view.y = displayCanvas.height / 2 - p.y * view.zoom;
    document.getElementById('zoom-level').textContent = Math.round(view.zoom * 100);
    draw();
  }

  function switchTab(tabId) {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
    const targetTab = document.getElementById(`tab-${tabId}`);
    if (targetTab) targetTab.classList.add('active');

    // Encuentra el botón que corresponde al switchTab
    const btns = document.querySelectorAll('.tab-btn');
    btns.forEach(b => {
      if (b.getAttribute('onclick') && b.getAttribute('onclick').includes(`'${tabId}'`)) {
        b.classList.add('active');
      }
    });

    if (tabId === 'cleaning') {
      setMode('pan');
    }
  }

  function setMode(mode) {
    currentMode = mode;
    activeTemplate = null;
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    const btn = document.getElementById(`btn-${mode}`);
    if (btn) btn.classList.add('active');
    pendingGeometry = null;

    const canvas = document.getElementById('mapCanvas');
    if (mode === 'pan') {
      canvas.className = 'mode-pan';
    } else {
      canvas.className = '';
    }

    const instructions = {
      pan: '💡 <b>Modo Navegar:</b> Arrastra con el ratón para moverte y usa la rueda para hacer zoom.',
      erase: '🧹 <b>Goma de Borrar:</b> Pinta en blanco (espacio libre) para eliminar puntos de ruido o personas.',
      clear_box: '📦 <b>Limpiar Rectángulo:</b> Haz clic y arrastra un rectángulo para vaciar toda el área a espacio libre.',
      obstacle: '✏️ <b>Lápiz de Pared:</b> Pinta en negro obstáculos o paredes faltantes a mano alzada.',
      line_wall: '📏 <b>Línea de Pared:</b> Haz clic en el punto inicial y arrastra para trazar una pared recta.',
      unknown: '🌫️ <b>Pincel Gris:</b> Pinta en gris espacio desconocido fuera de los límites de la habitación.',
      draw_vol_box: '📦 <b>Caja Volumétrica 3D:</b> Arrastra una caja sobre el mapa para definir ancho y largo físico.',
      draw_vol_poly: '📐 <b>Polígono Volumétrico 3D:</b> Haz clics para marcar los vértices de la base. Doble clic para terminar.',
      zone: '📐 <b>Dibujar Zona Libre:</b> Haz clic para agregar vértices al polígono. Doble clic para cerrar.',
      waypoint: '📍 <b>Colocar Waypoint:</b> Haz clic donde irá el robot y arrastra la flecha hacia donde mirará (Yaw).',
      keepout: '🚫 <b>Zona Prohibida:</b> Marca el polígono de área restringida donde el robot no debe entrar.',
      place_object: '🎯 <b>Colocar Objeto:</b> Haz clic sobre la mesa para situar el objeto (o arrastra para definir la posición de aproximación).'
    };
    document.getElementById('hud-instructions').innerHTML = instructions[mode] || '';
    draw();
  }

  // --- Activadores de Opciones y Plantillas Volumétricas ---
  function placeVolumeTemplate(targetCollection, name, type, heightCm, solidify, armGesture = 'none', capacity = 1) {
    activeTemplate = { targetCollection, name, type, heightCm, solidify, armGesture, capacity };
    currentMode = 'draw_vol_box';
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active-tpl');
    const icon = targetCollection === 'seat' ? '🛋️' : '🪑';
    document.getElementById('hud-instructions').innerHTML =
      `${icon} <b>Opción '${name}' (${heightCm}cm):</b> Haz clic y arrastra sobre el mapa para delimitar el volumen 3D (ancho × largo).`;
    draw();
  }

  function startFreeVolumeBox(targetCollection) {
    const isSeat = targetCollection === 'seat';
    activeTemplate = {
      targetCollection,
      name: isSeat ? 'Nuevo Asiento' : 'Nueva Mesa',
      type: isSeat ? 'couch' : 'table',
      heightCm: isSeat ? 85 : 76,
      solidify: true,
      armGesture: isSeat ? 'point_couch' : 'none',
      capacity: isSeat ? 2 : 1
    };
    currentMode = 'draw_vol_box';
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active');
    document.getElementById('hud-instructions').innerHTML =
      `📦 <b>Caja Volumétrica 3D:</b> Haz clic y arrastra sobre el mapa para delimitar las dimensiones físicas del elemento.`;
    draw();
  }

  function startFreeVolumePoly(targetCollection) {
    const isSeat = targetCollection === 'seat';
    activeTemplate = {
      targetCollection,
      name: isSeat ? 'Nuevo Asiento' : 'Nueva Mesa',
      type: isSeat ? 'couch' : 'table',
      heightCm: isSeat ? 85 : 76,
      solidify: true,
      armGesture: isSeat ? 'point_couch' : 'none',
      capacity: isSeat ? 2 : 1
    };
    currentMode = 'draw_vol_poly';
    pendingGeometry = [];
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active');
    document.getElementById('hud-instructions').innerHTML =
      `📐 <b>Polígono Volumétrico 3D:</b> Haz clics para marcar los vértices de la base del elemento. Doble clic para cerrar.`;
    draw();
  }

  function placeWaypointTemplate(name, zone) {
    activeTemplate = { category: 'waypoint', name, zone };
    currentMode = 'place_wp_template';
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active-tpl');
    document.getElementById('hud-instructions').innerHTML =
      `📍 <b>Plantilla '${name}':</b> Haz clic en el punto deseado y arrastra la flecha indicando hacia dónde mirará el robot (Yaw).`;
    draw();
  }

  function placeZoneTemplate(name, type, color) {
    activeTemplate = { category: 'zone', name, type, color };
    currentMode = 'place_zone_template';
    pendingGeometry = [];
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active-tpl');
    document.getElementById('hud-instructions').innerHTML =
      `📐 <b>Plantilla '${name}':</b> Haz clics sucesivos sobre el mapa para marcar los límites de la habitación. Doble clic para terminar.`;
    draw();
  }

  function placeObjectTemplate(name, objClass, label) {
    activeTemplate = { category: 'object', name, objClass, label };
    currentMode = 'place_object';
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    if (window.event && window.event.currentTarget) window.event.currentTarget.classList.add('active-tpl');
    document.getElementById('hud-instructions').innerHTML =
      `🎯 <b>Plantilla '${label}':</b> Haz clic sobre la mesa para colocarlo (o arrastra para fijar la posición de aproximación).`;
    draw();
  }

  function startPlaceObject() {
    activeTemplate = { category: 'object', name: 'pringles', objClass: 'can', label: 'Nuevo Objeto' };
    currentMode = 'place_object';
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.template-card').forEach(c => c.classList.remove('active-tpl'));
    const btn = document.getElementById('btn-place_object');
    if (btn) btn.classList.add('active');
    document.getElementById('hud-instructions').innerHTML =
      `🎯 <b>Colocar Objeto:</b> Haz clic en el mapa sobre la mesa para situarlo (o arrastra para definir la aproximación del robot).`;
    draw();
  }

  function updateBrushSize(val) {
    brushSize = parseInt(val, 10);
    document.getElementById('brush-size-val').textContent = `${brushSize} px`;
    document.getElementById('brush-meters-val').textContent = `${(brushSize * mapInfo.resolution * 100).toFixed(0)} cm`;
    document.getElementById('brush-slider').value = brushSize;
    document.querySelectorAll('.preset-btn').forEach(btn => {
      btn.classList.toggle('active', btn.textContent.startsWith(`${brushSize}px`));
    });
  }

  function applyPixelBrush(px, py, colorVal) {
    rawCtx.fillStyle = `rgb(${colorVal}, ${colorVal}, ${colorVal})`;
    rawCtx.beginPath();
    rawCtx.arc(px, py, brushSize / 2, 0, Math.PI * 2);
    rawCtx.fill();
  }

  function filterItemList(listId, query) {
    const list = document.getElementById(listId);
    if (!list) return;
    const q = (query || '').toLowerCase().trim();
    const cards = list.querySelectorAll('.item-card');
    cards.forEach(card => {
      const text = card.textContent.toLowerCase();
      card.style.display = text.includes(q) ? 'flex' : 'none';
    });
  }

  // --- Renderizado del Mapa y Capas Semánticas ---
  function draw() {
    ctx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);

    ctx.save();
    ctx.translate(view.x, view.y);
    ctx.scale(view.zoom, view.zoom);

    // 1. Capa Mapa PGM
    if (document.getElementById('layer-map').checked) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(colorCanvas, 0, 0);
    }

    // 2. Capa Zonas Prohibidas (Keep-out)
    if (document.getElementById('layer-keepout').checked) {
      for (const [id, kz] of Object.entries(keepout_zones)) {
        if (!kz.polygon || kz.polygon.length < 3) continue;
        ctx.beginPath();
        const p0 = worldToPixel(kz.polygon[0][0], kz.polygon[0][1]);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < kz.polygon.length; i++) {
          const p = worldToPixel(kz.polygon[i][0], kz.polygon[i][1]);
          ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = 'rgba(255, 51, 102, 0.25)';
        ctx.fill();
        ctx.strokeStyle = '#ff3366';
        ctx.lineWidth = 2 / view.zoom;
        ctx.stroke();
      }
    }

    // 3. Capa Zonas Semánticas
    if (document.getElementById('layer-zones').checked) {
      for (const [id, zone] of Object.entries(zones)) {
        if (!zone.polygon || zone.polygon.length < 3) continue;
        ctx.beginPath();
        const p0 = worldToPixel(zone.polygon[0][0], zone.polygon[0][1]);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < zone.polygon.length; i++) {
          const p = worldToPixel(zone.polygon[i][0], zone.polygon[i][1]);
          ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = hexToRgba(zone.color || '#00d2ff', 0.22);
        ctx.fill();
        ctx.strokeStyle = zone.color || '#00d2ff';
        ctx.lineWidth = 2 / view.zoom;
        ctx.stroke();

        const center = getPolygonCenter(zone.polygon);
        const cp = worldToPixel(center.x, center.y);
        drawBadge(cp.x, cp.y, zone.name || id, zone.color || '#00d2ff');
      }
    }

    // 4. Capa Mesas y Mobiliario Volumétrico
    if (document.getElementById('layer-furniture').checked) {
      for (const [id, item] of Object.entries(furniture)) {
        if (item.polygon && item.polygon.length >= 3) {
          drawVolumetricExtrusion(item.polygon, item.height_m, '#ff9d00', item.name || id, '🪑', item.solidified_in_map);
        }
      }
    }

    // 5. Capa Waypoints Nav2
    if (document.getElementById('layer-waypoints').checked) {
      for (const [name, wp] of Object.entries(waypoints)) {
        const p = worldToPixel(wp.x, wp.y);
        drawWaypointPin(p.x, p.y, wp.yaw_deg || 0, name);
      }
    }

    // 6. Capa Asientos Volumétricos Receptionist
    if (document.getElementById('layer-seats').checked) {
      for (const [id, seat] of Object.entries(seats)) {
        if (seat.polygon && seat.polygon.length >= 3) {
          const icon = seat.type === 'couch' ? '🛋️' : '🪑';
          drawVolumetricExtrusion(seat.polygon, seat.height_m || 0.85, '#b388ff', seat.name || id, icon, seat.solidified_in_map);
        } else if (seat.x !== undefined && seat.y !== undefined) {
          const p = worldToPixel(seat.x, seat.y);
          drawSeatPin(p.x, p.y, seat.name || id, seat.arm_gesture || '');
        }
      }
    }

    // 7. Capa Objetos Manipulables
    const layerObjCheck = document.getElementById('layer-objects');
    if (layerObjCheck && layerObjCheck.checked) {
      for (const [id, obj] of Object.entries(objects)) {
        if (obj.x !== undefined && obj.y !== undefined) {
          const pObj = worldToPixel(obj.x, obj.y);
          let appP = null;
          let yawDeg = 0;
          if (obj.approach && obj.approach.x !== undefined && obj.approach.y !== undefined) {
            appP = worldToPixel(obj.approach.x, obj.approach.y);
            yawDeg = obj.approach.yaw_deg || 0;
          } else {
            appP = { x: pObj.x + 20 / view.zoom, y: pObj.y };
          }
          drawObjectMarker(pObj.x, pObj.y, obj.number || 1, obj.name || id, obj.class || 'other', appP.x, appP.y, yawDeg);
        }
      }
    }

    // 8. Previsualizaciones en Vivo
    if (isMouseDown && dragStartPixel) {
      if (currentMode === 'clear_box') {
        const minX = Math.min(dragStartPixel.x, mousePixel.x);
        const minY = Math.min(dragStartPixel.y, mousePixel.y);
        const w = Math.abs(dragStartPixel.x - mousePixel.x);
        const h = Math.abs(dragStartPixel.y - mousePixel.y);

        ctx.strokeStyle = '#00f090';
        ctx.lineWidth = 2 / view.zoom;
        ctx.setLineDash([4 / view.zoom, 4 / view.zoom]);
        ctx.strokeRect(minX, minY, w, h);
        ctx.setLineDash([]);

        const wm = (w * mapInfo.resolution).toFixed(2);
        const hm = (h * mapInfo.resolution).toFixed(2);
        drawBadge(minX + w / 2, minY - 10 / view.zoom, `${wm}m x ${hm}m`, '#00f090');
      } else if (currentMode === 'draw_vol_box') {
        const minX = Math.min(dragStartPixel.x, mousePixel.x);
        const minY = Math.min(dragStartPixel.y, mousePixel.y);
        const w = Math.abs(dragStartPixel.x - mousePixel.x);
        const h = Math.abs(dragStartPixel.y - mousePixel.y);

        const wM = (w * mapInfo.resolution);
        const lM = (h * mapInfo.resolution);
        const hCm = activeTemplate && activeTemplate.heightCm ? activeTemplate.heightCm : 76;
        const hM = hCm / 100.0;
        const volM3 = (wM * lM * hM);

        const isSeat = activeTemplate && activeTemplate.targetCollection === 'seat';
        const color = isSeat ? '#b388ff' : '#ff9d00';

        const hPx = (hM / mapInfo.resolution);
        const isoDx = hPx * 0.28;
        const isoDy = -hPx * 0.42;

        // Base 2D en suelo
        ctx.strokeStyle = color;
        ctx.fillStyle = hexToRgba(color, 0.2);
        ctx.lineWidth = 2 / view.zoom;
        ctx.strokeRect(minX, minY, w, h);
        ctx.fillRect(minX, minY, w, h);

        // Aristas 3D
        ctx.beginPath();
        ctx.moveTo(minX, minY); ctx.lineTo(minX + isoDx, minY + isoDy);
        ctx.moveTo(minX + w, minY); ctx.lineTo(minX + w + isoDx, minY + isoDy);
        ctx.moveTo(minX + w, minY + h); ctx.lineTo(minX + w + isoDx, minY + h + isoDy);
        ctx.moveTo(minX, minY + h); ctx.lineTo(minX + isoDx, minY + h + isoDy);
        ctx.strokeStyle = hexToRgba(color, 0.6);
        ctx.lineWidth = 1.2 / view.zoom;
        ctx.setLineDash([3 / view.zoom, 3 / view.zoom]);
        ctx.stroke();
        ctx.setLineDash([]);

        // Tapa superior 3D
        ctx.fillStyle = hexToRgba(color, 0.4);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.2 / view.zoom;
        ctx.fillRect(minX + isoDx, minY + isoDy, w, h);
        ctx.strokeRect(minX + isoDx, minY + isoDy, w, h);

        drawBadge(
          minX + w / 2 + isoDx,
          minY - 14 / view.zoom + isoDy,
          `📦 ${wM.toFixed(2)}m × ${lM.toFixed(2)}m × ${hM.toFixed(2)}m (${volM3.toFixed(2)}m³)`,
          color
        );
      } else if (currentMode === 'line_wall') {
        ctx.beginPath();
        ctx.moveTo(dragStartPixel.x, dragStartPixel.y);
        ctx.lineTo(mousePixel.x, mousePixel.y);
        ctx.strokeStyle = '#080c13';
        ctx.lineWidth = brushSize;
        ctx.lineCap = 'round';
        ctx.stroke();

        const distPx = Math.hypot(mousePixel.x - dragStartPixel.x, mousePixel.y - dragStartPixel.y);
        const distM = (distPx * mapInfo.resolution).toFixed(2);
        drawBadge((dragStartPixel.x + mousePixel.x) / 2, (dragStartPixel.y + mousePixel.y) / 2 - 12 / view.zoom, `${distM} m`, '#38bdf8');
      } else if (currentMode === 'waypoint' || currentMode === 'place_wp_template') {
        const dx = mousePixel.x - dragStartPixel.x;
        const dy = mousePixel.y - dragStartPixel.y;
        const rad = Math.atan2(-dy, dx);
        const yawDeg = Math.round((rad * 180) / Math.PI);
        const name = activeTemplate && activeTemplate.name ? activeTemplate.name : 'waypoint';
        drawWaypointPin(dragStartPixel.x, dragStartPixel.y, yawDeg, name);
      }
    }

    // Polígonos en Construcción (Zonas, Keep-out, Volúmenes)
    if (pendingGeometry && pendingGeometry.length > 0) {
      let polyColor = '#00d2ff';
      if (currentMode === 'keepout') polyColor = '#ff3366';
      else if (currentMode === 'draw_vol_poly') polyColor = '#ff9d00';

      ctx.beginPath();
      const p0 = worldToPixel(pendingGeometry[0][0], pendingGeometry[0][1]);
      ctx.moveTo(p0.x, p0.y);
      for (let i = 1; i < pendingGeometry.length; i++) {
        const p = worldToPixel(pendingGeometry[i][0], pendingGeometry[i][1]);
        ctx.lineTo(p.x, p.y);
      }
      ctx.lineTo(mousePixel.x, mousePixel.y);
      ctx.strokeStyle = polyColor;
      ctx.lineWidth = 2 / view.zoom;
      ctx.setLineDash([4 / view.zoom, 4 / view.zoom]);
      ctx.stroke();
      ctx.setLineDash([]);

      for (let i = 0; i < pendingGeometry.length; i++) {
        const p = worldToPixel(pendingGeometry[i][0], pendingGeometry[i][1]);
        ctx.fillStyle = polyColor;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 4 / view.zoom, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // Previsualización de Cursor en Modos de Pincel
    if (['erase', 'obstacle', 'unknown'].includes(currentMode)) {
      ctx.beginPath();
      ctx.arc(mousePixel.x, mousePixel.y, brushSize / 2, 0, Math.PI * 2);
      ctx.strokeStyle = currentMode === 'erase' ? '#10b981' : (currentMode === 'obstacle' ? '#f43f5e' : '#94a3b8');
      ctx.lineWidth = 1.5 / view.zoom;
      ctx.stroke();
    }

    ctx.restore();
  }

  function drawVolumetricExtrusion(polyWorld, heightM, color, label, icon, isSolidified) {
    if (!polyWorld || polyWorld.length < 3) return;
    const basePts = polyWorld.map(pt => worldToPixel(pt[0], pt[1]));
    const hM = heightM || 0.76;
    const hPx = (hM / mapInfo.resolution);
    const isoDx = hPx * 0.28;
    const isoDy = -hPx * 0.42;
    const topPts = basePts.map(p => ({ x: p.x + isoDx, y: p.y + isoDy }));

    // 1. Huella base en el suelo (footprint 2D)
    ctx.beginPath();
    ctx.moveTo(basePts[0].x, basePts[0].y);
    for (let i = 1; i < basePts.length; i++) ctx.lineTo(basePts[i].x, basePts[i].y);
    ctx.closePath();
    ctx.fillStyle = hexToRgba(color, 0.20);
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.8 / view.zoom;
    ctx.stroke();

    // 2. Pilares verticales (aristas de extrusión 3D)
    ctx.beginPath();
    for (let i = 0; i < basePts.length; i++) {
      ctx.moveTo(basePts[i].x, basePts[i].y);
      ctx.lineTo(topPts[i].x, topPts[i].y);
    }
    ctx.strokeStyle = hexToRgba(color, 0.6);
    ctx.lineWidth = 1.2 / view.zoom;
    ctx.setLineDash([3 / view.zoom, 3 / view.zoom]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Cara superior 3D (tapa elevada del volumen)
    ctx.beginPath();
    ctx.moveTo(topPts[0].x, topPts[0].y);
    for (let i = 1; i < topPts.length; i++) ctx.lineTo(topPts[i].x, topPts[i].y);
    ctx.closePath();
    ctx.fillStyle = hexToRgba(color, 0.42);
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2 / view.zoom;
    ctx.stroke();

    // 4. Puntos de anclaje en las esquinas superiores
    ctx.fillStyle = color;
    for (const p of topPts) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3.5 / view.zoom, 0, Math.PI * 2);
      ctx.fill();
    }

    // 5. Etiqueta flotante 3D sobre la tapa superior
    let topCx = 0, topCy = 0;
    for (const p of topPts) { topCx += p.x; topCy += p.y; }
    topCx /= topPts.length;
    topCy /= topPts.length;

    const hLabel = `${Math.round(hM * 100)}cm`;
    const solidBadge = isSolidified ? ' ⚡' : '';
    drawBadge(topCx, topCy, `${icon} ${label} (${hLabel})${solidBadge}`, color);
  }

  function drawBadge(x, y, text, color) {
    const fontSize = Math.max(7.5, Math.min(10, 8.5 / Math.sqrt(view.zoom)));
    ctx.font = `600 ${fontSize}px sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    ctx.save();
    ctx.shadowColor = 'rgba(0, 0, 0, 0.9)';
    ctx.shadowBlur = 2;
    ctx.strokeStyle = 'rgba(8, 12, 18, 0.85)';
    ctx.lineWidth = 2.0 / view.zoom;
    ctx.strokeText(text, x, y);

    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
    ctx.restore();
  }

  function drawWaypointPin(px, py, yawDeg, label) {
    const r = 8 / view.zoom;
    ctx.fillStyle = '#00f090';
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#080c13';
    ctx.lineWidth = 2 / view.zoom;
    ctx.stroke();

    const rad = (-yawDeg * Math.PI) / 180;
    const len = 20 / view.zoom;
    const ex = px + len * Math.cos(rad);
    const ey = py + len * Math.sin(rad);

    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = '#00f090';
    ctx.lineWidth = 3 / view.zoom;
    ctx.stroke();

    drawBadge(px, py - 14 / view.zoom, label, '#00f090');
  }

  function drawSeatPin(px, py, label, gesture) {
    ctx.fillStyle = '#b388ff';
    ctx.beginPath();
    ctx.arc(px, py, 7 / view.zoom, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5 / view.zoom;
    ctx.stroke();

    drawBadge(px, py - 14 / view.zoom, `🛋️ ${label}`, '#b388ff');
  }

  function drawObjectMarker(px, py, number, name, objClass, appPx, appPy, yawDeg) {
    // 1. Línea punteada desde approach hacia el objeto
    ctx.beginPath();
    ctx.moveTo(appPx, appPy);
    ctx.lineTo(px, py);
    ctx.strokeStyle = '#ff9d00';
    ctx.lineWidth = 1.5 / view.zoom;
    ctx.setLineDash([4 / view.zoom, 3 / view.zoom]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 2. Marcador de approach
    const appR = 5 / view.zoom;
    ctx.beginPath();
    ctx.arc(appPx, appPy, appR, 0, Math.PI * 2);
    ctx.fillStyle = '#00f090';
    ctx.fill();
    ctx.strokeStyle = '#080c13';
    ctx.lineWidth = 1.5 / view.zoom;
    ctx.stroke();

    // Flecha de orientación (Yaw)
    const rad = (-yawDeg * Math.PI) / 180;
    const arrowLen = 14 / view.zoom;
    const ex = appPx + arrowLen * Math.cos(rad);
    const ey = appPy + arrowLen * Math.sin(rad);
    ctx.beginPath();
    ctx.moveTo(appPx, appPy);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = '#00f090';
    ctx.lineWidth = 2.5 / view.zoom;
    ctx.stroke();

    // 3. Círculo naranja con el número
    const r = 11 / view.zoom;
    ctx.beginPath();
    ctx.arc(px, py, r, 0, Math.PI * 2);
    ctx.fillStyle = '#ff7700';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2 / view.zoom;
    ctx.stroke();

    ctx.save();
    ctx.font = `bold ${Math.max(9, Math.round(11 / view.zoom))}px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif`;
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${number}`, px, py);
    ctx.restore();

    drawBadge(px, py - 16 / view.zoom, `${name} (${objClass})`, '#ff7700');
  }

  function getPolygonCenter(pts) {
    let sx = 0, sy = 0;
    for (const p of pts) { sx += p[0]; sy += p[1]; }
    return { x: sx / pts.length, y: sy / pts.length };
  }

  function hexToRgba(hex, alpha) {
    const r = parseInt(hex.slice(1, 3), 16) || 0;
    const g = parseInt(hex.slice(3, 5), 16) || 0;
    const b = parseInt(hex.slice(5, 7), 16) || 0;
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  // --- Manejo de Eventos del Ratón ---
  displayCanvas.addEventListener('mousedown', (e) => {
    isMouseDown = true;
    const rect = displayCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    dragStartPixel = screenToPixel(sx, sy);
    mousePixel = { ...dragStartPixel };

    if (e.button === 1) {
      return; // Middle click para pan
    }

    if (currentMode === 'erase') {
      applyPixelBrush(dragStartPixel.x, dragStartPixel.y, 254);
      updateColorCanvas();
      draw();
    } else if (currentMode === 'obstacle') {
      applyPixelBrush(dragStartPixel.x, dragStartPixel.y, 0);
      updateColorCanvas();
      draw();
    } else if (currentMode === 'unknown') {
      applyPixelBrush(dragStartPixel.x, dragStartPixel.y, 205);
      updateColorCanvas();
      draw();
    }
  });

  displayCanvas.addEventListener('mousemove', (e) => {
    const rect = displayCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const prevMousePixel = { ...mousePixel };
    mousePixel = screenToPixel(sx, sy);

    const world = pixelToWorld(mousePixel.x, mousePixel.y);
    document.getElementById('hud-x').textContent = world.x.toFixed(2);
    document.getElementById('hud-y').textContent = world.y.toFixed(2);
    document.getElementById('hud-px').textContent = Math.round(mousePixel.x);
    document.getElementById('hud-py').textContent = Math.round(mousePixel.y);

    if (isMouseDown) {
      if (currentMode === 'pan' || e.buttons === 4) {
        view.x += e.movementX;
        view.y += e.movementY;
        draw();
      } else if (currentMode === 'erase') {
        applyPixelBrush(mousePixel.x, mousePixel.y, 254);
        updateColorCanvas();
        draw();
      } else if (currentMode === 'obstacle') {
        applyPixelBrush(mousePixel.x, mousePixel.y, 0);
        updateColorCanvas();
        draw();
      } else if (currentMode === 'unknown') {
        applyPixelBrush(mousePixel.x, mousePixel.y, 205);
        updateColorCanvas();
        draw();
      } else if (['clear_box', 'line_wall', 'draw_vol_box', 'waypoint', 'place_wp_template'].includes(currentMode)) {
        draw();
      }
    } else {
      if (['erase', 'obstacle', 'unknown', 'draw_vol_poly', 'zone', 'keepout', 'place_zone_template'].includes(currentMode)) {
        draw();
      }
    }
  });

  window.addEventListener('mouseup', (e) => {
    if (!isMouseDown) return;
    isMouseDown = false;

    if (currentMode === 'clear_box' && dragStartPixel) {
      const minX = Math.min(dragStartPixel.x, mousePixel.x);
      const minY = Math.min(dragStartPixel.y, mousePixel.y);
      const w = Math.abs(dragStartPixel.x - mousePixel.x);
      const h = Math.abs(dragStartPixel.y - mousePixel.y);

      if (w > 2 && h > 2) {
        rawCtx.fillStyle = 'rgb(254, 254, 254)';
        rawCtx.fillRect(minX, minY, w, h);
        updateColorCanvas();
        pushUndo();
        showToast('Área rectangular limpiada');
      }
      dragStartPixel = null;
      draw();
    } else if (currentMode === 'line_wall' && dragStartPixel) {
      const dist = Math.hypot(mousePixel.x - dragStartPixel.x, mousePixel.y - dragStartPixel.y);
      if (dist > 2) {
        rawCtx.beginPath();
        rawCtx.moveTo(dragStartPixel.x, dragStartPixel.y);
        rawCtx.lineTo(mousePixel.x, mousePixel.y);
        rawCtx.strokeStyle = 'rgb(0, 0, 0)';
        rawCtx.lineWidth = brushSize;
        rawCtx.lineCap = 'round';
        rawCtx.stroke();
        updateColorCanvas();
        pushUndo();
        showToast('Pared recta añadida');
      }
      dragStartPixel = null;
      draw();
    } else if (currentMode === 'draw_vol_box' && dragStartPixel) {
      const minPxX = Math.min(dragStartPixel.x, mousePixel.x);
      const maxPxX = Math.max(dragStartPixel.x, mousePixel.x);
      const minPxY = Math.min(dragStartPixel.y, mousePixel.y);
      const maxPxY = Math.max(dragStartPixel.y, mousePixel.y);

      const p0 = pixelToWorld(minPxX, maxPxY); // bottom-left
      const p1 = pixelToWorld(maxPxX, maxPxY); // bottom-right
      const p2 = pixelToWorld(maxPxX, minPxY); // top-right
      const p3 = pixelToWorld(minPxX, minPxY); // top-left

      const wM = Math.hypot(p1.x - p0.x, p1.y - p0.y);
      const lM = Math.hypot(p3.x - p0.x, p3.y - p0.y);

      if (wM > 0.15 && lM > 0.15) {
        pendingVolumePolygon = [
          [round4(p0.x), round4(p0.y)],
          [round4(p1.x), round4(p1.y)],
          [round4(p2.x), round4(p2.y)],
          [round4(p3.x), round4(p3.y)]
        ];
        openVolumeModal();
      }
      dragStartPixel = null;
      draw();
    } else if ((currentMode === 'waypoint' || currentMode === 'place_wp_template') && dragStartPixel) {
      const dx = mousePixel.x - dragStartPixel.x;
      const dy = mousePixel.y - dragStartPixel.y;
      const rad = Math.atan2(-dy, dx);
      let yawDeg = Math.round((rad * 180) / Math.PI);
      if (Math.hypot(dx, dy) < 5) yawDeg = 0;

      const wpPos = pixelToWorld(dragStartPixel.x, dragStartPixel.y);
      let wpName = activeTemplate && activeTemplate.name ? activeTemplate.name : `punto_${Object.keys(waypoints).length + 1}`;

      if (currentMode === 'waypoint') {
        const inputName = prompt('Nombre identificador para este Waypoint Nav2:', wpName);
        if (inputName) wpName = inputName.trim();
        else { dragStartPixel = null; draw(); return; }
      }

      waypoints[wpName] = {
        name: wpName,
        x: round4(wpPos.x),
        y: round4(wpPos.y),
        yaw_deg: yawDeg,
        zone: activeTemplate && activeTemplate.zone ? activeTemplate.zone : 'default'
      };

      showToast(`✔ Waypoint '${wpName}' guardado (Yaw: ${yawDeg}°)`);
      dragStartPixel = null;
      setMode('pan');
      updateSidebar();
      setUnsaved(true);
      draw();
    } else if (['erase', 'obstacle', 'unknown'].includes(currentMode)) {
      pushUndo();
    }
  });

  displayCanvas.addEventListener('click', (e) => {
    if (['zone', 'keepout', 'draw_vol_poly', 'place_zone_template'].includes(currentMode)) {
      const rect = displayCanvas.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const pix = screenToPixel(sx, sy);
      const w = pixelToWorld(pix.x, pix.y);
      if (!pendingGeometry) pendingGeometry = [];
      pendingGeometry.push([round4(w.x), round4(w.y)]);
      draw();
    }
  });

  displayCanvas.addEventListener('dblclick', (e) => {
    if (['zone', 'keepout', 'place_zone_template'].includes(currentMode)) {
      if (pendingGeometry && pendingGeometry.length >= 3) {
        if (currentMode === 'keepout') {
          const id = `prohibido_${Date.now()}`;
          keepout_zones[id] = { name: 'Área Prohibida', polygon: pendingGeometry };
          showToast('🚫 Zona prohibida agregada');
        } else {
          let name = activeTemplate && activeTemplate.name ? activeTemplate.name : `Zona_${Object.keys(zones).length + 1}`;
          let color = activeTemplate && activeTemplate.color ? activeTemplate.color : '#00d2ff';
          if (currentMode === 'zone') {
            const inputName = prompt('Nombre de la zona semántica:', name);
            if (inputName) name = inputName.trim();
          }
          const id = name.toLowerCase().replace(/\s+/g, '_');
          zones[id] = { name, color, polygon: pendingGeometry, type: activeTemplate ? activeTemplate.type : 'general' };
          showToast(`📐 Zona '${name}' guardada`);
        }
        pendingGeometry = null;
        setMode('pan');
        updateSidebar();
        setUnsaved(true);
        draw();
      }
    } else if (currentMode === 'draw_vol_poly') {
      if (pendingGeometry && pendingGeometry.length >= 3) {
        pendingVolumePolygon = [...pendingGeometry];
        pendingGeometry = null;
        openVolumeModal();
      }
    }
  });

  displayCanvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = displayCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    const newZoom = Math.max(0.1, Math.min(25, view.zoom * zoomFactor));

    view.x = sx - (sx - view.x) * (newZoom / view.zoom);
    view.y = sy - (sy - view.y) * (newZoom / view.zoom);
    view.zoom = newZoom;
    document.getElementById('zoom-level').textContent = Math.round(view.zoom * 100);
    draw();
  });

  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

    if (e.ctrlKey && e.key === 'z') { e.preventDefault(); undo(); }
    else if (e.ctrlKey && (e.key === 'y' || (e.shiftKey && e.key === 'Z'))) { e.preventDefault(); redo(); }
    else if (e.ctrlKey && e.key === 's') { e.preventDefault(); saveAll(); }
    else if (e.key === '1') setMode('pan');
    else if (e.key === '2') setMode('erase');
    else if (e.key === '3') setMode('clear_box');
    else if (e.key === '4') setMode('obstacle');
    else if (e.key === '5') setMode('line_wall');
    else if (e.key === '6') setMode('unknown');
    else if (e.key === 'f' || e.key === 'F') resetView();
    else if (e.key === 'r' || e.key === 'R') setZoom100();
    else if (e.key === '?' || (e.shiftKey && e.key === '/')) openShortcutsModal();
    else if (e.key === 'Escape') {
      pendingGeometry = null;
      setMode('pan');
      closeVolumeModal();
      closeObjectModal();
      closeShortcutsModal();
    }
  });

  let pendingVolumePolygon = null;

  function openVolumeModal() {
    if (!pendingVolumePolygon || pendingVolumePolygon.length < 3) return;

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const pt of pendingVolumePolygon) {
      minX = Math.min(minX, pt[0]); maxX = Math.max(maxX, pt[0]);
      minY = Math.min(minY, pt[1]); maxY = Math.max(maxY, pt[1]);
    }
    const w = Math.max(0.1, maxX - minX);
    const l = Math.max(0.1, maxY - minY);
    const h = parseInt(document.getElementById('vol-height-input').value, 10) / 100.0;
    const vol = w * l * h;

    document.getElementById('vol-metric-w').textContent = `${w.toFixed(2)} m`;
    document.getElementById('vol-metric-l').textContent = `${l.toFixed(2)} m`;
    document.getElementById('vol-metric-v').textContent = `${vol.toFixed(2)} m³`;

    const isSeat = activeTemplate && activeTemplate.targetCollection === 'seat';
    document.getElementById('vol-modal-title').innerHTML = isSeat ? '🛋️ Registrar Asiento Receptionist' : '📦 Registrar Mesa / Mobiliario 3D';
    document.getElementById('vol-name-input').value = activeTemplate ? activeTemplate.name : (isSeat ? 'Sofá' : 'Mesa de Centro');
    document.getElementById('vol-type-input').value = activeTemplate ? activeTemplate.type : (isSeat ? 'couch' : 'table');
    document.getElementById('vol-height-input').value = activeTemplate ? activeTemplate.heightCm : (isSeat ? 85 : 76);
    document.getElementById('vol-solidify-check').checked = activeTemplate ? activeTemplate.solidify : true;

    onVolTypeChange(document.getElementById('vol-type-input').value);
    document.getElementById('volume-modal').classList.add('active');
  }

  function updateModalVolume() {
    if (!pendingVolumePolygon) return;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const pt of pendingVolumePolygon) {
      minX = Math.min(minX, pt[0]); maxX = Math.max(maxX, pt[0]);
      minY = Math.min(minY, pt[1]); maxY = Math.max(maxY, pt[1]);
    }
    const w = Math.max(0.1, maxX - minX);
    const l = Math.max(0.1, maxY - minY);
    const hCm = parseInt(document.getElementById('vol-height-input').value, 10) || 76;
    const vol = (w * l * (hCm / 100.0));
    document.getElementById('vol-metric-v').textContent = `${vol.toFixed(2)} m³`;
  }

  function onVolTypeChange(val) {
    const isSeat = ['couch', 'armchair'].includes(val);
    const gestGroup = document.getElementById('vol-gesture-group');
    if (gestGroup) gestGroup.style.display = isSeat ? 'flex' : 'none';
  }

  function closeVolumeModal() {
    document.getElementById('volume-modal').classList.remove('active');
    pendingVolumePolygon = null;
    draw();
  }

  function confirmSaveVolume() {
    if (!pendingVolumePolygon) return;

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const pt of pendingVolumePolygon) {
      minX = Math.min(minX, pt[0]); maxX = Math.max(maxX, pt[0]);
      minY = Math.min(minY, pt[1]); maxY = Math.max(maxY, pt[1]);
    }
    const w = round4(Math.max(0.1, maxX - minX));
    const l = round4(Math.max(0.1, maxY - minY));
    const hCm = parseInt(document.getElementById('vol-height-input').value, 10) || 76;
    const hM = round4(hCm / 100.0);
    const volM3 = round4(w * l * hM);

    const name = document.getElementById('vol-name-input').value.trim() || 'Mesa';
    const type = document.getElementById('vol-type-input').value;
    const solidify = document.getElementById('vol-solidify-check').checked;
    const isSeat = ['couch', 'armchair'].includes(type);

    const id = name.toLowerCase().replace(/[^a-z0-9_]/g, '_').replace(/_+/g, '_');

    const itemData = {
      name: name,
      type: type,
      height_m: hM,
      polygon: pendingVolumePolygon,
      solidified_in_map: solidify,
      volume: { width_m: w, length_m: l, height_m: hM, volume_m3: volM3 }
    };

    if (isSeat) {
      itemData.arm_gesture = document.getElementById('vol-gesture-input').value;
      itemData.capacity = type === 'couch' ? 2 : 1;
      seats[id] = itemData;
    } else {
      furniture[id] = itemData;
    }

    if (solidify) {
      solidifyGeometry(pendingVolumePolygon, true);
    }

    showToast(`✔ Volumen '${name}' registrado exitosamente (${hCm}cm)`);
    closeVolumeModal();
    setMode('pan');
    updateSidebar();
    setUnsaved(true);
    draw();
  }

  function solidifyGeometry(polygon, shouldSolidify) {
    if (!polygon || polygon.length < 3) return;
    const ptsPx = polygon.map(p => worldToPixel(p[0], p[1]));

    rawCtx.beginPath();
    rawCtx.moveTo(ptsPx[0].x, ptsPx[0].y);
    for (let i = 1; i < ptsPx.length; i++) rawCtx.lineTo(ptsPx[i].x, ptsPx[i].y);
    ctx.closePath();

    rawCtx.fillStyle = shouldSolidify ? 'rgb(0, 0, 0)' : 'rgb(254, 254, 254)';
    rawCtx.fill();
    updateColorCanvas();
    pushUndo();
  }

  function toggleSolidify(collection, id) {
    const coll = collection === 'seats' ? seats : furniture;
    if (!coll[id] || !coll[id].polygon) return;

    coll[id].solidified_in_map = !coll[id].solidified_in_map;
    solidifyGeometry(coll[id].polygon, coll[id].solidified_in_map);
    showToast(coll[id].solidified_in_map ? '⚡ Solidificada en PGM' : 'Huella liberada en PGM');
    updateSidebar();
    setUnsaved(true);
    draw();
  }

  function deleteItem(collection, id) {
    if (confirm(`¿Eliminar '${id}' de tu mapa?`)) {
      if (collection === 'furniture') delete furniture[id];
      else if (collection === 'seats') delete seats[id];
      else if (collection === 'zones') delete zones[id];
      else if (collection === 'waypoints') delete waypoints[id];
      else if (collection === 'objects') delete objects[id];
      showToast('Elemento eliminado');
      updateSidebar();
      setUnsaved(true);
      draw();
    }
  }

  function updateSidebar() {
    // 1. Mesas
    const fList = document.getElementById('furniture-list');
    fList.innerHTML = '';
    const fKeys = Object.keys(furniture);
    document.getElementById('furniture-count').textContent = fKeys.length;
    document.getElementById('badge-furniture').textContent = fKeys.length;
    if (fKeys.length === 0) {
      fList.innerHTML = '<div class="empty-state">No hay mesas en tu mapa.<br>Selecciona una opción arriba y dibuja su volumen 3D.</div>';
    } else {
      for (const [id, f] of Object.entries(furniture)) {
        const card = document.createElement('div');
        card.className = 'item-card';
        const volDesc = f.volume ? `${f.volume.width_m}×${f.volume.length_m}m (${f.volume.volume_m3}m³)` : `${Math.round((f.height_m || 0.76) * 100)}cm`;
        const center = f.polygon && f.polygon.length >= 3 ? getPolygonCenter(f.polygon) : { x: 0, y: 0 };
        card.innerHTML = `
          <div class="item-info">
            <span class="item-title">🪑 ${f.name}</span>
            <span class="item-subtitle">${volDesc} | ${f.solidified_in_map ? '⚡ Solidificada' : 'Huella libre'}</span>
          </div>
          <div class="item-actions">
            <button class="icon-btn focus-btn" title="Centrar en mapa" onclick="focusItem(${center.x}, ${center.y})">🎯</button>
            <button class="icon-btn solidify" title="Solidificar/Despejar en PGM" onclick="toggleSolidify('furniture', '${id}')">⚡</button>
            <button class="icon-btn danger" title="Eliminar" onclick="deleteItem('furniture', '${id}')">🗑️</button>
          </div>
        `;
        fList.appendChild(card);
      }
    }

    // 2. Asientos
    const sList = document.getElementById('seats-list');
    sList.innerHTML = '';
    const sKeys = Object.keys(seats);
    document.getElementById('seat-count').textContent = sKeys.length;
    document.getElementById('badge-seats').textContent = sKeys.length;
    if (sKeys.length === 0) {
      sList.innerHTML = '<div class="empty-state">No hay asientos colocados.<br>Selecciona una opción arriba y arrastra la caja de su volumen.</div>';
    } else {
      for (const [id, s] of Object.entries(seats)) {
        const card = document.createElement('div');
        card.className = 'item-card';
        const volDesc = s.volume ? `${s.volume.width_m}×${s.volume.length_m}×${Math.round((s.height_m || 0.85)*100)}cm` : '';
        const center = s.polygon && s.polygon.length >= 3 ? getPolygonCenter(s.polygon) : { x: s.x || 0, y: s.y || 0 };
        card.innerHTML = `
          <div class="item-info">
            <span class="item-title">${s.type === 'couch' ? '🛋️' : '🪑'} ${s.name || id}</span>
            <span class="item-subtitle">${volDesc ? volDesc + ' | ' : ''}Gesto: ${s.arm_gesture || 'point_couch'} | ${s.solidified_in_map ? '⚡ PGM' : 'Libre'}</span>
          </div>
          <div class="item-actions">
            <button class="icon-btn focus-btn" title="Centrar en mapa" onclick="focusItem(${center.x}, ${center.y})">🎯</button>
            <button class="icon-btn solidify" title="Solidificar/Despejar en PGM" onclick="toggleSolidify('seats', '${id}')">⚡</button>
            <button class="icon-btn danger" title="Eliminar" onclick="deleteItem('seats', '${id}')">🗑️</button>
          </div>
        `;
        sList.appendChild(card);
      }
    }

    // 3. Objetos
    const oList = document.getElementById('objects-list');
    if (oList) {
      oList.innerHTML = '';
      const oKeys = Object.keys(objects);
      document.getElementById('object-count').textContent = oKeys.length;
      document.getElementById('badge-objects').textContent = oKeys.length;
      if (oKeys.length === 0) {
        oList.innerHTML = '<div class="empty-state">No hay objetos colocados todavía.<br>Selecciona una plantilla o herramienta arriba y haz clic en el mapa.</div>';
      } else {
        const sortedObjects = Object.entries(objects).sort((a, b) => (a[1].number || 0) - (b[1].number || 0));
        for (const [id, obj] of sortedObjects) {
          const card = document.createElement('div');
          card.className = 'item-card';
          const iconMap = { can: '🥫', bottle: '🍾', cup: '☕', box: '📦', other: '🎯' };
          const icon = iconMap[obj.class] || '🎯';
          const onDesc = obj.on ? `Mesa: ${obj.on}` : 'Suelo';
          const appDesc = obj.approach ? `Approach (${obj.approach.x.toFixed(2)}, ${obj.approach.y.toFixed(2)}, ${obj.approach.yaw_deg}°)` : '';
          card.innerHTML = `
            <div class="item-info">
              <span class="item-title">${icon} [#${obj.number}] ${obj.name}</span>
              <span class="item-subtitle">${obj.class} | En ${onDesc} | Z: ${obj.z}m<br><span style="font-size: 10px; color: var(--accent-cyan);">${appDesc}</span></span>
            </div>
            <div class="item-actions">
              <button class="icon-btn focus-btn" title="Centrar en mapa" onclick="focusItem(${obj.x}, ${obj.y})">🎯</button>
              <button class="icon-btn" title="Editar Objeto" onclick="editObject('${id}')">✏️</button>
              <button class="icon-btn danger" title="Eliminar" onclick="deleteItem('objects', '${id}')">🗑️</button>
            </div>
          `;
          oList.appendChild(card);
        }
      }
    }

    // 4. Zonas
    const zList = document.getElementById('zones-list');
    zList.innerHTML = '';
    const zKeys = Object.keys(zones);
    document.getElementById('zone-count').textContent = zKeys.length;
    if (zKeys.length === 0) {
      zList.innerHTML = '<div class="empty-state">No hay zonas delimitadas todavía.</div>';
    } else {
      for (const [id, z] of Object.entries(zones)) {
        const card = document.createElement('div');
        card.className = 'item-card';
        const center = z.polygon && z.polygon.length >= 3 ? getPolygonCenter(z.polygon) : { x: 0, y: 0 };
        card.innerHTML = `
          <div class="item-info">
            <span class="item-title"><span style="color: ${z.color};">■</span> ${z.name || id}</span>
            <span class="item-subtitle">${z.type || 'zone'} (${z.polygon ? z.polygon.length : 0} vértices)</span>
          </div>
          <div class="item-actions">
            <button class="icon-btn focus-btn" title="Centrar en mapa" onclick="focusItem(${center.x}, ${center.y})">🎯</button>
            <button class="icon-btn danger" onclick="deleteItem('zones', '${id}')">🗑️</button>
          </div>
        `;
        zList.appendChild(card);
      }
    }

    // 5. Waypoints
    const wList = document.getElementById('waypoints-list');
    wList.innerHTML = '';
    const wKeys = Object.keys(waypoints);
    document.getElementById('wp-count').textContent = wKeys.length;
    document.getElementById('badge-zones').textContent = zKeys.length + wKeys.length;
    if (wKeys.length === 0) {
      wList.innerHTML = '<div class="empty-state">No hay waypoints colocados todavía.</div>';
    } else {
      for (const [name, wp] of Object.entries(waypoints)) {
        const card = document.createElement('div');
        card.className = 'item-card';
        card.innerHTML = `
          <div class="item-info">
            <span class="item-title">📍 ${name}</span>
            <span class="item-subtitle">X:${wp.x.toFixed(2)}, Y:${wp.y.toFixed(2)} | Yaw:${wp.yaw_deg || 0}°</span>
          </div>
          <div class="item-actions">
            <button class="icon-btn focus-btn" title="Centrar en mapa" onclick="focusItem(${wp.x}, ${wp.y})">🎯</button>
            <button class="icon-btn danger" onclick="deleteItem('waypoints', '${name}')">🗑️</button>
          </div>
        `;
        wList.appendChild(card);
      }
    }
  }

  async function saveAll() {
    const saveBtn = document.getElementById('btn-save-all');
    if (saveBtn) {
      saveBtn.classList.add('saving');
      saveBtn.innerHTML = '<span>⏳ Guardando...</span>';
    }
    showToast('Guardando cambios en disco...');
    try {
      const pgmBase64 = rawCanvas.toDataURL('image/png');
      const imgRes = await fetch('/api/save_map_image', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ image_data: pgmBase64 })
      });
      const imgData = await imgRes.json();

      const semRes = await fetch('/api/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          zones: zones,
          furniture: furniture,
          keepout_zones: keepout_zones,
          waypoints: waypoints,
          seats: seats,
          objects: objects
        })
      });
      const semData = await semRes.json();

      if (imgData.status === 'ok' && semData.status === 'ok') {
        const nav = imgData.nav || {};
        const navOk = nav.state === 'reloaded' || nav.state === 'nav_off';
        showToast((navOk ? '✔ Guardado con éxito. ' : '⚠ Guardado, pero: ') + (nav.message || ''), navOk ? 'success' : 'warning');
        setUnsaved(false);
      } else {
        showToast('⚠ Error al guardar algunos archivos', 'error');
      }
    } catch (e) {
      console.error(e);
      showToast('❌ Error de conexión al guardar', 'error');
    } finally {
      if (saveBtn) {
        saveBtn.classList.remove('saving');
        saveBtn.innerHTML = '<span>💾 Guardar Todo (PGM + Semántico)</span><span class=\"btn-shortcut\">Ctrl+S</span>';
      }
    }
  }

  async function restoreOriginalMap() {
    if (confirm('¿Restaurar el mapa PGM a su versión original (.orig)? Todos los cambios de dibujo se revertirán.')) {
      const res = await fetch('/api/restore_backup', { method: 'POST' });
      const data = await res.json();
      if (data.status === 'ok') {
        showToast('✔ Mapa restaurado a su versión original', 'success');
        loadData();
        setUnsaved(false);
      } else {
        showToast('No se encontró respaldo original (.orig)', 'warning');
      }
    }
  }

  function round4(num) { return Math.round(num * 10000) / 10000; }
  function round3(num) { return Math.round(num * 1000) / 1000; }

  function jsPointInPolygon(x, y, polygon) {
    if (!polygon || polygon.length < 3) return false;
    let inside = false;
    const n = polygon.length;
    let j = n - 1;
    for (let i = 0; i < n; i++) {
      const xi = polygon[i][0], yi = polygon[i][1];
      const xj = polygon[j][0], yj = polygon[j][1];
      const intersect = ((yi > y) !== (yj > y)) && (x < ((xj - xi) * (y - yi)) / (yj - yi) + xi);
      if (intersect) inside = !inside;
      j = i;
    }
    return inside;
  }

  function jsFindContainingFurniture(x, y) {
    for (const [fid, f] of Object.entries(furniture)) {
      if (f.polygon && f.polygon.length >= 3 && jsPointInPolygon(x, y, f.polygon)) {
        return fid;
      }
    }
    return null;
  }

  function jsGetNextFreeNumber() {
    const used = new Set();
    for (const obj of Object.values(objects)) {
      if (obj.number !== undefined && obj.number !== null) {
        used.add(parseInt(obj.number, 10));
      }
    }
    let n = 1;
    while (used.has(n)) n++;
    return n;
  }

  function jsIsNumberUnique(num, ignoreId = null) {
    const target = parseInt(num, 10);
    for (const [oid, obj] of Object.entries(objects)) {
      if (ignoreId && oid === ignoreId) continue;
      if (parseInt(obj.number, 10) === target) return false;
    }
    return true;
  }

  function jsCalculateApproach(objX, objY, appX, appY) {
    const dx = objX - appX;
    const dy = objY - appY;
    const dist = Math.hypot(dx, dy);
    const yawDeg = Math.round((Math.atan2(dy, dx) * 180) / Math.PI);
    return {
      x: round4(appX),
      y: round4(appY),
      yaw_deg: yawDeg,
      dist_m: round3(dist)
    };
  }

  function jsTableApproach(objX, objY, tablePoly, standDist = 0.60) {
    let bestDist = Infinity;
    let bestProj = [objX, objY];
    let bestEdge = null;
    const n = tablePoly.length;

    for (let i = 0; i < n; i++) {
      const a = tablePoly[i];
      const b = tablePoly[(i + 1) % n];
      const ab = [b[0] - a[0], b[1] - a[1]];
      const abLenSq = ab[0] * ab[0] + ab[1] * ab[1];
      if (abLenSq === 0) continue;

      const ap = [objX - a[0], objY - a[1]];
      const t = Math.max(0.0, Math.min(1.0, (ap[0] * ab[0] + ap[1] * ab[1]) / abLenSq));
      const proj = [a[0] + t * ab[0], a[1] + t * ab[1]];
      const d = Math.hypot(objX - proj[0], objY - proj[1]);
      if (d < bestDist) {
        bestDist = d;
        bestProj = proj;
        bestEdge = [a, b];
      }
    }

    if (!bestEdge) return jsCalculateApproach(objX, objY, objX - standDist, objY);

    const a = bestEdge[0], b = bestEdge[1];
    const edgeDx = b[0] - a[0], edgeDy = b[1] - a[1];
    const edgeLen = Math.hypot(edgeDx, edgeDy);
    if (edgeLen === 0) return jsCalculateApproach(objX, objY, objX - standDist, objY);

    const normal = [-edgeDy / edgeLen, edgeDx / edgeLen];
    const cand1 = [bestProj[0] + normal[0] * standDist, bestProj[1] + normal[1] * standDist];
    const cand2 = [bestProj[0] - normal[0] * standDist, bestProj[1] - normal[1] * standDist];

    const in1 = jsPointInPolygon(cand1[0], cand1[1], tablePoly);
    const chosen = in1 ? cand2 : cand1;
    return jsCalculateApproach(objX, objY, chosen[0], chosen[1]);
  }

  function openObjectModal(objX, objY, manualApproach = null, existingId = null) {
    editingObjectId = existingId;
    const isEdit = Boolean(existingId);
    const existing = isEdit ? objects[existingId] : null;

    const tableName = jsFindContainingFurniture(objX, objY);
    let tableHeight = 0.76;
    if (tableName && furniture[tableName]) {
      tableHeight = furniture[tableName].height_m || 0.76;
    }

    let defaultApproach;
    if (manualApproach) {
      defaultApproach = manualApproach;
    } else if (tableName && furniture[tableName] && furniture[tableName].polygon) {
      defaultApproach = jsTableApproach(objX, objY, furniture[tableName].polygon, 0.60);
    } else {
      defaultApproach = jsCalculateApproach(objX, objY, objX - 0.60, objY);
    }

    const freeNum = isEdit ? existing.number : jsGetNextFreeNumber();

    pendingObject = {
      x: round4(objX),
      y: round4(objY),
      z: isEdit ? existing.z : round3(tableHeight + 0.105),
      on: tableName,
      approach: defaultApproach,
      number: freeNum,
      name: isEdit ? existing.name : (activeTemplate ? activeTemplate.name : 'pringles'),
      class: isEdit ? existing.class : (activeTemplate ? activeTemplate.objClass : 'can')
    };

    document.getElementById('obj-modal-title').innerHTML = isEdit ? `🎯 Editar Objeto #${freeNum}` : '🎯 Registrar Objeto Manipulable';
    document.getElementById('obj-number-input').value = freeNum;
    document.getElementById('obj-name-input').value = pendingObject.name;
    document.getElementById('obj-class-input').value = pendingObject.class;
    document.getElementById('obj-z-input').value = pendingObject.z;
    document.getElementById('obj-on-input').value = tableName || '(Suelo)';

    document.getElementById('obj-app-x-disp').textContent = `${defaultApproach.x.toFixed(2)} m`;
    document.getElementById('obj-app-y-disp').textContent = `${defaultApproach.y.toFixed(2)} m`;
    document.getElementById('obj-app-yaw-disp').textContent = `${defaultApproach.yaw_deg}°`;

    validateObjectModalNumber();
    document.getElementById('object-modal').classList.add('active');
  }

  function closeObjectModal() {
    document.getElementById('object-modal').classList.remove('active');
    pendingObject = null;
    editingObjectId = null;
    draw();
  }

  function validateObjectModalNumber() {
    const input = document.getElementById('obj-number-input');
    const warning = document.getElementById('obj-number-warning');
    const btn = document.getElementById('btn-save-object');
    const val = parseInt(input.value, 10);

    if (isNaN(val) || val <= 0) {
      warning.textContent = '⚠ El número debe ser un entero positivo';
      warning.style.display = 'block';
      btn.disabled = true;
      return false;
    }

    const ok = jsIsNumberUnique(val, editingObjectId);
    if (!ok) {
      warning.textContent = `⚠ El número ${val} ya está en uso por otro objeto`;
      warning.style.display = 'block';
      btn.disabled = true;
      return false;
    }

    warning.style.display = 'none';
    btn.disabled = false;
    return true;
  }

  function confirmSaveObject() {
    if (!pendingObject || !validateObjectModalNumber()) return;

    const num = parseInt(document.getElementById('obj-number-input').value, 10);
    const name = document.getElementById('obj-name-input').value.trim() || 'objeto';
    const objClass = document.getElementById('obj-class-input').value;
    const z = parseFloat(document.getElementById('obj-z-input').value) || 0.85;

    let targetId = editingObjectId;
    if (!targetId) {
      const baseId = name.toLowerCase().replace(/[^a-z0-9_]/g, '_');
      targetId = `${baseId}_${num}`;
      let counter = 1;
      while (objects[targetId]) {
        targetId = `${baseId}_${num}_${counter}`;
        counter++;
      }
    }

    objects[targetId] = {
      number: num,
      name: name,
      class: objClass,
      x: pendingObject.x,
      y: pendingObject.y,
      z: round3(z),
      on: pendingObject.on,
      approach: pendingObject.approach
    };

    closeObjectModal();
    updateSidebar();
    setUnsaved(true);
    draw();
    showToast(`✔ Objeto #${num} '${name}' guardado`);
  }

  function editObject(id) {
    if (!objects[id]) return;
    const obj = objects[id];
    openObjectModal(obj.x, obj.y, obj.approach, id);
  }

  function showToast(msg, type = 'success') {
    const t = document.getElementById('toast');
    t.textContent = msg;
    t.className = type;
    t.classList.add('show');
    setTimeout(() => {
      t.classList.remove('show');
    }, 3200);
  }

  window.onload = () => {
    initTheme();
    resizeDisplayCanvas();
    loadData();

    // Comprobar parámetros de URL (útil para testing y accesos directos)
    const urlParams = new URLSearchParams(window.location.search);
    if (urlParams.get('tab')) {
      setTimeout(() => switchTab(urlParams.get('tab')), 200);
    }
    if (urlParams.get('theme')) {
      setTheme(urlParams.get('theme'));
    }
    if (urlParams.get('modal') === 'volume') {
      setTimeout(() => {
        pendingVolumePolygon = [[0, 0], [1.2, 0], [1.2, 0.8], [0, 0.8]];
        openVolumeModal();
      }, 400);
    } else if (urlParams.get('modal') === 'object') {
      setTimeout(() => {
        pendingObject = { name: 'pringles', class: 'can', x: 1.0, y: 2.0, z: 0.86, on: 'table_1' };
        openObjectModal(1.0, 2.0, null, null);
      }, 400);
    } else if (urlParams.get('modal') === 'shortcuts') {
      setTimeout(openShortcutsModal, 300);
    }
  };
</script>
</body>
</html>
'''


class MapZoneEditorHandler(BaseHTTPRequestHandler):
    """Manejador HTTP REST para servir la web y persistir mapas PGM y YAML."""
    manager: Optional[SemanticMapManager] = None

    def log_message(self, format, *args):
        return

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

        elif self.path.startswith("/api/map_raw.png"):
            if self.manager and self.manager.raw_png_bytes:
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.end_headers()
                self.wfile.write(self.manager.raw_png_bytes)
            else:
                self.send_error(404, "Imagen cruda del mapa no disponible")

        elif self.path == "/api/data":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            payload = {
                "map_info": self.manager.map_info,
                "zones": self.manager.zones,
                "furniture": self.manager.furniture,
                "keepout_zones": self.manager.keepout_zones,
                "waypoints": self.manager.waypoints,
                "seats": self.manager.seats,
                "objects": self.manager.objects,
            }
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8"))

        else:
            self.send_error(404, "Ruta no encontrada")

    def do_POST(self):
        if self.path == "/api/save":
            try:
                content_len = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_len)
                data = json.loads(body.decode("utf-8"))

                success = self.manager.save_data(data)
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                resp = {"status": "ok" if success else "error"}
                self.wfile.write(json.dumps(resp).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

        elif self.path == "/api/save_map_image":
            try:
                content_len = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_len)
                data = json.loads(body.decode("utf-8"))
                img_b64 = data.get("image_data", "")

                success = self.manager.save_map_image(img_b64)
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                resp = {"status": "ok" if success else "error"}
                if success:
                    resp["nav"] = reload_nav_map(self.manager.map_yaml_path)
                    print(f"[Editor] {resp['nav']['message']}")
                self.wfile.write(json.dumps(resp).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

        elif self.path == "/api/restore_backup":
            try:
                success = self.manager.restore_original_map()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                resp = {"status": "ok" if success else "error"}
                self.wfile.write(json.dumps(resp).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))
        else:
            self.send_error(404, "Ruta no encontrada")


def main():
    parser = argparse.ArgumentParser(description="Base@Home - Studio Web de Edición de Mapas PGM y Zonas Semánticas")
    parser.add_argument("--map", type=str, default=DEFAULT_MAP_YAML, help="Ruta al archivo mapa.yaml")
    parser.add_argument("--semantic", type=str, default=DEFAULT_SEMANTIC_YAML, help="Ruta al archivo semantic_map.yaml")
    parser.add_argument("--port", type=int, default=8080, help="Puerto del servidor web local")
    parser.add_argument("--no-browser", action="store_true", help="No abrir el navegador automáticamente")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("🗺️  BASE@HOME - STUDIO WEB DE EDICIÓN DE MAPAS PGM Y ZONAS SEMÁNTICAS")
    print("=" * 70)
    print(f"  Mapa Base:       {args.map}")
    print(f"  Mapa Semántico:  {args.semantic}")
    print(f"  URL del Visor:   http://localhost:{args.port}/")
    print("=" * 70 + "\n")

    manager = SemanticMapManager(map_yaml_path=args.map, semantic_yaml_path=args.semantic)
    MapZoneEditorHandler.manager = manager

    server = ThreadedHTTPServer(("0.0.0.0", args.port), MapZoneEditorHandler)

    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(f"http://localhost:{args.port}/")).start()

    try:
        print(f"🚀 Servidor activo en http://localhost:{args.port}/")
        print("   Modo limpio activado: No hay elementos ficticios precargados en el mapa.")
        print("   Opciones y plantillas de RoboCup @Home disponibles en la barra lateral.")
        print("   Presiona Ctrl+C para detener el servidor.\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Editor]: Cerrando servidor...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
