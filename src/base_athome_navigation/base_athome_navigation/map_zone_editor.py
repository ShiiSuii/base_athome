#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Studio Web de Edición de Mapas PGM y Zonas Semánticas
# Herramientas para:
#  - Limpieza de obstáculos fantasma (goma/rectángulo libre)
#  - Dibujo de paredes y obstáculos faltantes (lápiz/línea recta)
#  - Delimitación de mesas y objetos elevados (cajas 2D/3D sobre las patas)
#  - Solidificación de mesas en el mapa PGM para evitar colisiones de Nav2
#  - Áreas prohibidas (Keep-out / No-Go zones)
#  - Zonas poligonales, Waypoints Nav2 con orientación (Yaw) y Asientos
# ==============================================================================

import argparse
import base64
import datetime
import io
import json
import math
import os
import shutil
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
                # Codificar en PNG de 8 bits sin pérdida
                ret, buf = cv2.imencode(".png", gray)
                if ret:
                    self.raw_png_bytes = buf.tobytes()
                    print(f"✔ Mapa PGM cargado: {self.map_info['width']}x{self.map_info['height']} px | Res: {self.map_info['resolution']} m/px | Origen: {self.map_info['origin']}")
        else:
            print(f"⚠ Imagen PGM no encontrada en: {img_abs}")

    def load_semantic_data(self):
        """Carga zonas, mesas, áreas prohibidas, waypoints y asientos guardados."""
        if os.path.exists(self.semantic_yaml_path):
            try:
                with open(self.semantic_yaml_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                self.zones = data.get("zones", {})
                self.furniture = data.get("furniture", {})
                self.keepout_zones = data.get("keepout_zones", {})
                self.waypoints = data.get("waypoints", {})
                self.seats = data.get("seats", {})
                print(f"✔ Datos semánticos cargados: {len(self.zones)} zonas, {len(self.furniture)} muebles/mesas, {len(self.waypoints)} waypoints, {len(self.seats)} asientos.")
                return
            except Exception as e:
                print(f"⚠ Error cargando semantic_map.yaml: {e}")

        # Plantilla inicial con valores por defecto si no existe
        self.zones = {
            "entrance": {
                "name": "Entrada",
                "type": "entrance",
                "color": "#00d2ff",
                "polygon": [[0.4, -0.6], [1.3, -0.6], [1.3, 0.6], [0.4, 0.6]],
            },
            "living_room": {
                "name": "Sala de Estar",
                "type": "living_room",
                "color": "#00f090",
                "polygon": [[1.2, -0.8], [2.4, -0.8], [2.4, 0.8], [1.2, 0.8]],
            },
            "couch_area": {
                "name": "Zona del Sofá",
                "type": "seating",
                "color": "#ffaa00",
                "polygon": [[1.5, -0.7], [2.2, -0.7], [2.2, -0.1], [1.5, -0.1]],
            },
            "chair_area": {
                "name": "Zona de Sillas",
                "type": "seating",
                "color": "#ff5500",
                "polygon": [[1.5, 0.1], [2.2, 0.1], [2.2, 0.7], [1.5, 0.7]],
            },
        }
        self.furniture = {
            "mesa_centro": {
                "name": "Mesa de Centro",
                "type": "table",
                "height_m": 0.76,
                "solidified_in_map": True,
                "polygon": [[1.5, -0.5], [2.1, -0.5], [2.1, -0.1], [1.5, -0.1]],
                "bbox": {"x_min": 1.5, "x_max": 2.1, "y_min": -0.5, "y_max": -0.1, "center_x": 1.8, "center_y": -0.3, "width_m": 0.6, "length_m": 0.4},
            }
        }
        self.keepout_zones = {}
        self.waypoints = {
            "door": {"name": "door", "zone": "entrance", "x": 1.001, "y": 0.065, "yaw_deg": 18.0},
            "living_room": {"name": "living_room", "zone": "living_room", "x": 1.598, "y": -0.355, "yaw_deg": 159.0},
            "couch": {"name": "couch", "zone": "couch_area", "x": 1.673, "y": -0.043, "yaw_deg": 68.0},
            "chair": {"name": "chair", "zone": "chair_area", "x": 1.716, "y": 0.364, "yaw_deg": 10.0},
        }
        self.seats = {
            "couch": {"name": "Sofá", "zone": "living_room", "type": "couch", "arm_gesture": "point_couch", "capacity": 2, "occupied": False},
            "chair": {"name": "Silla Individual", "zone": "living_room", "type": "armchair", "arm_gesture": "point_chair", "capacity": 1, "occupied": False},
        }

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

            # Cuantizar a valores exactos del mapa ROS 2
            # 254 = Espacio libre, 0 = Obstáculo, 205 = Desconocido
            result_pgm = np.full_like(img, 205, dtype=np.uint8)
            result_pgm[img > 220] = 254
            result_pgm[img < 40] = 0

            pgm_path = self.map_info.get("image_path") or "/home/athome/mapa_base_athome.pgm"
            os.makedirs(os.path.dirname(os.path.abspath(pgm_path)), exist_ok=True)
            os.makedirs(BACKUP_DIR, exist_ok=True)

            # 1. Respaldo permanente del original si no existe
            orig_backup = pgm_path + ".orig"
            if os.path.exists(pgm_path) and not os.path.exists(orig_backup):
                shutil.copy2(pgm_path, orig_backup)
                print(f"✔ [Editor] Respaldo original permanente creado en: {orig_backup}")

            # 2. Respaldo con timestamp
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            ts_backup = os.path.join(BACKUP_DIR, f"mapa_base_athome_{ts}.pgm")
            if os.path.exists(pgm_path):
                shutil.copy2(pgm_path, ts_backup)

            # 3. Sobrescribir archivo PGM oficial
            cv2.imwrite(pgm_path, result_pgm)
            print(f"✔ [Editor] Mapa PGM guardado exitosamente en: {pgm_path} (Respaldo: {ts_backup})")

            # 4. Recargar mapa en memoria para sincronización inmediata
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

            # 1. Guardar mapa semántico enriquecido
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
            }

            os.makedirs(os.path.dirname(self.semantic_yaml_path), exist_ok=True)
            with open(self.semantic_yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(sem_payload, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

            # 2. Sincronizar automáticamente con waypoints.yaml para Nav2
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

            print(f"✔ [Editor] Datos semánticos guardados: {len(self.zones)} zonas, {len(self.furniture)} mesas, {len(self.waypoints)} waypoints.")
            return True
        except Exception as e:
            print(f"❌ Error guardando datos semánticos: {e}")
            return False

HTML_PAGE = r'''<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Base@Home - Studio de Edición de Mapas PGM y Zonas Semánticas</title>
<style>
  :root {
    --bg-dark: #080c13;
    --panel-bg: #121824;
    --panel-border: #1e2838;
    --panel-header: #182030;
    --accent-cyan: #00d2ff;
    --accent-green: #00f090;
    --accent-orange: #ff9d00;
    --accent-red: #ff3366;
    --accent-purple: #b388ff;
    --text-main: #f0f4f8;
    --text-muted: #8b9bb0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg-dark); color: var(--text-main); display: flex; height: 100vh; overflow: hidden; user-select: none; }

  /* Barra Lateral */
  #sidebar {
    width: 390px;
    background: var(--panel-bg);
    border-right: 1px solid var(--panel-border);
    display: flex;
    flex-direction: column;
    z-index: 10;
    box-shadow: 4px 0 20px rgba(0,0,0,0.5);
  }
  .header-bar {
    padding: 14px 16px;
    border-bottom: 1px solid var(--panel-border);
    background: var(--panel-header);
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .header-title h1 { font-size: 16px; color: var(--accent-cyan); display: flex; align-items: center; gap: 8px; font-weight: 700; }
  .header-title p { font-size: 11px; color: var(--text-muted); margin-top: 2px; }
  .status-badge { background: rgba(0, 240, 144, 0.15); color: var(--accent-green); border: 1px solid var(--accent-green); padding: 3px 8px; border-radius: 12px; font-size: 11px; font-weight: 600; display: flex; align-items: center; gap: 4px; }
  .status-badge::before { content: ""; width: 6px; height: 6px; border-radius: 50%; background: var(--accent-green); }

  /* Selector de Pestañas Principales */
  .tab-nav {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    background: #0d121c;
    border-bottom: 1px solid var(--panel-border);
  }
  .tab-btn {
    background: none;
    border: none;
    border-bottom: 2px solid transparent;
    color: var(--text-muted);
    padding: 10px 4px;
    font-size: 12px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.2s;
    text-align: center;
  }
  .tab-btn:hover { color: var(--text-main); background: rgba(255,255,255,0.02); }
  .tab-btn.active { color: var(--accent-cyan); border-bottom-color: var(--accent-cyan); background: rgba(0, 210, 255, 0.08); }

  /* Contenido de Pestañas */
  .tab-content { display: none; flex: 1; flex-direction: column; overflow-y: auto; padding: 14px; gap: 14px; }
  .tab-content.active { display: flex; }

  /* Cuadrícula de Herramientas */
  .tool-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .tool-btn {
    background: #182030;
    border: 1px solid var(--panel-border);
    color: var(--text-main);
    padding: 10px 10px;
    border-radius: 6px;
    font-size: 12px;
    cursor: pointer;
    display: flex;
    align-items: center;
    gap: 8px;
    transition: all 0.15s;
    font-weight: 500;
  }
  .tool-btn:hover { background: #222d42; border-color: rgba(0, 210, 255, 0.5); }
  .tool-btn.active { background: rgba(0, 210, 255, 0.18); border-color: var(--accent-cyan); color: var(--accent-cyan); font-weight: 700; }
  .tool-btn.danger.active { background: rgba(255, 51, 102, 0.2); border-color: var(--accent-red); color: var(--accent-red); }

  /* Controles de Pincel */
  .control-box {
    background: #161e2c;
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 10px 12px;
  }
  .control-header { display: flex; justify-content: space-between; font-size: 12px; color: var(--text-muted); margin-bottom: 8px; }
  .control-header span b { color: var(--accent-cyan); }
  .slider-row { display: flex; align-items: center; gap: 10px; }
  .slider-row input[type="range"] { flex: 1; accent-color: var(--accent-cyan); cursor: pointer; }
  .preset-row { display: flex; gap: 6px; margin-top: 8px; }
  .preset-btn { flex: 1; background: #0f1522; border: 1px solid var(--panel-border); color: var(--text-main); font-size: 11px; padding: 4px; border-radius: 4px; cursor: pointer; text-align: center; }
  .preset-btn:hover, .preset-btn.active { border-color: var(--accent-cyan); color: var(--accent-cyan); }

  /* Acciones de Deshacer / Rehacer */
  .history-row { display: flex; gap: 8px; }
  .sub-btn {
    flex: 1;
    background: #182030;
    border: 1px solid var(--panel-border);
    color: var(--text-main);
    padding: 8px;
    border-radius: 6px;
    font-size: 12px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
  }
  .sub-btn:hover:not(:disabled) { background: #222d42; border-color: var(--accent-cyan); }
  .sub-btn:disabled { opacity: 0.4; cursor: not-allowed; }

  /* Lista de Elementos */
  .section-title { font-size: 11px; text-transform: uppercase; letter-spacing: 0.8px; color: var(--text-muted); margin-top: 4px; display: flex; justify-content: space-between; }
  .item-list { display: flex; flex-direction: column; gap: 6px; max-height: 280px; overflow-y: auto; }
  .item-card {
    background: #161e2c;
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 8px 10px;
    display: flex;
    align-items: center;
    justify-content: space-between;
    transition: all 0.15s;
  }
  .item-card:hover { border-color: rgba(255,255,255,0.25); background: #1c2638; }
  .item-card.selected { border-color: var(--accent-cyan); background: rgba(0, 210, 255, 0.08); }
  .item-info { display: flex; flex-direction: column; gap: 2px; }
  .item-title { font-size: 13px; font-weight: 600; display: flex; align-items: center; gap: 6px; }
  .item-subtitle { font-size: 11px; color: var(--text-muted); }
  .item-actions { display: flex; align-items: center; gap: 4px; }
  .icon-btn { background: none; border: none; color: var(--text-muted); cursor: pointer; padding: 4px 6px; border-radius: 4px; font-size: 13px; }
  .icon-btn:hover { color: var(--text-main); background: rgba(255,255,255,0.1); }
  .icon-btn.danger:hover { color: var(--accent-red); background: rgba(255,51,102,0.15); }
  .icon-btn.solidify { color: var(--accent-orange); }

  /* Capas de Visualización */
  .layer-toggles {
    background: #0d121c;
    border-top: 1px solid var(--panel-border);
    padding: 10px 14px;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
    font-size: 11px;
  }
  .layer-item { display: flex; align-items: center; gap: 6px; color: var(--text-muted); cursor: pointer; }
  .layer-item input[type="checkbox"] { accent-color: var(--accent-cyan); cursor: pointer; }

  /* Footer Principal */
  .footer-actions {
    padding: 12px 14px;
    border-top: 1px solid var(--panel-border);
    background: var(--panel-header);
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .save-btn {
    background: linear-gradient(135deg, #00d2ff, #00f090);
    color: #080c13;
    border: none;
    padding: 12px;
    border-radius: 6px;
    font-size: 14px;
    font-weight: 700;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    box-shadow: 0 4px 14px rgba(0, 240, 144, 0.3);
    transition: all 0.2s;
  }
  .save-btn:hover { transform: translateY(-1px); box-shadow: 0 6px 18px rgba(0, 240, 144, 0.5); }

  /* Área Principal del Mapa */
  #map-container {
    flex: 1;
    position: relative;
    background: #05080e;
    overflow: hidden;
  }
  #mapCanvas { position: absolute; top: 0; left: 0; cursor: crosshair; }

  /* HUD Flotante */
  .hud-coordinates {
    position: absolute;
    bottom: 16px;
    left: 16px;
    background: rgba(14, 20, 32, 0.88);
    backdrop-filter: blur(8px);
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 8px 14px;
    font-family: monospace;
    font-size: 12px;
    color: var(--accent-cyan);
    pointer-events: none;
    display: flex;
    gap: 16px;
    box-shadow: 0 4px 16px rgba(0,0,0,0.5);
  }
  .hud-instructions {
    position: absolute;
    top: 16px;
    left: 16px;
    background: rgba(14, 20, 32, 0.92);
    backdrop-filter: blur(8px);
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 10px 16px;
    font-size: 13px;
    color: var(--text-main);
    box-shadow: 0 4px 16px rgba(0,0,0,0.5);
    max-width: 550px;
    display: flex;
    align-items: center;
    gap: 10px;
  }

  /* Modal de Configuración de Mesa */
  .modal-overlay {
    position: fixed;
    top: 0; left: 0; width: 100vw; height: 100vh;
    background: rgba(0,0,0,0.7);
    backdrop-filter: blur(4px);
    display: none;
    align-items: center;
    justify-content: center;
    z-index: 100;
  }
  .modal-overlay.active { display: flex; }
  .modal-card {
    background: var(--panel-bg);
    border: 1px solid var(--accent-cyan);
    border-radius: 8px;
    width: 420px;
    padding: 20px;
    box-shadow: 0 10px 30px rgba(0,0,0,0.8);
    display: flex;
    flex-direction: column;
    gap: 14px;
  }
  .modal-card h3 { font-size: 16px; color: var(--accent-cyan); display: flex; align-items: center; gap: 8px; }
  .form-group { display: flex; flex-direction: column; gap: 6px; font-size: 12px; color: var(--text-muted); }
  .form-group input, .form-group select {
    background: #182030;
    border: 1px solid var(--panel-border);
    color: var(--text-main);
    padding: 8px 10px;
    border-radius: 4px;
    font-size: 13px;
    outline: none;
  }
  .form-group input:focus { border-color: var(--accent-cyan); }
  .checkbox-group { display: flex; align-items: center; gap: 8px; font-size: 12px; color: var(--text-main); margin-top: 4px; cursor: pointer; }
  .modal-actions { display: flex; gap: 10px; margin-top: 10px; }
  .modal-btn { flex: 1; padding: 10px; border-radius: 6px; font-weight: 600; cursor: pointer; border: none; font-size: 13px; }
  .modal-btn.primary { background: var(--accent-cyan); color: #080c13; }
  .modal-btn.cancel { background: #1f2b3e; color: var(--text-main); }

  /* Notificaciones Toast */
  #toast {
    position: fixed;
    top: 24px;
    right: 24px;
    background: var(--accent-green);
    color: #080c13;
    padding: 12px 22px;
    border-radius: 6px;
    font-weight: 700;
    font-size: 14px;
    box-shadow: 0 4px 20px rgba(0, 240, 144, 0.4);
    opacity: 0;
    transform: translateY(-10px);
    pointer-events: none;
    transition: all 0.3s;
    z-index: 1000;
  }
  #toast.show { opacity: 1; transform: translateY(0); }
</style>
</head>
<body>

<div id="sidebar">
  <div class="header-bar">
    <div class="header-title">
      <h1>🤖 Base@Home Studio</h1>
      <p id="map-meta-info">Mapa PGM: 324x582 px (0.05 m/px)</p>
    </div>
    <div class="status-badge">Conectado</div>
  </div>

  <div class="tab-nav">
    <button class="tab-btn active" onclick="switchTab('cleaning')">🧹 Limpieza</button>
    <button class="tab-btn" onclick="switchTab('furniture')">🪑 Mesas</button>
    <button class="tab-btn" onclick="switchTab('semantics')">📍 Zonas/WPs</button>
  </div>

  <!-- PESTAÑA 1: LIMPIEZA Y EDICIÓN DEL MAPA PGM -->
  <div class="tab-content active" id="tab-cleaning">
    <div class="section-title">
      <span>Herramientas de Edición PGM</span>
    </div>
    <div class="tool-grid">
      <button class="tool-btn active" id="btn-pan" onclick="setMode('pan')">🖐️ Navegar (Pan)</button>
      <button class="tool-btn" id="btn-erase" onclick="setMode('erase')">🧹 Goma (Libre)</button>
      <button class="tool-btn" id="btn-clear_box" onclick="setMode('clear_box')">📦 Limpiar Rect</button>
      <button class="tool-btn" id="btn-obstacle" onclick="setMode('obstacle')">✏️ Lápiz Pared</button>
      <button class="tool-btn" id="btn-line_wall" onclick="setMode('line_wall')">📏 Línea Pared</button>
      <button class="tool-btn" id="btn-unknown" onclick="setMode('unknown')">🌫️ Pincel Gris</button>
    </div>

    <div class="control-box">
      <div class="control-header">
        <span>Grosor del Pincel: <b id="brush-size-val">3 px</b> (<span id="brush-meters-val">15 cm</span>)</span>
      </div>
      <div class="slider-row">
        <input type="range" id="brush-slider" min="1" max="40" value="3" oninput="updateBrushSize(this.value)">
      </div>
      <div class="preset-row">
        <button class="preset-btn" onclick="updateBrushSize(1)">1px (Punto)</button>
        <button class="preset-btn active" onclick="updateBrushSize(3)">3px (15cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(6)">6px (30cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(12)">12px (60cm)</button>
        <button class="preset-btn" onclick="updateBrushSize(24)">24px (1.2m)</button>
      </div>
    </div>

    <div class="history-row">
      <button class="sub-btn" id="btn-undo" onclick="undo()" disabled>↩️ Deshacer (Ctrl+Z)</button>
      <button class="sub-btn" id="btn-redo" onclick="redo()" disabled>↪️ Rehacer (Ctrl+Y)</button>
    </div>

    <button class="sub-btn danger" style="margin-top: 4px;" onclick="restoreOriginalMap()">🔄 Revertir al Mapa Original (.orig)</button>
  </div>

  <!-- PESTAÑA 2: MESAS Y MOBILIARIO -->
  <div class="tab-content" id="tab-furniture">
    <div class="section-title">
      <span>Delimitar Mesas & Obstáculos 2D/3D</span>
    </div>
    <div class="tool-grid">
      <button class="tool-btn" id="btn-table_rect" onclick="setMode('table_rect')">➕ Caja de Mesa (Drag)</button>
      <button class="tool-btn" id="btn-table_poly" onclick="setMode('table_poly')">📐 Mesa Polígono</button>
    </div>

    <div class="section-title">
      <span>Mesas y Muebles Registrados</span>
      <span id="furniture-count">0</span>
    </div>
    <div class="item-list" id="furniture-list"></div>
  </div>

  <!-- PESTAÑA 3: ZONAS SEMÁNTICAS, WAYPOINTS Y ASIENTOS -->
  <div class="tab-content" id="tab-semantics">
    <div class="section-title">
      <span>Entorno Semántico</span>
    </div>
    <div class="tool-grid">
      <button class="tool-btn" id="btn-zone" onclick="setMode('zone')">📐 Dibujar Zona</button>
      <button class="tool-btn" id="btn-waypoint" onclick="setMode('waypoint')">📍 Waypoint (Yaw)</button>
      <button class="tool-btn" id="btn-seat" onclick="setMode('seat')">🛋️ Asiento</button>
      <button class="tool-btn danger" id="btn-keepout" onclick="setMode('keepout')">🚫 Zona Prohibida</button>
    </div>

    <div class="section-title">
      <span>Zonas Delimitadas</span>
      <span id="zone-count">0</span>
    </div>
    <div class="item-list" id="zones-list"></div>

    <div class="section-title">
      <span>Waypoints Nav2</span>
      <span id="wp-count">0</span>
    </div>
    <div class="item-list" id="waypoints-list"></div>

    <div class="section-title">
      <span>Asientos Receptionist</span>
      <span id="seat-count">0</span>
    </div>
    <div class="item-list" id="seats-list"></div>
  </div>

  <!-- Capas de Visualización -->
  <div class="layer-toggles">
    <label class="layer-item"><input type="checkbox" id="layer-map" checked onchange="draw()"> 🗺️ Mapa PGM</label>
    <label class="layer-item"><input type="checkbox" id="layer-furniture" checked onchange="draw()"> 🪑 Mesas/Muebles</label>
    <label class="layer-item"><input type="checkbox" id="layer-zones" checked onchange="draw()"> 📐 Zonas Semánticas</label>
    <label class="layer-item"><input type="checkbox" id="layer-waypoints" checked onchange="draw()"> 📍 Waypoints Nav2</label>
    <label class="layer-item"><input type="checkbox" id="layer-seats" checked onchange="draw()"> 🛋️ Asientos</label>
    <label class="layer-item"><input type="checkbox" id="layer-keepout" checked onchange="draw()"> 🚫 Áreas Prohibidas</label>
  </div>

  <!-- Acciones Finales -->
  <div class="footer-actions">
    <button class="save-btn" onclick="saveAll()">💾 Guardar Todo (PGM + Semántico)</button>
    <button class="sub-btn" onclick="resetView()">🎯 Centrar Mapa en Pantalla</button>
  </div>
</div>

<div id="map-container">
  <canvas id="mapCanvas"></canvas>

  <div class="hud-instructions" id="hud-instructions">
    💡 <b>Modo Navegar:</b> Arrastra con el ratón para moverte y usa la rueda para hacer zoom.
  </div>

  <div class="hud-coordinates">
    <span>X: <b id="coord-x">0.00</b> m</span>
    <span>Y: <b id="coord-y">0.00</b> m</span>
    <span>Pixel: <b id="coord-px">0, 0</b></span>
    <span>Valor: <b id="coord-val">Libre (254)</b></span>
    <span>Zoom: <b id="zoom-level">100</b>%</span>
  </div>
</div>

<!-- Modal de Registro de Mesa -->
<div class="modal-overlay" id="table-modal">
  <div class="modal-card">
    <h3>🪑 Delimitar Nueva Mesa / Mueble</h3>
    <div class="form-group">
      <label>Nombre del Mueble:</label>
      <input type="text" id="table-name-input" value="Mesa de Centro" placeholder="ej. Mesa de Centro, Escritorio">
    </div>
    <div class="form-group">
      <label>Tipo de Mueble:</label>
      <select id="table-type-input">
        <option value="table">Mesa de Comedor / Centro (table)</option>
        <option value="desk">Escritorio (desk)</option>
        <option value="shelf">Estante / Repisa (shelf)</option>
        <option value="counter">Mostrador / Barra (counter)</option>
        <option value="box">Caja / Obstáculo Elevado (box)</option>
      </select>
    </div>
    <div class="form-group">
      <label>Altura del Tablero / Superficie (cm):</label>
      <input type="number" id="table-height-input" value="76" min="10" max="250" step="1">
    </div>
    <label class="checkbox-group">
      <input type="checkbox" id="table-solidify-check" checked>
      <b>⚡ Solidificar en el mapa PGM (Rellenar patas en negro para evitar colisión de Nav2)</b>
    </label>
    <div class="modal-actions">
      <button class="modal-btn cancel" onclick="closeTableModal()">Cancelar</button>
      <button class="modal-btn primary" onclick="confirmSaveTable()">Guardar Mesa</button>
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

  let currentMode = 'pan'; // pan, erase, clear_box, obstacle, line_wall, unknown, table_rect, table_poly, zone, waypoint, seat, keepout
  let brushSize = 3; // px
  let mousePixel = { x: 0, y: 0 };
  let isMouseDown = false;
  let dragStartPixel = null;
  let pendingGeometry = null;

  // Transformación de Vista (Pan & Zoom)
  let view = { x: 50, y: 50, zoom: 1.5 };

  // Canvases de Renderizado
  // 1. rawCanvas: tamaño real de la imagen PGM (324x582), contiene los valores de 8-bits exactos [0, 205, 254]
  const rawCanvas = document.createElement('canvas');
  const rawCtx = rawCanvas.getContext('2d', { willReadFrequently: true });

  // 2. colorCanvas: representación visual en color de alto contraste
  const colorCanvas = document.createElement('canvas');
  const colorCtx = colorCanvas.getContext('2d');

  // 3. displayCanvas: canvas visible en el navegador
  const displayCanvas = document.getElementById('mapCanvas');
  const ctx = displayCanvas.getContext('2d');
  const container = document.getElementById('map-container');

  // Pilas de Deshacer / Rehacer para el mapa PGM
  let undoStack = [];
  let redoStack = [];
  const MAX_HISTORY = 25;

  function resizeDisplayCanvas() {
    displayCanvas.width = container.clientWidth;
    displayCanvas.height = container.clientHeight;
    draw();
  }
  window.addEventListener('resize', resizeDisplayCanvas);

  // --- Conversión de Coordenadas Métricas <-> Píxeles ---
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
    return {
      x: (sx - view.x) / view.zoom,
      y: (sy - view.y) / view.zoom
    };
  }

  function pixelToScreen(px, py) {
    return {
      x: view.x + px * view.zoom,
      y: view.y + py * view.zoom
    };
  }

  function worldToScreen(x, y) {
    const p = worldToPixel(x, y);
    return pixelToScreen(p.x, p.y);
  }

  // --- Carga Inicial de Datos ---
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

      document.getElementById('map-meta-info').textContent =
        `Mapa PGM: ${mapInfo.width}x${mapInfo.height} px (${mapInfo.resolution} m/px)`;

      // Configurar dimensiones de los canvases internos
      rawCanvas.width = mapInfo.width;
      rawCanvas.height = mapInfo.height;
      colorCanvas.width = mapInfo.width;
      colorCanvas.height = mapInfo.height;

      // Cargar imagen cruda PGM
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
    }
  }

  // --- Actualización de ColorCanvas desde RawCanvas ---
  function updateColorCanvas() {
    const imgData = rawCtx.getImageData(0, 0, mapInfo.width, mapInfo.height);
    const pixels = imgData.data;
    const colorData = colorCtx.createImageData(mapInfo.width, mapInfo.height);
    const out = colorData.data;

    for (let i = 0; i < pixels.length; i += 4) {
      const val = pixels[i];
      if (val > 220) {
        // Espacio Libre (254) -> Blanco suave cálido
        out[i] = 240; out[i+1] = 244; out[i+2] = 250; out[i+3] = 255;
      } else if (val < 40) {
        // Espacio Ocupado / Pared (0) -> Negro azulado sólido
        out[i] = 10; out[i+1] = 14; out[i+2] = 20; out[i+3] = 255;
      } else {
        // Desconocido (205) -> Gris azulado intermedio
        out[i] = 45; out[i+1] = 54; out[i+2] = 70; out[i+3] = 255;
      }
    }
    colorCtx.putImageData(colorData, 0, 0);
  }

  // --- Sistema de Deshacer / Rehacer (Undo / Redo) ---
  function pushUndo() {
    if (undoStack.length >= MAX_HISTORY) undoStack.shift();
    undoStack.push(rawCtx.getImageData(0, 0, mapInfo.width, mapInfo.height));
    redoStack = [];
    updateUndoRedoButtons();
  }

  function undo() {
    if (undoStack.length > 1) {
      redoStack.push(undoStack.pop());
      const previousState = undoStack[undoStack.length - 1];
      rawCtx.putImageData(previousState, 0, 0);
      updateColorCanvas();
      draw();
      updateUndoRedoButtons();
      showToast('Deshecho');
    }
  }

  function redo() {
    if (redoStack.length > 0) {
      const nextState = redoStack.pop();
      undoStack.push(nextState);
      rawCtx.putImageData(nextState, 0, 0);
      updateColorCanvas();
      draw();
      updateUndoRedoButtons();
      showToast('Rehecho');
    }
  }

  function updateUndoRedoButtons() {
    document.getElementById('btn-undo').disabled = undoStack.length <= 1;
    document.getElementById('btn-redo').disabled = redoStack.length === 0;
  }

  // --- Centrar Vista ---
  function resetView() {
    const scaleX = (displayCanvas.width * 0.85) / mapInfo.width;
    const scaleY = (displayCanvas.height * 0.85) / mapInfo.height;
    view.zoom = Math.min(scaleX, scaleY, 4.0);
    view.x = (displayCanvas.width - mapInfo.width * view.zoom) / 2;
    view.y = (displayCanvas.height - mapInfo.height * view.zoom) / 2;
    draw();
  }

  function switchTab(tabId) {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
    document.getElementById(`tab-${tabId}`).classList.add('active');
    event.currentTarget.classList.add('active');
  }

  function setMode(mode) {
    currentMode = mode;
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    const btn = document.getElementById(`btn-${mode}`);
    if (btn) btn.classList.add('active');
    pendingGeometry = null;

    const instructions = {
      pan: '💡 <b>Modo Navegar:</b> Arrastra con el ratón para moverte y usa la rueda para hacer zoom.',
      erase: '🧹 <b>Goma de Borrar:</b> Pinta en blanco (espacio libre) para eliminar puntos de ruido o personas.',
      clear_box: '📦 <b>Limpiar Rectángulo:</b> Haz clic y arrastra un rectángulo para vaciar toda el área a espacio libre.',
      obstacle: '✏️ <b>Lápiz de Pared:</b> Pinta en negro obstáculos o paredes faltantes a mano alzada.',
      line_wall: '📏 <b>Línea de Pared:</b> Haz clic en el punto inicial y arrastra para trazar una pared recta.',
      unknown: '🌫️ <b>Pincel Gris:</b> Pinta en gris espacio desconocido fuera de los límites de la habitación.',
      table_rect: '🪑 <b>Delimitar Mesa:</b> Haz clic y arrastra una caja cubriendo las 4 patas del LiDAR.',
      table_poly: '📐 <b>Mesa Polígono:</b> Haz clic en cada pata o esquina para trazar la mesa. Doble clic para terminar.',
      zone: '📐 <b>Dibujar Zona:</b> Haz clic para agregar vértices al polígono. Doble clic para cerrar.',
      waypoint: '📍 <b>Colocar Waypoint:</b> Haz clic donde irá el robot y arrastra la flecha hacia donde mirará (Yaw).',
      seat: '🛋️ <b>Colocar Asiento:</b> Haz clic en el mapa donde está el sofá o silla a señalar con el brazo.',
      keepout: '🚫 <b>Zona Prohibida:</b> Marca el polígono de área restringida donde el robot no debe entrar.'
    };
    document.getElementById('hud-instructions').innerHTML = instructions[mode] || '';
    draw();
  }

  function updateBrushSize(val) {
    brushSize = parseInt(val, 10);
    document.getElementById('brush-size-val').textContent = `${brushSize} px`;
    document.getElementById('brush-meters-val').textContent = `${(brushSize * mapInfo.resolution * 100).toFixed(0)} cm`;
    document.getElementById('brush-slider').value = brushSize;
    document.querySelectorAll('.preset-btn').forEach(b => {
      b.classList.toggle('active', parseInt(b.textContent, 10) === brushSize);
    });
    draw();
  }

  // --- Pintar en RawCanvas (Píxeles exactos PGM) ---
  function applyPixelBrush(px, py, colorVal) {
    rawCtx.fillStyle = colorVal;
    rawCtx.beginPath();
    rawCtx.arc(px, py, brushSize / 2, 0, Math.PI * 2);
    rawCtx.fill();
  }

  // --- Renderizado Gráfico Principal ---
  function draw() {
    ctx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);

    ctx.save();
    ctx.translate(view.x, view.y);
    ctx.scale(view.zoom, view.zoom);

    // 1. Capa Mapa PGM (escalado pixel-art sin blur)
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

        // Etiqueta
        const center = getPolygonCenter(zone.polygon);
        const cp = worldToPixel(center.x, center.y);
        drawBadge(cp.x, cp.y, zone.name || id, zone.color || '#00d2ff');
      }
    }

    // 4. Capa Mesas y Muebles (Delimitación de Patas & Tableros)
    if (document.getElementById('layer-furniture').checked) {
      for (const [id, item] of Object.entries(furniture)) {
        if (!item.polygon || item.polygon.length < 3) continue;
        ctx.beginPath();
        const p0 = worldToPixel(item.polygon[0][0], item.polygon[0][1]);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < item.polygon.length; i++) {
          const p = worldToPixel(item.polygon[i][0], item.polygon[i][1]);
          ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = 'rgba(255, 157, 0, 0.25)';
        ctx.fill();
        ctx.strokeStyle = '#ff9d00';
        ctx.lineWidth = 2.5 / view.zoom;
        ctx.stroke();

        // Dibujar patas en las esquinas
        ctx.fillStyle = '#ff3366';
        for (const pt of item.polygon) {
          const p = worldToPixel(pt[0], pt[1]);
          ctx.beginPath();
          ctx.arc(p.x, p.y, 4 / view.zoom, 0, Math.PI * 2);
          ctx.fill();
        }

        // Etiqueta con nombre y altura
        const center = getPolygonCenter(item.polygon);
        const cp = worldToPixel(center.x, center.y);
        const hLabel = item.height_m ? ` (${(item.height_m * 100).toFixed(0)}cm)` : '';
        drawBadge(cp.x, cp.y, `🪑 ${item.name}${hLabel}`, '#ff9d00');
      }
    }

    // 5. Capa Waypoints Nav2
    if (document.getElementById('layer-waypoints').checked) {
      for (const [name, wp] of Object.entries(waypoints)) {
        const p = worldToPixel(wp.x, wp.y);
        drawWaypointPin(p.x, p.y, wp.yaw_deg || 0, name);
      }
    }

    // 6. Capa Asientos Receptionist
    if (document.getElementById('layer-seats').checked) {
      for (const [id, seat] of Object.entries(seats)) {
        if (seat.x !== undefined && seat.y !== undefined) {
          const p = worldToPixel(seat.x, seat.y);
          drawSeatPin(p.x, p.y, seat.name || id, seat.arm_gesture || '');
        }
      }
    }

    // 7. Previsualizaciones en Vivo de Herramientas
    if (isMouseDown && dragStartPixel) {
      if (currentMode === 'clear_box' || currentMode === 'table_rect') {
        const minX = Math.min(dragStartPixel.x, mousePixel.x);
        const minY = Math.min(dragStartPixel.y, mousePixel.y);
        const w = Math.abs(dragStartPixel.x - mousePixel.x);
        const h = Math.abs(dragStartPixel.y - mousePixel.y);

        ctx.strokeStyle = currentMode === 'clear_box' ? '#00f090' : '#ff9d00';
        ctx.lineWidth = 2 / view.zoom;
        ctx.setLineDash([4 / view.zoom, 4 / view.zoom]);
        ctx.strokeRect(minX, minY, w, h);
        ctx.setLineDash([]);

        // Dimensiones en metros
        const wm = (w * mapInfo.resolution).toFixed(2);
        const hm = (h * mapInfo.resolution).toFixed(2);
        drawBadge(minX + w / 2, minY - 10 / view.zoom, `${wm}m x ${hm}m`, currentMode === 'clear_box' ? '#00f090' : '#ff9d00');
      } else if (currentMode === 'line_wall') {
        ctx.strokeStyle = '#000000';
        ctx.lineWidth = brushSize;
        ctx.beginPath();
        ctx.moveTo(dragStartPixel.x, dragStartPixel.y);
        ctx.lineTo(mousePixel.x, mousePixel.y);
        ctx.stroke();

        const distPx = Math.hypot(mousePixel.x - dragStartPixel.x, mousePixel.y - dragStartPixel.y);
        const distM = (distPx * mapInfo.resolution).toFixed(2);
        drawBadge((dragStartPixel.x + mousePixel.x) / 2, (dragStartPixel.y + mousePixel.y) / 2 - 10 / view.zoom, `${distM}m`, '#00d2ff');
      } else if (currentMode === 'waypoint') {
        const dx = mousePixel.x - dragStartPixel.x;
        const dy = -(mousePixel.y - dragStartPixel.y);
        const yawDeg = Math.round((Math.atan2(dy, dx) * 180) / Math.PI);
        drawWaypointPin(dragStartPixel.x, dragStartPixel.y, yawDeg, 'Nuevo WP');
      }
    }

    // 8. Previsualización de Polígonos en Construcción
    if (pendingGeometry && pendingGeometry.length > 0) {
      ctx.beginPath();
      ctx.moveTo(pendingGeometry[0].x, pendingGeometry[0].y);
      for (let i = 1; i < pendingGeometry.length; i++) {
        ctx.lineTo(pendingGeometry[i].x, pendingGeometry[i].y);
      }
      ctx.lineTo(mousePixel.x, mousePixel.y);
      ctx.strokeStyle = '#00d2ff';
      ctx.lineWidth = 2 / view.zoom;
      ctx.setLineDash([4 / view.zoom, 4 / view.zoom]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // 9. Círculo de Pincel bajo el Cursor (para Goma, Lápiz y Gris)
    if (['erase', 'obstacle', 'unknown'].includes(currentMode)) {
      ctx.beginPath();
      ctx.arc(mousePixel.x, mousePixel.y, brushSize / 2, 0, Math.PI * 2);
      ctx.strokeStyle = currentMode === 'erase' ? '#00f090' : (currentMode === 'obstacle' ? '#ff3366' : '#8b9bb0');
      ctx.lineWidth = 1.5 / view.zoom;
      ctx.stroke();
    }

    ctx.restore();
  }

  // --- Funciones de Dibujo Auxiliares ---
  function drawBadge(x, y, text, color) {
    ctx.font = `bold ${Math.max(10, 11 / Math.sqrt(view.zoom))}px sans-serif`;
    const tw = ctx.measureText(text).width;
    const pad = 4 / view.zoom;

    ctx.fillStyle = 'rgba(10, 14, 22, 0.85)';
    ctx.fillRect(x - tw / 2 - pad, y - 8 / view.zoom, tw + pad * 2, 16 / view.zoom);

    ctx.strokeStyle = color;
    ctx.lineWidth = 1 / view.zoom;
    ctx.strokeRect(x - tw / 2 - pad, y - 8 / view.zoom, tw + pad * 2, 16 / view.zoom);

    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y);
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

    // Flecha Yaw
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

    if (e.button === 2 || e.spaceKey || currentMode === 'pan') {
      // Pan
      return;
    }

    if (currentMode === 'erase') {
      applyPixelBrush(mousePixel.x, mousePixel.y, '#ffffff');
      updateColorCanvas();
      draw();
    } else if (currentMode === 'obstacle') {
      applyPixelBrush(mousePixel.x, mousePixel.y, '#000000');
      updateColorCanvas();
      draw();
    } else if (currentMode === 'unknown') {
      applyPixelBrush(mousePixel.x, mousePixel.y, '#cdcdcd');
      updateColorCanvas();
      draw();
    } else if (currentMode === 'seat') {
      const world = pixelToWorld(mousePixel.x, mousePixel.y);
      const name = prompt('Nombre del Asiento (ej. Sofá, Silla):', 'Silla');
      if (name) {
        const id = name.toLowerCase().replace(/\s+/g, '_');
        seats[id] = {
          name: name,
          zone: 'living_room',
          x: round4(world.x),
          y: round4(world.y),
          type: id.includes('sofa') || id.includes('couch') ? 'couch' : 'armchair',
          arm_gesture: id.includes('sofa') || id.includes('couch') ? 'point_couch' : 'point_chair',
          capacity: 1,
          occupied: false
        };
        updateSidebar();
        showToast('Asiento registrado');
      }
      setMode('pan');
    }
  });

  displayCanvas.addEventListener('mousemove', (e) => {
    const rect = displayCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const p = screenToPixel(sx, sy);
    mousePixel = p;

    // Actualizar HUD de coordenadas
    const world = pixelToWorld(p.x, p.y);
    document.getElementById('coord-x').textContent = world.x.toFixed(2);
    document.getElementById('coord-y').textContent = world.y.toFixed(2);
    document.getElementById('coord-px').textContent = `${Math.round(p.x)}, ${Math.round(p.y)}`;
    document.getElementById('zoom-level').textContent = `${Math.round(view.zoom * 100)}`;

    if (p.x >= 0 && p.x < mapInfo.width && p.y >= 0 && p.y < mapInfo.height) {
      const pixelVal = rawCtx.getImageData(Math.floor(p.x), Math.floor(p.y), 1, 1).data[0];
      let valStr = pixelVal > 220 ? 'Libre (254)' : (pixelVal < 40 ? 'Obstáculo (0)' : 'Desconocido (205)');
      document.getElementById('coord-val').textContent = valStr;
    }

    if (isMouseDown) {
      if (e.buttons === 2 || e.spaceKey || currentMode === 'pan') {
        view.x += e.movementX;
        view.y += e.movementY;
        draw();
        return;
      }

      if (currentMode === 'erase') {
        applyPixelBrush(p.x, p.y, '#ffffff');
        updateColorCanvas();
      } else if (currentMode === 'obstacle') {
        applyPixelBrush(p.x, p.y, '#000000');
        updateColorCanvas();
      } else if (currentMode === 'unknown') {
        applyPixelBrush(p.x, p.y, '#cdcdcd');
        updateColorCanvas();
      }
    }
    draw();
  });

  window.addEventListener('mouseup', (e) => {
    if (!isMouseDown) return;
    isMouseDown = false;

    if (['erase', 'obstacle', 'unknown'].includes(currentMode)) {
      pushUndo();
      return;
    }

    if (!dragStartPixel) return;

    if (currentMode === 'clear_box') {
      const minX = Math.round(Math.min(dragStartPixel.x, mousePixel.x));
      const minY = Math.round(Math.min(dragStartPixel.y, mousePixel.y));
      const w = Math.round(Math.abs(dragStartPixel.x - mousePixel.x));
      const h = Math.round(Math.abs(dragStartPixel.y - mousePixel.y));

      if (w > 2 && h > 2) {
        rawCtx.fillStyle = '#ffffff';
        rawCtx.fillRect(minX, minY, w, h);
        updateColorCanvas();
        pushUndo();
        showToast('Área limpiada (Espacio libre)');
      }
      setMode('pan');
    } else if (currentMode === 'line_wall') {
      rawCtx.strokeStyle = '#000000';
      rawCtx.lineWidth = brushSize;
      rawCtx.beginPath();
      rawCtx.moveTo(dragStartPixel.x, dragStartPixel.y);
      rawCtx.lineTo(mousePixel.x, mousePixel.y);
      rawCtx.stroke();
      updateColorCanvas();
      pushUndo();
      showToast('Pared trazada');
      setMode('pan');
    } else if (currentMode === 'table_rect') {
      const minX = Math.min(dragStartPixel.x, mousePixel.x);
      const minY = Math.min(dragStartPixel.y, mousePixel.y);
      const maxX = Math.max(dragStartPixel.x, mousePixel.x);
      const maxY = Math.max(dragStartPixel.y, mousePixel.y);

      if (Math.abs(maxX - minX) > 4 && Math.abs(maxY - minY) > 4) {
        const p1 = pixelToWorld(minX, minY);
        const p2 = pixelToWorld(maxX, minY);
        const p3 = pixelToWorld(maxX, maxY);
        const p4 = pixelToWorld(minX, maxY);

        pendingTablePolygon = [
          [round4(p1.x), round4(p1.y)],
          [round4(p2.x), round4(p2.y)],
          [round4(p3.x), round4(p3.y)],
          [round4(p4.x), round4(p4.y)]
        ];
        openTableModal();
      }
    } else if (currentMode === 'waypoint') {
      const dx = mousePixel.x - dragStartPixel.x;
      const dy = -(mousePixel.y - dragStartPixel.y);
      const yawDeg = Math.round((Math.atan2(dy, dx) * 180) / Math.PI);
      const world = pixelToWorld(dragStartPixel.x, dragStartPixel.y);

      const name = prompt('Nombre del Waypoint Nav2 (ej. mesa_pringles, sofa, cocina):', 'punto_' + Object.keys(waypoints).length);
      if (name) {
        waypoints[name] = {
          name: name,
          x: round4(world.x),
          y: round4(world.y),
          yaw_deg: yawDeg
        };
        updateSidebar();
        showToast(`Waypoint '${name}' registrado`);
      }
      setMode('pan');
    }

    dragStartPixel = null;
    draw();
  });

  // Manejo de Clics para Polígonos
  displayCanvas.addEventListener('click', (e) => {
    if (['zone', 'keepout', 'table_poly'].includes(currentMode)) {
      if (!pendingGeometry) pendingGeometry = [];
      pendingGeometry.push({ ...mousePixel });
      draw();
    }
  });

  displayCanvas.addEventListener('dblclick', (e) => {
    if (['zone', 'keepout', 'table_poly'].includes(currentMode)) {
      if (pendingGeometry && pendingGeometry.length >= 3) {
        const polyWorld = pendingGeometry.map(p => {
          const w = pixelToWorld(p.x, p.y);
          return [round4(w.x), round4(w.y)];
        });

        if (currentMode === 'zone') {
          const name = prompt('Nombre de la Zona Semántica:', 'Nueva Zona');
          if (name) {
            const id = name.toLowerCase().replace(/\s+/g, '_');
            zones[id] = { name: name, type: 'room', color: '#00d2ff', polygon: polyWorld };
            updateSidebar();
            showToast('Zona guardada');
          }
        } else if (currentMode === 'keepout') {
          const name = prompt('Nombre del Área Prohibida:', 'Cables / Desnivel');
          if (name) {
            const id = name.toLowerCase().replace(/\s+/g, '_');
            keepout_zones[id] = { name: name, polygon: polyWorld };
            updateSidebar();
            showToast('Área prohibida guardada');
          }
        } else if (currentMode === 'table_poly') {
          pendingTablePolygon = polyWorld;
          openTableModal();
        }
      }
      pendingGeometry = null;
      setMode('pan');
    }
  });

  // Zoom con Rueda
  displayCanvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = displayCanvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    const newZoom = Math.min(Math.max(view.zoom * zoomFactor, 0.2), 25.0);

    view.x = sx - (sx - view.x) * (newZoom / view.zoom);
    view.y = sy - (sy - view.y) * (newZoom / view.zoom);
    view.zoom = newZoom;
    draw();
  });

  // Atajos de Teclado
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;

    if (e.ctrlKey && e.key === 'z') { e.preventDefault(); undo(); }
    else if (e.ctrlKey && (e.key === 'y' || (e.shiftKey && e.key === 'Z'))) { e.preventDefault(); redo(); }
    else if (e.key === '1') setMode('pan');
    else if (e.key === '2') setMode('erase');
    else if (e.key === '3') setMode('clear_box');
    else if (e.key === '4') setMode('obstacle');
    else if (e.key === '5') setMode('line_wall');
    else if (e.key === '6') setMode('unknown');
    else if (e.key === '7') setMode('table_rect');
    else if (e.key === 'Escape') { pendingGeometry = null; setMode('pan'); closeTableModal(); }
  });

  // --- Modal de Mesa ---
  let pendingTablePolygon = null;
  function openTableModal() {
    document.getElementById('table-modal').classList.add('active');
  }
  function closeTableModal() {
    document.getElementById('table-modal').classList.remove('active');
    pendingTablePolygon = null;
    setMode('pan');
  }
  function confirmSaveTable() {
    if (!pendingTablePolygon) return;
    const name = document.getElementById('table-name-input').value.trim() || 'Mesa';
    const type = document.getElementById('table-type-input').value;
    const heightCm = parseFloat(document.getElementById('table-height-input').value) || 76;
    const solidify = document.getElementById('table-solidify-check').checked;

    const id = name.toLowerCase().replace(/\s+/g, '_');
    furniture[id] = {
      name: name,
      type: type,
      height_m: heightCm / 100.0,
      solidified_in_map: solidify,
      polygon: pendingTablePolygon
    };

    if (solidify) {
      solidifyTableGeometry(pendingTablePolygon, true);
    }

    closeTableModal();
    updateSidebar();
    showToast(`Mesa '${name}' guardada`);
  }

  function solidifyTableGeometry(polyWorld, isObstacle) {
    if (!polyWorld || polyWorld.length < 3) return;
    rawCtx.fillStyle = isObstacle ? '#000000' : '#ffffff';
    rawCtx.beginPath();
    const p0 = worldToPixel(polyWorld[0][0], polyWorld[0][1]);
    rawCtx.moveTo(p0.x, p0.y);
    for (let i = 1; i < polyWorld.length; i++) {
      const p = worldToPixel(polyWorld[i][0], polyWorld[i][1]);
      rawCtx.lineTo(p.x, p.y);
    }
    rawCtx.closePath();
    rawCtx.fill();
    updateColorCanvas();
    pushUndo();
    draw();
  }

  function toggleSolidify(id) {
    const item = furniture[id];
    if (!item) return;
    item.solidified_in_map = !item.solidified_in_map;
    solidifyTableGeometry(item.polygon, item.solidified_in_map);
    updateSidebar();
    showToast(item.solidified_in_map ? 'Mesa solidificada en PGM' : 'Mesa despejada');
  }

  function deleteItem(collectionName, id) {
    if (confirm(`¿Eliminar '${id}'?`)) {
      if (collectionName === 'furniture') delete furniture[id];
      else if (collectionName === 'zones') delete zones[id];
      else if (collectionName === 'waypoints') delete waypoints[id];
      else if (collectionName === 'seats') delete seats[id];
      updateSidebar();
      draw();
    }
  }

  // --- Actualización de la Barra Lateral ---
  function updateSidebar() {
    // Mesas
    const fList = document.getElementById('furniture-list');
    fList.innerHTML = '';
    const fKeys = Object.keys(furniture);
    document.getElementById('furniture-count').textContent = fKeys.length;
    for (const [id, f] of Object.entries(furniture)) {
      const card = document.createElement('div');
      card.className = 'item-card';
      card.innerHTML = `
        <div class="item-info">
          <span class="item-title">🪑 ${f.name}</span>
          <span class="item-subtitle">${(f.height_m * 100).toFixed(0)}cm alt | ${f.solidified_in_map ? '⚡ Solidificada' : 'Huella libre'}</span>
        </div>
        <div class="item-actions">
          <button class="icon-btn solidify" title="Solidificar/Despejar" onclick="toggleSolidify('${id}')">⚡</button>
          <button class="icon-btn danger" title="Eliminar" onclick="deleteItem('furniture', '${id}')">🗑️</button>
        </div>
      `;
      fList.appendChild(card);
    }

    // Zonas
    const zList = document.getElementById('zones-list');
    zList.innerHTML = '';
    document.getElementById('zone-count').textContent = Object.keys(zones).length;
    for (const [id, z] of Object.entries(zones)) {
      const card = document.createElement('div');
      card.className = 'item-card';
      card.innerHTML = `
        <div class="item-info">
          <span class="item-title"><span style="color: ${z.color};">■</span> ${z.name || id}</span>
          <span class="item-subtitle">${z.type || 'zone'} (${z.polygon ? z.polygon.length : 0} vértices)</span>
        </div>
        <div class="item-actions">
          <button class="icon-btn danger" onclick="deleteItem('zones', '${id}')">🗑️</button>
        </div>
      `;
      zList.appendChild(card);
    }

    // Waypoints
    const wList = document.getElementById('waypoints-list');
    wList.innerHTML = '';
    document.getElementById('wp-count').textContent = Object.keys(waypoints).length;
    for (const [name, wp] of Object.entries(waypoints)) {
      const card = document.createElement('div');
      card.className = 'item-card';
      card.innerHTML = `
        <div class="item-info">
          <span class="item-title">📍 ${name}</span>
          <span class="item-subtitle">X:${wp.x.toFixed(2)}, Y:${wp.y.toFixed(2)} | Yaw:${wp.yaw_deg || 0}°</span>
        </div>
        <div class="item-actions">
          <button class="icon-btn danger" onclick="deleteItem('waypoints', '${name}')">🗑️</button>
        </div>
      `;
      wList.appendChild(card);
    }

    // Asientos
    const sList = document.getElementById('seats-list');
    sList.innerHTML = '';
    document.getElementById('seat-count').textContent = Object.keys(seats).length;
    for (const [id, s] of Object.entries(seats)) {
      const card = document.createElement('div');
      card.className = 'item-card';
      card.innerHTML = `
        <div class="item-info">
          <span class="item-title">🛋️ ${s.name || id}</span>
          <span class="item-subtitle">Gesto: ${s.arm_gesture || 'point_couch'}</span>
        </div>
        <div class="item-actions">
          <button class="icon-btn danger" onclick="deleteItem('seats', '${id}')">🗑️</button>
        </div>
      `;
      sList.appendChild(card);
    }
  }

  // --- Guardar Todo (PGM + Datos Semánticos) ---
  async function saveAll() {
    showToast('Guardando cambios...');
    try {
      // 1. Guardar Imagen PGM
      const pgmBase64 = rawCanvas.toDataURL('image/png');
      const imgRes = await fetch('/api/save_map_image', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ image_data: pgmBase64 })
      });
      const imgData = await imgRes.json();

      // 2. Guardar Datos Semánticos
      const semRes = await fetch('/api/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          zones: zones,
          furniture: furniture,
          keepout_zones: keepout_zones,
          waypoints: waypoints,
          seats: seats
        })
      });
      const semData = await semRes.json();

      if (imgData.status === 'ok' && semData.status === 'ok') {
        showToast('✔ ¡Mapa PGM y Datos Semánticos guardados exitosamente!');
      } else {
        showToast('⚠ Error al guardar algunos archivos');
      }
    } catch (e) {
      console.error(e);
      showToast('❌ Error en el guardado');
    }
  }

  async function restoreOriginalMap() {
    if (confirm('¿Restaurar el mapa PGM a su versión original (.orig)? Todos los cambios de dibujo se revertirán.')) {
      const res = await fetch('/api/restore_backup', { method: 'POST' });
      const data = await res.json();
      if (data.status === 'ok') {
        showToast('Mapa restaurado');
        loadData();
      } else {
        showToast('No se encontró respaldo original');
      }
    }
  }

  function round4(num) { return Math.round(num * 10000) / 10000; }

  function showToast(msg) {
    const t = document.getElementById('toast');
    t.textContent = msg;
    t.classList.add('show');
    setTimeout(() => t.classList.remove('show'), 3000);
  }

  window.onload = () => {
    resizeDisplayCanvas();
    loadData();
  };
</script>
</body>
</html>
'''


class MapZoneEditorHandler(BaseHTTPRequestHandler):
    """Manejador HTTP REST para servir la web y persistir mapas PGM y YAML."""
    manager: Optional[SemanticMapManager] = None

    def log_message(self, format, *args):
        # Silenciar logs ruidosos de peticiones estáticas
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
        print("   Herramientas activas: Limpieza PGM, Goma, Paredes, Mesas 2D/3D, Solidificación, Zonas y WPs.")
        print("   Presiona Ctrl+C para detener el servidor.\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Editor]: Cerrando servidor...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
