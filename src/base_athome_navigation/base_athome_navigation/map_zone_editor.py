#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Visor y Editor Web de Zonas Semánticas y Waypoints en Localhost
# Permite delimitar polígonos (Entrada, Sala, Sofá, Silla), waypoints Nav2 y asientos.
# ==============================================================================

import argparse
import datetime
import io
import json
import math
import os
import sys
import threading
import time
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import cv2
import numpy as np
import yaml

DEFAULT_MAP_YAML = "/home/athome/mapa_base_athome.yaml"
DEFAULT_SEMANTIC_YAML = "/home/athome/base_athome_ws/src/base_athome_navigation/config/semantic_map.yaml"
DEFAULT_WAYPOINTS_YAML = "/home/athome/base_athome_ws/src/base_athome_navigation/config/waypoints.yaml"


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Servidor HTTP multihilo para atender peticiones web concurrentes sin bloqueo."""
    daemon_threads = True


class SemanticMapManager:
    """Gestiona la carga, transformación de coordenadas y guardado del mapa semántico."""

    def __init__(self, map_yaml_path: str = DEFAULT_MAP_YAML, semantic_yaml_path: str = DEFAULT_SEMANTIC_YAML):
        self.map_yaml_path = os.path.abspath(map_yaml_path)
        self.semantic_yaml_path = os.path.abspath(semantic_yaml_path)
        self.waypoints_yaml_path = DEFAULT_WAYPOINTS_YAML

        self.map_info = {
            "resolution": 0.05,
            "origin": [-10.0, -10.0, 0.0],
            "width": 500,
            "height": 500,
            "image_path": "",
        }
        self.png_bytes = b""
        self.zones = {}
        self.waypoints = {}
        self.seats = {}

        self.load_map()
        self.load_semantic_data()

    def load_map(self):
        """Carga el mapa PGM y sus metadatos YAML."""
        if not os.path.exists(self.map_yaml_path):
            print(f"⚠ Mapa YAML no encontrado en: {self.map_yaml_path}")
            return

        with open(self.map_yaml_path, "r") as f:
            data = yaml.safe_load(f)

        self.map_info["resolution"] = float(data.get("resolution", 0.05))
        self.map_info["origin"] = [float(x) for x in data.get("origin", [-10.0, -10.0, 0.0])]

        img_rel = data.get("image", "")
        img_abs = os.path.join(os.path.dirname(self.map_yaml_path), img_rel)
        if not os.path.exists(img_abs):
            img_abs = os.path.expanduser(f"~/{img_rel}")

        if os.path.exists(img_abs):
            self.map_info["image_path"] = img_abs
            gray = cv2.imread(img_abs, cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                self.map_info["height"], self.map_info["width"] = gray.shape[:2]
                # Convertir a imagen RGB de alta visibilidad (fondo oscuro suave para modo nocturno)
                # En mapas ROS: 254 = espacio libre, 0 = ocupado, 205 = desconocido
                rgb = np.zeros((gray.shape[0], gray.shape[1], 3), dtype=np.uint8)
                # Espacio libre: blanco cálido
                rgb[gray > 240] = [235, 240, 245]
                # Espacio ocupado (paredes/obstáculos): negro azulado
                rgb[gray < 30] = [15, 20, 28]
                # Espacio desconocido: gris azulado oscuro
                rgb[(gray >= 30) & (gray <= 240)] = [45, 52, 65]

                ret, buf = cv2.imencode(".png", rgb)
                if ret:
                    self.png_bytes = buf.tobytes()
                    print(f"✔ Mapa cargado: {self.map_info['width']}x{self.map_info['height']} px ({self.map_info['resolution']} m/px)")
        else:
            print(f"⚠ Imagen PGM no encontrada: {img_abs}")

    def load_semantic_data(self):
        """Carga zonas, waypoints y asientos guardados en semantic_map.yaml."""
        if os.path.exists(self.semantic_yaml_path):
            try:
                with open(self.semantic_yaml_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                self.zones = data.get("zones", {})
                self.waypoints = data.get("waypoints", {})
                self.seats = data.get("seats", {})
                print(f"✔ Datos semánticos cargados: {len(self.zones)} zonas, {len(self.waypoints)} waypoints, {len(self.seats)} asientos.")
                return
            except Exception as e:
                print(f"⚠ Error cargando semantic_map.yaml: {e}")

        # Si no existe, crear datos iniciales para RoboCup @Home
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
        }
        self.waypoints = {
            "door": {
                "name": "door",
                "zone": "entrance",
                "x": 1.001,
                "y": 0.065,
                "yaw_deg": 18.0,
            },
            "living_room": {
                "name": "living_room",
                "zone": "living_room",
                "x": 1.598,
                "y": -0.355,
                "yaw_deg": 159.0,
            },
        }
        self.seats = {
            "couch": {
                "name": "Sofá Principal",
                "zone": "living_room",
                "type": "couch",
                "arm_gesture": "point_couch",
                "capacity": 2,
            },
            "chair": {
                "name": "Silla Individual",
                "zone": "living_room",
                "type": "chair",
                "arm_gesture": "point_chair",
                "capacity": 1,
            },
        }

    def save_data(self, new_data: dict) -> bool:
        """Persiste las zonas, waypoints y asientos tanto en semantic_map.yaml como en waypoints.yaml."""
        try:
            self.zones = new_data.get("zones", self.zones)
            self.waypoints = new_data.get("waypoints", self.waypoints)
            self.seats = new_data.get("seats", self.seats)

            # 1. Guardar mapa semántico completo
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

            # También sincronizar respaldo en ~/waypoints_base_athome.yaml
            user_wp = os.path.expanduser("~/waypoints_base_athome.yaml")
            with open(user_wp, "w", encoding="utf-8") as f:
                yaml.dump({"waypoints": wp_dict}, f, default_flow_style=False, sort_keys=False)

            print(f"✔ [Editor] Guardado exitoso: {len(self.zones)} zonas y {len(self.waypoints)} waypoints sincronizados con Nav2.")
            return True

        except Exception as e:
            print(f"❌ Error guardando datos semánticos: {e}")
            return False


HTML_PAGE = """<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Base@Home - Editor de Zonas y Mapa Semántico</title>
<style>
  :root {
    --bg-dark: #0b0f17;
    --panel-bg: #141b26;
    --panel-border: #1f2b3e;
    --accent-cyan: #00d2ff;
    --accent-green: #00f090;
    --accent-orange: #ff9d00;
    --accent-red: #ff3366;
    --text-main: #f0f4f8;
    --text-muted: #8b9bb0;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg-dark); color: var(--text-main); display: flex; height: 100vh; overflow: hidden; }

  /* Barra lateral de herramientas */
  #sidebar {
    width: 360px;
    background: var(--panel-bg);
    border-right: 1px solid var(--panel-border);
    display: flex;
    flex-direction: column;
    z-index: 10;
    box-shadow: 2px 0 12px rgba(0,0,0,0.4);
  }
  .header-bar {
    padding: 16px;
    border-bottom: 1px solid var(--panel-border);
    background: rgba(0, 210, 255, 0.05);
  }
  .header-bar h1 { font-size: 18px; color: var(--accent-cyan); display: flex; align-items: center; gap: 8px; }
  .header-bar p { font-size: 12px; color: var(--text-muted); margin-top: 4px; }

  /* Herramientas de dibujo */
  .toolbar {
    padding: 12px;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    border-bottom: 1px solid var(--panel-border);
  }
  .tool-btn {
    background: #1c2636;
    border: 1px solid var(--panel-border);
    color: var(--text-main);
    padding: 10px 8px;
    border-radius: 6px;
    font-size: 13px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    transition: all 0.2s;
  }
  .tool-btn:hover { background: #253348; border-color: var(--accent-cyan); }
  .tool-btn.active { background: rgba(0, 210, 255, 0.15); border-color: var(--accent-cyan); color: var(--accent-cyan); font-weight: bold; }

  /* Lista de Elementos */
  .content-list {
    flex: 1;
    overflow-y: auto;
    padding: 12px;
  }
  .section-title {
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 0.8px;
    color: var(--text-muted);
    margin: 12px 0 8px 4px;
    display: flex;
    justify-content: space-between;
  }
  .item-card {
    background: #182230;
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 10px 12px;
    margin-bottom: 8px;
    cursor: pointer;
    transition: all 0.2s;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }
  .item-card:hover { border-color: rgba(255,255,255,0.3); }
  .item-card.selected { border-color: var(--accent-cyan); background: rgba(0, 210, 255, 0.1); }
  .item-info { display: flex; align-items: center; gap: 8px; }
  .color-badge { width: 12px; height: 12px; border-radius: 3px; }
  .item-title { font-size: 14px; font-weight: 500; }
  .item-subtitle { font-size: 11px; color: var(--text-muted); }
  .del-btn { color: #889; background: none; border: none; cursor: pointer; padding: 4px; border-radius: 4px; }
  .del-btn:hover { color: var(--accent-red); background: rgba(255,50,50,0.1); }

  /* Botones inferiores */
  .footer-actions {
    padding: 14px;
    border-top: 1px solid var(--panel-border);
    background: #111722;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .primary-btn {
    background: var(--accent-cyan);
    color: #0b0f17;
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
    transition: 0.2s;
  }
  .primary-btn:hover { background: #33dcff; box-shadow: 0 0 14px rgba(0, 210, 255, 0.4); }

  /* Área Principal del Mapa */
  #map-container {
    flex: 1;
    position: relative;
    background: #080c12;
    overflow: hidden;
  }
  #mapCanvas { position: absolute; top: 0; left: 0; cursor: crosshair; }

  /* Barra de Información Flotante */
  .hud-coordinates {
    position: absolute;
    bottom: 16px;
    left: 16px;
    background: rgba(14, 20, 29, 0.85);
    backdrop-filter: blur(8px);
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 8px 14px;
    font-family: monospace;
    font-size: 13px;
    color: var(--accent-cyan);
    pointer-events: none;
    display: flex;
    gap: 16px;
  }
  .hud-instructions {
    position: absolute;
    top: 16px;
    left: 16px;
    background: rgba(14, 20, 29, 0.88);
    backdrop-filter: blur(8px);
    border: 1px solid var(--panel-border);
    border-radius: 6px;
    padding: 10px 16px;
    font-size: 13px;
    color: var(--text-main);
    box-shadow: 0 4px 16px rgba(0,0,0,0.5);
  }

  /* Notificaciones Toast */
  #toast {
    position: fixed;
    top: 24px;
    right: 24px;
    background: var(--accent-green);
    color: #000;
    padding: 12px 20px;
    border-radius: 6px;
    font-weight: 700;
    box-shadow: 0 4px 16px rgba(0, 240, 144, 0.4);
    opacity: 0;
    pointer-events: none;
    transition: opacity 0.3s;
    z-index: 100;
  }
  #toast.show { opacity: 1; }
</style>
</head>
<body>

<div id="sidebar">
  <div class="header-bar">
    <h1>🤖 Base@Home Editor</h1>
    <p>Delimitador de Zonas y Waypoints Semánticos</p>
  </div>

  <div class="toolbar">
    <button class="tool-btn active" id="btn-pan" onclick="setMode('pan')">🖐️ Navegar</button>
    <button class="tool-btn" id="btn-zone" onclick="setMode('zone')">📐 Dibujar Zona</button>
    <button class="tool-btn" id="btn-waypoint" onclick="setMode('waypoint')">📍 Waypoint</button>
    <button class="tool-btn" id="btn-seat" onclick="setMode('seat')">🛋️ Asiento</button>
  </div>

  <div class="content-list">
    <div class="section-title">
      <span>Zonas Delimitadas</span>
      <span id="zone-count">0</span>
    </div>
    <div id="zones-list"></div>

    <div class="section-title">
      <span>Waypoints Nav2</span>
      <span id="wp-count">0</span>
    </div>
    <div id="waypoints-list"></div>

    <div class="section-title">
      <span>Asientos Receptionist</span>
      <span id="seat-count">0</span>
    </div>
    <div id="seats-list"></div>
  </div>

  <div class="footer-actions">
    <button class="primary-btn" onclick="saveData()">💾 Guardar y Actualizar Nav2</button>
    <button class="tool-btn" style="width: 100%;" onclick="resetView()">🎯 Centrar Mapa</button>
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
    <span>Zoom: <b id="zoom-level">100</b>%</span>
  </div>
</div>

<div id="toast">¡Guardado con éxito!</div>

<script>
  let mapInfo = { resolution: 0.05, origin: [-10, -10, 0], width: 500, height: 500 };
  let mapImage = new Image();
  let zones = {};
  let waypoints = {};
  let seats = {};

  let currentMode = 'pan'; // 'pan', 'zone', 'waypoint', 'seat'
  let selectedId = null;

  // Transformación de la vista (Pan & Zoom)
  let view = { x: 50, y: 50, zoom: 1.2 };
  let isDragging = false;
  let dragStart = { x: 0, y: 0 };

  // Variables para dibujo de zonas poligonales
  let currentPolygon = [];

  // Variables para colocar waypoint con orientación
  let wpPlacement = null; // { x, y, yaw_deg }

  const canvas = document.getElementById('mapCanvas');
  const ctx = canvas.getContext('2d');
  const container = document.getElementById('map-container');

  function resizeCanvas() {
    canvas.width = container.clientWidth;
    canvas.height = container.clientHeight;
    draw();
  }
  window.addEventListener('resize', resizeCanvas);

  // --- Conversión de Coordenadas ---
  // Mapa ROS: origen (x_orig, y_orig) es esquina inferior-izquierda. Imagen PGM: (0,0) superior-izquierda.
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

  function screenToWorld(sx, sy) {
    const p = screenToPixel(sx, sy);
    return pixelToWorld(p.x, p.y);
  }

  // --- Carga Inicial de Datos ---
  async function loadData() {
    try {
      const res = await fetch('/api/data');
      const data = await res.json();
      mapInfo = data.map_info;
      zones = data.zones || {};
      waypoints = data.waypoints || {};
      seats = data.seats || {};

      mapImage.src = '/api/map.png';
      mapImage.onload = () => {
        resetView();
        updateSidebar();
      };
    } catch (e) {
      console.error('Error cargando datos:', e);
    }
  }

  function resetView() {
    if (!mapImage.complete) return;
    const scaleX = (canvas.width * 0.8) / mapImage.width;
    const scaleY = (canvas.height * 0.8) / mapImage.height;
    view.zoom = Math.min(scaleX, scaleY, 2.5);
    view.x = (canvas.width - mapImage.width * view.zoom) / 2;
    view.y = (canvas.height - mapImage.height * view.zoom) / 2;
    draw();
  }

  function setMode(mode) {
    currentMode = mode;
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(`btn-${mode}`).classList.add('active');

    const instructions = {
      pan: '💡 <b>Modo Navegar:</b> Arrastra con el ratón para moverte y usa la rueda para hacer zoom.',
      zone: '📐 <b>Dibujar Zona:</b> Haz clic en varios puntos para trazar los vértices del polígono. Doble clic para terminar.',
      waypoint: '📍 <b>Colocar Waypoint:</b> Haz clic donde se ubicará el robot y arrastra hacia la dirección donde debe mirar (Yaw).',
      seat: '🛋️ <b>Colocar Asiento:</b> Haz clic en el mapa donde está el sofá o silla a señalar con el brazo.'
    };
    document.getElementById('hud-instructions').innerHTML = instructions[mode] || '';

    currentPolygon = [];
    wpPlacement = null;
    draw();
  }

  // --- Renderizado Principal del Canvas ---
  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Dibujar Imagen del Mapa Ocupacional
    if (mapImage.complete && mapImage.width > 0) {
      ctx.drawImage(
        mapImage,
        view.x,
        view.y,
        mapImage.width * view.zoom,
        mapImage.height * view.zoom
      );
    }

    // 2. Dibujar Zonas Poligonales
    for (const [id, z] of Object.entries(zones)) {
      if (!z.polygon || z.polygon.length < 3) continue;
      ctx.beginPath();
      const first = worldToScreen(z.polygon[0][0], z.polygon[0][1]);
      ctx.moveTo(first.x, first.y);
      for (let i = 1; i < z.polygon.length; i++) {
        const pt = worldToScreen(z.polygon[i][0], z.polygon[i][1]);
        ctx.lineTo(pt.x, pt.y);
      }
      ctx.closePath();

      // Relleno translúcido
      ctx.fillStyle = z.color ? hexToRgba(z.color, 0.28) : 'rgba(0, 210, 255, 0.25)';
      ctx.fill();
      ctx.strokeStyle = z.color || '#00d2ff';
      ctx.lineWidth = (selectedId === id) ? 3.5 : 2;
      ctx.stroke();

      // Etiqueta centrada en la zona
      let cx = 0, cy = 0;
      z.polygon.forEach(p => { cx += p[0]; cy += p[1]; });
      cx /= z.polygon.length;
      cy /= z.polygon.length;
      const sCenter = worldToScreen(cx, cy);

      ctx.fillStyle = 'rgba(11, 15, 23, 0.75)';
      ctx.font = 'bold 12px sans-serif';
      const textWidth = ctx.measureText(z.name).width;
      ctx.fillRect(sCenter.x - textWidth/2 - 6, sCenter.y - 10, textWidth + 12, 20);

      ctx.fillStyle = z.color || '#00d2ff';
      ctx.textAlign = 'center';
      ctx.fillText(z.name, sCenter.x, sCenter.y + 4);
    }

    // Dibujar polígono en progreso (mientras se dibuja)
    if (currentPolygon.length > 0) {
      ctx.beginPath();
      const p0 = worldToScreen(currentPolygon[0][0], currentPolygon[0][1]);
      ctx.moveTo(p0.x, p0.y);
      for (let i = 1; i < currentPolygon.length; i++) {
        const pt = worldToScreen(currentPolygon[i][0], currentPolygon[i][1]);
        ctx.lineTo(pt.x, pt.y);
      }
      ctx.strokeStyle = '#ff9d00';
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      currentPolygon.forEach(pt => {
        const s = worldToScreen(pt[0], pt[1]);
        ctx.fillStyle = '#ff9d00';
        ctx.beginPath();
        ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
        ctx.fill();
      });
    }

    // 3. Dibujar Waypoints con Flecha de Orientación (Yaw)
    for (const [id, wp] of Object.entries(waypoints)) {
      const s = worldToScreen(wp.x, wp.y);
      const isSel = (selectedId === id);

      // Círculo base del waypoint
      ctx.fillStyle = isSel ? '#00f090' : '#00d2ff';
      ctx.beginPath();
      ctx.arc(s.x, s.y, isSel ? 7 : 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Flecha de orientación Yaw (en ROS: 0 rad = +X hacia adelante, en pantalla rotar según el mapa)
      const yawRad = (wp.yaw_deg || 0) * (Math.PI / 180.0);
      const arrowLen = 22 * Math.min(view.zoom, 1.8);
      // Nota: en la pantalla Y crece hacia abajo, por eso el seno es negativo
      const endX = s.x + Math.cos(yawRad) * arrowLen;
      const endY = s.y - Math.sin(yawRad) * arrowLen;

      ctx.beginPath();
      ctx.moveTo(s.x, s.y);
      ctx.lineTo(endX, endY);
      ctx.strokeStyle = '#00f090';
      ctx.lineWidth = 2.5;
      ctx.stroke();

      // Punta de flecha
      const angle = Math.atan2(s.y - endY, endX - s.x);
      ctx.fillStyle = '#00f090';
      ctx.beginPath();
      ctx.moveTo(endX, endY);
      ctx.lineTo(endX - 7 * Math.cos(angle - Math.PI / 6), endY + 7 * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(endX - 7 * Math.cos(angle + Math.PI / 6), endY + 7 * Math.sin(angle + Math.PI / 6));
      ctx.fill();

      // Nombre del waypoint
      ctx.font = 'bold 11px sans-serif';
      ctx.fillStyle = '#fff';
      ctx.textAlign = 'left';
      ctx.fillText(wp.name, s.x + 10, s.y - 6);
    }

    // Dibujar flecha provisional de waypoint si se está arrastrando
    if (wpPlacement && wpPlacement.endScreen) {
      ctx.beginPath();
      ctx.moveTo(wpPlacement.startScreen.x, wpPlacement.startScreen.y);
      ctx.lineTo(wpPlacement.endScreen.x, wpPlacement.endScreen.y);
      ctx.strokeStyle = '#ff3366';
      ctx.lineWidth = 3;
      ctx.stroke();
    }
  }

  function hexToRgba(hex, alpha) {
    const c = hex.replace('#', '');
    const r = parseInt(c.substring(0, 2), 16) || 0;
    const g = parseInt(c.substring(2, 4), 16) || 0;
    const b = parseInt(c.substring(4, 6), 16) || 0;
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  // --- Manejadores de Ratón ---
  canvas.addEventListener('mousedown', (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    if (currentMode === 'pan') {
      isDragging = true;
      dragStart = { x: sx - view.x, y: sy - view.y };
    } else if (currentMode === 'zone') {
      const w = screenToWorld(sx, sy);
      currentPolygon.push([Math.round(w.x * 100) / 100, Math.round(w.y * 100) / 100]);
      draw();
    } else if (currentMode === 'waypoint') {
      const w = screenToWorld(sx, sy);
      wpPlacement = {
        worldPos: w,
        startScreen: { x: sx, y: sy },
        endScreen: { x: sx, y: sy }
      };
      isDragging = true;
    } else if (currentMode === 'seat') {
      const w = screenToWorld(sx, sy);
      promptNewSeat(w.x, w.y);
    }
  });

  canvas.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;

    // Actualizar coordenadas métricas del cursor
    const w = screenToWorld(sx, sy);
    document.getElementById('coord-x').innerText = (w.x >= 0 ? '+' : '') + w.x.toFixed(2);
    document.getElementById('coord-y').innerText = (w.y >= 0 ? '+' : '') + w.y.toFixed(2);
    document.getElementById('zoom-level').innerText = Math.round(view.zoom * 100);

    if (isDragging) {
      if (currentMode === 'pan') {
        view.x = sx - dragStart.x;
        view.y = sy - dragStart.y;
        draw();
      } else if (currentMode === 'waypoint' && wpPlacement) {
        wpPlacement.endScreen = { x: sx, y: sy };
        draw();
      }
    }
  });

  window.addEventListener('mouseup', (e) => {
    if (isDragging) {
      isDragging = false;
      if (currentMode === 'waypoint' && wpPlacement) {
        // Calcular ángulo Yaw en grados
        const dx = wpPlacement.endScreen.x - wpPlacement.startScreen.x;
        const dy = wpPlacement.startScreen.y - wpPlacement.endScreen.y; // Invertir Y para ROS
        let yawRad = Math.atan2(dy, dx);
        let yawDeg = Math.round(yawRad * (180.0 / Math.PI));
        promptNewWaypoint(wpPlacement.worldPos.x, wpPlacement.worldPos.y, yawDeg);
        wpPlacement = null;
        draw();
      }
    }
  });

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;

    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    const newZoom = Math.max(0.4, Math.min(10.0, view.zoom * zoomFactor));

    view.x = mx - (mx - view.x) * (newZoom / view.zoom);
    view.y = my - (my - view.y) * (newZoom / view.zoom);
    view.zoom = newZoom;
    draw();
  });

  // Doble clic para terminar el polígono de una zona
  canvas.addEventListener('dblclick', (e) => {
    if (currentMode === 'zone' && currentPolygon.length >= 3) {
      promptNewZone();
    }
  });

  // --- Diálogos de Creación ---
  function promptNewZone() {
    const defaultName = `Zona_${Object.keys(zones).length + 1}`;
    const name = prompt('Nombre de la nueva Zona (ej: Entrada, Sala, Sofá):', defaultName);
    if (!name) {
      currentPolygon = [];
      draw();
      return;
    }

    const id = name.toLowerCase().replace(/\\s+/g, '_');
    const colors = ['#00d2ff', '#00f090', '#ffaa00', '#ff3366', '#a070ff', '#ffdd00'];
    const chosenColor = colors[Object.keys(zones).length % colors.length];

    zones[id] = {
      name: name,
      type: id.includes('entr') ? 'entrance' : (id.includes('sala') || id.includes('living') ? 'living_room' : 'custom'),
      color: chosenColor,
      polygon: [...currentPolygon]
    };

    currentPolygon = [];
    setMode('pan');
    updateSidebar();
    showToast(`Zona '${name}' creada con éxito.`);
  }

  function promptNewWaypoint(x, y, yawDeg) {
    const defaultName = `punto_${Object.keys(waypoints).length + 1}`;
    const name = prompt('Nombre del Waypoint (ej: door, living_room, mesa):', defaultName);
    if (!name) return;

    waypoints[name] = {
      name: name,
      zone: 'living_room',
      x: Math.round(x * 1000) / 1000,
      y: Math.round(y * 1000) / 1000,
      yaw_deg: yawDeg
    };

    setMode('pan');
    updateSidebar();
    showToast(`Waypoint '${name}' creado.`);
  }

  function promptNewSeat(x, y) {
    const name = prompt('Nombre del Asiento (ej: Sofá, Silla 1, Sillón):', 'Sofá');
    if (!name) return;

    const id = name.toLowerCase().replace(/\\s+/g, '_');
    seats[id] = {
      name: name,
      zone: 'living_room',
      type: id.includes('sill') ? 'armchair' : 'couch',
      arm_gesture: id.includes('sill') ? 'point_chair' : 'point_couch',
      capacity: id.includes('sof') ? 2 : 1,
      x: Math.round(x * 100) / 100,
      y: Math.round(y * 100) / 100
    };

    setMode('pan');
    updateSidebar();
    showToast(`Asiento '${name}' registrado.`);
  }

  // --- Actualización de la Barra Lateral ---
  function updateSidebar() {
    // Zonas
    const zContainer = document.getElementById('zones-list');
    zContainer.innerHTML = '';
    const zKeys = Object.keys(zones);
    document.getElementById('zone-count').innerText = zKeys.length;

    zKeys.forEach(id => {
      const z = zones[id];
      const card = document.createElement('div');
      card.className = `item-card ${selectedId === id ? 'selected' : ''}`;
      card.innerHTML = `
        <div class="item-info">
          <div class="color-badge" style="background: ${z.color || '#00d2ff'};"></div>
          <div>
            <div class="item-title">${z.name}</div>
            <div class="item-subtitle">${z.polygon ? z.polygon.length : 0} vértices | ${z.type || 'área'}</div>
          </div>
        </div>
        <button class="del-btn" onclick="deleteItem('zone', '${id}', event)">✕</button>
      `;
      card.onclick = () => selectItem(id);
      zContainer.appendChild(card);
    });

    // Waypoints
    const wpContainer = document.getElementById('waypoints-list');
    wpContainer.innerHTML = '';
    const wpKeys = Object.keys(waypoints);
    document.getElementById('wp-count').innerText = wpKeys.length;

    wpKeys.forEach(id => {
      const wp = waypoints[id];
      const card = document.createElement('div');
      card.className = `item-card ${selectedId === id ? 'selected' : ''}`;
      card.innerHTML = `
        <div class="item-info">
          <div>📍</div>
          <div>
            <div class="item-title">${wp.name}</div>
            <div class="item-subtitle">X: ${wp.x.toFixed(2)}, Y: ${wp.y.toFixed(2)} | Yaw: ${wp.yaw_deg}°</div>
          </div>
        </div>
        <button class="del-btn" onclick="deleteItem('waypoint', '${id}', event)">✕</button>
      `;
      card.onclick = () => selectItem(id);
      wpContainer.appendChild(card);
    });

    // Asientos
    const seatContainer = document.getElementById('seats-list');
    seatContainer.innerHTML = '';
    const sKeys = Object.keys(seats);
    document.getElementById('seat-count').innerText = sKeys.length;

    sKeys.forEach(id => {
      const s = seats[id];
      const card = document.createElement('div');
      card.className = `item-card ${selectedId === id ? 'selected' : ''}`;
      card.innerHTML = `
        <div class="item-info">
          <div>🛋️</div>
          <div>
            <div class="item-title">${s.name}</div>
            <div class="item-subtitle">Gesto: ${s.arm_gesture || 'point_couch'} | Cap: ${s.capacity || 1}</div>
          </div>
        </div>
        <button class="del-btn" onclick="deleteItem('seat', '${id}', event)">✕</button>
      `;
      card.onclick = () => selectItem(id);
      seatContainer.appendChild(card);
    });

    draw();
  }

  function selectItem(id) {
    selectedId = (selectedId === id) ? null : id;
    updateSidebar();
  }

  function deleteItem(type, id, event) {
    event.stopPropagation();
    if (!confirm(`¿Eliminar ${id}?`)) return;

    if (type === 'zone') delete zones[id];
    else if (type === 'waypoint') delete waypoints[id];
    else if (type === 'seat') delete seats[id];

    if (selectedId === id) selectedId = null;
    updateSidebar();
    showToast(`Elemento '${id}' eliminado.`);
  }

  async function saveData() {
    try {
      const payload = { zones: zones, waypoints: waypoints, seats: seats };
      const res = await fetch('/api/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      if (data.status === 'ok') {
        showToast('✔ Zonas y Waypoints guardados y sincronizados con Nav2!');
      } else {
        alert('Error guardando: ' + (data.message || 'error desconocido'));
      }
    } catch (e) {
      alert('Error de conexión con el servidor: ' + e);
    }
  }

  function showToast(msg) {
    const t = document.getElementById('toast');
    t.innerText = msg;
    t.classList.add('show');
    setTimeout(() => t.classList.remove('show'), 2800);
  }

  // Iniciar al cargar la ventana
  window.onload = () => {
    resizeCanvas();
    loadData();
  };
</script>
</body>
</html>
"""


class MapZoneEditorHandler(BaseHTTPRequestHandler):
    """Manejador de rutas HTTP y API REST para el editor web."""

    manager: SemanticMapManager = None

    def log_message(self, format, *args):
        # Silenciar logs ruidosos de peticiones estáticas
        return

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))

        elif self.path == "/api/map.png":
            if self.manager.png_bytes:
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Cache-Control", "public, max-age=3600")
                self.end_headers()
                self.wfile.write(self.manager.png_bytes)
            else:
                self.send_error(404, "Imagen del mapa no disponible")

        elif self.path == "/api/data":
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.end_headers()
            payload = {
                "map_info": self.manager.map_info,
                "zones": self.manager.zones,
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
        else:
            self.send_error(404, "Ruta no encontrada")


def main():
    parser = argparse.ArgumentParser(description="Base@Home - Editor Web de Zonas y Mapa Semántico")
    parser.add_argument("--map", type=str, default=DEFAULT_MAP_YAML, help="Ruta al archivo mapa.yaml")
    parser.add_argument("--semantic", type=str, default=DEFAULT_SEMANTIC_YAML, help="Ruta al archivo semantic_map.yaml")
    parser.add_argument("--port", type=int, default=8080, help="Puerto del servidor web local")
    parser.add_argument("--no-browser", action="store_true", help="No abrir el navegador automáticamente")
    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("🗺️  BASE@HOME - FRAMEWORK Y EDITOR WEB DE ZONAS SEMANTICAS")
    print("=" * 70)
    print(f"  Mapa Base:       {args.map}")
    print(f"  Mapa Semántico:  {args.semantic}")
    print(f"  URL del Visor:   http://localhost:{args.port}/")
    print("=" * 70 + "\n")

    manager = SemanticMapManager(map_yaml_path=args.map, semantic_yaml_path=args.semantic)
    MapZoneEditorHandler.manager = manager

    server = ThreadedHTTPServer(("0.0.0.0", args.port), MapZoneEditorHandler)

    if not args.no_browser:
        # Abrir navegador tras medio segundo
        threading.Timer(0.6, lambda: webbrowser.open(f"http://localhost:{args.port}/")).start()

    try:
        print(f"🚀 Servidor activo en http://localhost:{args.port}/")
        print("   Presiona Ctrl+C para detener el servidor.\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Editor]: Cerrando servidor...")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
