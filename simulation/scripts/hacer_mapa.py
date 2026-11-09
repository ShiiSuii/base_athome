#!/usr/bin/env python3
"""Hace el mapa de ocupacion de un mundo de Gazebo a partir de sus modelos estaticos.
Uso: hacer_mapa.py mundo.sdf salida [origen_x origen_y ancho_m alto_m]      (por defecto: origen (-5, -5), 12 x 10 m)
Escribe salida.pgm y salida.yaml, como los que guarda el robot con su mapeo. Solo entiende cajas: es lo que usan estos mundos."""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

RES_M = 0.05
ORIGIN = (-5.0, -5.0)
SIZE_M = (12.0, 10.0)
LASER_Z_M = 0.38          # el lidar solo ve lo que cruza esta altura, y el mapa real sale de el


def _pose(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))


def occupancy(world_path, origin=ORIGIN, size_m=SIZE_M):
    w, h = int(round(size_m[0] / RES_M)), int(round(size_m[1] / RES_M))
    grid = np.zeros((h, w), bool)
    for model in ET.parse(world_path).getroot().iter("model"):
        if model.findtext("static") != "true":
            continue
        mx, my, _, _, _, myaw = _pose(model.findtext("pose"))
        for col in model.iter("collision"):
            box = col.find("./geometry/box/size")
            if box is None:
                continue
            sx, sy, sz = [float(v) for v in box.text.split()]
            cx, cy, cz, _, _, cyaw = _pose(col.findtext("pose"))
            if not cz - sz / 2.0 <= LASER_Z_M <= cz + sz / 2.0:
                continue
            c, s = math.cos(myaw), math.sin(myaw)
            ox, oy = mx + c * cx - s * cy, my + s * cx + c * cy
            yaw = myaw + cyaw
            for i in np.arange(-sx / 2.0, sx / 2.0 + RES_M / 2.0, RES_M / 2.0):
                for j in np.arange(-sy / 2.0, sy / 2.0 + RES_M / 2.0, RES_M / 2.0):
                    x = ox + i * math.cos(yaw) - j * math.sin(yaw)
                    y = oy + i * math.sin(yaw) + j * math.cos(yaw)
                    col_i, row = int((x - origin[0]) / RES_M), int((y - origin[1]) / RES_M)
                    if 0 <= col_i < w and 0 <= row < h:
                        grid[row, col_i] = True
    return grid


def main():
    world, out = sys.argv[1], sys.argv[2]
    origin, size_m = ORIGIN, SIZE_M
    if len(sys.argv) >= 7:
        origin, size_m = (float(sys.argv[3]), float(sys.argv[4])), (float(sys.argv[5]), float(sys.argv[6]))
    grid = occupancy(world, origin, size_m)
    image = np.where(grid, 0, 254).astype(np.uint8)[::-1]
    with open(out + ".pgm", "wb") as f:
        f.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        f.write(image.tobytes())
    with open(out + ".yaml", "w", encoding="utf-8") as f:
        f.write(f"image: {os.path.basename(out)}.pgm\nmode: trinary\nresolution: {RES_M}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"{out}: {int(grid.sum())} celdas ocupadas")


if __name__ == "__main__":
    main()
