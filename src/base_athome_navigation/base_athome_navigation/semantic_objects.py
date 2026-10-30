#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Módulo de gestión de objetos semánticos para Base@Home (RoboCup@Home).

Provee funciones puras para:
- Asignación y validación de número único de objeto.
- Detección de mesa contenedora mediante prueba punto-en-polígono.
- Cálculo automático de posición de aproximación (approach) a distancia configurable (default 0.60m)
  con orientación (yaw) mirando hacia el objeto.
- Creación y validación de entradas de objetos en semantic_map.yaml.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

ALLOWED_OBJECT_CLASSES = ["can", "bottle", "cup", "box", "other"]
DEFAULT_APPROACH_DISTANCE_M = 0.60
DEFAULT_OBJECT_CLEARANCE_M = 0.10
FALLBACK_OBJECT_Z_M = 0.865


def point_in_polygon(x: float, y: float, polygon: List[List[float]]) -> bool:
    """
    Determina si el punto (x, y) cae dentro del polígono 2D (algoritmo PNPOLY ray-casting).
    """
    if not polygon or len(polygon) < 3:
        return False
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = float(polygon[i][0]), float(polygon[i][1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def find_containing_furniture(
    x: float, y: float, furniture: Dict[str, Any]
) -> Tuple[Optional[str], Optional[Dict[str, Any]]]:
    """
    Busca si las coordenadas (x, y) caen dentro del polígono de alguna mesa/mueble.
    Retorna (furniture_id, furniture_data) o (None, None).
    """
    if not furniture:
        return None, None

    for f_id, f_data in furniture.items():
        if not isinstance(f_data, dict):
            continue
        poly = f_data.get("polygon")
        if poly and point_in_polygon(x, y, poly):
            return f_id, f_data

    return None, None


def get_next_free_number(objects: Dict[str, Any]) -> int:
    """
    Retorna el siguiente número entero positivo libre (1, 2, 3...) no utilizado en objects.
    """
    if not objects:
        return 1

    used_numbers = set()
    for obj in objects.values():
        if isinstance(obj, dict) and "number" in obj:
            try:
                num = int(obj["number"])
                if num > 0:
                    used_numbers.add(num)
            except (ValueError, TypeError):
                continue

    candidate = 1
    while candidate in used_numbers:
        candidate += 1
    return candidate


def is_number_unique(
    objects: Dict[str, Any], number: int, current_id: Optional[str] = None
) -> bool:
    """
    Verifica si el número especificado no se repite entre los demás objetos.
    Permite ignorar el objeto con ID current_id en caso de edición.
    """
    try:
        num = int(number)
    except (ValueError, TypeError):
        return False

    for obj_id, obj in objects.items():
        if current_id and obj_id == current_id:
            continue
        if isinstance(obj, dict) and obj.get("number") == num:
            return False

    return True


def calculate_approach_pose(
    obj_x: float,
    obj_y: float,
    ref_center: Optional[Tuple[float, float]] = None,
    distance: float = DEFAULT_APPROACH_DISTANCE_M,
) -> Dict[str, float]:
    """
    Calcula el punto de approach a 'distance' metros del objeto.
    Si se provee ref_center (por ejemplo, centro de la mesa), el approach se ubica
    alejándose del centro hacia el objeto (hacia el espacio exterior de la mesa).
    El ángulo yaw del approach siempre queda orientado mirando directamente hacia el objeto.
    """
    if ref_center is not None:
        cx, cy = ref_center
        dx = obj_x - cx
        dy = obj_y - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-4:
            # Si el objeto está exactamente en el centro, alejarse en dirección +X por defecto
            ux, uy = 1.0, 0.0
        else:
            ux = dx / dist
            uy = dy / dist
    else:
        # Si no hay centro de referencia, por defecto aproximar desde -X (robot se para en obj_x - dist)
        # o alejándose del origen (0, 0)
        dist_from_origin = math.hypot(obj_x, obj_y)
        if dist_from_origin > 1e-4:
            ux = obj_x / dist_from_origin
            uy = obj_y / dist_from_origin
        else:
            ux, uy = 1.0, 0.0

    app_x = round(obj_x + distance * ux, 4)
    app_y = round(obj_y + distance * uy, 4)

    # Vector desde la postura de approach hacia el objeto
    look_dx = obj_x - app_x
    look_dy = obj_y - app_y
    yaw_rad = math.atan2(look_dy, look_dx)
    yaw_deg = round(math.degrees(yaw_rad), 1)

    return {
        "x": app_x,
        "y": app_y,
        "yaw_deg": yaw_deg,
    }


def create_object_entry(
    number: int,
    name: str,
    obj_class: str,
    x: float,
    y: float,
    z: Optional[float] = None,
    on: Optional[str] = None,
    approach: Optional[Dict[str, float]] = None,
    furniture: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Crea una entrada de objeto válida y estandarizada para semantic_map.yaml.
    Detecta automáticamente la mesa contenedora y la altura Z por defecto si no se especifican.
    """
    obj_class_clean = str(obj_class).strip().lower()
    if obj_class_clean not in ALLOWED_OBJECT_CLASSES:
        obj_class_clean = "other"

    detected_on = ""
    table_height = 0.76
    ref_center = None

    if furniture is not None:
        f_id, f_data = find_containing_furniture(x, y, furniture)
        if f_id and f_data:
            detected_on = f_id
            table_height = float(f_data.get("height_m", 0.76))
            center = f_data.get("center")
            if center and "x" in center and "y" in center:
                ref_center = (float(center["x"]), float(center["y"]))

    final_on = detected_on if on is None else str(on)

    if z is None:
        if detected_on:
            final_z = round(table_height + DEFAULT_OBJECT_CLEARANCE_M, 3)
        else:
            final_z = FALLBACK_OBJECT_Z_M
    else:
        final_z = round(float(z), 3)

    if approach is None:
        approach = calculate_approach_pose(
            x, y, ref_center=ref_center, distance=DEFAULT_APPROACH_DISTANCE_M
        )
    else:
        # Asegurar cálculo de yaw si no viene o recalcular
        app_x = float(approach.get("x", x))
        app_y = float(approach.get("y", y))
        yaw_deg = approach.get("yaw_deg")
        if yaw_deg is None:
            look_dx = x - app_x
            look_dy = y - app_y
            yaw_deg = round(math.degrees(math.atan2(look_dy, look_dx)), 1)
        approach = {
            "x": round(app_x, 4),
            "y": round(app_y, 4),
            "yaw_deg": round(float(yaw_deg), 1),
        }

    return {
        "number": int(number),
        "name": str(name).strip(),
        "class": obj_class_clean,
        "x": round(float(x), 4),
        "y": round(float(y), 4),
        "z": final_z,
        "on": final_on,
        "approach": approach,
    }


def validate_object_entry(
    entry: Dict[str, Any],
    existing_objects: Optional[Dict[str, Any]] = None,
    current_id: Optional[str] = None,
) -> Tuple[bool, str]:
    """
    Valida que una entrada cumpla con todos los requisitos de tipo y unicidad.
    """
    if not isinstance(entry, dict):
        return False, "La entrada del objeto debe ser un diccionario"

    for req_key in ["number", "name", "class", "x", "y", "z", "on", "approach"]:
        if req_key not in entry:
            return False, f"Falta el campo requerido '{req_key}'"

    try:
        num = int(entry["number"])
        if num <= 0:
            return False, "El número del objeto debe ser un entero positivo"
    except (ValueError, TypeError):
        return False, "El número del objeto debe ser un entero válido"

    if existing_objects and not is_number_unique(existing_objects, num, current_id):
        return False, f"El número {num} ya está asignado a otro objeto"

    if entry["class"] not in ALLOWED_OBJECT_CLASSES:
        return False, f"Clase no válida '{entry['class']}'. Debe ser una de: {ALLOWED_OBJECT_CLASSES}"

    app = entry.get("approach")
    if not isinstance(app, dict) or "x" not in app or "y" not in app or "yaw_deg" not in app:
        return False, "El campo 'approach' debe contener x, y, yaw_deg"

    return True, "Válido"
