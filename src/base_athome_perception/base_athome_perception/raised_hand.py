#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Deteccion de Clientes con Mano Levantada (Restaurant / HRI)
# ==============================================================================
"""Logica pura y determinista para clasificar manos levantadas en personas,
filtrar gestos transitorios con una ventana temporal (N de M cuadros) y ordenar
los clientes detectados segun distancia al robot u orden de aparicion.

Soporta esqueletos en formato COCO-17 (usado por YOLOv8-pose y person_pose_detector)
y OpenPose BODY_25 mediante mapeo documentado de keypoints. Sin dependencias de ROS.
"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

# ------------------------------------------------------------------------------
# Indices Anatomicos COCO-17
# ------------------------------------------------------------------------------
COCO_NOSE = 0
COCO_L_EYE = 1
COCO_R_EYE = 2
COCO_L_EAR = 3
COCO_R_EAR = 4
COCO_L_SHOULDER = 5
COCO_R_SHOULDER = 6
COCO_L_ELBOW = 7
COCO_R_ELBOW = 8
COCO_L_WRIST = 9
COCO_R_WRIST = 10
COCO_L_HIP = 11
COCO_R_HIP = 12
COCO_L_KNEE = 13
COCO_R_KNEE = 14
COCO_L_ANKLE = 15
COCO_R_ANKLE = 16

# ------------------------------------------------------------------------------
# Mapeo OpenPose BODY_25 -> COCO-17
#
# Formato OpenPose BODY_25 (Cao et al.):
#  0: Nose, 1: Neck, 2: RShoulder, 3: RElbow, 4: RWrist,
#  5: LShoulder, 6: LElbow, 7: LWrist, 8: MidHip,
#  9: RHip, 10: RKnee, 11: RAnkle, 12: LHip, 13: LKnee, 14: LAnkle,
#  15: REye, 16: LEye, 17: REar, 18: LEar,
#  19-24: Pies (L/R BigToe, SmallToe, Heel)
#
# Correspondencia biunivoca hacia los 17 puntos COCO:
# ------------------------------------------------------------------------------
BODY25_TO_COCO_INDEXES: Dict[int, int] = {
    COCO_NOSE: 0,          # Nose
    COCO_L_EYE: 16,        # LEye
    COCO_R_EYE: 15,        # REye
    COCO_L_EAR: 18,        # LEar
    COCO_R_EAR: 17,        # REar
    COCO_L_SHOULDER: 5,    # LShoulder
    COCO_R_SHOULDER: 2,    # RShoulder
    COCO_L_ELBOW: 6,       # LElbow
    COCO_R_ELBOW: 3,       # RElbow
    COCO_L_WRIST: 7,       # LWrist
    COCO_R_WRIST: 4,       # RWrist
    COCO_L_HIP: 12,        # LHip
    COCO_R_HIP: 9,         # RHip
    COCO_L_KNEE: 13,       # LKnee
    COCO_R_KNEE: 10,       # RKnee
    COCO_L_ANKLE: 14,      # LAnkle
    COCO_R_ANKLE: 11,      # RAnkle
}


def body25_to_coco(
    keypoints: Sequence[Sequence[float]],
    conf: Optional[Sequence[float]] = None
) -> Tuple[List[Tuple[float, float]], List[float]]:
    """Convierte un esqueleto OpenPose BODY_25 (25 keypoints) a COCO-17.

    Acepta:
      - keypoints: lista/array de forma (25, 2) o (25, 3) si incluye confianza.
      - conf: lista/array opcional de forma (25,).
    Devuelve:
      - (coco_points, coco_conf) donde coco_points tiene 17 puntos (x, y) y coco_conf 17 valores.
    """
    coco_points: List[Tuple[float, float]] = []
    coco_conf: List[float] = []

    for coco_idx in range(17):
        b25_idx = BODY25_TO_COCO_INDEXES[coco_idx]
        if b25_idx < len(keypoints):
            pt = keypoints[b25_idx]
            x, y = float(pt[0]), float(pt[1])
            if conf is not None and b25_idx < len(conf):
                c = float(conf[b25_idx])
            elif len(pt) >= 3:
                c = float(pt[2])
            else:
                c = 1.0
        else:
            x, y, c = 0.0, 0.0, 0.0

        coco_points.append((x, y))
        coco_conf.append(c)

    return coco_points, coco_conf


def normalize_keypoints(
    keypoints: Any,
    conf: Optional[Any] = None
) -> Tuple[List[Tuple[float, float]], List[float]]:
    """Normaliza keypoints de entrada (COCO-17, BODY_25, dicts de person_pose_detector)
    a tuplas estandar (puntos_xy, confianzas) de 17 elementos.
    """
    # Si viene empaquetado como dict (como en person_pose_detector.detect_poses)
    if isinstance(keypoints, dict):
        raw_kpts = keypoints.get("keypoints", [])
        raw_conf = keypoints.get("keypoints_conf", None)
        return normalize_keypoints(raw_kpts, raw_conf)

    pts_list = list(keypoints)
    n_pts = len(pts_list)

    if n_pts == 25:
        # Formato BODY_25
        return body25_to_coco(pts_list, conf)
    elif n_pts == 17:
        # Formato COCO-17
        out_pts: List[Tuple[float, float]] = []
        out_conf: List[float] = []
        for i, pt in enumerate(pts_list):
            x, y = float(pt[0]), float(pt[1])
            if conf is not None and i < len(conf):
                c = float(conf[i])
            elif len(pt) >= 3:
                c = float(pt[2])
            else:
                c = 1.0
            out_pts.append((x, y))
            out_conf.append(c)
        return out_pts, out_conf
    else:
        # Caso degradado: devolver lista rellena o vacia
        out_pts = [(float(pt[0]), float(pt[1])) for pt in pts_list[:17]]
        out_conf = [float(conf[i]) if (conf is not None and i < len(conf)) else 1.0 for i in range(len(out_pts))]
        while len(out_pts) < 17:
            out_pts.append((0.0, 0.0))
            out_conf.append(0.0)
        return out_pts, out_conf


# ------------------------------------------------------------------------------
# Clasificacion de Mano Levantada (Cuadro Individual)
# ------------------------------------------------------------------------------
@dataclass
class RaisedHandState:
    NONE: str = "none"
    LEFT: str = "left"
    RIGHT: str = "right"
    BOTH: str = "both"


@dataclass
class RaisedHandClassification:
    state: str                     # "none", "left", "right", "both"
    is_raised: bool                # True si state != "none"
    left_raised: bool
    right_raised: bool
    left_wrist_conf: float
    right_wrist_conf: float
    description: str


def classify_raised_hand(
    keypoints: Any,
    conf: Optional[Any] = None,
    conf_thresh: float = 0.40
) -> RaisedHandClassification:
    """Evalua si una persona tiene la mano levantada en un solo cuadro.

    Regla geometrica (coordenadas de imagen: Y crece hacia abajo):
      - Muneca sobre hombro respectivo (y_wrist < y_shoulder)
      - O muneca sobre la nariz (y_wrist < y_nose)
      - Sujeto a que las articulaciones tengan confianza >= conf_thresh.
    """
    pts, confs = normalize_keypoints(keypoints, conf)

    nose_valid = confs[COCO_NOSE] >= conf_thresh
    nose_y = pts[COCO_NOSE][1]

    # Evaluacion brazo izquierdo
    l_wrist_conf = confs[COCO_L_WRIST]
    l_shoulder_conf = confs[COCO_L_SHOULDER]
    l_wrist_y = pts[COCO_L_WRIST][1]
    l_shoulder_y = pts[COCO_L_SHOULDER][1]

    l_up = False
    if l_wrist_conf >= conf_thresh:
        over_shoulder = (l_shoulder_conf >= conf_thresh) and (l_wrist_y < l_shoulder_y)
        over_nose = nose_valid and (l_wrist_y < nose_y)
        l_up = over_shoulder or over_nose

    # Evaluacion brazo derecho
    r_wrist_conf = confs[COCO_R_WRIST]
    r_shoulder_conf = confs[COCO_R_SHOULDER]
    r_wrist_y = pts[COCO_R_WRIST][1]
    r_shoulder_y = pts[COCO_R_SHOULDER][1]

    r_up = False
    if r_wrist_conf >= conf_thresh:
        over_shoulder = (r_shoulder_conf >= conf_thresh) and (r_wrist_y < r_shoulder_y)
        over_nose = nose_valid and (r_wrist_y < nose_y)
        r_up = over_shoulder or over_nose

    if l_up and r_up:
        state = RaisedHandState.BOTH
        desc = "Ambas manos levantadas"
    elif l_up:
        state = RaisedHandState.LEFT
        desc = "Mano izquierda levantada"
    elif r_up:
        state = RaisedHandState.RIGHT
        desc = "Mano derecha levantada"
    else:
        state = RaisedHandState.NONE
        desc = "Sin manos levantadas"

    return RaisedHandClassification(
        state=state,
        is_raised=(state != RaisedHandState.NONE),
        left_raised=l_up,
        right_raised=r_up,
        left_wrist_conf=l_wrist_conf,
        right_wrist_conf=r_wrist_conf,
        description=desc,
    )


# ------------------------------------------------------------------------------
# Filtro Temporal (N de M cuadros)
# ------------------------------------------------------------------------------
class TemporalRaisedHandFilter:
    """Filtro de ventana deslizante temporal para evitar disparar con ruidos,
    falsos positivos transitorios o gestos de un unico cuadro.

    Requiere que al menos `min_raised_frames` (N) de los ultimos `window_size` (M)
    cuadros hayan clasificado la mano como levantada.
    """

    def __init__(self, window_size: int = 5, min_raised_frames: int = 3):
        if min_raised_frames > window_size:
            raise ValueError("min_raised_frames no puede ser mayor que window_size.")
        self.window_size = window_size
        self.min_raised_frames = min_raised_frames
        self.history: deque = deque(maxlen=window_size)
        self.state_history: deque = deque(maxlen=window_size)

    def update(
        self,
        keypoints_or_is_raised: Union[bool, Any],
        conf: Optional[Any] = None,
        conf_thresh: float = 0.40
    ) -> Tuple[bool, str]:
        """Agrega un cuadro al historial y devuelve (is_confirmed_raised, dominant_state).

        Puede recibir directamente un booleano `is_raised` o los keypoints para clasificar.
        """
        if isinstance(keypoints_or_is_raised, bool):
            is_raised = keypoints_or_is_raised
            state = RaisedHandState.LEFT if is_raised else RaisedHandState.NONE
        else:
            cls = classify_raised_hand(keypoints_or_is_raised, conf=conf, conf_thresh=conf_thresh)
            is_raised = cls.is_raised
            state = cls.state

        self.history.append(is_raised)
        self.state_history.append(state)

        raised_count = sum(1 for val in self.history if val)
        is_confirmed = (raised_count >= self.min_raised_frames)

        if not is_confirmed:
            return False, RaisedHandState.NONE

        # Determinar el estado dominante (both > left/right)
        raised_states = [s for s in self.state_history if s != RaisedHandState.NONE]
        if not raised_states:
            return True, RaisedHandState.LEFT

        # Si hay mayoria de both
        if raised_states.count(RaisedHandState.BOTH) >= (len(raised_states) / 2):
            dominant = RaisedHandState.BOTH
        else:
            dominant = max(set(raised_states), key=raised_states.count)

        return True, dominant

    def reset(self) -> None:
        """Reinicia el historial de la ventana."""
        self.history.clear()
        self.state_history.clear()


class MultiPersonRaisedHandTracker:
    """Mantiene filtros temporales independientes indexados por id de persona."""

    def __init__(self, window_size: int = 5, min_raised_frames: int = 3):
        self.window_size = window_size
        self.min_raised_frames = min_raised_frames
        self.filters: Dict[Any, TemporalRaisedHandFilter] = {}

    def update_person(
        self,
        person_id: Any,
        keypoints: Any,
        conf: Optional[Any] = None,
        conf_thresh: float = 0.40
    ) -> Tuple[bool, str]:
        if person_id not in self.filters:
            self.filters[person_id] = TemporalRaisedHandFilter(
                window_size=self.window_size,
                min_raised_frames=self.min_raised_frames
            )
        return self.filters[person_id].update(keypoints, conf=conf, conf_thresh=conf_thresh)

    def reset_person(self, person_id: Any) -> None:
        if person_id in self.filters:
            self.filters[person_id].reset()

    def clear(self) -> None:
        self.filters.clear()


# ------------------------------------------------------------------------------
# Ordenamiento de Clientes
# ------------------------------------------------------------------------------
def _extract_distance(client: Dict[str, Any], robot_pos: Tuple[float, float]) -> float:
    """Extrae o calcula la distancia de un cliente respecto a robot_pos."""
    if "distance" in client and client["distance"] is not None:
        return float(client["distance"])
    if "dist" in client and client["dist"] is not None:
        return float(client["dist"])

    # Calcular distancia euclidiana si viene posicion espacial
    pos = client.get("position") or client.get("pos") or client.get("map_pos") or client.get("coords")
    if pos is not None and len(pos) >= 2:
        dx = float(pos[0]) - float(robot_pos[0])
        dy = float(pos[1]) - float(robot_pos[1])
        return math.hypot(dx, dy)

    # Si no hay informacion de distancia, devolver infinito
    return float("inf")


def order_clients(
    clients: Sequence[Dict[str, Any]],
    by: str = "distance",
    robot_pos: Tuple[float, float] = (0.0, 0.0),
    reverse: bool = False,
    only_calling: bool = False
) -> List[Dict[str, Any]]:
    """Ordena una coleccion de clientes segun criterio especificado.

    Parametros:
      - clients: lista de diccionarios representando clientes.
      - by: "distance" (mas cercano primero) o "detection" / "order" (orden temporal).
      - robot_pos: posicion (x, y) de referencia para el calculo de distancias.
      - reverse: invertir el orden resultante.
      - only_calling: si es True, descarta a los clientes que no esten llamando
        (aquellos con is_calling=False o hand_raised=False).
    """
    selected: List[Dict[str, Any]] = []

    for idx, c in enumerate(clients):
        # Filtrar si solo se requieren los que estan llamando
        if only_calling:
            is_calling = c.get("is_calling")
            if is_calling is None:
                is_calling = c.get("hand_raised") or c.get("is_raised") or False
            if not is_calling:
                continue

        # Copia superficial para preservar indices originales si no existen
        c_entry = dict(c)
        if "detection_index" not in c_entry:
            c_entry["detection_index"] = idx

        selected.append(c_entry)

    if by in ("distance", "dist"):
        selected.sort(key=lambda c: _extract_distance(c, robot_pos), reverse=reverse)
    elif by in ("detection", "order", "index"):
        selected.sort(key=lambda c: c.get("detection_index", 0), reverse=reverse)
    else:
        # Criterio desconocido: mantener orden estable
        pass

    return selected
