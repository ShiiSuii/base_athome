#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Emparejamiento de Personas para GPSR (RoboCup@Home)
# Gesto, Postura, Ropa y Secuencia de Nombres por Voz
# ==============================================================================
"""Logica pura y determinista para verificar si una persona detectada cumple
con los descriptores de personas utilizados en RoboCup@Home GPSR:
  - Gestos: waving, raising their left arm, raising their right arm,
            raising their arm, pointing to the left, pointing to the right.
  - Posturas: standing, sitting, lying (lying down).
  - Ropa: color + prenda (camisa/remera/chaqueta vs pantalon), integrado con
          PersonAttributes.
  - Nombres: estrategia de confirmacion por dialogo verbal (HRI social),
             dado que no se realiza reconocimiento facial sin entrenamiento.

Regla fundamental:
  Lo que no se pueda decidir con certeza geometrica o confianza de keypoints
  se clasifica estrictamente como 'desconocido' (UNKNOWN); NUNCA se adivina.
Sin dependencias de ROS ni llamadas de red.
"""

from dataclasses import dataclass, field
import math
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

try:
    from .raised_hand import (
        COCO_NOSE, COCO_L_EYE, COCO_R_EYE, COCO_L_EAR, COCO_R_EAR,
        COCO_L_SHOULDER, COCO_R_SHOULDER, COCO_L_ELBOW, COCO_R_ELBOW,
        COCO_L_WRIST, COCO_R_WRIST, COCO_L_HIP, COCO_R_HIP,
        COCO_L_KNEE, COCO_R_KNEE, COCO_L_ANKLE, COCO_R_ANKLE,
        normalize_keypoints,
    )
    from .person_attributes import (
        PersonAttributes, AttributeResult,
        ALL_COLORS, COLOR_DARK, COLOR_BLACK, COLOR_GRAY,
    )
except (ImportError, ValueError):
    # Fallback para ejecucion directa como script o test sin el paquete instalado
    try:
        from raised_hand import (
            COCO_NOSE, COCO_L_EYE, COCO_R_EYE, COCO_L_EAR, COCO_R_EAR,
            COCO_L_SHOULDER, COCO_R_SHOULDER, COCO_L_ELBOW, COCO_R_ELBOW,
            COCO_L_WRIST, COCO_R_WRIST, COCO_L_HIP, COCO_R_HIP,
            COCO_L_KNEE, COCO_R_KNEE, COCO_L_ANKLE, COCO_R_ANKLE,
            normalize_keypoints,
        )
        from person_attributes import (
            PersonAttributes, AttributeResult,
            ALL_COLORS, COLOR_DARK, COLOR_BLACK, COLOR_GRAY,
        )
    except (ImportError, ValueError):
        # Definicion autocontenida de constantes COCO si fallan las importaciones
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

        ALL_COLORS = (
            "red", "orange", "yellow", "green", "blue", "purple",
            "pink", "brown", "white", "gray", "dark", "black",
        )
        COLOR_DARK = "dark"
        COLOR_BLACK = "black"
        COLOR_GRAY = "gray"
        PersonAttributes = None
        AttributeResult = None
        normalize_keypoints = None


# ------------------------------------------------------------------------------
# Estados y Modelos de Datos
# ------------------------------------------------------------------------------
class MatchStatus:
    MATCH = "MATCH"
    NO_MATCH = "NO_MATCH"
    UNKNOWN = "UNKNOWN"


@dataclass
class PersonMatchResult:
    """Resultado de coincidencia de un descriptor GPSR sobre una persona."""
    matched: Optional[bool]  # True: coincide; False: no coincide; None: desconocido
    status: str              # MatchStatus: "MATCH", "NO_MATCH", "UNKNOWN"
    score: float             # Confianza / puntaje de coincidencia en [0.0, 1.0]
    reason: str              # Explicacion fundamentada del veredicto
    details: Dict[str, Any] = field(default_factory=dict)

    @property
    def is_match(self) -> bool:
        return self.matched is True

    @property
    def is_unknown(self) -> bool:
        return self.status == MatchStatus.UNKNOWN or self.matched is None


# ------------------------------------------------------------------------------
# Listas Oficiales GPSR y Sinonimos
# ------------------------------------------------------------------------------
OFFICIAL_GESTURES = (
    "pointing to the left",
    "pointing to the right",
    "raising their left arm",
    "raising their right arm",
    "raising their arm",
    "waving",
)

OFFICIAL_POSTURES = (
    "standing",
    "sitting",
    "lying",
)

GESTURE_SYNONYMS = {
    "pointing to the left": "pointing_left",
    "pointing to the right": "pointing_right",
    "pointing left": "pointing_left",
    "pointing right": "pointing_right",
    "point to the left": "pointing_left",
    "point to the right": "pointing_right",
    "point left": "pointing_left",
    "point right": "pointing_right",
    "raising their left arm": "raising_left_arm",
    "raising their right arm": "raising_right_arm",
    "raising left arm": "raising_left_arm",
    "raising right arm": "raising_right_arm",
    "raising their arm": "raising_arm",
    "raising arm": "raising_arm",
    "raising their hands": "raising_arm",
    "raising hands": "raising_arm",
    "waving": "waving",
    "wave": "waving",
    "saludando": "waving",
}

POSTURE_SYNONYMS = {
    "standing": "standing",
    "stand": "standing",
    "standing up": "standing",
    "de pie": "standing",
    "sitting": "sitting",
    "sit": "sitting",
    "sitting down": "sitting",
    "sentado": "sitting",
    "lying": "lying",
    "lying down": "lying",
    "lie": "lying",
    "acostado": "lying",
    "tumbado": "lying",
}

CLOTHING_TORSO_ITEMS = (
    "shirt", "t-shirt", "tshirt", "t shirt", "sweater", "jacket",
    "hoodie", "top", "coat", "blouse", "polo", "vest",
)

CLOTHING_LEGS_ITEMS = (
    "trousers", "pants", "jeans", "shorts", "slacks",
)


# ------------------------------------------------------------------------------
# Normalizacion y Auxiliares Geometricos
# ------------------------------------------------------------------------------
def _normalize_keypoints_local(
    keypoints: Any,
    conf: Optional[Any] = None
) -> Tuple[List[Tuple[float, float]], List[float]]:
    """Normaliza keypoints de entrada a lista de 17 tuplas (x, y) y 17 confianzas."""
    if normalize_keypoints is not None:
        try:
            return normalize_keypoints(keypoints, conf)
        except Exception:
            pass

    if isinstance(keypoints, dict):
        raw_kpts = keypoints.get("keypoints", [])
        raw_conf = keypoints.get("keypoints_conf", None)
        return _normalize_keypoints_local(raw_kpts, raw_conf)

    pts_list = list(keypoints)
    n_pts = len(pts_list)
    out_pts: List[Tuple[float, float]] = []
    out_conf: List[float] = []

    for i in range(17):
        if i < n_pts:
            pt = pts_list[i]
            x, y = float(pt[0]), float(pt[1])
            if conf is not None and i < len(conf):
                c = float(conf[i])
            elif len(pt) >= 3:
                c = float(pt[2])
            else:
                c = 1.0
        else:
            x, y, c = 0.0, 0.0, 0.0
        out_pts.append((x, y))
        out_conf.append(c)

    return out_pts, out_conf


def _knee_angle(
    hip: Tuple[float, float],
    knee: Tuple[float, float],
    ankle: Tuple[float, float]
) -> float:
    """Calcula el angulo en grados formado en la rodilla (cadera-rodilla-tobillo)."""
    v1 = (hip[0] - knee[0], hip[1] - knee[1])
    v2 = (ankle[0] - knee[0], ankle[1] - knee[1])
    m1 = math.hypot(v1[0], v1[1])
    m2 = math.hypot(v2[0], v2[1])
    if m1 < 1e-3 or m2 < 1e-3:
        return 180.0
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    cos_a = max(-1.0, min(1.0, dot / (m1 * m2)))
    return math.degrees(math.acos(cos_a))


def _elbow_angle(
    shoulder: Tuple[float, float],
    elbow: Tuple[float, float],
    wrist: Tuple[float, float]
) -> float:
    """Calcula el angulo en grados en el codo (hombro-codo-muneca)."""
    v1 = (shoulder[0] - elbow[0], shoulder[1] - elbow[1])
    v2 = (wrist[0] - elbow[0], wrist[1] - elbow[1])
    m1 = math.hypot(v1[0], v1[1])
    m2 = math.hypot(v2[0], v2[1])
    if m1 < 1e-3 or m2 < 1e-3:
        return 180.0
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    cos_a = max(-1.0, min(1.0, dot / (m1 * m2)))
    return math.degrees(math.acos(cos_a))


# ------------------------------------------------------------------------------
# 1. Clasificacion y Coincidencia de Gestos
# ------------------------------------------------------------------------------
def classify_gesture(
    keypoints: Any,
    conf: Optional[Any] = None,
    conf_thresh: float = 0.35,
    frame: str = "camera",
) -> Dict[str, Any]:
    """Evalua los gestos de los brazos de una persona en un cuadro 2D.

    Gestos evaluados:
      - raising_left_arm: persona levanta su brazo izquierdo anatomico por encima
        del hombro o la nariz, con brazo derecho abajo.
      - raising_right_arm: persona levanta su brazo derecho anatomico por encima
        del hombro o la nariz, con brazo izquierdo abajo.
      - raising_arm: al menos un brazo levantado.
      - waving: al menos una mano levantada por encima del hombro/cabeza (brazo
        en alto para saludar).
      - pointing_left: brazo extendido lateralmente apuntando hacia la izquierda
        del campo visual / camara (dx < 0).
      - pointing_right: brazo extendido lateralmente apuntando hacia la derecha
        del campo visual / camara (dx > 0).

    Si los puntos criticos de los brazos no tienen suficiente confianza,
    marca 'unknown_arms=True' para evitar adivinaciones.
    """
    pts, confs = _normalize_keypoints_local(keypoints, conf)

    # Keypoints necesarios para miembros superiores
    l_sh_ok = confs[COCO_L_SHOULDER] >= conf_thresh
    r_sh_ok = confs[COCO_R_SHOULDER] >= conf_thresh
    l_wr_ok = confs[COCO_L_WRIST] >= conf_thresh
    r_wr_ok = confs[COCO_R_WRIST] >= conf_thresh
    l_el_ok = confs[COCO_L_ELBOW] >= conf_thresh
    r_el_ok = confs[COCO_R_ELBOW] >= conf_thresh
    nose_ok = confs[COCO_NOSE] >= conf_thresh

    nose_y = pts[COCO_NOSE][1] if nose_ok else float("inf")

    # Si ninguno de los dos brazos tiene hombro y muneca visibles, no se puede evaluar
    left_arm_visible = l_sh_ok and l_wr_ok
    right_arm_visible = r_sh_ok and r_wr_ok

    if not left_arm_visible and not right_arm_visible:
        return {
            "status": MatchStatus.UNKNOWN,
            "reason": "desconocido: keypoints de brazos no visibles o con baja confianza",
            "raising_left_arm": False,
            "raising_right_arm": False,
            "raising_arm": False,
            "waving": False,
            "pointing_left": False,
            "pointing_right": False,
            "scores": {},
        }

    # 1. Evaluacion de elevacion de brazos (brazo izquierdo vs derecho)
    # Coordenadas de imagen: Y crece hacia abajo (menor Y = mas arriba)
    l_up = False
    l_score = 0.0
    if left_arm_visible:
        over_sh = pts[COCO_L_WRIST][1] < pts[COCO_L_SHOULDER][1]
        over_nose = nose_ok and (pts[COCO_L_WRIST][1] < nose_y)
        l_up = over_sh or over_nose
        l_score = (confs[COCO_L_SHOULDER] + confs[COCO_L_WRIST]) / 2.0

    r_up = False
    r_score = 0.0
    if right_arm_visible:
        over_sh = pts[COCO_R_WRIST][1] < pts[COCO_R_SHOULDER][1]
        over_nose = nose_ok and (pts[COCO_R_WRIST][1] < nose_y)
        r_up = over_sh or over_nose
        r_score = (confs[COCO_R_SHOULDER] + confs[COCO_R_WRIST]) / 2.0

    # 2. Evaluacion de senalamiento (pointing left / pointing right)
    # Brazo extendido lateralmente: vector hombro -> muneca
    pointing_left = False
    pointing_right = False
    p_left_score = 0.0
    p_right_score = 0.0
    pointing_reasons: List[str] = []

    # Escala anatomica de referencia para descartar brazos colgando
    torso_scale = 100.0
    if l_sh_ok and r_sh_ok:
        torso_scale = max(30.0, abs(pts[COCO_L_SHOULDER][0] - pts[COCO_R_SHOULDER][0]))

    for is_left_arm, sh_idx, el_idx, wr_idx, sh_ok, el_ok, wr_ok, arm_name in [
        (False, COCO_R_SHOULDER, COCO_R_ELBOW, COCO_R_WRIST, r_sh_ok, r_el_ok, r_wr_ok, "derecho"),
        (True, COCO_L_SHOULDER, COCO_L_ELBOW, COCO_L_WRIST, l_sh_ok, l_el_ok, l_wr_ok, "izquierdo"),
    ]:
        if not (sh_ok and wr_ok):
            continue

        sh_pt = pts[sh_idx]
        wr_pt = pts[wr_idx]
        dx = wr_pt[0] - sh_pt[0]
        dy = wr_pt[1] - sh_pt[1]
        arm_len = math.hypot(dx, dy)

        # Si el brazo mide menos del 40% del ancho del torso, esta retraido o incompleto
        if arm_len < 0.40 * torso_scale:
            continue

        # Si el brazo esta colgando verticalmente (dx pequeno y dy hacia abajo), no es senalamiento
        # Para senalar lateralmente, la componente horizontal |dx| / arm_len debe ser sustancial (>= 0.45)
        # o el angulo respecto a la vertical debe superar los 30 grados
        horizontal_ratio = abs(dx) / arm_len
        is_lateral = horizontal_ratio >= 0.45

        # Si el codo es visible, verificar que no este totalmente flexionado hacia el pecho (< 90 deg)
        elbow_extended = True
        if el_ok:
            e_ang = _elbow_angle(sh_pt, pts[el_idx], wr_pt)
            if e_ang < 95.0:
                elbow_extended = False

        if is_lateral and elbow_extended:
            arm_conf = (confs[sh_idx] + confs[wr_idx]) / 2.0
            if dx < -0.35 * arm_len:
                # Apunta hacia la izquierda de la imagen (viewer/camera left)
                pointing_left = True
                p_left_score = max(p_left_score, arm_conf)
                pointing_reasons.append(f"brazo {arm_name} extendido a la izquierda (dx={dx:.1f})")
            elif dx > 0.35 * arm_len:
                # Apunta hacia la derecha de la imagen (viewer/camera right)
                pointing_right = True
                p_right_score = max(p_right_score, arm_conf)
                pointing_reasons.append(f"brazo {arm_name} extendido a la derecha (dx={dx:.1f})")

    # Waving: al menos una mano levantada por encima del hombro
    waving = l_up or r_up
    waving_score = max(l_score if l_up else 0.0, r_score if r_up else 0.0)

    # Raising arm general
    raising_arm = l_up or r_up
    raising_arm_score = waving_score

    # Raising specific arm (uno levantado y el otro no, salvo que ambos esten)
    raising_left_only = l_up and not r_up
    raising_right_only = r_up and not l_up

    return {
        "status": MatchStatus.MATCH if (waving or pointing_left or pointing_right) else MatchStatus.NO_MATCH,
        "left_arm_visible": left_arm_visible,
        "right_arm_visible": right_arm_visible,
        "left_arm_up": l_up,
        "right_arm_up": r_up,
        "raising_left_arm": l_up,
        "raising_left_arm_only": raising_left_only,
        "raising_right_arm": r_up,
        "raising_right_arm_only": raising_right_only,
        "raising_arm": raising_arm,
        "waving": waving,
        "pointing_left": pointing_left,
        "pointing_right": pointing_right,
        "scores": {
            "raising_left_arm": l_score if l_up else 0.0,
            "raising_right_arm": r_score if r_up else 0.0,
            "raising_arm": raising_arm_score,
            "waving": waving_score,
            "pointing_left": p_left_score,
            "pointing_right": p_right_score,
        },
        "pointing_reasons": pointing_reasons,
    }


def match_gesture(
    keypoints: Any,
    target_gesture: str,
    conf: Optional[Any] = None,
    conf_thresh: float = 0.35,
    frame: str = "camera",
) -> PersonMatchResult:
    """Verifica si la persona realiza el gesto GPSR solicitado.

    Si los keypoints para decidir el gesto especifico estan ausentes o por debajo
    del umbral de confianza, devuelve status=UNKNOWN y reason con 'desconocido'.
    """
    g_raw = target_gesture.strip().lower()
    canonical = GESTURE_SYNONYMS.get(g_raw, None)
    if canonical is None:
        for k, v in GESTURE_SYNONYMS.items():
            if k in g_raw:
                canonical = v
                break

    if canonical is None:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=f"desconocido: gesto no reconocido en vocabulario oficial GPSR ('{target_gesture}')",
        )

    res = classify_gesture(keypoints, conf=conf, conf_thresh=conf_thresh, frame=frame)
    if res.get("status") == MatchStatus.UNKNOWN:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=res.get("reason", "desconocido: keypoints insuficientes para evaluar gesto"),
            details=res,
        )

    pts, confs = _normalize_keypoints_local(keypoints, conf)

    # 1. Raising left arm
    if canonical == "raising_left_arm":
        if not res["left_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: keypoints del brazo izquierdo no visibles o con baja confianza",
                details=res,
            )
        if res["raising_left_arm"]:
            # Si se exige especificamente el brazo izquierdo y el derecho tambien esta levantado,
            # sigue teniendo el brazo izquierdo arriba
            score = res["scores"]["raising_left_arm"]
            extra = " (ambos brazos arriba)" if res["right_arm_up"] else ""
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=score,
                reason=f"brazo izquierdo levantado por encima del hombro{extra}",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="brazo izquierdo no esta levantado",
                details=res,
            )

    # 2. Raising right arm
    elif canonical == "raising_right_arm":
        if not res["right_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: keypoints del brazo derecho no visibles o con baja confianza",
                details=res,
            )
        if res["raising_right_arm"]:
            score = res["scores"]["raising_right_arm"]
            extra = " (ambos brazos arriba)" if res["left_arm_up"] else ""
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=score,
                reason=f"brazo derecho levantado por encima del hombro{extra}",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="brazo derecho no esta levantado",
                details=res,
            )

    # 3. Raising arm (cualquiera)
    elif canonical == "raising_arm":
        if not res["left_arm_visible"] and not res["right_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: ningun brazo visible para evaluar elevacion",
                details=res,
            )
        if res["raising_arm"]:
            arm_str = "ambos brazos" if (res["left_arm_up"] and res["right_arm_up"]) else (
                "brazo izquierdo" if res["left_arm_up"] else "brazo derecho"
            )
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=res["scores"]["raising_arm"],
                reason=f"{arm_str} levantado",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="ningun brazo esta levantado",
                details=res,
            )

    # 4. Waving (saludo)
    elif canonical == "waving":
        if not res["left_arm_visible"] and not res["right_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: brazos no visibles para evaluar saludo (waving)",
                details=res,
            )
        if res["waving"]:
            arm_str = "ambos brazos" if (res["left_arm_up"] and res["right_arm_up"]) else (
                "brazo izquierdo" if res["left_arm_up"] else "brazo derecho"
            )
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=res["scores"]["waving"],
                reason=f"gesto de saludo detectado con {arm_str} en alto",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="sin gesto de saludo (brazos en reposo hacia abajo)",
                details=res,
            )

    # 5. Pointing to the left
    elif canonical == "pointing_left":
        if not res["left_arm_visible"] and not res["right_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: brazos no visibles para evaluar senalamiento hacia la izquierda",
                details=res,
            )
        if res["pointing_left"]:
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=res["scores"]["pointing_left"],
                reason="persona senalando hacia la izquierda del campo visual",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="persona no esta senalando hacia la izquierda",
                details=res,
            )

    # 6. Pointing to the right
    elif canonical == "pointing_right":
        if not res["left_arm_visible"] and not res["right_arm_visible"]:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: brazos no visibles para evaluar senalamiento hacia la derecha",
                details=res,
            )
        if res["pointing_right"]:
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=res["scores"]["pointing_right"],
                reason="persona senalando hacia la derecha del campo visual",
                details=res,
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason="persona no esta senalando hacia la derecha",
                details=res,
            )

    return PersonMatchResult(
        matched=None,
        status=MatchStatus.UNKNOWN,
        score=0.0,
        reason=f"desconocido: gesto no categorizado ({target_gesture})",
    )


# ------------------------------------------------------------------------------
# 2. Clasificacion y Coincidencia de Posturas (Standing, Sitting, Lying)
# ------------------------------------------------------------------------------
def classify_posture(
    keypoints: Any,
    conf: Optional[Any] = None,
    bbox: Optional[Sequence[float]] = None,
    conf_thresh: float = 0.35,
) -> Dict[str, Any]:
    """Clasifica la postura humana segun la configuracion cinematica del esqueleto.

    Posturas:
      - 'lying': tronco o cuerpo con orientacion predominantemente horizontal
                 (dx_tronco > 1.2 * dy_tronco).
      - 'standing': tronco vertical, piernas extendidas hacia abajo
                    (angulo en rodilla >= 145 deg, dy_muslo comparable a dy_pantorrilla).
      - 'sitting': tronco vertical, muslos horizontales o flexionados
                   (angulo en rodilla entre 65 y 135 deg, o dy_muslo < 0.55 * dy_pantorrilla).
      - 'unknown': keypoints insuficientes o ambiguos (NUNCA adivina).
    """
    pts, confs = _normalize_keypoints_local(keypoints, conf)

    # 1. Verificar visibilidad del tronco (hombros y caderas)
    l_sh_ok = confs[COCO_L_SHOULDER] >= conf_thresh
    r_sh_ok = confs[COCO_R_SHOULDER] >= conf_thresh
    l_hip_ok = confs[COCO_L_HIP] >= conf_thresh
    r_hip_ok = confs[COCO_R_HIP] >= conf_thresh

    sh_ok = l_sh_ok or r_sh_ok
    hip_ok = l_hip_ok or r_hip_ok

    if not sh_ok or not hip_ok:
        return {
            "posture": "unknown",
            "score": 0.0,
            "status": MatchStatus.UNKNOWN,
            "reason": "desconocido: hombros o caderas no visibles para evaluar postura",
        }

    # Centro de hombros
    if l_sh_ok and r_sh_ok:
        sh_pt = ((pts[COCO_L_SHOULDER][0] + pts[COCO_R_SHOULDER][0]) / 2.0,
                 (pts[COCO_L_SHOULDER][1] + pts[COCO_R_SHOULDER][1]) / 2.0)
        sh_conf = (confs[COCO_L_SHOULDER] + confs[COCO_R_SHOULDER]) / 2.0
    elif l_sh_ok:
        sh_pt = pts[COCO_L_SHOULDER]
        sh_conf = confs[COCO_L_SHOULDER]
    else:
        sh_pt = pts[COCO_R_SHOULDER]
        sh_conf = confs[COCO_R_SHOULDER]

    # Centro de caderas
    if l_hip_ok and r_hip_ok:
        hip_pt = ((pts[COCO_L_HIP][0] + pts[COCO_R_HIP][0]) / 2.0,
                  (pts[COCO_L_HIP][1] + pts[COCO_R_HIP][1]) / 2.0)
        hip_conf = (confs[COCO_L_HIP] + confs[COCO_R_HIP]) / 2.0
    elif l_hip_ok:
        hip_pt = pts[COCO_L_HIP]
        hip_conf = confs[COCO_L_HIP]
    else:
        hip_pt = pts[COCO_R_HIP]
        hip_conf = confs[COCO_R_HIP]

    dx_torso = abs(hip_pt[0] - sh_pt[0])
    dy_torso = abs(hip_pt[1] - sh_pt[1])

    # 2. Comprobar si esta acostado / tumbado (LYING)
    # En posicion acostada, el tronco es horizontal: dx_torso > 1.2 * dy_torso
    is_horizontal_torso = (dx_torso > 1.2 * dy_torso)
    is_wide_bbox = False
    if bbox is not None and len(bbox) >= 4:
        bw = abs(float(bbox[2]) - float(bbox[0]))
        bh = abs(float(bbox[3]) - float(bbox[1]))
        if bh > 0 and (bw / bh) >= 1.25:
            is_wide_bbox = True

    if is_horizontal_torso or (is_wide_bbox and dx_torso > dy_torso):
        score = (sh_conf + hip_conf) / 2.0
        return {
            "posture": "lying",
            "score": score,
            "status": MatchStatus.MATCH,
            "reason": f"cuerpo en orientacion horizontal (dx_tronco={dx_torso:.1f} > dy_tronco={dy_torso:.1f})",
        }

    # Si el tronco no es vertical (por ejemplo, angulo intermedio 45 deg sin claridad), es desconocido
    if dy_torso < 1.1 * dx_torso:
        return {
            "posture": "unknown",
            "score": 0.0,
            "status": MatchStatus.UNKNOWN,
            "reason": "desconocido: inclinacion de tronco ambigua entre horizontal y vertical",
        }

    # 3. Tronco vertical: diferenciar entre de pie (STANDING) y sentado (SITTING)
    l_knee_ok = confs[COCO_L_KNEE] >= conf_thresh
    r_knee_ok = confs[COCO_R_KNEE] >= conf_thresh
    l_ank_ok = confs[COCO_L_ANKLE] >= conf_thresh
    r_ank_ok = confs[COCO_R_ANKLE] >= conf_thresh

    knee_ok = l_knee_ok or r_knee_ok
    ank_ok = l_ank_ok or r_ank_ok

    # Si no se ve ninguna rodilla ni tobillo, imposible saber si esta de pie o sentado
    if not knee_ok:
        return {
            "posture": "unknown",
            "score": 0.0,
            "status": MatchStatus.UNKNOWN,
            "reason": "desconocido: rodillas no visibles para distinguir de pie vs sentado",
        }

    # Evaluar pierna con mejor confianza
    leg_scores: List[float] = []
    knee_angles: List[float] = []
    thigh_dy_list: List[float] = []
    calf_dy_list: List[float] = []

    for h_idx, k_idx, a_idx, h_ok, k_ok, a_ok in [
        (COCO_L_HIP, COCO_L_KNEE, COCO_L_ANKLE, l_hip_ok, l_knee_ok, l_ank_ok),
        (COCO_R_HIP, COCO_R_KNEE, COCO_R_ANKLE, r_hip_ok, r_knee_ok, r_ank_ok),
    ]:
        if not (h_ok and k_ok):
            continue
        h_p = pts[h_idx]
        k_p = pts[k_idx]
        thigh_dy = abs(k_p[1] - h_p[1])
        thigh_dy_list.append(thigh_dy)

        if a_ok:
            a_p = pts[a_idx]
            calf_dy = abs(a_p[1] - k_p[1])
            calf_dy_list.append(calf_dy)
            ang = _knee_angle(h_p, k_p, a_p)
            knee_angles.append(ang)
            leg_scores.append((confs[h_idx] + confs[k_idx] + confs[a_idx]) / 3.0)
        else:
            leg_scores.append((confs[h_idx] + confs[k_idx]) / 2.0)

    # Caso A: Se dispone de angulos de rodilla (tobillo visible)
    if knee_angles:
        avg_angle = sum(knee_angles) / len(knee_angles)
        avg_score = sum(leg_scores) / len(leg_scores)

        # Rodillas flexionadas tipicas de sentado: angulo < 135 deg
        if avg_angle < 135.0:
            return {
                "posture": "sitting",
                "score": avg_score,
                "status": MatchStatus.MATCH,
                "reason": f"persona sentada con rodillas flexionadas (angulo={avg_angle:.1f} deg)",
                "knee_angle": avg_angle,
            }
        # Rodillas extendidas tipicas de de pie: angulo >= 145 deg
        elif avg_angle >= 145.0:
            return {
                "posture": "standing",
                "score": avg_score,
                "status": MatchStatus.MATCH,
                "reason": f"persona de pie con piernas extendidas (angulo rodilla={avg_angle:.1f} deg)",
                "knee_angle": avg_angle,
            }
        else:
            # Zona intermedia (135 - 145 deg)
            # Revisar relacion thigh_dy vs calf_dy
            if thigh_dy_list and calf_dy_list:
                thigh = sum(thigh_dy_list) / len(thigh_dy_list)
                calf = sum(calf_dy_list) / len(calf_dy_list)
                if thigh < 0.55 * calf:
                    return {
                        "posture": "sitting",
                        "score": avg_score * 0.9,
                        "status": MatchStatus.MATCH,
                        "reason": f"persona sentada (muslo vertical reducido={thigh:.1f} vs pantorrilla={calf:.1f})",
                    }
                else:
                    return {
                        "posture": "standing",
                        "score": avg_score * 0.9,
                        "status": MatchStatus.MATCH,
                        "reason": f"persona de pie (muslo vertical={thigh:.1f} y pantorrilla={calf:.1f})",
                    }

    # Caso B: Rodillas visibles pero tobillos ocultos (comun al estar sentado tras mesa)
    if thigh_dy_list:
        avg_thigh = sum(thigh_dy_list) / len(thigh_dy_list)
        avg_score = sum(leg_scores) / len(leg_scores)

        # Si el muslo apenas tiene proyeccion vertical (rodillas a la misma altura que caderas)
        if avg_thigh < 0.40 * dy_torso:
            return {
                "posture": "sitting",
                "score": avg_score * 0.85,
                "status": MatchStatus.MATCH,
                "reason": f"persona sentada con muslos horizontales (dy_muslo={avg_thigh:.1f} < 0.4*dy_tronco={dy_torso:.1f})",
            }
        elif avg_thigh >= 0.70 * dy_torso:
            # Muslos claramente orientados hacia abajo
            return {
                "posture": "standing",
                "score": avg_score * 0.85,
                "status": MatchStatus.MATCH,
                "reason": f"persona de pie con muslos extendidos verticalmente (dy_muslo={avg_thigh:.1f})",
            }

    return {
        "posture": "unknown",
        "score": 0.0,
        "status": MatchStatus.UNKNOWN,
        "reason": "desconocido: configuracion de piernas ambigua entre de pie y sentado",
    }


def match_posture(
    keypoints: Any,
    target_posture: str,
    conf: Optional[Any] = None,
    bbox: Optional[Sequence[float]] = None,
    conf_thresh: float = 0.35,
) -> PersonMatchResult:
    """Verifica si la postura de la persona coincide con la solicitada en GPSR."""
    p_raw = target_posture.strip().lower()
    canonical = POSTURE_SYNONYMS.get(p_raw, None)
    if canonical is None:
        for k, v in POSTURE_SYNONYMS.items():
            if k in p_raw:
                canonical = v
                break

    if canonical is None:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=f"desconocido: postura no reconocida en vocabulario oficial GPSR ('{target_posture}')",
        )

    res = classify_posture(keypoints, conf=conf, bbox=bbox, conf_thresh=conf_thresh)
    detected = res.get("posture")

    if detected == "unknown" or res.get("status") == MatchStatus.UNKNOWN:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=res.get("reason", "desconocido: keypoints insuficientes para evaluar postura"),
            details=res,
        )

    if detected == canonical:
        return PersonMatchResult(
            matched=True,
            status=MatchStatus.MATCH,
            score=res.get("score", 0.9),
            reason=f"postura coincide: {canonical} ({res.get('reason', '')})",
            details=res,
        )
    else:
        return PersonMatchResult(
            matched=False,
            status=MatchStatus.NO_MATCH,
            score=0.0,
            reason=f"postura no coincide: esperada '{canonical}', detectada '{detected}' ({res.get('reason', '')})",
            details=res,
        )


# ------------------------------------------------------------------------------
# 3. Clasificacion y Coincidencia de Ropa
# ------------------------------------------------------------------------------
def parse_target_clothing(text: str) -> Tuple[Optional[str], Optional[str]]:
    """Extrae el color y el tipo de prenda (torso vs piernas) de un texto GPSR.

    Ejemplos:
      - 'blue sweater' -> ('blue', 'torso')
      - 'red jacket' -> ('red', 'torso')
      - 'orange t shirt' -> ('orange', 'torso')
      - 'dark trousers' -> ('dark', 'legs')
      - 'black pants' -> ('black', 'legs')
      - 'wearing red' -> ('red', None)
    """
    text_clean = text.lower().strip()

    # 1. Buscar color
    found_color = None
    all_colors_set = set(ALL_COLORS) | {"grey", "black", "violet"}
    for color in sorted(all_colors_set, key=len, reverse=True):
        if re.search(r"\b" + re.escape(color) + r"\b", text_clean):
            if color == "grey":
                found_color = COLOR_GRAY
            elif color == "black":
                found_color = COLOR_DARK
            else:
                found_color = color
            break

    # 2. Buscar tipo de prenda
    target_part = None
    for item in sorted(CLOTHING_TORSO_ITEMS, key=len, reverse=True):
        if re.search(r"\b" + re.escape(item) + r"\b", text_clean):
            target_part = "torso"
            break

    if target_part is None:
        for item in sorted(CLOTHING_LEGS_ITEMS, key=len, reverse=True):
            if re.search(r"\b" + re.escape(item) + r"\b", text_clean):
                target_part = "legs"
                break

    return found_color, target_part


def match_clothing(
    attributes: Optional[Union[PersonAttributes, Dict[str, Any]]],
    target_clothing: str,
    min_confidence: float = 0.45,
) -> PersonMatchResult:
    """Verifica si la vestimenta de la persona coincide con el descriptor GPSR.

    Si no se dispone de atributos de ropa o la confianza es baja, devuelve UNKNOWN
    con 'desconocido'. Nunca inventa ni adivina colores.
    """
    target_color, target_part = parse_target_clothing(target_clothing)

    if target_color is None:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=f"desconocido: color no identificado en descriptor de ropa ('{target_clothing}')",
        )

    if attributes is None:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason="desconocido: sin atributos visuales de ropa disponibles",
        )

    # Extraer torso y legs segun sea objeto PersonAttributes o dict
    torso_color = None
    torso_conf = 0.0
    legs_color = None
    legs_conf = 0.0

    if hasattr(attributes, "torso"):
        if attributes.torso is not None:
            torso_color = attributes.torso.color
            torso_conf = getattr(attributes.torso, "confidence", 0.0)
        if attributes.legs is not None:
            legs_color = attributes.legs.color
            legs_conf = getattr(attributes.legs, "confidence", 0.0)
    elif isinstance(attributes, dict):
        torso = attributes.get("torso")
        if torso and isinstance(torso, dict):
            torso_color = torso.get("color")
            torso_conf = float(torso.get("confidence", 0.0))
        elif isinstance(torso, str):
            torso_color = torso
            torso_conf = 0.8
        legs = attributes.get("legs")
        if legs and isinstance(legs, dict):
            legs_color = legs.get("color")
            legs_conf = float(legs.get("confidence", 0.0))
        elif isinstance(legs, str):
            legs_color = legs
            legs_conf = 0.8

    def _color_matches(detected: Optional[str], target: str) -> bool:
        if not detected:
            return False
        d = detected.lower().strip()
        t = target.lower().strip()
        if t in (COLOR_DARK, COLOR_BLACK) and d in (COLOR_DARK, COLOR_BLACK):
            return True
        if t == COLOR_GRAY and d in ("gray", "grey"):
            return True
        return d == t

    # 1. El objetivo especifica prenda de torso (ej. blue sweater, red jacket)
    if target_part == "torso":
        if torso_color is None or torso_conf < min_confidence:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: color de prenda superior no visible o con baja confianza",
            )
        if _color_matches(torso_color, target_color):
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=torso_conf,
                reason=f"color de prenda superior coincide: {torso_color} (conf={torso_conf:.2f})",
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason=f"color de prenda superior no coincide: esperado '{target_color}', detectado '{torso_color}'",
            )

    # 2. El objetivo especifica prenda de piernas (ej. dark trousers, black pants)
    elif target_part == "legs":
        if legs_color is None or legs_conf < min_confidence:
            return PersonMatchResult(
                matched=None,
                status=MatchStatus.UNKNOWN,
                score=0.0,
                reason="desconocido: color de prenda inferior no visible o con baja confianza",
            )
        if _color_matches(legs_color, target_color):
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=legs_conf,
                reason=f"color de prenda inferior coincide: {legs_color} (conf={legs_conf:.2f})",
            )
        else:
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason=f"color de prenda inferior no coincide: esperado '{target_color}', detectado '{legs_color}'",
            )

    # 3. No especifica prenda (ej. 'wearing red', 'blue clothing'): evaluar ambas
    else:
        torso_ok = _color_matches(torso_color, target_color) and (torso_conf >= min_confidence)
        legs_ok = _color_matches(legs_color, target_color) and (legs_conf >= min_confidence)

        if torso_ok or legs_ok:
            score = max(torso_conf if torso_ok else 0.0, legs_conf if legs_ok else 0.0)
            part = "superior" if torso_ok else "inferior"
            return PersonMatchResult(
                matched=True,
                status=MatchStatus.MATCH,
                score=score,
                reason=f"color coincide en prenda {part}: {target_color}",
            )

        # Si al menos una prenda fue vista claramente y no coincide
        if (torso_color and torso_conf >= min_confidence) or (legs_color and legs_conf >= min_confidence):
            seen = f"torso={torso_color}, legs={legs_color}"
            return PersonMatchResult(
                matched=False,
                status=MatchStatus.NO_MATCH,
                score=0.0,
                reason=f"color de ropa no coincide: esperado '{target_color}', detectado ({seen})",
            )

        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason="desconocido: vestimenta no detectable con suficiente confianza",
        )


# ------------------------------------------------------------------------------
# 4. Emparejamiento Completo de Descriptores GPSR (match_person)
# ------------------------------------------------------------------------------
def match_person(
    person_or_kpts: Any,
    descriptor: Dict[str, Any],
    conf: Optional[Any] = None,
    bbox: Optional[Sequence[float]] = None,
    attributes: Optional[Any] = None,
    conf_thresh: float = 0.35,
    frame: str = "camera",
) -> PersonMatchResult:
    """Verifica si una persona cumple con los criterios de un descriptor GPSR.

    Parametros:
      - person_or_kpts: Dict de deteccion (con 'keypoints', 'keypoints_conf', 'bbox', 'attributes')
        o lista/array de keypoints COCO-17.
      - descriptor: Dict con claves opcionales 'gesture', 'posture', 'clothing', 'name'.
      - conf: Array opcional de confianzas si person_or_kpts solo contiene puntos xy.
      - bbox: [x1, y1, x2, y2] opcional.
      - attributes: PersonAttributes o dict opcional.
      - conf_thresh: Umbral minimo para keypoints anatomicos.
      - frame: "camera" o "robot".

    Comportamiento:
      - Si cualquier criterio evaluado resulta NO_MATCH -> veredicto NO_MATCH.
      - Si todos los criterios evaluados resultan MATCH -> veredicto MATCH.
      - Si ninguno es NO_MATCH pero alguno o todos son UNKNOWN -> veredicto UNKNOWN ('desconocido').
      - NUNCA adivina.
    """
    # Si viene un diccionario de deteccion (como produce person_pose_detector)
    keypoints = person_or_kpts
    if isinstance(person_or_kpts, dict):
        keypoints = person_or_kpts.get("keypoints", [])
        if conf is None:
            conf = person_or_kpts.get("keypoints_conf", None)
        if bbox is None:
            bbox = person_or_kpts.get("bbox", None)
        if attributes is None:
            attributes = person_or_kpts.get("attributes", None)

    # Identificar criterios presentes
    has_gesture = bool(descriptor.get("gesture"))
    has_posture = bool(descriptor.get("posture"))
    has_clothing = bool(descriptor.get("clothing"))
    has_name = bool(descriptor.get("name") and descriptor.get("name") != "me")

    if not has_gesture and not has_posture and not has_clothing and not has_name:
        # Descriptor vacio o 'me'
        return PersonMatchResult(
            matched=True,
            status=MatchStatus.MATCH,
            score=1.0,
            reason="descriptor sin restricciones especificas de persona",
        )

    results: Dict[str, PersonMatchResult] = {}

    # 1. Gesto
    if has_gesture:
        results["gesture"] = match_gesture(
            keypoints,
            descriptor["gesture"],
            conf=conf,
            conf_thresh=conf_thresh,
            frame=frame,
        )

    # 2. Postura
    if has_posture:
        results["posture"] = match_posture(
            keypoints,
            descriptor["posture"],
            conf=conf,
            bbox=bbox,
            conf_thresh=conf_thresh,
        )

    # 3. Ropa
    if has_clothing:
        results["clothing"] = match_clothing(
            attributes,
            descriptor["clothing"],
        )

    # 4. Nombre
    # El nombre NO se puede deducir por vision. Si es el unico criterio,
    # requiere confirmacion verbal.
    if has_name and not (has_gesture or has_posture or has_clothing):
        target_name = descriptor["name"]
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason=f"desconocido: el nombre '{target_name}' no se reconoce por vision; requiere confirmacion verbal por voz",
            details={"name": target_name, "requires_voice": True},
        )

    # Evaluar conjuncion de criterios visuales
    any_no_match = False
    any_unknown = False
    match_scores: List[float] = []
    reasons: List[str] = []

    for crit, r in results.items():
        if r.status == MatchStatus.NO_MATCH:
            any_no_match = True
            reasons.append(r.reason)
        elif r.status == MatchStatus.UNKNOWN:
            any_unknown = True
            reasons.append(r.reason)
        elif r.status == MatchStatus.MATCH:
            match_scores.append(r.score)
            reasons.append(r.reason)

    if any_no_match:
        return PersonMatchResult(
            matched=False,
            status=MatchStatus.NO_MATCH,
            score=0.0,
            reason="; ".join(reasons),
            details={"criteria": results},
        )

    if any_unknown:
        return PersonMatchResult(
            matched=None,
            status=MatchStatus.UNKNOWN,
            score=0.0,
            reason="; ".join(reasons),
            details={"criteria": results},
        )

    # Todos los criterios visuales cumplieron MATCH
    final_score = min(match_scores) if match_scores else 1.0
    voice_note = ""
    if has_name:
        voice_note = f" (pendiente confirmar nombre '{descriptor['name']}' por voz)"

    return PersonMatchResult(
        matched=True,
        status=MatchStatus.MATCH,
        score=final_score,
        reason="; ".join(reasons) + voice_note,
        details={"criteria": results, "has_name": has_name},
    )


# ==============================================================================
# RESOLUCION DE IDENTIDAD DE PERSONAS POR VOZ (HRI SOCIAL)
# ==============================================================================
# Propuesta arquitectonica para RoboCup@Home (GPSR):
# En entornos de competencia domestica, el reconocimiento facial presenta severas
# limitaciones operativas: ausencia de imagenes de enrolamiento previas para
# personas desconocidas, riesgo de penalizaciones por falsas identificaciones
# biometricas y variabilidad de angulos/iluminacion.
#
# La solucion estandar y socialmente natural recomendada en RoboCup@Home es la
# resolucion verbal interactiva (HRI conversacional):
# 1. Localizacion espacial: El robot navega a la sala indicada en el comando GPSR.
# 2. Enumeracion visual: Identifica las personas candidatas presentes mediante
#    YOLOv8-pose y las ordena espacialmente (por ejemplo, por proximidad o azimut).
# 3. Interaccion secuencial:
#    a. El robot se acerca (approach_person) a la persona candidata i.
#    b. Orienta la camara / pan-tilt y formula verbalmente la pregunta mediante TTS:
#       "Excuse me, are you <Name>?"
#    c. Escucha la respuesta durante una ventana acotada de ASR (Voice Understanding).
#    d. La funcion pura de evaluacion clasifica la respuesta:
#       - AFIRMATIVA ('yes', 'yeah', 'I am', etc.): Persona confirmada. Se detiene
#         la busqueda con exito y se ejecuta la siguiente accion (seguir, hablar, entregar).
#       - NEGATIVA ('no', 'nope', 'I am not', etc.): La persona es descartada. El robot
#         agradece cortesmente ("Thank you, excuse me") y avanza al siguiente candidato.
#       - AMBIGUA O SILENCIO: Se puede repreguntar una unica vez o considerar descartada.
# 4. Terminacion determinista: Si se agotan los candidatos sin confirmacion, el robot
#    anuncia que la persona <Name> no fue encontrada en la habitacion.
# ==============================================================================

@dataclass
class VoiceQueryStep:
    """Paso planificado en la secuencia de identificacion de personas por voz."""
    candidate_id: Any
    question: str
    confirmation_phrase: str
    rejection_phrase: str


@dataclass
class VoiceResolutionResult:
    """Resultado final o intermedio de la resolucion de una persona por voz."""
    matched_id: Optional[Any]
    status: str              # "CONFIRMED", "NOT_FOUND", "AMBIGUOUS"
    reason: str
    dialogue_log: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def is_confirmed(self) -> bool:
        return self.status == "CONFIRMED" and self.matched_id is not None


VOICE_AFFIRMATIVE_PATTERNS = [
    r"\b(yes|yeah|yep|yup|sure|correct|exactly|affirmative)\b",
    r"\b(that\'?s\s+me|it\'?s\s+me|i\s+am|i\s+am\s+indeed)\b",
    r"\b(s[ií]|afirmativo|correcto|soy\s+yo)\b",
]

VOICE_NEGATIVE_PATTERNS = [
    r"\b(no|nope|nah|negative)\b",
    r"\b(not\s+me|i\'?m\s+not|i\s+am\s+not)\b",
    r"\b(wrong\s+person|no\s+soy)\b",
]


def build_name_question(target_name: str, polite: bool = True) -> str:
    """Genera la frase formal para preguntar a un candidato si es la persona buscada."""
    name_clean = target_name.strip()
    if polite:
        return f"Excuse me, are you {name_clean}?"
    return f"Are you {name_clean}?"


def classify_yes_no_response(response_text: str) -> Optional[bool]:
    """Clasifica una respuesta hablada en afirmativa (True), negativa (False) o ambigua (None).

    Logica pura y determinista con soporte de patrones en ingles y espanol.
    """
    clean = response_text.lower().strip()
    if not clean:
        return None

    # Verificar negaciones primero para evitar falsos positivos con frases como "No, I am not"
    for p in VOICE_NEGATIVE_PATTERNS:
        if re.search(p, clean):
            return False

    for p in VOICE_AFFIRMATIVE_PATTERNS:
        if re.search(p, clean):
            return True

    return None


def plan_voice_identification_dialogue(
    target_name: str,
    candidate_ids: Sequence[Any],
) -> List[VoiceQueryStep]:
    """Genera la secuencia ordenada de preguntas verbales para los candidatos visibles."""
    steps = []
    question = build_name_question(target_name, polite=True)
    confirmation = f"Thank you, {target_name}. Nice to meet you."
    rejection = "Thank you, sorry to disturb you."

    for cid in candidate_ids:
        steps.append(VoiceQueryStep(
            candidate_id=cid,
            question=question,
            confirmation_phrase=confirmation,
            rejection_phrase=rejection,
        ))
    return steps


def evaluate_voice_candidate_response(
    target_name: str,
    candidate_id: Any,
    response_text: str,
) -> Tuple[Optional[bool], str]:
    """Evalua la respuesta verbal de un candidato individual.

    Retorna:
      (True, motivo) si confirmo afirmativamente ser la persona.
      (False, motivo) si nego ser la persona.
      (None, motivo) si la respuesta es ambigua o ininteligible.
    """
    verdict = classify_yes_no_response(response_text)
    if verdict is True:
        return True, f"Candidato {candidate_id} confirmo ser '{target_name}' ('{response_text}')"
    elif verdict is False:
        return False, f"Candidato {candidate_id} nego ser '{target_name}' ('{response_text}')"
    else:
        return None, f"Respuesta ambigua o no concluyente de candidato {candidate_id} ('{response_text}')"


def resolve_person_by_voice(
    target_name: str,
    candidate_responses: Sequence[Tuple[Any, str]],
) -> VoiceResolutionResult:
    """Simula o resuelve la secuencia completa de dialogo verbal sobre una lista de candidatos.

    Recibe una secuencia de tuplas (candidate_id, respuesta_hablada) en el orden
    en que fueron entrevistados por el robot.

    Detiene la busqueda ante la primera respuesta afirmativa. Si todos niegan,
    retorna status='NOT_FOUND'.
    """
    log: List[Dict[str, Any]] = []

    for cid, resp in candidate_responses:
        verdict, reason = evaluate_voice_candidate_response(target_name, cid, resp)
        log.append({
            "candidate_id": cid,
            "response": resp,
            "verdict": verdict,
            "reason": reason,
        })
        if verdict is True:
            return VoiceResolutionResult(
                matched_id=cid,
                status="CONFIRMED",
                reason=f"Persona {cid} confirmada verbalmente como '{target_name}'",
                dialogue_log=log,
            )

    return VoiceResolutionResult(
        matched_id=None,
        status="NOT_FOUND",
        reason=f"Ninguno de los {len(candidate_responses)} candidatos confirmo ser '{target_name}'",
        dialogue_log=log,
    )
