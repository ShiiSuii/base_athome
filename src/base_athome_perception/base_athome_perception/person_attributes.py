"""Extraccion de atributos visuales de una persona (ropa y color).

Modulo puro sin dependencias de ROS para RoboCup@Home HRI Challenge.
A partir de una imagen BGR y los keypoints COCO (17 puntos) provistos por YOLOv8-pose,
identifica el color dominante del torso (prenda superior) y las piernas (prenda inferior)
en palabras simples en ingles ('red shirt', 'dark trousers'), computa una medida de
confianza fundamentada, y genera una frase corta en ingles apta para el anfitrion.

Si la confianza no alcanza el umbral establecido (debido a ropa multicolor, baja luz,
oclusiones o keypoints faltantes), devuelve None para evitar penalidades por error en HRI.
"""

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np

# Indices anatomicos estandar COCO (17 keypoints)
NOSE = 0
LEFT_EYE = 1
RIGHT_EYE = 2
LEFT_EAR = 3
RIGHT_EAR = 4
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10
LEFT_HIP = 11
RIGHT_HIP = 12
LEFT_KNEE = 13
RIGHT_KNEE = 14
LEFT_ANKLE = 15
RIGHT_ANKLE = 16

# Vocabulario de colores simples en ingles para HRI
COLOR_RED = "red"
COLOR_ORANGE = "orange"
COLOR_YELLOW = "yellow"
COLOR_GREEN = "green"
COLOR_BLUE = "blue"
COLOR_PURPLE = "purple"
COLOR_PINK = "pink"
COLOR_BROWN = "brown"
COLOR_WHITE = "white"
COLOR_GRAY = "gray"
COLOR_DARK = "dark"
COLOR_BLACK = "black"

ALL_COLORS = (
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
    COLOR_PINK,
    COLOR_BROWN,
    COLOR_WHITE,
    COLOR_GRAY,
    COLOR_DARK,
)

# Nombres de prendas para formulacion de etiquetas
ITEM_SHIRT = "shirt"
ITEM_TROUSERS = "trousers"

# Umbrales por defecto
DEFAULT_MIN_KP_CONF = 0.35
DEFAULT_MIN_CONFIDENCE = 0.50
DEFAULT_LOW_LIGHT_THRESH = 30.0
MIN_ROI_PIXELS = 30


@dataclass
class AttributeResult:
    """Resultado del analisis de una prenda individual (torso o piernas)."""
    color: str
    item: str
    label: str
    confidence: float
    color_distribution: Dict[str, float] = field(default_factory=dict)

    @property
    def is_dark(self) -> bool:
        """Indica si el color es oscuro / negro."""
        return self.color in (COLOR_DARK, COLOR_BLACK)

    def matches_color(self, target: str) -> bool:
        """Verifica coincidencia de color tolerando sinonimos como dark/black."""
        tgt = target.lower().strip()
        if tgt in (COLOR_DARK, COLOR_BLACK) and self.is_dark:
            return True
        return self.color.lower() == tgt


@dataclass
class PersonAttributes:
    """Atributos visuales globales de una persona y frase para dialogo HRI."""
    torso: Optional[AttributeResult] = None
    legs: Optional[AttributeResult] = None
    spoken_phrase: str = ""
    confidence: float = 0.0

    @property
    def phrase(self) -> str:
        return self.spoken_phrase

    @property
    def description(self) -> str:
        return self.spoken_phrase

    @property
    def torso_color(self) -> Optional[str]:
        return self.torso.color if self.torso else None

    @property
    def torso_label(self) -> Optional[str]:
        return self.torso.label if self.torso else None

    @property
    def legs_color(self) -> Optional[str]:
        return self.legs.color if self.legs else None

    @property
    def legs_label(self) -> Optional[str]:
        return self.legs.label if self.legs else None

    def to_dict(self) -> Dict[str, Any]:
        """Serializa los atributos a diccionario estandar."""
        return {
            "torso": {
                "color": self.torso.color,
                "item": self.torso.item,
                "label": self.torso.label,
                "confidence": self.torso.confidence,
                "distribution": self.torso.color_distribution,
            } if self.torso else None,
            "legs": {
                "color": self.legs.color,
                "item": self.legs.item,
                "label": self.legs.label,
                "confidence": self.legs.confidence,
                "distribution": self.legs.color_distribution,
            } if self.legs else None,
            "spoken_phrase": self.spoken_phrase,
            "confidence": self.confidence,
        }

    def __getitem__(self, key: str) -> Any:
        if key == "torso":
            return self.torso
        elif key == "legs":
            return self.legs
        elif key in ("spoken_phrase", "phrase", "description"):
            return self.spoken_phrase
        elif key == "confidence":
            return self.confidence
        elif key == "torso_label":
            return self.torso_label
        elif key == "legs_label":
            return self.legs_label
        elif key == "torso_color":
            return self.torso_color
        elif key == "legs_color":
            return self.legs_color
        raise KeyError(f"Clave de atributo desconocida: {key}")


def get_english_article(word: str) -> str:
    """Devuelve 'an' o 'a' segun comience con sonido vocalico en ingles."""
    if not word:
        return "a"
    w = word.strip().lower()
    return "an" if w[0] in "aeiou" else "a"


def format_spoken_phrase(
    torso: Optional[AttributeResult],
    legs: Optional[AttributeResult]
) -> str:
    """Genera la frase hablada en ingles para la introduccion HRI."""
    if torso is not None and legs is not None:
        art = get_english_article(torso.label)
        return f"The person is wearing {art} {torso.label} and {legs.label}."
    elif torso is not None:
        art = get_english_article(torso.label)
        return f"The person is wearing {art} {torso.label}."
    elif legs is not None:
        return f"The person is wearing {legs.label}."
    return ""


def classify_pixels_hsv(
    hsv_pixels: np.ndarray,
    dark_v_thresh: int = 50,
) -> Tuple[np.ndarray, Dict[int, str]]:
    """Clasifica un arreglo de pixeles HSV (N, 3) en clases de color discretas.
    
    Espacio HSV en OpenCV: H in [0, 179], S in [0, 255], V in [0, 255].
    """
    h = hsv_pixels[:, 0]
    s = hsv_pixels[:, 1]
    v = hsv_pixels[:, 2]

    color_map = {
        1: COLOR_DARK,
        2: COLOR_WHITE,
        3: COLOR_GRAY,
        4: COLOR_BROWN,
        5: COLOR_RED,
        6: COLOR_ORANGE,
        7: COLOR_YELLOW,
        8: COLOR_GREEN,
        9: COLOR_BLUE,
        10: COLOR_PURPLE,
        11: COLOR_PINK,
    }

    ids = np.zeros(h.shape, dtype=np.int32)

    # 1. Acromaticos por brillo y saturacion
    dark_mask = v < dark_v_thresh
    ids[dark_mask] = 1

    white_mask = (~dark_mask) & (s < 40) & (v >= 180)
    ids[white_mask] = 2

    gray_mask = (~dark_mask) & (~white_mask) & (s < 45) & (v < 180)
    ids[gray_mask] = 3

    # 2. Colores cromaticos saturados
    sat_mask = (~dark_mask) & (~white_mask) & (~gray_mask)

    # Marron: matiz naranja/amarillo con brillo medio-bajo y buena saturacion
    brown_mask = sat_mask & (h >= 10) & (h <= 25) & (v <= 135) & (s >= 40)
    ids[brown_mask] = 4

    # Rojo: extremos del canal H
    red_mask = sat_mask & (~brown_mask) & ((h < 10) | (h >= 168))
    ids[red_mask] = 5

    # Naranja
    orange_mask = sat_mask & (~brown_mask) & (h >= 10) & (h <= 22)
    ids[orange_mask] = 6

    # Amarillo
    yellow_mask = sat_mask & (~brown_mask) & (h >= 23) & (h <= 35)
    ids[yellow_mask] = 7

    # Verde
    green_mask = sat_mask & (h >= 36) & (h <= 82)
    ids[green_mask] = 8

    # Azul (incluye tonos cyan hasta azul oscuro)
    blue_mask = sat_mask & (h >= 83) & (h <= 130)
    ids[blue_mask] = 9

    # Violeta / purpura
    purple_mask = sat_mask & (h >= 131) & (h <= 155)
    ids[purple_mask] = 10

    # Rosa
    pink_mask = sat_mask & (h >= 156) & (h <= 167)
    ids[pink_mask] = 11

    return ids, color_map


def compute_dominant_color_and_confidence(
    bgr_pixels: np.ndarray,
    dark_v_thresh: int = 50,
) -> Tuple[Optional[str], float, Dict[str, float]]:
    """Calcula el color dominante, su distribucion y la confianza del veredicto.
    
    La confianza se modela combinando la pureza de la proporcion dominante con el
    margen relativo sobre el segundo color. Ropa multicolor o estampada presenta
    alta entropia y escaso margen, arrojando baja confianza.
    """
    if bgr_pixels.size == 0 or len(bgr_pixels) < MIN_ROI_PIXELS:
        return None, 0.0, {}

    # Convertir a HSV
    hsv = cv2.cvtColor(bgr_pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    color_ids, color_map = classify_pixels_hsv(hsv, dark_v_thresh=dark_v_thresh)

    total_valid = len(color_ids)
    if total_valid < MIN_ROI_PIXELS:
        return None, 0.0, {}

    # Conteo de cada clase de color
    counts = np.bincount(color_ids, minlength=12)
    dist: Dict[str, float] = {}

    color_fractions = []
    for cid, cname in color_map.items():
        frac = float(counts[cid]) / total_valid
        if frac > 0.001:
            dist[cname] = round(frac, 4)
        color_fractions.append((frac, cname))

    color_fractions.sort(key=lambda x: x[0], reverse=True)

    top_frac, top_color = color_fractions[0]
    second_frac, _ = color_fractions[1] if len(color_fractions) > 1 else (0.0, "")

    if top_frac < 0.35:
        # Menos del 35% del area pertenece a un solo color: fuertemente multicolor
        confidence = float(top_frac * 0.5)
        return top_color, confidence, dist

    # Margen relativo sobre el competidor mas cercano
    margin_ratio = max(0.0, 1.0 - (second_frac / (top_frac + 1e-7)))
    confidence = float(top_frac * math.sqrt(margin_ratio))
    confidence = max(0.0, min(1.0, confidence))

    return top_color, round(confidence, 4), dist


def check_is_low_light(
    image: np.ndarray,
    roi_bgr: Optional[np.ndarray] = None,
    threshold: float = DEFAULT_LOW_LIGHT_THRESH,
) -> bool:
    """Detecta si la escena o la region presenta iluminacion deficiente (poca luz)."""
    if image is None or image.size == 0:
        return True

    # Brillo medio en canal V global
    # Usar sampleo rapido si la imagen es grande
    if image.shape[0] * image.shape[1] > 10000:
        step_y = max(1, image.shape[0] // 100)
        step_x = max(1, image.shape[1] // 100)
        sample = image[::step_y, ::step_x]
    else:
        sample = image

    v_channel = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
    mean_v = float(np.mean(v_channel))
    max_v = float(np.max(v_channel))

    # Escena global oscura (poca luz severa en toda la toma)
    if mean_v < threshold and max_v < 60.0:
        return True

    # Si se pasa un ROI especifico, verificar que no este en penumbra completa
    if roi_bgr is not None and roi_bgr.size > 0:
        roi_v = cv2.cvtColor(roi_bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2GRAY)
        roi_mean = float(np.mean(roi_v))
        roi_max = float(np.max(roi_v))
        if roi_mean < 18.0 and roi_max < 30.0:
            return True

    return False


def parse_coco_keypoints(
    keypoints: Any,
    keypoints_conf: Optional[np.ndarray] = None
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Normaliza diversos formatos de keypoints COCO a (pts (17,2), conf (17,))."""
    if keypoints is None:
        return None, None

    # Formato diccionario (como el devuelto por person_pose_detector.py)
    if isinstance(keypoints, dict):
        conf_arr = keypoints.get("keypoints_conf")
        if conf_arr is None:
            conf_arr = keypoints.get("conf")
        pts_arr = keypoints.get("keypoints")
        if pts_arr is None:
            pts_arr = keypoints.get("xy")
        if pts_arr is None:
            return None, None
        return parse_coco_keypoints(pts_arr, conf_arr)

    arr = np.asarray(keypoints, dtype=np.float32)

    # Eliminar dimensiones extra e.g. (1, 17, 2) -> (17, 2)
    if arr.ndim == 3 and arr.shape[0] == 1:
        arr = arr[0]

    if arr.ndim != 2 or arr.shape[0] < 17:
        return None, None

    if arr.shape[1] >= 3:
        pts = arr[:17, :2]
        confs = arr[:17, 2] if keypoints_conf is None else np.asarray(keypoints_conf, dtype=np.float32)[:17]
    elif arr.shape[1] == 2:
        pts = arr[:17, :2]
        if keypoints_conf is not None:
            confs = np.asarray(keypoints_conf, dtype=np.float32)[:17]
        else:
            # Puntos con coord valida (>0) tienen confianza 1.0 por omision
            confs = np.where((pts[:, 0] > 0) | (pts[:, 1] > 0), 1.0, 0.0).astype(np.float32)
    else:
        return None, None

    return pts, confs


class PersonAttributesExtractor:
    """Extractor parametrizable de atributos visuales de una persona."""

    def __init__(
        self,
        min_kp_conf: float = DEFAULT_MIN_KP_CONF,
        min_confidence: float = DEFAULT_MIN_CONFIDENCE,
        low_light_thresh: float = DEFAULT_LOW_LIGHT_THRESH,
        require_both: bool = False,
    ):
        self.min_kp_conf = min_kp_conf
        self.min_confidence = min_confidence
        self.low_light_thresh = low_light_thresh
        self.require_both = require_both

    def extract(
        self,
        image_bgr: np.ndarray,
        keypoints: Any,
        keypoints_conf: Optional[np.ndarray] = None,
        min_confidence: Optional[float] = None,
        require_both: Optional[bool] = None,
    ) -> Optional[PersonAttributes]:
        """Extrae atributos visuales de la persona en la imagen BGR.
        
        Devuelve None si:
        - La imagen o keypoints son invalidos o nulos.
        - Las condiciones de luz son deficientes.
        - Faltan keypoints clave anatomicos.
        - La confianza de los atributos no alcanza min_confidence.
        """
        req_both = self.require_both if require_both is None else require_both
        thresh = self.min_confidence if min_confidence is None else min_confidence

        if image_bgr is None or not isinstance(image_bgr, np.ndarray) or image_bgr.size == 0:
            return None

        if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
            return None

        pts, confs = parse_coco_keypoints(keypoints, keypoints_conf)
        if pts is None or confs is None:
            return None

        # Verificacion global de baja iluminacion
        if check_is_low_light(image_bgr, threshold=self.low_light_thresh):
            return None

        img_h, img_w = image_bgr.shape[:2]

        # 1. Extraccion de atributos del Torso
        torso_attr = self._extract_torso(image_bgr, pts, confs, img_h, img_w, thresh)

        # 2. Extraccion de atributos de las Piernas
        legs_attr = self._extract_legs(image_bgr, pts, confs, img_h, img_w, thresh)

        # Evaluacion de requisitos de aceptacion segun la confianza
        if req_both:
            if torso_attr is None or legs_attr is None:
                return None
        else:
            if torso_attr is None and legs_attr is None:
                return None

        # Calculo de confianza media
        confs_list = []
        if torso_attr is not None:
            confs_list.append(torso_attr.confidence)
        if legs_attr is not None:
            confs_list.append(legs_attr.confidence)

        overall_conf = float(np.mean(confs_list)) if confs_list else 0.0

        spoken = format_spoken_phrase(torso_attr, legs_attr)

        return PersonAttributes(
            torso=torso_attr,
            legs=legs_attr,
            spoken_phrase=spoken,
            confidence=round(overall_conf, 4),
        )

    def _extract_torso(
        self,
        image_bgr: np.ndarray,
        pts: np.ndarray,
        confs: np.ndarray,
        img_h: int,
        img_w: int,
        min_conf: float,
    ) -> Optional[AttributeResult]:
        """Recorta y analiza la region central del torso."""
        # Se requieren ambos hombros y ambas caderas
        for idx in (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP):
            if confs[idx] < self.min_kp_conf:
                return None

        p_ls = pts[LEFT_SHOULDER]
        p_rs = pts[RIGHT_SHOULDER]
        p_lh = pts[LEFT_HIP]
        p_rh = pts[RIGHT_HIP]

        # Los hombros deben situarse arriba de las caderas en coordenadas de imagen (y menor)
        y_shoulders = (p_ls[1] + p_rs[1]) / 2.0
        y_hips = (p_lh[1] + p_rh[1]) / 2.0
        if y_shoulders >= y_hips:
            return None

        y_top_raw = min(p_ls[1], p_rs[1])
        y_bottom_raw = max(p_lh[1], p_rh[1])
        x_left_raw = min(p_ls[0], p_rs[0], p_lh[0], p_rh[0])
        x_right_raw = max(p_ls[0], p_rs[0], p_lh[0], p_rh[0])

        h_torso = y_bottom_raw - y_top_raw
        w_torso = x_right_raw - x_left_raw
        if h_torso < 15.0 or w_torso < 15.0:
            return None

        # Recorte interior centrado:
        # Excluye cuello/menton arriba (15%) y cintura/pantalon abajo (10%)
        # Excluye brazos/fondo lateral (20% en cada lateral)
        y1 = max(0, int(round(y_top_raw + 0.15 * h_torso)))
        y2 = min(img_h, int(round(y_bottom_raw - 0.10 * h_torso)))
        x1 = max(0, int(round(x_left_raw + 0.20 * w_torso)))
        x2 = min(img_w, int(round(x_right_raw - 0.20 * w_torso)))

        if y2 <= y1 or x2 <= x1:
            return None

        roi = image_bgr[y1:y2, x1:x2]
        if check_is_low_light(image_bgr, roi_bgr=roi, threshold=self.low_light_thresh):
            return None

        pixels = roi.reshape(-1, 3)
        color, conf, dist = compute_dominant_color_and_confidence(pixels)

        if color is None or conf < min_conf:
            return None

        label = f"{color} {ITEM_SHIRT}"

        return AttributeResult(
            color=color,
            item=ITEM_SHIRT,
            label=label,
            confidence=conf,
            color_distribution=dist,
        )

    def _extract_legs(
        self,
        image_bgr: np.ndarray,
        pts: np.ndarray,
        confs: np.ndarray,
        img_h: int,
        img_w: int,
        min_conf: float,
    ) -> Optional[AttributeResult]:
        """Muestrea franjas sobre ambas piernas excluyendo el hueco central de fondo."""
        # Se requieren ambas caderas y ambas rodillas como minimo
        for idx in (LEFT_HIP, RIGHT_HIP, LEFT_KNEE, RIGHT_KNEE):
            if confs[idx] < self.min_kp_conf:
                return None

        p_lh = pts[LEFT_HIP]
        p_rh = pts[RIGHT_HIP]
        p_lk = pts[LEFT_KNEE]
        p_rk = pts[RIGHT_KNEE]

        # Validacion de orientacion vertical (caderas arriba de rodillas)
        if min(p_lk[1], p_rk[1]) <= max(p_lh[1], p_rh[1]):
            return None

        ankles_ok = (confs[LEFT_ANKLE] >= self.min_kp_conf and
                     confs[RIGHT_ANKLE] >= self.min_kp_conf)

        if ankles_ok:
            p_la = pts[LEFT_ANKLE]
            p_ra = pts[RIGHT_ANKLE]
            y_top_raw = min(p_lh[1], p_rh[1])
            y_bottom_raw = max(p_la[1], p_ra[1])
            leg_h = y_bottom_raw - y_top_raw
            if leg_h < 15.0:
                return None
            # Excluir cintura arriba (10%) y calzado abajo (10%)
            y1 = max(0, int(round(y_top_raw + 0.10 * leg_h)))
            y2 = min(img_h, int(round(y_bottom_raw - 0.10 * leg_h)))
            left_x_mid = (p_lh[0] + p_la[0]) / 2.0
            right_x_mid = (p_rh[0] + p_ra[0]) / 2.0
        else:
            # Solo muslos / hasta rodillas
            y_top_raw = min(p_lh[1], p_rh[1])
            y_bottom_raw = max(p_lk[1], p_rk[1])
            leg_h = y_bottom_raw - y_top_raw
            if leg_h < 15.0:
                return None
            y1 = max(0, int(round(y_top_raw + 0.15 * leg_h)))
            y2 = min(img_h, int(round(y_bottom_raw - 0.05 * leg_h)))
            left_x_mid = (p_lh[0] + p_lk[0]) / 2.0
            right_x_mid = (p_rh[0] + p_rk[0]) / 2.0

        if y2 <= y1:
            return None

        hip_dist = max(10.0, abs(p_lh[0] - p_rh[0]))
        half_width = max(4, int(round(hip_dist * 0.22)))

        lx1 = max(0, int(round(left_x_mid - half_width)))
        lx2 = min(img_w, int(round(left_x_mid + half_width)))
        rx1 = max(0, int(round(right_x_mid - half_width)))
        rx2 = min(img_w, int(round(right_x_mid + half_width)))

        collected_pixels = []
        if lx2 > lx1:
            left_roi = image_bgr[y1:y2, lx1:lx2]
            if left_roi.size > 0:
                collected_pixels.append(left_roi.reshape(-1, 3))
        if rx2 > rx1:
            right_roi = image_bgr[y1:y2, rx1:rx2]
            if right_roi.size > 0:
                collected_pixels.append(right_roi.reshape(-1, 3))

        if not collected_pixels:
            return None

        all_pixels = np.vstack(collected_pixels)
        if all_pixels.shape[0] < MIN_ROI_PIXELS:
            return None

        # Para pantalones, brillo V < 55 califica como dark / black
        color, conf, dist = compute_dominant_color_and_confidence(all_pixels, dark_v_thresh=55)

        if color is None or conf < min_conf:
            return None

        label = f"{color} {ITEM_TROUSERS}"

        return AttributeResult(
            color=color,
            item=ITEM_TROUSERS,
            label=label,
            confidence=conf,
            color_distribution=dist,
        )


def extract_person_attributes(
    image_bgr: np.ndarray,
    keypoints: Any,
    keypoints_conf: Optional[np.ndarray] = None,
    min_confidence: float = DEFAULT_MIN_CONFIDENCE,
    require_both: bool = False,
    low_light_thresh: float = DEFAULT_LOW_LIGHT_THRESH,
    min_kp_conf: float = DEFAULT_MIN_KP_CONF,
) -> Optional[PersonAttributes]:
    """Funcion principal pura para extraccion de atributos visuales de una persona.
    
    Argumentos:
        image_bgr: Imagen en formato BGR (numpy.ndarray de 3 canales uint8).
        keypoints: Keypoints COCO (17 puntos) en formato ndarray (17,2)/(17,3),
                   diccionario de deteccion o lista.
        keypoints_conf: Confianzas de los 17 puntos si se proveen por separado.
        min_confidence: Umbral minimo de confianza para aceptar un atributo.
        require_both: Si True, exige que tanto torso como piernas superen el umbral;
                      si False, acepta al menos un atributo valido.
        low_light_thresh: Umbral de brillo medio en V para descartar poca luz.
        min_kp_conf: Umbral minimo para considerar valido un keypoint anatomico.

    Retorna:
        PersonAttributes si se detectaron atributos con suficiente confianza, o None.
    """
    extractor = PersonAttributesExtractor(
        min_kp_conf=min_kp_conf,
        min_confidence=min_confidence,
        low_light_thresh=low_light_thresh,
        require_both=require_both,
    )
    return extractor.extract(
        image_bgr=image_bgr,
        keypoints=keypoints,
        keypoints_conf=keypoints_conf,
        min_confidence=min_confidence,
        require_both=require_both,
    )
