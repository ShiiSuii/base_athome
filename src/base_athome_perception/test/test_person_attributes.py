"""Tests unitarios para person_attributes.py.

Modulo puro sin ROS: extraccion de atributos visuales de personas (color dominante
de torso y piernas, confianza y frase corta HRI).
Pruebas exhaustivas con imagenes sinteticas:
- Caso patron: red shirt, dark trousers
- Variaciones cromaticas completas (blue, green, yellow, orange, white, purple, etc.)
- Ropa multicolor (cuadricula 4 colores, rayas 50/50) -> rechazo por baja confianza
- Poca luz / escenas subexpuestas -> rechazo seguro (None)
- Keypoints faltantes (hombros, caderas, rodillas, keypoints en cero, fuera de limites)
- Deteccion parcial (torso solo, piernas solas) y modo require_both
- Compatibilidad de formatos de entrada (ndarray 17x2, 17x3, dict de person_pose_detector)
- Gramatica de articulos en ingles ('a red shirt' vs 'an orange shirt')
"""

import os
import sys
from typing import Optional, Tuple
import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from base_athome_perception.person_attributes import (
    COLOR_BLACK,
    COLOR_BLUE,
    COLOR_BROWN,
    COLOR_DARK,
    COLOR_GRAY,
    COLOR_GREEN,
    COLOR_ORANGE,
    COLOR_PINK,
    COLOR_PURPLE,
    COLOR_RED,
    COLOR_WHITE,
    COLOR_YELLOW,
    LEFT_ANKLE,
    LEFT_HIP,
    LEFT_KNEE,
    LEFT_SHOULDER,
    RIGHT_ANKLE,
    RIGHT_HIP,
    RIGHT_KNEE,
    RIGHT_SHOULDER,
    PersonAttributes,
    PersonAttributesExtractor,
    extract_person_attributes,
    format_spoken_phrase,
    get_english_article,
)


# BGR de referencia para generacion sintetica
BGR_PALETTE = {
    COLOR_RED: (0, 0, 220),
    COLOR_BLUE: (220, 50, 0),
    COLOR_GREEN: (0, 180, 0),
    COLOR_YELLOW: (0, 220, 220),
    COLOR_ORANGE: (0, 140, 240),
    COLOR_PURPLE: (180, 20, 140),
    COLOR_PINK: (180, 130, 245),
    COLOR_WHITE: (245, 245, 245),
    COLOR_GRAY: (128, 128, 128),
    COLOR_DARK: (25, 25, 25),
    COLOR_BROWN: (30, 60, 120),
}


def make_synthetic_person_image(
    torso_bgr: Tuple[int, int, int] = (0, 0, 220),       # Red
    legs_bgr: Tuple[int, int, int] = (25, 25, 25),        # Dark / Black
    bg_bgr: Tuple[int, int, int] = (190, 190, 190),
    img_h: int = 480,
    img_w: int = 640,
    kpt_conf: float = 0.90,
) -> Tuple[np.ndarray, np.ndarray]:
    """Genera una imagen sintetica de una persona de pie con keypoints COCO."""
    img = np.ones((img_h, img_w, 3), dtype=np.uint8)
    img[:] = bg_bgr

    # Cabeza (piel sintetica)
    cv2.circle(img, (320, 60), 30, (140, 175, 215), -1)

    # Torso: y in [100, 240], x in [260, 380]
    cv2.rectangle(img, (260, 100), (380, 240), torso_bgr, -1)

    # Piernas: y in [240, 450], franjas izquierda y derecha
    cv2.rectangle(img, (270, 240), (315, 440), legs_bgr, -1)  # Pierna der (img izq)
    cv2.rectangle(img, (325, 240), (370, 440), legs_bgr, -1)  # Pierna izq (img der)

    # Keypoints COCO (17 x 3: x, y, conf)
    kpts = np.zeros((17, 3), dtype=np.float32)
    kpts[0] = [320, 55, kpt_conf]    # Nose
    kpts[1] = [330, 50, kpt_conf]    # L_Eye
    kpts[2] = [310, 50, kpt_conf]    # R_Eye
    kpts[3] = [345, 55, kpt_conf]    # L_Ear
    kpts[4] = [295, 55, kpt_conf]    # R_Ear
    kpts[5] = [360, 105, kpt_conf]   # L_Shoulder
    kpts[6] = [280, 105, kpt_conf]   # R_Shoulder
    kpts[7] = [385, 170, kpt_conf]   # L_Elbow
    kpts[8] = [255, 170, kpt_conf]   # R_Elbow
    kpts[9] = [390, 235, kpt_conf]   # L_Wrist
    kpts[10] = [250, 235, kpt_conf]  # R_Wrist
    kpts[11] = [350, 240, kpt_conf]  # L_Hip
    kpts[12] = [290, 240, kpt_conf]  # R_Hip
    kpts[13] = [345, 340, kpt_conf]  # L_Knee
    kpts[14] = [295, 340, kpt_conf]  # R_Knee
    kpts[15] = [345, 435, kpt_conf]  # L_Ankle
    kpts[16] = [295, 435, kpt_conf]  # R_Ankle

    return img, kpts


# ==============================================================================
# 1. CASO PATRON Y COMBINACIONES BASICAS
# ==============================================================================

def test_red_shirt_dark_trousers_standard():
    """Prueba el caso especifico del enunciado: red shirt, dark trousers."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=BGR_PALETTE[COLOR_RED],
        legs_bgr=BGR_PALETTE[COLOR_DARK],
    )
    result = extract_person_attributes(img, kpts)

    assert result is not None
    assert result.torso is not None
    assert result.torso.label == "red shirt"
    assert result.torso.color == "red"
    assert result.torso.confidence >= 0.80

    assert result.legs is not None
    assert result.legs.label == "dark trousers"
    assert result.legs.color in ("dark", "black")
    assert result.legs.confidence >= 0.80

    assert result.spoken_phrase == "The person is wearing a red shirt and dark trousers."
    assert result.confidence >= 0.80


def test_blue_shirt_gray_trousers():
    """Prueba blue shirt y gray trousers con frase hablada correspondiente."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=BGR_PALETTE[COLOR_BLUE],
        legs_bgr=BGR_PALETTE[COLOR_GRAY],
    )
    result = extract_person_attributes(img, kpts)

    assert result is not None
    assert result.torso_label == "blue shirt"
    assert result.legs_label == "gray trousers"
    assert result.spoken_phrase == "The person is wearing a blue shirt and gray trousers."


@pytest.mark.parametrize("shirt_color,shirt_bgr,expected_shirt_label", [
    (COLOR_GREEN, BGR_PALETTE[COLOR_GREEN], "green shirt"),
    (COLOR_YELLOW, BGR_PALETTE[COLOR_YELLOW], "yellow shirt"),
    (COLOR_ORANGE, BGR_PALETTE[COLOR_ORANGE], "orange shirt"),
    (COLOR_WHITE, BGR_PALETTE[COLOR_WHITE], "white shirt"),
    (COLOR_PURPLE, BGR_PALETTE[COLOR_PURPLE], "purple shirt"),
    (COLOR_PINK, BGR_PALETTE[COLOR_PINK], "pink shirt"),
    (COLOR_BROWN, BGR_PALETTE[COLOR_BROWN], "brown shirt"),
    (COLOR_DARK, BGR_PALETTE[COLOR_DARK], "dark shirt"),
])
def test_various_shirt_colors(shirt_color, shirt_bgr, expected_shirt_label):
    """Verifica que diversos colores de camiseta se identifiquen correctamente."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=shirt_bgr,
        legs_bgr=BGR_PALETTE[COLOR_DARK],
    )
    result = extract_person_attributes(img, kpts)
    assert result is not None
    assert result.torso_label == expected_shirt_label
    assert result.legs_label == "dark trousers"


@pytest.mark.parametrize("trousers_color,trousers_bgr,expected_trousers_label", [
    (COLOR_BLUE, BGR_PALETTE[COLOR_BLUE], "blue trousers"),
    (COLOR_WHITE, BGR_PALETTE[COLOR_WHITE], "white trousers"),
    (COLOR_GRAY, BGR_PALETTE[COLOR_GRAY], "gray trousers"),
    (COLOR_BROWN, BGR_PALETTE[COLOR_BROWN], "brown trousers"),
    (COLOR_DARK, BGR_PALETTE[COLOR_DARK], "dark trousers"),
])
def test_various_trousers_colors(trousers_color, trousers_bgr, expected_trousers_label):
    """Verifica la clasificacion correcta de diversos colores de pantalon."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=BGR_PALETTE[COLOR_RED],
        legs_bgr=trousers_bgr,
    )
    result = extract_person_attributes(img, kpts)
    assert result is not None
    assert result.torso_label == "red shirt"
    assert result.legs_label == expected_trousers_label


# ==============================================================================
# 2. ROPA MULTICOLOR (RECHAZO POR BAJA CONFIANZA -> None)
# ==============================================================================

def test_multicolor_quadrant_torso_returns_none():
    """Torso dividido en 4 cuadrantes de colores distintos: rechazo seguro."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=(0, 0, 0),
        legs_bgr=BGR_PALETTE[COLOR_DARK],
    )
    # Rellenar torso con 4 cuadrantes: rojo, azul, verde, amarillo (25% c/u)
    cv2.rectangle(img, (260, 100), (320, 170), (0, 0, 220), -1)    # Rojo
    cv2.rectangle(img, (320, 100), (380, 170), (220, 0, 0), -1)    # Azul
    cv2.rectangle(img, (260, 170), (320, 240), (0, 220, 0), -1)    # Verde
    cv2.rectangle(img, (320, 170), (380, 240), (0, 220, 220), -1)  # Amarillo

    # Si se exige require_both=True, debe devolver None directamente
    result_both = extract_person_attributes(img, kpts, require_both=True)
    assert result_both is None

    # Incluso en require_both=False, el torso NO debe tener atributo
    result_any = extract_person_attributes(img, kpts, require_both=False)
    if result_any is not None:
        assert result_any.torso is None
        assert result_any.legs_label == "dark trousers"


def test_striped_50_50_torso_rejected_for_torso():
    """Camiseta a rayas 50% roja y 50% blanca: ningun color supera el margen."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=(0, 0, 0),
        legs_bgr=BGR_PALETTE[COLOR_DARK],
    )
    # Dibujar rayas horizontales alternadas de 10 px
    for y in range(100, 240, 10):
        color = (0, 0, 220) if (y // 10) % 2 == 0 else (250, 250, 250)
        cv2.rectangle(img, (260, y), (380, min(240, y + 10)), color, -1)

    result = extract_person_attributes(img, kpts, require_both=True)
    assert result is None, "La ropa rayada 50/50 debe ser rechazada para evitar restar puntos"


def test_full_multicolor_person_returns_none():
    """Persona con torso multicolor y pantalones multicolor: debe retornar None."""
    img, kpts = make_synthetic_person_image()
    # Torso 4 colores
    cv2.rectangle(img, (260, 100), (320, 170), (0, 0, 220), -1)
    cv2.rectangle(img, (320, 100), (380, 170), (220, 0, 0), -1)
    cv2.rectangle(img, (260, 170), (320, 240), (0, 220, 0), -1)
    cv2.rectangle(img, (320, 170), (380, 240), (0, 220, 220), -1)
    # Piernas a rayas amarillo/azul/rojo
    for y in range(240, 440, 15):
        c = [(0, 220, 220), (220, 0, 0), (0, 0, 220)][(y // 15) % 3]
        cv2.rectangle(img, (270, y), (370, min(440, y + 15)), c, -1)

    result = extract_person_attributes(img, kpts)
    assert result is None


# ==============================================================================
# 3. POCA LUZ (ESCENAS OSCURAS -> RETORNO SEGURO None)
# ==============================================================================

def test_severely_underexposed_scene_returns_none():
    """Escena con iluminacion minima (< 15 en todos los canales): retorna None."""
    img, kpts = make_synthetic_person_image()
    # Reducir imagen a penumbra completa
    dark_img = (img.astype(np.float32) * 0.05).astype(np.uint8)

    result = extract_person_attributes(dark_img, kpts)
    assert result is None, "En penumbra no debe arriesgar un atributo incorrecto"


def test_low_light_threshold_config():
    """Verifica que el umbral de baja iluminacion pueda ajustarse."""
    img, kpts = make_synthetic_person_image()
    dim_img = (img.astype(np.float32) * 0.20).astype(np.uint8)  # Brillo medio ~ 35

    # Con umbral alto (45), la imagen atenuada se descarta
    res_strict = extract_person_attributes(dim_img, kpts, low_light_thresh=50.0)
    assert res_strict is None

    # Con umbral permisivo (10), se procesa
    res_permissive = extract_person_attributes(dim_img, kpts, low_light_thresh=10.0)
    # Si la confianza es suficiente devuelve resultado o None, pero no falla por excepcion
    assert res_permissive is None or isinstance(res_permissive, PersonAttributes)


# ==============================================================================
# 4. KEYPOINTS FALTANTES Y CASOS DE BORDE ANATOMICOS
# ==============================================================================

def test_missing_shoulders_keypoints():
    """Hombros con confianza cero: no se puede estimar el torso."""
    img, kpts = make_synthetic_person_image()
    kpts[LEFT_SHOULDER, 2] = 0.0
    kpts[RIGHT_SHOULDER, 2] = 0.0

    # Con require_both=True, falla completo
    assert extract_person_attributes(img, kpts, require_both=True) is None

    # Con require_both=False, devuelve solo pantalones
    res = extract_person_attributes(img, kpts, require_both=False)
    assert res is not None
    assert res.torso is None
    assert res.legs_label == "dark trousers"
    assert res.spoken_phrase == "The person is wearing dark trousers."


def test_missing_hips_keypoints_returns_none():
    """Caderas no detectadas: anula tanto torso como piernas -> None."""
    img, kpts = make_synthetic_person_image()
    kpts[LEFT_HIP, 2] = 0.1
    kpts[RIGHT_HIP, 2] = 0.1

    result = extract_person_attributes(img, kpts)
    assert result is None


def test_missing_knees_and_ankles():
    """Solo visible el torso (persona detras de una mesa): piernas None."""
    img, kpts = make_synthetic_person_image()
    kpts[LEFT_KNEE, 2] = 0.0
    kpts[RIGHT_KNEE, 2] = 0.0
    kpts[LEFT_ANKLE, 2] = 0.0
    kpts[RIGHT_ANKLE, 2] = 0.0

    res = extract_person_attributes(img, kpts, require_both=False)
    assert res is not None
    assert res.torso_label == "red shirt"
    assert res.legs is None
    assert res.spoken_phrase == "The person is wearing a red shirt."


def test_all_keypoints_zeros_returns_none():
    """Arreglo de keypoints vacio / todo en ceros."""
    img, _ = make_synthetic_person_image()
    zero_kpts = np.zeros((17, 3), dtype=np.float32)
    assert extract_person_attributes(img, zero_kpts) is None


def test_inverted_keypoints_rejected():
    """Hombros situados debajo de las caderas (pose corrupta/ruido)."""
    img, kpts = make_synthetic_person_image()
    # Invertir coordenadas Y de hombros y caderas
    kpts[LEFT_SHOULDER, 1] = 300
    kpts[RIGHT_SHOULDER, 1] = 300
    kpts[LEFT_HIP, 1] = 100
    kpts[RIGHT_HIP, 1] = 100

    assert extract_person_attributes(img, kpts, require_both=True) is None


# ==============================================================================
# 5. VALIDACION DE ENTRADAS Y ROBUSTEZ
# ==============================================================================

def test_invalid_images_handled_gracefully():
    """Imagenes None, vacias o de formato incorrecto."""
    kpts = np.zeros((17, 3), dtype=np.float32)
    assert extract_person_attributes(None, kpts) is None
    assert extract_person_attributes(np.zeros((0, 0, 3), dtype=np.uint8), kpts) is None
    assert extract_person_attributes(np.zeros((100, 100), dtype=np.uint8), kpts) is None  # 1 canal


def test_invalid_keypoints_handled_gracefully():
    """Keypoints None, de longitud menor o dimension incorrecta."""
    img, _ = make_synthetic_person_image()
    assert extract_person_attributes(img, None) is None
    assert extract_person_attributes(img, np.zeros((10, 2))) is None
    assert extract_person_attributes(img, []) is None


def test_dict_formatted_keypoints_compatibility():
    """Compatibilidad con salida de person_pose_detector.py."""
    img, kpts_arr = make_synthetic_person_image()
    pose_dict = {
        "keypoints": kpts_arr[:, :2],
        "keypoints_conf": kpts_arr[:, 2],
        "bbox": np.array([250, 40, 390, 450]),
        "score": 0.92,
    }
    result = extract_person_attributes(img, pose_dict)
    assert result is not None
    assert result.torso_label == "red shirt"
    assert result.legs_label == "dark trousers"


def test_separate_keypoints_conf_argument():
    """Paso de coordenadas (17,2) y confianzas (17,) como argumentos separados."""
    img, kpts_arr = make_synthetic_person_image()
    pts_2d = kpts_arr[:, :2]
    confs = kpts_arr[:, 2]

    result = extract_person_attributes(img, pts_2d, keypoints_conf=confs)
    assert result is not None
    assert result.torso_label == "red shirt"
    assert result.legs_label == "dark trousers"


# ==============================================================================
# 6. GRAMATICA DE FRASES Y ARTICULOS EN INGLES
# ==============================================================================

def test_english_articles_handling():
    """Verifica el uso de 'a' vs 'an' segun la vocal inicial del adjetivo."""
    assert get_english_article("orange shirt") == "an"
    assert get_english_article("red shirt") == "a"
    assert get_english_article("blue shirt") == "a"
    assert get_english_article("emerald shirt") == "an"


def test_phrase_with_orange_shirt():
    """'an orange shirt' en la frase compuesta."""
    img, kpts = make_synthetic_person_image(
        torso_bgr=BGR_PALETTE[COLOR_ORANGE],
        legs_bgr=BGR_PALETTE[COLOR_DARK],
    )
    result = extract_person_attributes(img, kpts)
    assert result is not None
    assert result.spoken_phrase == "The person is wearing an orange shirt and dark trousers."


# ==============================================================================
# 7. METODOS DE LA CLASE Y PROPIEDADES DE PersonAttributes
# ==============================================================================

def test_person_attributes_dict_and_index_access():
    """Acceso por indexacion [], propiedades de conveniencia y to_dict()."""
    img, kpts = make_synthetic_person_image()
    attrs = extract_person_attributes(img, kpts)
    assert attrs is not None

    # Acceso por indexacion
    assert attrs["torso_label"] == "red shirt"
    assert attrs["legs_label"] == "dark trousers"
    assert "red shirt" in attrs["phrase"]
    assert attrs["confidence"] > 0.0

    # Propiedades
    assert attrs.phrase == attrs.spoken_phrase
    assert attrs.description == attrs.spoken_phrase
    assert attrs.torso_color == "red"
    assert attrs.legs_color in ("dark", "black")

    # to_dict
    d = attrs.to_dict()
    assert isinstance(d, dict)
    assert d["torso"]["label"] == "red shirt"
    assert d["legs"]["label"] == "dark trousers"
    assert "distribution" in d["torso"]


def test_extractor_class_instance_reuse():
    """Uso directo de PersonAttributesExtractor reutilizando la instancia."""
    extractor = PersonAttributesExtractor(min_confidence=0.60)
    img, kpts = make_synthetic_person_image()

    res1 = extractor.extract(img, kpts)
    res2 = extractor.extract(img, kpts)

    assert res1 is not None and res2 is not None
    assert res1.torso_label == res2.torso_label == "red shirt"
