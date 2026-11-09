#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Tests Unitarios para Emparejamiento de Personas (GPSR)
# Gestos, Posturas, Ropa, Ruido, Keypoints Faltantes y Dialogo Verbal
# ==============================================================================

import random
from typing import Any, Dict, Sequence, Tuple
import pytest
import numpy as np

from base_athome_perception.person_matcher import (
    MatchStatus,
    PersonMatchResult,
    classify_gesture,
    match_gesture,
    classify_posture,
    match_posture,
    parse_target_clothing,
    match_clothing,
    match_person,
    build_name_question,
    classify_yes_no_response,
    plan_voice_identification_dialogue,
    evaluate_voice_candidate_response,
    resolve_person_by_voice,
    COCO_NOSE,
    COCO_L_SHOULDER,
    COCO_R_SHOULDER,
    COCO_L_ELBOW,
    COCO_R_ELBOW,
    COCO_L_WRIST,
    COCO_R_WRIST,
    COCO_L_HIP,
    COCO_R_HIP,
    COCO_L_KNEE,
    COCO_R_KNEE,
    COCO_L_ANKLE,
    COCO_R_ANKLE,
)
from base_athome_perception.person_attributes import (
    PersonAttributes,
    AttributeResult,
)


# ------------------------------------------------------------------------------
# Generador de Esqueletos Sinteticos COCO-17
# ------------------------------------------------------------------------------
def make_skeleton(
    posture: str = "standing",
    gesture: str = "none",
    noise: float = 0.0,
    seed: int = 42,
    conf_val: float = 0.95,
) -> Tuple[np.ndarray, np.ndarray]:
    """Genera un esqueleto sintetico COCO-17 geometricamente consistente."""
    kpts = np.zeros((17, 2), dtype=float)
    conf = np.ones(17, dtype=float) * conf_val

    # 1. Postura base del tronco y miembros inferiores
    if posture == "standing":
        kpts[COCO_NOSE] = [300.0, 150.0]
        kpts[COCO_L_SHOULDER] = [340.0, 200.0]
        kpts[COCO_R_SHOULDER] = [260.0, 200.0]
        kpts[COCO_L_HIP] = [330.0, 400.0]
        kpts[COCO_R_HIP] = [270.0, 400.0]
        kpts[COCO_L_KNEE] = [330.0, 600.0]
        kpts[COCO_R_KNEE] = [270.0, 600.0]
        kpts[COCO_L_ANKLE] = [330.0, 800.0]
        kpts[COCO_R_ANKLE] = [270.0, 800.0]
    elif posture == "sitting":
        kpts[COCO_NOSE] = [300.0, 250.0]
        kpts[COCO_L_SHOULDER] = [340.0, 300.0]
        kpts[COCO_R_SHOULDER] = [260.0, 300.0]
        kpts[COCO_L_HIP] = [330.0, 450.0]
        kpts[COCO_R_HIP] = [270.0, 450.0]
        # Muslos horizontales hacia los lados/adelante
        kpts[COCO_L_KNEE] = [380.0, 470.0]
        kpts[COCO_R_KNEE] = [220.0, 470.0]
        # Pantorrillas verticales
        kpts[COCO_L_ANKLE] = [380.0, 650.0]
        kpts[COCO_R_ANKLE] = [220.0, 650.0]
    elif posture == "lying":
        # Tronco y extremidades orientadas horizontalmente a lo largo del eje X
        kpts[COCO_NOSE] = [150.0, 500.0]
        kpts[COCO_L_SHOULDER] = [250.0, 470.0]
        kpts[COCO_R_SHOULDER] = [250.0, 530.0]
        kpts[COCO_L_HIP] = [450.0, 470.0]
        kpts[COCO_R_HIP] = [450.0, 530.0]
        kpts[COCO_L_KNEE] = [650.0, 470.0]
        kpts[COCO_R_KNEE] = [650.0, 530.0]
        kpts[COCO_L_ANKLE] = [850.0, 470.0]
        kpts[COCO_R_ANKLE] = [850.0, 530.0]

    # 2. Brazos segun el gesto
    # Por defecto: brazos en reposo hacia abajo
    if gesture == "none":
        kpts[COCO_L_ELBOW] = [350.0, 300.0]
        kpts[COCO_L_WRIST] = [350.0, 400.0]
        kpts[COCO_R_ELBOW] = [250.0, 300.0]
        kpts[COCO_R_WRIST] = [250.0, 400.0]
    elif gesture == "raising_left_arm":
        # Brazo izquierdo levantado por encima del hombro y la cabeza
        kpts[COCO_L_ELBOW] = [350.0, 150.0]
        kpts[COCO_L_WRIST] = [350.0, 80.0]
        # Brazo derecho en reposo
        kpts[COCO_R_ELBOW] = [250.0, 300.0]
        kpts[COCO_R_WRIST] = [250.0, 400.0]
    elif gesture == "raising_right_arm":
        # Brazo derecho levantado
        kpts[COCO_R_ELBOW] = [250.0, 150.0]
        kpts[COCO_R_WRIST] = [250.0, 80.0]
        # Brazo izquierdo en reposo
        kpts[COCO_L_ELBOW] = [350.0, 300.0]
        kpts[COCO_L_WRIST] = [350.0, 400.0]
    elif gesture == "waving":
        # Gesto de saludo: mano levantada con antebrazo en alto
        kpts[COCO_L_ELBOW] = [360.0, 140.0]
        kpts[COCO_L_WRIST] = [360.0, 70.0]
        kpts[COCO_R_ELBOW] = [250.0, 300.0]
        kpts[COCO_R_WRIST] = [250.0, 400.0]
    elif gesture == "pointing_left":
        # Brazo derecho extendido hacia la izquierda de la imagen (viewer left)
        kpts[COCO_R_ELBOW] = [180.0, 200.0]
        kpts[COCO_R_WRIST] = [100.0, 200.0]
        # Brazo izquierdo en reposo
        kpts[COCO_L_ELBOW] = [350.0, 300.0]
        kpts[COCO_L_WRIST] = [350.0, 400.0]
    elif gesture == "pointing_right":
        # Brazo izquierdo extendido hacia la derecha de la imagen (viewer right)
        kpts[COCO_L_ELBOW] = [420.0, 200.0]
        kpts[COCO_L_WRIST] = [500.0, 200.0]
        # Brazo derecho en reposo
        kpts[COCO_R_ELBOW] = [250.0, 300.0]
        kpts[COCO_R_WRIST] = [250.0, 400.0]

    # Aplicar ruido opcional
    if noise > 0.0:
        rng = np.random.default_rng(seed)
        noise_arr = rng.uniform(-noise, noise, size=kpts.shape)
        kpts = kpts + noise_arr

    return kpts, conf


# ------------------------------------------------------------------------------
# 1. Tests de Gestos
# ------------------------------------------------------------------------------
class TestGestures:
    """Verifica el reconocimiento determinista de todos los gestos oficiales GPSR."""

    def test_waving(self):
        kpts, conf = make_skeleton("standing", "waving")
        res = match_gesture(kpts, "waving", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70
        assert "saludo" in res.reason.lower() or "levantado" in res.reason.lower()

    def test_raising_left_arm(self):
        kpts, conf = make_skeleton("standing", "raising_left_arm")
        res = match_gesture(kpts, "raising their left arm", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70
        assert "izquierdo" in res.reason.lower()

        # No debe coincidir con brazo derecho levantado
        res_right = match_gesture(kpts, "raising their right arm", conf=conf)
        assert res_right.matched is False
        assert res_right.status == MatchStatus.NO_MATCH

    def test_raising_right_arm(self):
        kpts, conf = make_skeleton("standing", "raising_right_arm")
        res = match_gesture(kpts, "raising their right arm", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70
        assert "derecho" in res.reason.lower()

        # No debe coincidir con brazo izquierdo levantado
        res_left = match_gesture(kpts, "raising their left arm", conf=conf)
        assert res_left.matched is False
        assert res_left.status == MatchStatus.NO_MATCH

    def test_raising_any_arm(self):
        # Brazo izquierdo levantado coincide con "raising their arm"
        kpts, conf = make_skeleton("standing", "raising_left_arm")
        res = match_gesture(kpts, "raising their arm", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH

        # Brazo derecho levantado tambien coincide
        kpts_r, conf_r = make_skeleton("standing", "raising_right_arm")
        res_r = match_gesture(kpts_r, "raising their arm", conf=conf_r)
        assert res_r.matched is True
        assert res_r.status == MatchStatus.MATCH

    def test_pointing_to_the_left(self):
        kpts, conf = make_skeleton("standing", "pointing_left")
        res = match_gesture(kpts, "pointing to the left", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70
        assert "izquierda" in res.reason.lower()

        # No debe coincidir con pointing to the right
        res_r = match_gesture(kpts, "pointing to the right", conf=conf)
        assert res_r.matched is False
        assert res_r.status == MatchStatus.NO_MATCH

    def test_pointing_to_the_right(self):
        kpts, conf = make_skeleton("standing", "pointing_right")
        res = match_gesture(kpts, "pointing to the right", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70
        assert "derecha" in res.reason.lower()

        # No debe coincidir con pointing to the left
        res_l = match_gesture(kpts, "pointing to the left", conf=conf)
        assert res_l.matched is False
        assert res_l.status == MatchStatus.NO_MATCH

    def test_no_gesture_hands_down(self):
        # Persona con brazos caidos a los lados
        kpts, conf = make_skeleton("standing", "none")
        for g in ("waving", "raising their left arm", "raising their right arm",
                  "pointing to the left", "pointing to the right"):
            res = match_gesture(kpts, g, conf=conf)
            assert res.matched is False
            assert res.status == MatchStatus.NO_MATCH


# ------------------------------------------------------------------------------
# 2. Tests de Posturas
# ------------------------------------------------------------------------------
class TestPostures:
    """Verifica la clasificacion y coincidencia de posturas oficiales GPSR."""

    def test_standing(self):
        kpts, conf = make_skeleton("standing", "none")
        res = match_posture(kpts, "standing", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70

        # No debe clasificar como sitting ni lying
        assert match_posture(kpts, "sitting", conf=conf).matched is False
        assert match_posture(kpts, "lying", conf=conf).matched is False

    def test_sitting(self):
        kpts, conf = make_skeleton("sitting", "none")
        res = match_posture(kpts, "sitting", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70

        # No debe clasificar como standing ni lying
        assert match_posture(kpts, "standing", conf=conf).matched is False
        assert match_posture(kpts, "lying", conf=conf).matched is False

    def test_lying(self):
        kpts, conf = make_skeleton("lying", "none")
        res = match_posture(kpts, "lying", conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70

        # Coincide tambien con sinonimo 'lying down'
        res_down = match_posture(kpts, "lying down", conf=conf)
        assert res_down.matched is True

        # No debe clasificar como standing ni sitting
        assert match_posture(kpts, "standing", conf=conf).matched is False
        assert match_posture(kpts, "sitting", conf=conf).matched is False


# ------------------------------------------------------------------------------
# 3. Tests de Robustez ante Ruido
# ------------------------------------------------------------------------------
class TestNoiseRobustness:
    """Verifica que pequenas perturbaciones en coordenadas no alteren la decision."""

    @pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
    def test_postures_with_noise(self, seed):
        # Ruido de +- 8 pixeles
        kpts_stand, conf = make_skeleton("standing", "none", noise=8.0, seed=seed)
        assert match_posture(kpts_stand, "standing", conf=conf).matched is True

        kpts_sit, conf = make_skeleton("sitting", "none", noise=8.0, seed=seed + 10)
        assert match_posture(kpts_sit, "sitting", conf=conf).matched is True

        kpts_lie, conf = make_skeleton("lying", "none", noise=8.0, seed=seed + 20)
        assert match_posture(kpts_lie, "lying", conf=conf).matched is True

    @pytest.mark.parametrize("seed", [10, 20, 30])
    def test_gestures_with_noise(self, seed):
        kpts_wave, conf = make_skeleton("standing", "waving", noise=7.0, seed=seed)
        assert match_gesture(kpts_wave, "waving", conf=conf).matched is True

        kpts_pl, conf = make_skeleton("standing", "pointing_left", noise=7.0, seed=seed)
        assert match_gesture(kpts_pl, "pointing to the left", conf=conf).matched is True

        kpts_pr, conf = make_skeleton("standing", "pointing_right", noise=7.0, seed=seed)
        assert match_gesture(kpts_pr, "pointing to the right", conf=conf).matched is True


# ------------------------------------------------------------------------------
# 4. Tests de Keypoints Faltantes o Baja Confianza (Regla: Nunca Adivinar)
# ------------------------------------------------------------------------------
class TestMissingKeypoints:
    """Verifica que ante keypoints insuficientes devuelva 'desconocido' (UNKNOWN)."""

    def test_missing_arms_for_gesture(self):
        kpts, conf = make_skeleton("standing", "waving")
        # Ambos hombros y munecas con confianza cero
        conf[COCO_L_SHOULDER] = 0.0
        conf[COCO_R_SHOULDER] = 0.0
        conf[COCO_L_WRIST] = 0.0
        conf[COCO_R_WRIST] = 0.0

        res = match_gesture(kpts, "waving", conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()

    def test_missing_left_arm_when_requested(self):
        kpts, conf = make_skeleton("standing", "raising_left_arm")
        conf[COCO_L_WRIST] = 0.10  # Por debajo del umbral conf_thresh=0.35

        res = match_gesture(kpts, "raising their left arm", conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()

    def test_missing_legs_for_posture(self):
        kpts, conf = make_skeleton("standing", "none")
        # Rodillas y tobillos no detectados
        conf[COCO_L_KNEE] = 0.0
        conf[COCO_R_KNEE] = 0.0
        conf[COCO_L_ANKLE] = 0.0
        conf[COCO_R_ANKLE] = 0.0

        res = match_posture(kpts, "standing", conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()

    def test_missing_torso_for_posture(self):
        kpts, conf = make_skeleton("standing", "none")
        # Sin hombros ni caderas
        conf[COCO_L_SHOULDER] = 0.0
        conf[COCO_R_SHOULDER] = 0.0
        conf[COCO_L_HIP] = 0.0
        conf[COCO_R_HIP] = 0.0

        res = match_posture(kpts, "standing", conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()

    def test_unknown_gesture_phrase(self):
        kpts, conf = make_skeleton("standing", "none")
        res = match_gesture(kpts, "dancing macarena", conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()


# ------------------------------------------------------------------------------
# 5. Tests de Coincidencia de Ropa
# ------------------------------------------------------------------------------
class TestClothingMatching:
    """Verifica la coincidencia de prendas superiores e inferiores con PersonAttributes."""

    def test_parse_target_clothing_phrases(self):
        assert parse_target_clothing("blue sweater") == ("blue", "torso")
        assert parse_target_clothing("red jacket") == ("red", "torso")
        assert parse_target_clothing("orange t shirt") == ("orange", "torso")
        assert parse_target_clothing("dark trousers") == ("dark", "legs")
        assert parse_target_clothing("black pants") == ("dark", "legs")
        assert parse_target_clothing("wearing green") == ("green", None)

    def test_torso_matching(self):
        attrs = PersonAttributes(
            torso=AttributeResult(color="blue", item="shirt", label="blue shirt", confidence=0.85),
            legs=AttributeResult(color="dark", item="trousers", label="dark trousers", confidence=0.90),
            confidence=0.85,
        )

        # Prenda superior azul coincide con 'blue sweater'
        res_ok = match_clothing(attrs, "blue sweater")
        assert res_ok.matched is True
        assert res_ok.status == MatchStatus.MATCH
        assert res_ok.score == 0.85

        # Discrepancia de color: espera rojo
        res_fail = match_clothing(attrs, "red jacket")
        assert res_fail.matched is False
        assert res_fail.status == MatchStatus.NO_MATCH

    def test_legs_matching(self):
        attrs = PersonAttributes(
            torso=AttributeResult(color="red", item="shirt", label="red shirt", confidence=0.80),
            legs=AttributeResult(color="dark", item="trousers", label="dark trousers", confidence=0.88),
            confidence=0.80,
        )

        res_ok = match_clothing(attrs, "dark trousers")
        assert res_ok.matched is True
        assert res_ok.status == MatchStatus.MATCH

        # 'black pants' es sinonimo de 'dark'
        res_syn = match_clothing(attrs, "black pants")
        assert res_syn.matched is True

        # Discrepancia
        res_fail = match_clothing(attrs, "yellow trousers")
        assert res_fail.matched is False

    def test_missing_clothing_attributes(self):
        # Sin atributos disponibles
        res_none = match_clothing(None, "blue sweater")
        assert res_none.matched is None
        assert res_none.status == MatchStatus.UNKNOWN
        assert "desconocido" in res_none.reason.lower()

        # Torso faltante cuando se pide prenda superior
        attrs_legs_only = PersonAttributes(
            torso=None,
            legs=AttributeResult(color="dark", item="trousers", label="dark trousers", confidence=0.80),
        )
        res_no_torso = match_clothing(attrs_legs_only, "red jacket")
        assert res_no_torso.matched is None
        assert res_no_torso.status == MatchStatus.UNKNOWN
        assert "desconocido" in res_no_torso.reason.lower()


# ------------------------------------------------------------------------------
# 6. Tests de Emparejamiento Multi-Criterio (match_person)
# ------------------------------------------------------------------------------
class TestMatchPerson:
    """Verifica el emparejador integral sobre diccionarios y descriptores GPSR."""

    def test_posture_and_gesture_conjunction(self):
        # Persona de pie y saludando
        kpts, conf = make_skeleton("standing", "waving")
        desc = {"posture": "standing", "gesture": "waving"}
        res = match_person(kpts, desc, conf=conf)
        assert res.matched is True
        assert res.status == MatchStatus.MATCH
        assert res.score >= 0.70

    def test_posture_match_but_gesture_fails(self):
        # Persona de pie pero con brazos abajo (no saluda)
        kpts, conf = make_skeleton("standing", "none")
        desc = {"posture": "standing", "gesture": "waving"}
        res = match_person(kpts, desc, conf=conf)
        assert res.matched is False
        assert res.status == MatchStatus.NO_MATCH

    def test_unknown_criterion_propagates_unknown(self):
        # Persona de pie, pero brazos ocultos (confianza 0)
        kpts, conf = make_skeleton("standing", "waving")
        conf[COCO_L_WRIST] = 0.0
        conf[COCO_R_WRIST] = 0.0
        desc = {"posture": "standing", "gesture": "waving"}
        res = match_person(kpts, desc, conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "desconocido" in res.reason.lower()

    def test_name_only_requires_voice(self):
        kpts, conf = make_skeleton("standing", "none")
        desc = {"name": "Robin"}
        res = match_person(kpts, desc, conf=conf)
        assert res.matched is None
        assert res.status == MatchStatus.UNKNOWN
        assert "voz" in res.reason.lower() or "verbal" in res.reason.lower()


# ------------------------------------------------------------------------------
# 7. Tests de Resolucion de Nombres por Secuencia Verbal
# ------------------------------------------------------------------------------
class TestVoiceNameResolution:
    """Verifica la logica pura de diálogo verbal para reconocimiento de personas por nombre."""

    def test_question_generation(self):
        q = build_name_question("Robin")
        assert "Robin" in q
        assert "Excuse me, are you Robin?" == q

    @pytest.mark.parametrize("affirmative", [
        "yes",
        "Yes, I am",
        "yeah",
        "Yeah, that's me!",
        "yep",
        "sure",
        "correct",
        "that's me",
        "I am",
        "si, soy yo",
    ])
    def test_classify_affirmative_responses(self, affirmative):
        assert classify_yes_no_response(affirmative) is True

    @pytest.mark.parametrize("negative", [
        "no",
        "No, I am not",
        "nope",
        "not me",
        "wrong person",
        "I'm not Robin",
        "negative",
        "no soy",
    ])
    def test_classify_negative_responses(self, negative):
        assert classify_yes_no_response(negative) is False

    @pytest.mark.parametrize("ambiguous", [
        "",
        "hello",
        "what?",
        "who are you?",
        "table",
        "can you repeat?",
    ])
    def test_classify_ambiguous_responses(self, ambiguous):
        assert classify_yes_no_response(ambiguous) is None

    def test_plan_voice_identification_dialogue(self):
        candidates = ["person_0", "person_1", "person_2"]
        steps = plan_voice_identification_dialogue("Angel", candidates)
        assert len(steps) == 3
        for idx, step in enumerate(steps):
            assert step.candidate_id == candidates[idx]
            assert "Angel" in step.question

    def test_evaluate_individual_candidate_response(self):
        verdict_yes, _ = evaluate_voice_candidate_response("Robin", "cand_1", "Yes, that is me!")
        assert verdict_yes is True

        verdict_no, _ = evaluate_voice_candidate_response("Robin", "cand_2", "No, I am not")
        assert verdict_no is False

        verdict_amb, _ = evaluate_voice_candidate_response("Robin", "cand_3", "Hello there")
        assert verdict_amb is None

    def test_resolve_sequence_first_candidate_matches(self):
        dialogue = [
            ("cand_1", "Yes I am!"),
            ("cand_2", "No"),
        ]
        res = resolve_person_by_voice("Robin", dialogue)
        assert res.is_confirmed
        assert res.matched_id == "cand_1"
        assert res.status == "CONFIRMED"
        assert len(res.dialogue_log) == 1  # Se detiene al encontrar al primero

    def test_resolve_sequence_second_candidate_matches(self):
        dialogue = [
            ("cand_1", "No, sorry"),
            ("cand_2", "Yeah, that's me"),
            ("cand_3", "No"),
        ]
        res = resolve_person_by_voice("Robin", dialogue)
        assert res.is_confirmed
        assert res.matched_id == "cand_2"
        assert res.status == "CONFIRMED"
        assert len(res.dialogue_log) == 2

    def test_resolve_sequence_all_rejected(self):
        dialogue = [
            ("cand_1", "No, not me"),
            ("cand_2", "I am not Robin"),
            ("cand_3", "Nope"),
        ]
        res = resolve_person_by_voice("Robin", dialogue)
        assert not res.is_confirmed
        assert res.matched_id is None
        assert res.status == "NOT_FOUND"
        assert len(res.dialogue_log) == 3
