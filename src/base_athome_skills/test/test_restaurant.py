"""Tests unitarios para la mision Restaurant (HOME-102).
Sin ROS y sin llamar a la red:
- Dos clientes -> dos misiones en orden estricto.
- Cliente sin pedido entendible -> pide repetir una vez y sigue con el otro.
- Gramatica GBNF solo con items del menu.
- Respuestas y frases en una sola oracion de <= 15 palabras.
- Pregunta de verificacion de bandeja con VLM y short_answer.
"""

from unittest.mock import patch
import pytest

from base_athome_skills.mission.plan import parse_plan, validate_plan
from base_athome_skills.mission.skills import default_registry
from base_athome_skills.mission import restaurant as rest
from base_athome_skills.mission import vlm


# ------------------------------------------------------------------------------
# 1. Frases del Robot (<= 15 palabras y oracion unica)
# ------------------------------------------------------------------------------
def test_all_phrases_are_single_sentence_and_at_most_15_words():
    phrases = [
        rest.greeting_phrase(),
        rest.confirm_order_phrase(["coke", "water"]),
        rest.confirm_order_phrase(["coffee"]),
        rest.repeat_order_phrase(),
        rest.cancel_order_phrase(),
        rest.barman_request_phrase(["coke", "water"], use_tray=True),
        rest.barman_request_phrase(["coke", "water"], use_tray=False),
        rest.deliver_phrase(["coke", "water"]),
    ]

    for p in phrases:
        words = p.strip().split()
        assert len(words) <= 15, f"Frase excede 15 palabras ({len(words)}): '{p}'"
        assert len(words) >= 3, f"Frase demasiado corta: '{p}'"
        assert p.endswith((".", "!", "?")), f"Frase no termina con puntuacion: '{p}'"


# ------------------------------------------------------------------------------
# 2. Gramatica GBNF y Validacion de Menu
# ------------------------------------------------------------------------------
def test_grammar_contains_only_menu_items():
    menu = ["coke", "water", "beer"]
    g = rest.grammar(menu)

    # Debe contener los items del menu
    assert '"\\"coke\\""' in g
    assert '"\\"water\\""' in g
    assert '"\\"beer\\""' in g

    # NO debe contener items ajenos al menu
    assert "pizza" not in g
    assert "whiskey" not in g
    assert "burger" not in g

    # Estructura GBNF valida de array JSON
    assert 'root ::= "[]" | "[" item (", " item)* "]"' in g
    assert 'item ::=' in g


def test_grammar_with_empty_menu():
    assert rest.grammar([]) == 'root ::= "[]"'


def test_to_order_extracts_menu_items_and_discards_unknown():
    menu = ["coke", "water", "beer", "coffee"]

    # Respuesta valida de la LLM
    reply_valid = '["coke", "water"]'
    assert rest.to_order(reply_valid, menu) == ["coke", "water"]

    # Respuesta con elementos fuera de menu (deben ser descartados)
    reply_mixed = '["coke", "poison", "pizza", "coffee"]'
    assert rest.to_order(reply_mixed, menu) == ["coke", "coffee"]

    # Respuesta vacia, malformada o no entendida
    assert rest.to_order("[]", menu) == []
    assert rest.to_order("invalid json text", menu) == []
    assert rest.to_order('{"not": "a list"}', menu) == []
    assert rest.to_order("", menu) == []


def test_messages_prompt_structure():
    menu = ["coke", "water"]
    msgs = rest.messages("I want a coke please", menu)
    assert len(msgs) >= 3
    assert msgs[0]["role"] == "system"
    assert "coke" in msgs[0]["content"] and "water" in msgs[0]["content"]
    assert msgs[-1]["role"] == "user"
    assert msgs[-1]["content"] == "I want a coke please"


def test_ask_order_offline_server_returns_empty_list():
    assert rest.ask_order("a coke", ["coke"], url="http://127.0.0.1:9/none", timeout_s=0.2) == []


# ------------------------------------------------------------------------------
# 3. Protocolo de Dialogo: Repetir una vez y seguir con el siguiente
# ------------------------------------------------------------------------------
def test_customer_unintelligible_order_retries_once_then_continues():
    handler = rest.RestaurantDialogHandler(menu=["coke", "water", "beer"])

    # Cliente 1 - Intento 1: orden ininteligible (ruido / fuera de menu)
    st1 = handler.handle_customer_speech(customer_id="cust_1", speech_text="blablabla incomprehensible")
    assert st1.status == "retry"
    assert st1.attempts == 1
    assert st1.items == []
    # El robot debe pedir repetir
    assert st1.speech == rest.repeat_order_phrase()
    assert len(st1.speech.split()) <= 15

    # Cliente 1 - Intento 2: sigue ininteligible
    st2 = handler.handle_customer_speech(customer_id="cust_1", speech_text="still noise 12345")
    assert st2.status == "failed"
    assert st2.attempts == 2
    assert st2.items == []
    # El robot se disculpa y avisa que atiende a la siguiente mesa
    assert st2.speech == rest.cancel_order_phrase()
    assert len(st2.speech.split()) <= 15

    # Siguiente cliente (Cliente 2): atencion independiente y limpia
    st_cust2 = handler.handle_customer_speech(customer_id="cust_2", speech_text="I want a beer and water")
    assert st_cust2.status == "confirmed"
    assert st_cust2.attempts == 1
    assert set(st_cust2.items) == {"beer", "water"}
    assert st_cust2.speech == rest.confirm_order_phrase(st_cust2.items)
    assert len(st_cust2.speech.split()) <= 15


def test_customer_understands_on_second_attempt():
    handler = rest.RestaurantDialogHandler(menu=["coke", "water"])

    # Intento 1 falla
    st1 = handler.handle_customer_speech("cust_x", "hummm errr")
    assert st1.status == "retry"

    # Intento 2 tiene exito
    st2 = handler.handle_customer_speech("cust_x", "a coke please")
    assert st2.status == "confirmed"
    assert st2.items == ["coke"]
    assert "coke" in st2.speech


# ------------------------------------------------------------------------------
# 4. Dos Clientes -> Dos Misiones en Orden Estricto (Planner)
# ------------------------------------------------------------------------------
def test_plan_two_customers_in_strict_sequential_order():
    clients = [
        {"id": "1", "place": "table_1"},
        {"id": "2", "place": "table_2"},
    ]
    orders = {
        "1": ["coke", "water"],
        "2": ["beer", "juice"],
    }
    bar_loc = "kitchen_bar"

    plan_dict = rest.plan_restaurant_missions(
        clients=clients,
        bar_location=bar_loc,
        orders=orders,
        use_tray=True,
        name="test_two_clients"
    )

    steps = plan_dict["steps"]
    skills = [s["skill"] for s in steps]
    step_ids = [s["id"] for s in steps]

    # Secuencia esperada por cliente:
    # navigate_to, speak, listen, speak, navigate_to, speak, wait(tray), navigate_to, speak, give_object
    expected_per_cust = [
        "navigate_to", "speak", "listen", "speak",
        "navigate_to", "speak", "wait",
        "navigate_to", "speak", "give_object"
    ]

    # Verificar que el plan completo son los pasos de cliente 1 seguidos de cliente 2
    assert skills == expected_per_cust + expected_per_cust

    # Verificar los destinos de navegacion del cliente 1
    cust1_steps = steps[:len(expected_per_cust)]
    assert cust1_steps[0]["args"] == {"place": "table_1"}
    assert cust1_steps[4]["args"] == {"place": "kitchen_bar"}
    assert cust1_steps[7]["args"] == {"place": "table_1"}

    # Verificar los destinos de navegacion del cliente 2
    cust2_steps = steps[len(expected_per_cust):]
    assert cust2_steps[0]["args"] == {"place": "table_2"}
    assert cust2_steps[4]["args"] == {"place": "kitchen_bar"}
    assert cust2_steps[7]["args"] == {"place": "table_2"}

    # Verificar orden estricto de ejecucion: todos los IDs de cust 1 preceden a cust 2
    assert all("c1_1" in sid for sid in step_ids[:len(expected_per_cust)])
    assert all("c2_2" in sid for sid in step_ids[len(expected_per_cust):])

    # Validar el plan formal contra el registro oficial de habilidades de Base@Home
    reg = default_registry()
    places = {
        "table_1": (1.0, 0.0, 0.0),
        "table_2": (2.0, 0.0, 0.0),
        "kitchen_bar": (0.0, 1.0, 0.0),
        "home": (0.0, 0.0, 0.0)
    }
    parsed = parse_plan(plan_dict)
    errors = validate_plan(parsed, reg, places)
    assert errors == [], f"Errores en el plan generado: {errors}"
    assert plan_dict["success_if"] == steps[-1]["id"]


def test_plan_with_manual_picking_when_not_using_tray():
    clients = [{"id": "1", "place": "table_1"}]
    orders = {"1": ["coke", "water"]}

    plan_dict = rest.plan_restaurant_missions(
        clients=clients,
        bar_location="kitchen_bar",
        orders=orders,
        use_tray=False
    )

    steps = plan_dict["steps"]
    skills = [s["skill"] for s in steps]

    # Debe contener pick_object para cada item en lugar de wait(tray)
    assert "pick_object" in skills
    picks = [s for s in steps if s["skill"] == "pick_object"]
    assert len(picks) == 2
    assert picks[0]["args"] == {"object": "coke"}
    assert picks[1]["args"] == {"object": "water"}


# ------------------------------------------------------------------------------
# 5. Verificacion Visual de Bandeja con VLM
# ------------------------------------------------------------------------------
def test_tray_verification_question_format():
    q = rest.tray_verification_question("coke")
    assert q == "Is there a coke on the tray? Answer yes or no."


def test_verify_item_on_tray_with_mock_vlm():
    # Simulando respuesta afirmativa
    with patch.object(vlm, "ask_vlm", return_value="Yes, there is a coke."):
        assert rest.verify_item_on_tray(b"fake_jpeg", "coke") is True

    # Simulando respuesta negativa
    with patch.object(vlm, "ask_vlm", return_value="No, the tray is empty."):
        assert rest.verify_item_on_tray(b"fake_jpeg", "coke") is False

    # Simulando respuesta irrelevante
    with patch.object(vlm, "ask_vlm", return_value="I only see a napkin."):
        assert rest.verify_item_on_tray(b"fake_jpeg", "coke") is False

    # Servidor desconectado (None)
    with patch.object(vlm, "ask_vlm", return_value=None):
        assert rest.verify_item_on_tray(b"fake_jpeg", "coke") is False
