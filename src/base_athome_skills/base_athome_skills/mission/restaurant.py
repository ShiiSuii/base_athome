"""Planificacion y toma de pedidos para la prueba Restaurant (RoboCup@Home). Logica pura sin ROS.

Organiza la atencion secuencial de clientes:
  1. Para cada cliente en orden:
     - Navega / se aproxima a la mesa del cliente.
     - Saluda brevemente (frase corta <= 15 palabras).
     - Escucha el pedido.
     - Confirma el pedido en una sola frase (<= 15 palabras). Si no entiende, pide repetir una vez;
       si vuelve a fallar, pide disculpas y continua con el siguiente cliente.
     - Navega al kitchen bar ('kitchen_bar' o parametro).
     - Pide al barman los items del pedido (frase corta <= 15 palabras).
     - Espera la colocacion en bandeja (o toma los items).
     - Regresa a la mesa del cliente.
     - Entrega la orden (frase corta <= 15 palabras).
     - Recien entonces pasa al siguiente cliente.
  2. Modulo de LLM (local): genera prompt messages() y gramatica GBNF limitada exclusivamente
     a los items del menu, valida y descarta items desconocidos con to_order().
  3. Modulo VLM: formula preguntas breves de confirmacion para verificar con vlm.short_answer
     la presencia de los objetos en la bandeja del robot.
"""

from dataclasses import dataclass, field
import json
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple
import urllib.error
import urllib.request

from . import vlm

# ------------------------------------------------------------------------------
# Constantes y Parametros de Dialogo
# ------------------------------------------------------------------------------
DEFAULT_BAR_LOCATION = "kitchen_bar"
DEFAULT_MENU = ("coke", "water", "beer", "juice", "coffee", "tea", "milk")
MAX_PHRASE_WORDS = 15

LLM_URL = "http://127.0.0.1:8771/intent"
LLM_TIMEOUT_S = 10.0


def limit_words(text: str, max_words: int = MAX_PHRASE_WORDS) -> str:
    """Garantiza que una frase tenga como maximo max_words palabras y termine con puntuacion."""
    words = text.strip().split()
    if not words:
        return ""
    if len(words) <= max_words:
        out = " ".join(words)
    else:
        out = " ".join(words[:max_words]).rstrip(",;:")
    if not out.endswith((".", "!", "?")):
        out += "."
    return out


# ------------------------------------------------------------------------------
# Generacion de Frases Breves (<= 15 palabras)
# ------------------------------------------------------------------------------
def greeting_phrase() -> str:
    """Saludo corto inicial al llegar a la mesa del cliente."""
    return limit_words("Hello! What would you like to order today?")


def confirm_order_phrase(items: Sequence[str]) -> str:
    """Confirmacion inmediata del pedido al comensal."""
    clean = [str(it).strip().lower() for it in items if str(it).strip()]
    if not clean:
        return repeat_order_phrase()
    if len(clean) == 1:
        text = f"I will bring you a {clean[0]} right away."
    else:
        text = f"I will bring you {' and '.join(clean)} right away."
    return limit_words(text)


def repeat_order_phrase() -> str:
    """Peticion de repetir el pedido cuando no se entendio el primer intento."""
    return limit_words("I did not understand. Could you please repeat your order?")


def cancel_order_phrase() -> str:
    """Aviso al cliente de que no se pudo tomar su pedido y se procede con el resto."""
    return limit_words("Sorry, I could not understand. I will serve the next table.")


def barman_request_phrase(items: Sequence[str], use_tray: bool = True) -> str:
    """Instruccion breve al barman en el Kitchen-bar."""
    clean = [str(it).strip().lower() for it in items if str(it).strip()]
    joined = " and ".join(clean) if clean else "the order"
    if use_tray:
        text = f"Please place {joined} on my tray."
    else:
        text = f"I am here to collect {joined}."
    return limit_words(text)


def deliver_phrase(items: Sequence[str]) -> str:
    """Frase de entrega de los items al cliente."""
    clean = [str(it).strip().lower() for it in items if str(it).strip()]
    joined = " and ".join(clean) if clean else "your order"
    return limit_words(f"Here is your order of {joined}. Enjoy!")


# ------------------------------------------------------------------------------
# Gramatica GBNF y Extraccion de Orden con LLM
# ------------------------------------------------------------------------------
SYSTEM_PROMPT = (
    "You are an assistant for a waiter robot in a restaurant. Extract the ordered food and drink items "
    "that appear in the menu as a JSON list of strings. If no valid menu items are requested, return []. "
    "Do not include commentary or items outside the provided menu."
)

FEW_SHOTS = (
    ("I would like a coke and a water, please", '["coke", "water"]'),
    ("Can you bring me a coffee?", '["coffee"]'),
    ("Two beers and a juice", '["beer", "juice"]'),
    ("What time is the restaurant closing?", "[]"),
    ("Hello how are you today", "[]"),
    ("I want a pizza and sushi", "[]"),
)


def _alternatives(names: Sequence[str]) -> str:
    clean = sorted({str(n).strip().lower().replace('"', "") for n in names if str(n).strip()})
    if not clean:
        return '""'
    return " | ".join('"\\"%s\\""' % n for n in clean)


def grammar(menu: Sequence[str]) -> str:
    """Genera gramatica GBNF que restringe la salida estricta a un array JSON con items del menu."""
    clean_menu = sorted({str(m).strip().lower() for m in menu if str(m).strip()})
    if not clean_menu:
        return 'root ::= "[]"'
    item_alts = _alternatives(clean_menu)
    return (
        'root ::= "[]" | "[" item (", " item)* "]"\n'
        f"item ::= {item_alts}"
    )


def messages(text: str, menu: Sequence[str]) -> List[Dict[str, str]]:
    """Construye los mensajes para la LLM local con contexto del menu."""
    clean_menu = sorted({str(m).strip().lower() for m in menu if str(m).strip()})
    menu_desc = f"Available menu items: {', '.join(clean_menu)}."
    out = [dict(role="system", content=f"{SYSTEM_PROMPT} {menu_desc}")]
    for user_ex, asst_ex in FEW_SHOTS:
        out.append(dict(role="user", content=user_ex))
        out.append(dict(role="assistant", content=asst_ex))
    out.append(dict(role="user", content=text))
    return out


def to_order(reply: str, menu: Sequence[str]) -> List[str]:
    """Valida la respuesta de la LLM contra el menu y descarta cualquier item desconocido."""
    if not reply or not reply.strip():
        return []
    try:
        data = json.loads(reply.strip())
    except (ValueError, TypeError, json.JSONDecodeError):
        # Intento de rescate por expresion regular si vino envuelto
        match = re.search(r"\[.*?\]", reply, re.DOTALL)
        if match:
            try:
                data = json.loads(match.group(0))
            except Exception:
                return []
        else:
            return []

    if not isinstance(data, list):
        return []

    valid_menu = {str(m).strip().lower() for m in menu if str(m).strip()}
    extracted: List[str] = []
    for elem in data:
        if isinstance(elem, str):
            clean_elem = elem.strip().lower()
            if clean_elem in valid_menu and clean_elem not in extracted:
                extracted.append(clean_elem)

    return extracted


def ask_order(
    text: str,
    menu: Sequence[str],
    url: str = LLM_URL,
    timeout_s: float = LLM_TIMEOUT_S
) -> List[str]:
    """Consulta al servidor local de LLM. Devuelve lista vacia si esta offline o falla."""
    payload = json.dumps(dict(
        messages=messages(text, menu),
        grammar=grammar(menu)
    )).encode("utf-8")
    req = urllib.request.Request(url, payload, {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as res:
            reply_json = json.load(res)
            return to_order(reply_json.get("reply", ""), menu)
    except (OSError, ValueError, urllib.error.URLError, TimeoutError):
        return []


# ------------------------------------------------------------------------------
# Verificacion Visual en Bandeja con VLM
# ------------------------------------------------------------------------------
def tray_verification_question(item: str) -> str:
    """Pregunta estandar de si/no para consultar al VLM sobre el contenido de la bandeja."""
    clean_item = str(item).strip().lower()
    return f"Is there a {clean_item} on the tray? Answer yes or no."


def verify_item_on_tray(
    image_bytes: bytes,
    item: str,
    url: str = vlm.URL,
    timeout_s: float = vlm.TIMEOUT_S
) -> bool:
    """Verifica si el item solicitado esta presente en la imagen usando vlm.short_answer."""
    if not image_bytes or not item:
        return False
    q = tray_verification_question(item)
    raw_answer = vlm.ask_vlm(image_bytes, q, url=url, timeout_s=timeout_s)
    if not raw_answer:
        return False
    ans = vlm.short_answer(raw_answer).strip().lower()
    return ("yes" in ans or "si" in ans or "true" in ans) and ("no" not in ans.split())


# ------------------------------------------------------------------------------
# Gestor de Sesion de Dialogo con el Cliente (Reintentos)
# ------------------------------------------------------------------------------
@dataclass
class DialogState:
    customer_id: str
    attempts: int = 0
    items: List[str] = field(default_factory=list)
    status: str = "pending"  # "pending", "confirmed", "retry", "failed"
    speech: str = ""


class RestaurantDialogHandler:
    """Maneja el protocolo de dialogo por cliente:
    - Primer intento no entendido -> solicita repetir una vez (status="retry").
    - Segundo intento no entendido -> cancela atencion de ese cliente y sigue con el otro (status="failed").
    - Pedido valido -> confirma y finaliza (status="confirmed").
    """

    def __init__(self, menu: Sequence[str] = DEFAULT_MENU):
        self.menu = menu
        self.sessions: Dict[str, DialogState] = {}

    def handle_customer_speech(self, customer_id: str, speech_text: str) -> DialogState:
        cid = str(customer_id)
        if cid not in self.sessions:
            self.sessions[cid] = DialogState(customer_id=cid)
        st = self.sessions[cid]
        st.attempts += 1

        items = to_order(speech_text, self.menu)
        # Si no vino por JSON directo, intentar match directo de nombres del menu en el texto
        if not items:
            lower_text = speech_text.lower()
            for m in self.menu:
                m_clean = m.strip().lower()
                if re.search(r"\b" + re.escape(m_clean) + r"\b", lower_text):
                    if m_clean not in items:
                        items.append(m_clean)

        if items:
            st.items = items
            st.status = "confirmed"
            st.speech = confirm_order_phrase(items)
        else:
            if st.attempts < 2:
                st.status = "retry"
                st.speech = repeat_order_phrase()
            else:
                st.status = "failed"
                st.speech = cancel_order_phrase()

        return st


# ------------------------------------------------------------------------------
# Generador de Plan de Mision (Planner)
# ------------------------------------------------------------------------------
def _resolve_customer_place(client: Dict[str, Any]) -> str:
    """Extrae el identificador de ubicacion del cliente para Nav2."""
    if "place" in client and client["place"]:
        return str(client["place"])
    if "location" in client and client["location"]:
        return str(client["location"])
    if "table" in client and client["table"]:
        return str(client["table"])
    cid = client.get("id", "1")
    return f"table_{cid}"


def plan_customer_steps(
    client: Dict[str, Any],
    order_items: Sequence[str],
    bar_location: str = DEFAULT_BAR_LOCATION,
    use_tray: bool = True,
    step_prefix: str = "c1"
) -> List[Dict[str, Any]]:
    """Genera la secuencia estricta de pasos de mision para un unico cliente:
    1. navigate_to mesa cliente
    2. speak saludo corto
    3. listen pedido
    4. speak confirmacion
    5. navigate_to bar
    6. speak pedido al barman
    7. tomar items / esperar colocacion en bandeja
    8. navigate_to mesa cliente
    9. speak entrega
    10. give_object entrega fisica
    """
    cust_place = _resolve_customer_place(client)
    items = [it.strip().lower() for it in order_items if it.strip()]
    steps: List[Dict[str, Any]] = []

    # 1. Acercarse a la mesa
    steps.append({
        "id": f"{step_prefix}_s1_nav_cust",
        "skill": "navigate_to",
        "args": {"place": cust_place},
        "retries": 1,
        "timeout_s": 120.0,
        "on_failure": "avisar",
    })

    # 2. Saludo corto
    steps.append({
        "id": f"{step_prefix}_s2_greet",
        "skill": "speak",
        "args": {"text": greeting_phrase()},
        "retries": 0,
        "timeout_s": 30.0,
        "on_failure": "continue",
    })

    # 3. Escuchar pedido
    steps.append({
        "id": f"{step_prefix}_s3_listen",
        "skill": "listen",
        "args": {"prompt": "What would you like to order?", "listen_timeout_s": 15.0},
        "retries": 1,
        "timeout_s": 25.0,
        "on_failure": "continue",
    })

    # 4. Confirmar en una sola frase
    steps.append({
        "id": f"{step_prefix}_s4_confirm",
        "skill": "speak",
        "args": {"text": confirm_order_phrase(items)},
        "retries": 0,
        "timeout_s": 30.0,
        "on_failure": "continue",
    })

    # 5. Ir al bar
    steps.append({
        "id": f"{step_prefix}_s5_nav_bar",
        "skill": "navigate_to",
        "args": {"place": bar_location},
        "retries": 1,
        "timeout_s": 120.0,
        "on_failure": "avisar",
    })

    # 6. Pedir al barman los items
    steps.append({
        "id": f"{step_prefix}_s6_speak_barman",
        "skill": "speak",
        "args": {"text": barman_request_phrase(items, use_tray=use_tray)},
        "retries": 0,
        "timeout_s": 30.0,
        "on_failure": "continue",
    })

    # 7. Tomar los items (o esperar a que el barman los coloque en la bandeja)
    if use_tray:
        steps.append({
            "id": f"{step_prefix}_s7_wait_tray",
            "skill": "wait",
            "args": {"seconds": 5.0},
            "retries": 0,
            "timeout_s": 30.0,
            "on_failure": "continue",
        })
    else:
        for idx, obj in enumerate(items):
            steps.append({
                "id": f"{step_prefix}_s7_pick_{idx + 1}_{obj}",
                "skill": "pick_object",
                "args": {"object": obj},
                "retries": 1,
                "timeout_s": 90.0,
                "on_failure": "avisar",
            })

    # 8. Volver a la mesa del cliente
    steps.append({
        "id": f"{step_prefix}_s8_return_cust",
        "skill": "navigate_to",
        "args": {"place": cust_place},
        "retries": 1,
        "timeout_s": 120.0,
        "on_failure": "avisar",
    })

    # 9. Avisar entrega
    steps.append({
        "id": f"{step_prefix}_s9_deliver_speak",
        "skill": "speak",
        "args": {"text": deliver_phrase(items)},
        "retries": 0,
        "timeout_s": 30.0,
        "on_failure": "continue",
    })

    # 10. Entregar
    steps.append({
        "id": f"{step_prefix}_s10_give",
        "skill": "give_object",
        "args": {},
        "retries": 0,
        "timeout_s": 30.0,
        "on_failure": "continue",
    })

    return steps


def plan_restaurant_missions(
    clients: Sequence[Dict[str, Any]],
    bar_location: str = DEFAULT_BAR_LOCATION,
    orders: Optional[Dict[Any, Sequence[str]]] = None,
    use_tray: bool = True,
    name: str = "restaurant_mission"
) -> Dict[str, Any]:
    """Compila el plan estructurado del ejecutor para atender una lista ordenada de clientes.
    Cada cliente se completa de principio a fin antes de comenzar con el siguiente.
    """
    if orders is None:
        orders = {}

    all_steps: List[Dict[str, Any]] = []

    for i, client in enumerate(clients):
        cid = client.get("id", str(i + 1))
        prefix = f"c{i + 1}_{cid}"
        client_items = orders.get(cid) or orders.get(client.get("id")) or ["coke", "water"]
        cust_steps = plan_customer_steps(
            client=client,
            order_items=client_items,
            bar_location=bar_location,
            use_tray=use_tray,
            step_prefix=prefix
        )
        all_steps.extend(cust_steps)

    recovery = [
        {"id": "avisar", "skill": "speak", "args": {"text": "I could not complete the mission."}},
        {"id": "volver", "skill": "return_home", "timeout_s": 180.0}
    ]

    last_step_id = all_steps[-1]["id"] if all_steps else None

    plan = {
        "name": name,
        "steps": all_steps,
        "recovery": recovery
    }
    if last_step_id:
        plan["success_if"] = last_step_id

    return plan
