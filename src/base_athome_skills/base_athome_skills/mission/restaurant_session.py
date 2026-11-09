"""Maquina de estados pura (sin ROS) para la prueba Restaurant (RoboCup@Home 2026.2 §5.5).

Estrategia:
  1. Inicio junto al kitchen bar: registra pose inicial como 'kitchen_bar'.
  2. Pide deteccion de clientes con la mano levantada (raised_hands).
  3. Informa cuantos clientes vio ("I see one customer" / "I see two customers").
  4. Por cada cliente (hasta 2, en orden de cercania):
     - Navega a un punto a ~0.9 m del cliente mirandolo de frente (orientacion hacia el comensal).
     - Saluda brevemente (<= 15 palabras).
     - Escucha el pedido; extrae items con LLM o fallback al menu.
     - Si no entiende, pide repetir una vez. Si vuelve a fallar, cancela la atencion de ese cliente.
     - Confirma el pedido en una frase breve (<= 15 palabras).
     - Navega al kitchen bar.
     - Comunica el pedido al barman y pide que lo coloque en la bandeja (frase corta <= 15 palabras).
     - Espera colocacion / respuesta "ready"/"done" o timeout de 30 s (opcionalmente verifica con VLM).
     - Regresa a la mesa del cliente mirandolo.
     - Avisa la entrega: "Here is your order, please take it from my tray."
  5. Regresa al kitchen bar y finaliza ("Mission complete. I am back at the kitchen bar.").

Reglas estrictas de competencia:
  - Sin confirmar direccion (-30 pts).
  - Mirar al cliente de frente mientras habla (orientacion y pan-tilt).
  - Frases breves de maximo 15 palabras.
  - Cancelacion inmediata con 'stop' por voz o señal de aborto.
  - Limite de tiempo global de 15 minutos (900 s).
  - Sin HRI alternativo (teclado/pantalla para el pedido penaliza -80 pts).
"""

from enum import Enum
import math
import re
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from . import restaurant as rest
from . import vlm


class SessionState(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    DETECTING_CUSTOMERS = "DETECTING_CUSTOMERS"
    APPROACHING_CUSTOMER = "APPROACHING_CUSTOMER"
    GREETING = "GREETING"
    TAKING_ORDER = "TAKING_ORDER"
    CONFIRMING_ORDER = "CONFIRMING_ORDER"
    RETURNING_TO_BAR = "RETURNING_TO_BAR"
    ORDERING_BARMAN = "ORDERING_BARMAN"
    WAITING_BARMAN = "WAITING_BARMAN"
    DELIVERING_TO_CUSTOMER = "DELIVERING_TO_CUSTOMER"
    RETURNING_TO_BAR_FINAL = "RETURNING_TO_BAR_FINAL"
    SUCCEEDED = "SUCCEEDED"
    ABORTED = "ABORTED"
    TIMEOUT = "TIMEOUT"
    FAILED = "FAILED"


def compute_approach_pose(
    customer_xy: Sequence[float],
    robot_pose: Tuple[float, float, float],
    standoff_dist: float = 0.9,
) -> Tuple[float, float, float]:
    """Calcula la pose (x, y, yaw) a ~standoff_dist metros del cliente mirandolo de frente."""
    cx, cy = float(customer_xy[0]), float(customer_xy[1])
    rx, ry, ryaw = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    dx = cx - rx
    dy = cy - ry
    dist = math.hypot(dx, dy)

    if dist > standoff_dist:
        ux = dx / dist
        uy = dy / dist
        tx = cx - standoff_dist * ux
        ty = cy - standoff_dist * uy
        tyaw = math.atan2(cy - ty, cx - tx)
    elif dist > 0.01:
        tx = rx
        ty = ry
        tyaw = math.atan2(dy, dx)
    else:
        tx, ty, tyaw = rx, ry, ryaw

    return (round(tx, 3), round(ty, 3), round(tyaw, 4))


class RestaurantSession:
    """Maquina de estados pura con interfaces inyectadas para la mision Restaurant."""

    def __init__(
        self,
        navigate_fn: Callable[[float, float, float], bool],
        speak_fn: Callable[[str], None],
        listen_fn: Callable[[float], Optional[str]],
        detect_customers_fn: Callable[[float, int], Any],
        ask_order_fn: Optional[Callable[[str, Sequence[str]], List[str]]] = None,
        ask_vlm_fn: Optional[Callable[[bytes, str], Optional[str]]] = None,
        get_pose_fn: Optional[Callable[[], Tuple[float, float, float]]] = None,
        clock_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        cancel_nav_fn: Optional[Callable[[], None]] = None,
        point_head_fn: Optional[Callable[[float, float], None]] = None,
        get_camera_image_fn: Optional[Callable[[], Optional[bytes]]] = None,
        on_state_change_fn: Optional[Callable[[Dict[str, Any]], None]] = None,
        menu: Sequence[str] = rest.DEFAULT_MENU,
        max_time_s: float = 900.0,
        standoff_dist: float = 0.9,
        use_tray: bool = True,
        max_customers: int = 2,
    ):
        self.navigate_fn = navigate_fn
        self.speak_fn = speak_fn
        self.listen_fn = listen_fn
        self.detect_customers_fn = detect_customers_fn
        self.ask_order_fn = ask_order_fn
        self.ask_vlm_fn = ask_vlm_fn
        self.get_pose_fn = get_pose_fn or (lambda: (0.0, 0.0, 0.0))
        self.clock_fn = clock_fn or time.time
        self.sleep_fn = sleep_fn or time.sleep
        self.cancel_nav_fn = cancel_nav_fn or (lambda: None)
        self.point_head_fn = point_head_fn
        self.get_camera_image_fn = get_camera_image_fn
        self.on_state_change_fn = on_state_change_fn

        self.menu = tuple(menu)
        self.max_time_s = float(max_time_s)
        self.standoff_dist = float(standoff_dist)
        self.use_tray = bool(use_tray)
        self.max_customers = int(max_customers)

        self.state: SessionState = SessionState.IDLE
        self.stopped: bool = False
        self.start_time: float = 0.0
        self.bar_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.customers: List[Dict[str, Any]] = []
        self.current_customer_idx: int = 0
        self.orders: Dict[str, List[str]] = {}
        self.spoken_phrases: List[str] = []

    def _speak(self, text: str) -> None:
        """Envia frase asegurando tope estricto de 15 palabras y registro."""
        clean = rest.limit_words(text, max_words=15)
        self.spoken_phrases.append(clean)
        self.speak_fn(clean)

    def _set_state(self, new_state: SessionState, info: Optional[Dict[str, Any]] = None) -> None:
        self.state = new_state
        if self.on_state_change_fn:
            payload = {
                "state": new_state.value,
                "elapsed_s": round(self.clock_fn() - self.start_time, 2) if self.start_time > 0 else 0.0,
                "customer_idx": self.current_customer_idx,
                "total_customers": len(self.customers),
                "orders": self.orders,
                "info": info or {},
            }
            try:
                self.on_state_change_fn(payload)
            except Exception:
                pass

    def _is_timeout(self) -> bool:
        if self.start_time <= 0:
            return False
        return (self.clock_fn() - self.start_time) >= self.max_time_s

    def stop(self) -> None:
        """Detiene inmediatamente la ejecucion."""
        self.stopped = True
        try:
            self.cancel_nav_fn()
        except Exception:
            pass

    def extract_order(self, speech_text: str) -> List[str]:
        """Extrae items validos del menu mediante LLM local o busqueda directa de palabras clave."""
        if not speech_text or not speech_text.strip():
            return []

        items: List[str] = []
        if self.ask_order_fn:
            try:
                items = self.ask_order_fn(speech_text, self.menu)
            except Exception:
                items = []

        if not items:
            lower = speech_text.lower()
            for m in self.menu:
                m_clean = m.strip().lower()
                if re.search(r"\b" + re.escape(m_clean) + r"\b", lower):
                    if m_clean not in items:
                        items.append(m_clean)

        valid_menu = {m.strip().lower() for m in self.menu}
        filtered = [it for it in items if it in valid_menu]
        return filtered[:2]

    def _wait_for_barman(self, order_items: Sequence[str], max_wait_s: float = 30.0) -> None:
        """Espera a que el barman coloque los objetos en la bandeja o pasen 30 s."""
        t0 = self.clock_fn()
        while self.clock_fn() - t0 < max_wait_s:
            if self.stopped or self._is_timeout():
                return

            if self.ask_vlm_fn and self.get_camera_image_fn and order_items:
                img_bytes = self.get_camera_image_fn()
                if img_bytes:
                    try:
                        if rest.verify_item_on_tray(img_bytes, order_items[0], timeout_s=3.0):
                            return
                    except Exception:
                        pass

            speech = self.listen_fn(min(3.0, max_wait_s - (self.clock_fn() - t0)))
            if speech:
                lower = speech.lower()
                if "stop" in lower.split():
                    self.stop()
                    return
                if any(w in lower for w in ("ready", "done", "finished", "here", "ok")):
                    return
            else:
                self.sleep_fn(0.2)

    def run(self) -> bool:
        """Ejecuta la mision completa."""
        self.start_time = self.clock_fn()
        self.stopped = False
        self._set_state(SessionState.STARTING)

        # 1. Pose de inicio como kitchen_bar
        self.bar_pose = self.get_pose_fn()

        # 2. Deteccion de clientes
        if self._is_timeout():
            return self._handle_timeout()
        if self.stopped:
            return self._handle_stop()

        self._set_state(SessionState.DETECTING_CUSTOMERS)
        raw_cust = self.detect_customers_fn(10.0, 3)

        customers_list: List[Dict[str, Any]] = []
        if isinstance(raw_cust, dict):
            if raw_cust.get("ok", True):
                customers_list = list(raw_cust.get("customers", []))
        elif isinstance(raw_cust, list):
            customers_list = list(raw_cust)

        if not customers_list:
            self._speak("I do not see any customers calling.")
            self._set_state(SessionState.SUCCEEDED, {"reason": "no_customers"})
            return True

        self.customers = customers_list[: self.max_customers]
        n_cust = len(self.customers)
        if n_cust == 1:
            self._speak("I see one customer.")
        else:
            self._speak(f"I see {n_cust} customers.")

        # 3. Atencion secuencial de clientes
        for idx, cust in enumerate(self.customers):
            self.current_customer_idx = idx

            if self._is_timeout():
                return self._handle_timeout()
            if self.stopped:
                return self._handle_stop()

            # Extraer coordenadas del cliente
            c_xy = cust.get("xy")
            if not c_xy:
                c_xy = (cust.get("x", 0.0), cust.get("y", 0.0))
            approach_pose = compute_approach_pose(c_xy, self.get_pose_fn(), self.standoff_dist)

            # Acercarse al cliente
            self._set_state(SessionState.APPROACHING_CUSTOMER, {"target": approach_pose, "customer_index": idx})
            nav_ok = self.navigate_fn(approach_pose[0], approach_pose[1], approach_pose[2])
            if self.stopped:
                return self._handle_stop()
            if not nav_ok:
                self._speak("I could not reach the customer.")
                continue

            # Apuntar la cabeza hacia el cliente si existe pan-tilt
            if self.point_head_fn:
                try:
                    self.point_head_fn(0.0, -10.0)
                except Exception:
                    pass

            # Saludo inicial
            if self._is_timeout():
                return self._handle_timeout()
            self._set_state(SessionState.GREETING)
            self._speak(rest.greeting_phrase())

            # Toma de orden con hasta 2 intentos
            self._set_state(SessionState.TAKING_ORDER)
            order_items: List[str] = []
            for attempt in range(2):
                if self._is_timeout():
                    return self._handle_timeout()
                if self.stopped:
                    return self._handle_stop()

                speech = self.listen_fn(15.0) or ""
                if "stop" in speech.lower().split():
                    return self._handle_stop()

                items = self.extract_order(speech)
                if items:
                    order_items = items
                    break
                else:
                    if attempt == 0:
                        self._speak(rest.repeat_order_phrase())
                    else:
                        self._speak(rest.cancel_order_phrase())

            cust_id = str(cust.get("id", idx + 1))
            self.orders[cust_id] = order_items

            if not order_items:
                # Orden fallida: continuar con el siguiente comensal
                continue

            # Confirmar pedido
            self._set_state(SessionState.CONFIRMING_ORDER, {"items": order_items})
            self._speak(rest.confirm_order_phrase(order_items))

            # Ir al kitchen bar
            if self._is_timeout():
                return self._handle_timeout()
            if self.stopped:
                return self._handle_stop()

            self._set_state(SessionState.RETURNING_TO_BAR)
            nav_bar_ok = self.navigate_fn(self.bar_pose[0], self.bar_pose[1], self.bar_pose[2])
            if self.stopped:
                return self._handle_stop()
            if not nav_bar_ok:
                self._speak("I could not reach the bar.")
                continue

            # Pedir al barman
            self._set_state(SessionState.ORDERING_BARMAN)
            self._speak(rest.barman_request_phrase(order_items, use_tray=self.use_tray))

            # Esperar barman (hasta 30 s o confirmacion)
            self._set_state(SessionState.WAITING_BARMAN)
            self._wait_for_barman(order_items, max_wait_s=30.0)
            if self.stopped:
                return self._handle_stop()
            if self._is_timeout():
                return self._handle_timeout()

            # Regresar al cliente
            self._set_state(SessionState.DELIVERING_TO_CUSTOMER)
            nav_back_ok = self.navigate_fn(approach_pose[0], approach_pose[1], approach_pose[2])
            if self.stopped:
                return self._handle_stop()
            if not nav_back_ok:
                self._speak("I could not return to the customer.")
                continue

            # Entregar orden
            self._speak("Here is your order, please take it from my tray.")

        # 4. Regreso final al kitchen bar
        if self._is_timeout():
            return self._handle_timeout()
        if self.stopped:
            return self._handle_stop()

        self._set_state(SessionState.RETURNING_TO_BAR_FINAL)
        self.navigate_fn(self.bar_pose[0], self.bar_pose[1], self.bar_pose[2])
        self._speak("Mission complete. I am back at the kitchen bar.")
        self._set_state(SessionState.SUCCEEDED)
        return True

    def start(self) -> bool:
        """Alias para iniciar la ejecucion."""
        return self.run()

    def _handle_stop(self) -> bool:
        try:
            self.cancel_nav_fn()
        except Exception:
            pass
        self._speak("Stopping mission.")
        self._set_state(SessionState.ABORTED)
        return False

    def _handle_timeout(self) -> bool:
        try:
            self.cancel_nav_fn()
        except Exception:
            pass
        self._speak("Time limit reached. Returning to the kitchen bar.")
        self._set_state(SessionState.TIMEOUT)
        try:
            self.navigate_fn(self.bar_pose[0], self.bar_pose[1], self.bar_pose[2])
        except Exception:
            pass
        return False
