"""Flujo Restaurant por voz con callbacks inyectados y sin dependencia de ROS.

La barra es la pose inicial medida. Busca al barman, registra caras si hay
adaptadores, confirma cada pedido y sirve en ciclos hasta el limite global.
Con callbacks None se omite reconocimiento/manipulacion y se usa la bandeja.
El audio es bloqueante y precede a la accion anunciada. batch_orders=False es
el flujo secuencial; True recoge el lote visible antes de preparar cada orden.
"""

from enum import Enum
import math
import re
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from . import restaurant as rest
from .restaurant_navigation import is_stop_command


HEAD_LOOK_AROUND_TIMEOUT_S = 80.0


class SessionState(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    FINDING_BARMAN = "FINDING_BARMAN"
    CONFIRMING_BARMAN = "CONFIRMING_BARMAN"
    ENROLLING_FACE = "ENROLLING_FACE"
    CLEARING_COUNTER = "CLEARING_COUNTER"
    CHECKING_ORDER = "CHECKING_ORDER"
    DETECTING_ITEM = "DETECTING_ITEM"
    PICKING_ITEM = "PICKING_ITEM"
    HANDING_OVER = "HANDING_OVER"
    RELEASING_ITEM = "RELEASING_ITEM"
    STOWING_ARM = "STOWING_ARM"
    WAITING_CUSTOMER = "WAITING_CUSTOMER"
    LOOKING_AROUND = "LOOKING_AROUND"
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


# Valor que Nav2 publica en OccupancyGrid para INSCRIBED_INFLATED_OBSTACLE (costo 253): con footprint
# circular el inflado ya contiene el radio del robot y SmacPlanner2D descarta toda celda >= este valor.
NAV2_INSCRIBED_VALUE = 99
NAV2_LETHAL_VALUE = 100
# Distancias al centro del cliente sentado. A 0.8-1.1 m entre centros, con medio ancho de chasis de
# ~0.3 m y ~0.25 m de cuerpo sentado, quedan 0.25-0.55 m libres: zona personal, se le habla de frente
# sin invadirlo. Mas cerca no entra el robot junto a la silla; mas lejos el ASR pierde la voz.
APPROACH_RADII = (0.8, 1.0, 1.1)
APPROACH_ANGLES = 24


def approach_candidates(
    customer_xy: Sequence[float],
    robot_pose: Tuple[float, float, float],
    grid: Optional[Any] = None,
    standoff_dist: float = 0.9,
    robot_radius: float = 0.3,
) -> List[Tuple[float, float, float]]:
    """Ordena poses de aproximacion validas para Nav2, sin importar ROS ni modificar la grilla.

    Recibe un objeto con los campos de OccupancyGrid (el /global_costmap/costmap de Nav2).
    Una pose es candidata si su celda central esta por debajo del costo inscrito (la misma
    condicion con la que SmacPlanner2D acepta una celda con footprint circular: el inflado ya
    contiene el radio de 0.44 m) y si todo el disco del chasis (robot_radius) esta observado;
    desconocido/fuera de la grilla = no candidata. Volver a exigir costo bajo en todo el disco
    contaria dos veces el footprint y descartaria las poses junto a un cliente contra la pared.
    Se ordenan por distancia al robot penalizada por el costo (prefiere poses mas despejadas).
    Sin grilla utilizable o sin candidatos conserva la pose rectilinea, que nunca se usa como
    alternativa de reintento.
    """
    fallback = compute_approach_pose(customer_xy, robot_pose, standoff_dist)
    if grid is None:
        return [fallback]
    info = grid.info
    if (not math.isfinite(info.resolution) or info.resolution <= 0
            or info.width <= 0 or info.height <= 0
            or len(grid.data) != info.width * info.height):
        return [fallback]
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    co, si = math.cos(yaw), math.sin(yaw)
    res = info.resolution

    def cost(pose) -> Optional[int]:
        """Costo de la celda central, o None si la pose no es valida para Nav2."""
        dx, dy = pose[0] - info.origin.position.x, pose[1] - info.origin.position.y
        x, y = co * dx + si * dy, -si * dx + co * dy
        cix, ciy = math.floor(x / res), math.floor(y / res)
        if not (0 <= cix < info.width and 0 <= ciy < info.height):
            return None
        center = grid.data[ciy * info.width + cix]
        if not 0 <= center < NAV2_INSCRIBED_VALUE:
            return None
        for iy in range(math.floor((y - robot_radius) / res), math.floor((y + robot_radius) / res) + 1):
            for ix in range(math.floor((x - robot_radius) / res), math.floor((x + robot_radius) / res) + 1):
                nx = max(ix * res - x, 0, x - (ix + 1) * res)
                ny = max(iy * res - y, 0, y - (iy + 1) * res)
                if math.hypot(nx, ny) > robot_radius:
                    continue
                if not (0 <= ix < info.width and 0 <= iy < info.height):
                    return None
                # El inflado del centro ya separa el footprint: el disco solo exige celdas observadas.
                if not 0 <= grid.data[iy * info.width + ix] < NAV2_LETHAL_VALUE:
                    return None
        return center

    cx, cy = float(customer_xy[0]), float(customer_xy[1])
    angle = math.atan2(robot_pose[1] - cy, robot_pose[0] - cx)
    scored = []
    for radius in dict.fromkeys((standoff_dist,) + APPROACH_RADII):
        for i in range(APPROACH_ANGLES):
            a = angle + i * math.tau / APPROACH_ANGLES
            x, y = cx + radius * math.cos(a), cy + radius * math.sin(a)
            pose = (x, y, math.atan2(cy - y, cx - x))
            c = cost(pose)
            if c is not None:
                # 1 m de penalizacion por costo casi inscrito: entre dos poses parecidas, la despejada.
                score = math.hypot(x - robot_pose[0], y - robot_pose[1]) + c / NAV2_INSCRIBED_VALUE
                scored.append((score, pose))
    scored.sort(key=lambda sp: sp[0])
    return [pose for _, pose in scored] or [fallback]


def progressive_advance_candidates(
    customer_xy: Sequence[float],
    robot_pose: Tuple[float, float, float],
    grid: Optional[Any] = None,
    standoff_dist: float = 0.9,
    robot_radius: float = 0.3,
) -> List[Tuple[float, float, float]]:
    """Genera poses intermedias en direccion al cliente para avanzar progresivamente sin mapa previo.

    Se usa cuando el cliente esta muy lejos o su distancia fue estimada sin profundidad (estimated=True),
    permitiendo acercarse ~1.0-2.2 m dentro del costmap SLAM antes de re-medir con la RealSense.
    """
    cx, cy = float(customer_xy[0]), float(customer_xy[1])
    rx, ry, _ = float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2])
    dx, dy = cx - rx, cy - ry
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    max_advance = max(0.6, dist - max(standoff_dist, 1.1))
    step_default = min(2.0, max(0.6, max_advance * 0.6))
    fallback = (
        round(rx + step_default * math.cos(bearing), 3),
        round(ry + step_default * math.sin(bearing), 3),
        round(bearing, 4),
    )
    if grid is None:
        return [fallback]
    info = grid.info
    if (not math.isfinite(info.resolution) or info.resolution <= 0
            or info.width <= 0 or info.height <= 0
            or len(grid.data) != info.width * info.height):
        return [fallback]
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    co, si = math.cos(yaw), math.sin(yaw)
    res = info.resolution

    def cost(pose) -> Optional[int]:
        px, py = pose[0] - info.origin.position.x, pose[1] - info.origin.position.y
        x, y = co * px + si * py, -si * px + co * py
        cix, ciy = math.floor(x / res), math.floor(y / res)
        if not (0 <= cix < info.width and 0 <= ciy < info.height):
            return None
        center = grid.data[ciy * info.width + cix]
        if not 0 <= center < NAV2_INSCRIBED_VALUE:
            return None
        for iy in range(math.floor((y - robot_radius) / res), math.floor((y + robot_radius) / res) + 1):
            for ix in range(math.floor((x - robot_radius) / res), math.floor((x + robot_radius) / res) + 1):
                nx = max(ix * res - x, 0, x - (ix + 1) * res)
                ny = max(iy * res - y, 0, y - (iy + 1) * res)
                if math.hypot(nx, ny) > robot_radius:
                    continue
                if not (0 <= ix < info.width and 0 <= iy < info.height):
                    return None
                if not 0 <= grid.data[iy * info.width + ix] < NAV2_LETHAL_VALUE:
                    return None
        return center

    scored = []
    for frac in (0.65, 0.50, 0.35, 0.80):
        step = min(2.4, max(0.6, max_advance * frac))
        for d_ang in (0.0, 0.22, -0.22, 0.45, -0.45):
            ang = bearing + d_ang
            tx, ty = rx + step * math.cos(ang), ry + step * math.sin(ang)
            rem_dist = math.hypot(cx - tx, cy - ty)
            if rem_dist >= dist - 0.35:
                continue
            pose = (tx, ty, math.atan2(cy - ty, cx - tx))
            c = cost(pose)
            if c is not None:
                score = rem_dist + c / NAV2_INSCRIBED_VALUE
                scored.append((score, pose))
    scored.sort(key=lambda sp: sp[0])
    return [pose for _, pose in scored] or [fallback]


def rank_customers(
    customers: Sequence[Dict[str, Any]],
    robot_pose: Tuple[float, float, float],
) -> List[Dict[str, Any]]:
    """Task planning para ordenar clientes detectados: primero posicion medida (no estimada) y mas cercana."""
    rx, ry = float(robot_pose[0]), float(robot_pose[1])
    ranked: List[Tuple[Tuple[int, float, int], Dict[str, Any]]] = []
    for idx, cust in enumerate(customers):
        c_copy = dict(cust)
        xy = c_copy.get("xy") or (c_copy.get("x", 0.0), c_copy.get("y", 0.0))
        try:
            cx, cy = float(xy[0]), float(xy[1])
        except (TypeError, ValueError, IndexError):
            cx, cy = 0.0, 0.0
        dist = math.hypot(cx - rx, cy - ry) if (math.isfinite(cx) and math.isfinite(cy)) else float("inf")
        est_rank = 1 if c_copy.get("estimated", False) else 0
        ranked.append(((est_rank, round(dist, 3), idx), c_copy))
    ranked.sort(key=lambda item: item[0])
    return [c for _, c in ranked]


def path_length(points: Sequence[Sequence[float]]) -> float:
    """Largo de una polilinea (x, y)."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def validate_with_planner(
    candidates: Sequence[Tuple[float, float, float]],
    plan_fn: Optional[Callable[[Tuple[float, float, float]], Optional[float]]],
    clock_fn: Callable[[], float] = time.monotonic,
    budget_s: float = 10.0,
    max_valid: int = 3,
    max_checks: int = 16,
) -> Tuple[List[Tuple[float, float, float]], bool]:
    """Pide a Nav2 un plan a cada candidata (en el orden dado) y devuelve (poses, validadas).

    plan_fn(pose) devuelve el largo del plan, None si el planner no encontro camino, o lanza si el
    planner no esta disponible. Se guardan hasta max_valid poses con plan, ordenadas por largo
    del plan (el camino real, no la recta). Si el planner no responde, se conservan las candidatas
    sin validar (validadas=False) para no empeorar el comportamiento previo.
    """
    if plan_fn is None or not candidates:
        return list(candidates), False
    end = clock_fn() + budget_s
    valid = []
    for checks, pose in enumerate(candidates):
        if checks >= max_checks or len(valid) >= max_valid or clock_fn() >= end:
            break
        try:
            length = plan_fn(pose)
        except Exception:
            if not valid:
                return list(candidates), False
            break
        if length is not None and math.isfinite(length):
            valid.append((length, checks, pose))
    valid.sort(key=lambda v: (v[0], v[1]))
    return [pose for _, _, pose in valid], True


class RestaurantSession:
    """Maquina de estados pura con interfaces inyectadas para Restaurant.

    Los siete callbacks de cara/manipulacion son opcionales. arm=False o
    pick_item_fn=None fuerza bandeja, incluso con use_tray=False (parametro
    heredado). delivery_wait_s se acepta por compatibilidad; ahora la descarga
    se confirma por voz. Los callbacks son sincronicos y deben respetar stop
    y sus propios limites de ejecucion; la sesion comprueba el deadline antes
    y despues de cada llamada.
    """

    def __init__(
        self,
        navigate_fn: Callable[..., bool],
        speak_fn: Callable[[str], None],
        listen_fn: Callable[[float], Optional[str]],
        detect_customers_fn: Callable[[float, int], Any],
        ask_order_fn: Optional[Callable[[str, Sequence[str]], List[str]]] = None,
        ask_vlm_fn: Optional[Callable[[bytes, str], Optional[str]]] = None,
        get_pose_fn: Optional[Callable[[], Optional[Tuple[float, float, float]]]] = None,
        clock_fn: Optional[Callable[[], float]] = None,
        sleep_fn: Optional[Callable[[float], None]] = None,
        cancel_nav_fn: Optional[Callable[[], None]] = None,
        point_head_fn: Optional[Callable[[float, float], None]] = None,
        get_camera_image_fn: Optional[Callable[[], Optional[bytes]]] = None,
        on_state_change_fn: Optional[Callable[[Dict[str, Any]], None]] = None,
        menu: Sequence[str] = rest.DEFAULT_MENU,
        max_time_s: float = 900.0,
        standoff_dist: float = 0.9,
        delivery_wait_s: float = 7.0,
        use_tray: bool = True,
        max_customers: int = 2,
        get_costmap_fn: Optional[Callable[[], Any]] = None,
        nav_aborted_fn: Optional[Callable[[], bool]] = None,
        pose_timeout_s: float = 5.0,
        plan_fn: Optional[Callable[[Tuple[float, float, float]], Optional[float]]] = None,
        spin_fn: Optional[Callable[[float], bool]] = None,
        batch_orders: bool = False,
        enroll_face_fn: Optional[Callable[[str, Optional[dict]], bool]] = None,
        identify_face_fn: Optional[Callable[[dict], Optional[str]]] = None,
        detect_item_fn: Optional[Callable[[str, float], bool]] = None,
        pick_item_fn: Optional[Callable[[str], bool]] = None,
        handover_fn: Optional[Callable[[], bool]] = None,
        release_fn: Optional[Callable[[], bool]] = None,
        stow_fn: Optional[Callable[[], bool]] = None,
        arm: bool = True,
    ):
        self.enroll_face_fn = enroll_face_fn
        self.identify_face_fn = identify_face_fn
        self.detect_item_fn = detect_item_fn
        self.pick_item_fn = pick_item_fn
        self.handover_fn = handover_fn
        self.release_fn = release_fn
        self.stow_fn = stow_fn
        self.arm = bool(arm)
        self.served_labels: set[str] = set()
        self._visited_customers: List[Dict[str, Any]] = []
        self._holding_item = False
        self.spin_fn = spin_fn
        self.plan_fn = plan_fn
        self.batch_orders = bool(batch_orders)
        self.get_costmap_fn = get_costmap_fn or (lambda: None)
        self.nav_aborted_fn = nav_aborted_fn or (lambda: False)
        self.navigate_fn = navigate_fn
        self.speak_fn = speak_fn
        self.listen_fn = listen_fn
        self.detect_customers_fn = detect_customers_fn
        self.ask_order_fn = ask_order_fn
        self.ask_vlm_fn = ask_vlm_fn
        self.get_pose_fn = get_pose_fn or (lambda: None)
        self.clock_fn = clock_fn or time.monotonic
        self.sleep_fn = sleep_fn or time.sleep
        self.cancel_nav_fn = cancel_nav_fn or (lambda: None)
        self.point_head_fn = point_head_fn
        self.get_camera_image_fn = get_camera_image_fn
        self.on_state_change_fn = on_state_change_fn

        self.menu = tuple(menu)
        self.max_time_s = float(max_time_s)
        self.pose_timeout_s = float(pose_timeout_s)
        if not math.isfinite(self.max_time_s) or self.max_time_s <= 0:
            raise ValueError("max_time_s must be positive and finite")
        if not math.isfinite(self.pose_timeout_s) or self.pose_timeout_s <= 0:
            raise ValueError("pose_timeout_s must be positive and finite")
        self.standoff_dist = float(standoff_dist)
        self.delivery_wait_s = float(delivery_wait_s)
        if not math.isfinite(self.delivery_wait_s) or self.delivery_wait_s < 0:
            raise ValueError("delivery_wait_s must be non-negative and finite")
        self.served = 0
        self.stop_look_on_customer = True     # cortar el barrido inicial en cuanto aparece un cliente
        self.use_tray = bool(use_tray)
        self.max_customers = int(max_customers)
        if self.max_customers <= 0:
            raise ValueError("max_customers must be positive")

        self.state: SessionState = SessionState.IDLE
        self.stopped: bool = False
        self.start_time: float = 0.0
        self.deadline_s: Optional[float] = None
        self.bar_pose: Optional[Tuple[float, float, float]] = None
        self.customers: List[Dict[str, Any]] = []
        self.current_customer_idx: int = 0
        self.orders: Dict[str, List[str]] = {}
        self.spoken_phrases: List[str] = []

    def _speak(self, text: str, *, terminal: bool = False) -> None:
        """Audio bloqueante, completo; no cortar instrucciones ni confirmaciones."""
        if not terminal and (self.stopped or self._is_timeout()):
            return
        clean = text.strip()
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
        return self.deadline_s is not None and self.clock_fn() >= self.deadline_s

    def remaining_s(self) -> float:
        return max(0.0, self.deadline_s - self.clock_fn()) if self.deadline_s is not None else 0.0

    def _navigate_to(self, pose: Tuple[float, float, float]) -> bool:
        """El adaptador recibe el deadline absoluto y debe cancelar/drenar al vencer."""
        if self.stopped or self._is_timeout():
            return False
        return self.navigate_fn(*pose, deadline_s=self.deadline_s)

    def _wait_for_pose(self) -> Optional[Tuple[float, float, float]]:
        deadline = min(self.deadline_s, self.clock_fn() + self.pose_timeout_s)
        while not self.stopped and self.clock_fn() < deadline:
            try:
                pose = self.get_pose_fn()
                if pose is not None and len(pose) == 3 and all(math.isfinite(v) for v in pose):
                    return tuple(float(v) for v in pose)
            except Exception:
                pass
            self.sleep_fn(min(0.1, max(0.0, deadline - self.clock_fn())))
        return None

    def _look_around(self) -> List[Dict[str, Any]]:
        """Tres vistas con pan fijo; spin_fn debe cancelar/drenar ante stop o deadline.

        Los ids de bag_finder son locales a cada pedido: asociar por distancia en map
        (0.8 m, igual que RaisedHandsAggregator), nunca por id ni por rango de camara.
        Sin adaptador Spin conserva la deteccion unica para clientes existentes.
        """
        self.detection_error = ""
        customers: List[Dict[str, Any]] = []
        angles = (math.pi / 2, -math.pi, math.pi / 2) if self.spin_fn else (None,)
        for angle in angles:
            if self.stopped or self._is_timeout():
                break
            spin_ok = True
            if angle is not None:
                if self.point_head_fn:
                    self.point_head_fn(0.0, 0.0)
                self._set_state(SessionState.LOOKING_AROUND, {"angle_rad": angle})
                spin_ok = self.spin_fn(angle)
            if self.stopped or self._is_timeout():
                break
            self._set_state(SessionState.DETECTING_CUSTOMERS)
            if self.stopped or self._is_timeout():
                break
            raw = self.detect_customers_fn(min(14.0 if self.spin_fn else HEAD_LOOK_AROUND_TIMEOUT_S, self.remaining_s()), 3)   # 8 s era poco con YOLO en la CPU
            if isinstance(raw, dict) and not raw.get("ok", True):
                self.detection_error = raw.get("error", "")
            view = (raw.get("customers", []) if raw.get("ok", True) else []) if isinstance(raw, dict) else raw
            if isinstance(view, list):
                for customer in view:
                    if not self._eligible_customer(customer):
                        continue
                    if not self.spin_fn:
                        customers.append(dict(customer))
                        continue
                    xy = customer.get("xy", (customer.get("x"), customer.get("y")))
                    try:
                        x, y = map(float, xy)
                    except (TypeError, ValueError):
                        continue
                    if not math.isfinite(x) or not math.isfinite(y):
                        continue
                    if not any(math.hypot(x - c["xy"][0], y - c["xy"][1]) < 0.8 for c in customers):
                        customers.append(dict(customer, xy=[x, y]))
            # Pedido del usuario: apenas ve a alguien con la mano levantada, corta el barrido y va hacia esa persona
            # (su posicion en el mapa ya quedo guardada). No sigue girando.
            if customers and self.spin_fn and self.stop_look_on_customer:
                self._set_state(SessionState.DETECTING_CUSTOMERS, {"stopped_look_at_angle": angle, "customers": len(customers)})
                break
            # Tras un giro fallido no se conoce el yaw alcanzado. Aprovechar esta vista
            # y las anteriores, sin encadenar giros relativos que ya no volverian al frente.
            if not spin_ok:
                break
        if self.spin_fn:
            customers.sort(key=lambda c: math.hypot(c["xy"][0] - self.bar_pose[0],
                                                    c["xy"][1] - self.bar_pose[1]))
            for idx, customer in enumerate(customers, 1):
                customer["id"] = str(idx)
        return customers

    def _handle_pose_failure(self) -> bool:
        if self.stopped:
            return self._handle_stop()
        if self._is_timeout():
            return self._handle_timeout()
        self.cancel_nav_fn()
        self._speak("Robot position unavailable. Stopping mission.")
        self._set_state(SessionState.FAILED, {"reason": "pose_unavailable"})
        return False

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
            found: List[Tuple[int, str]] = []
            for m in self.menu:
                m_clean = m.strip().lower()
                for match in re.finditer(r"\b" + re.escape(m_clean) + r"\b", lower):
                    found.append((match.start(), m_clean))
            found.sort(key=lambda x: x[0])
            for _, item in found:
                if item not in items:
                    items.append(item)

        valid_menu = {m.strip().lower() for m in self.menu}
        filtered = [it for it in items if it in valid_menu]
        return filtered[:2]

    def _wait_for_barman(self, order_items: Sequence[str], max_wait_s: float = 30.0) -> None:
        """Confirmacion por voz o espera acotada; firma heredada conserva order_items."""
        end = min(self.deadline_s, self.clock_fn() + max_wait_s)
        while not self.stopped and self.clock_fn() < end:
            speech = self._listen(min(3.0, end - self.clock_fn()))
            if self.stopped or self._is_timeout() or rest.yes_no_answer(speech) is True:
                return
            self.sleep_fn(min(0.2, max(0.0, end - self.clock_fn())))

    def _relocalize_customer_near(
        self, expected_xy: Sequence[float], max_gate_m: float = 2.5
    ) -> Optional[Tuple[float, float]]:
        """Re-detecta al cliente desde mas cerca para reemplazar una posicion estimada por una medida."""
        if self.stopped or self._is_timeout() or self.remaining_s() <= 1.0:
            return None
        try:
            raw = self.detect_customers_fn(min(6.0, self.remaining_s()), 2)
        except Exception:
            return None
        view = (raw.get("customers", []) if raw.get("ok", True) else []) if isinstance(raw, dict) else raw
        if not isinstance(view, list):
            return None
        best_xy: Optional[Tuple[float, float]] = None
        best_score = (2, float("inf"))
        for c in view:
            xy = c.get("xy", (c.get("x"), c.get("y")))
            try:
                x, y = float(xy[0]), float(xy[1])
            except (TypeError, ValueError, IndexError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            d = math.hypot(x - float(expected_xy[0]), y - float(expected_xy[1]))
            if d <= max_gate_m:
                est_rank = 1 if c.get("estimated", False) else 0
                if (est_rank, d) < best_score:
                    best_score = (est_rank, d)
                    best_xy = (x, y)
        return best_xy

    def _approach_customer(
        self, idx: int, cust: Dict[str, Any]
    ) -> Optional[Tuple[Tuple[float, float, float], List[Tuple[float, float, float]], int]]:
        """Acerca el robot al cliente (con avance progresivo si esta muy lejos o fue estimado sin depth)."""
        c_xy = cust.get("xy")
        if not c_xy:
            c_xy = (cust.get("x", 0.0), cust.get("y", 0.0))
        robot_pose = self._wait_for_pose()
        if robot_pose is None:
            self._abort_run = True
            self._handle_pose_failure()
            return None

        dist_to_cust = math.hypot(float(c_xy[0]) - robot_pose[0], float(c_xy[1]) - robot_pose[1])
        did_progressive = False

        if cust.get("estimated", False) and dist_to_cust > self.standoff_dist + 0.6:
            adv_cands = progressive_advance_candidates(
                c_xy, robot_pose, self.get_costmap_fn(), self.standoff_dist
            )
            adv_cands, _ = validate_with_planner(
                adv_cands, self.plan_fn, self.clock_fn, budget_s=min(6.0, self.remaining_s())
            )
            if self.stopped or self._is_timeout():
                return None
            if adv_cands:
                did_progressive = True
                step_pose = adv_cands[0]
                self._set_state(
                    SessionState.APPROACHING_CUSTOMER,
                    {"target": step_pose, "customer_index": idx, "customer_xy": list(c_xy), "progressive": True},
                )
                self._speak("Moving to the customer table.")
                step_ok = self._navigate_to(step_pose)
                if self.stopped or self._is_timeout():
                    return None
                if step_ok:
                    robot_pose = self._wait_for_pose()
                    if robot_pose is None:
                        self._abort_run = True
                        self._handle_pose_failure()
                        return None
                    refined = self._relocalize_customer_near(c_xy)
                    if self.stopped or self._is_timeout():
                        return None
                    if refined is not None:
                        c_xy = refined
                        cust["xy"] = [round(refined[0], 2), round(refined[1], 2)]
                        cust["estimated"] = False

        candidates = approach_candidates(c_xy, robot_pose, self.get_costmap_fn(), self.standoff_dist)
        n_free = len(candidates)
        candidates, validated = validate_with_planner(
            candidates, self.plan_fn, self.clock_fn, budget_s=min(10.0, self.remaining_s())
        )
        if self.stopped or self._is_timeout():
            return None

        if not candidates and not did_progressive and dist_to_cust >= 3.5:
            adv_cands = progressive_advance_candidates(
                c_xy, robot_pose, self.get_costmap_fn(), self.standoff_dist
            )
            adv_cands, _ = validate_with_planner(
                adv_cands, self.plan_fn, self.clock_fn, budget_s=min(6.0, self.remaining_s())
            )
            if self.stopped or self._is_timeout():
                return None
            if adv_cands:
                step_pose = adv_cands[0]
                self._set_state(
                    SessionState.APPROACHING_CUSTOMER,
                    {"target": step_pose, "customer_index": idx, "customer_xy": list(c_xy), "progressive": True},
                )
                self._speak("Moving to the customer table.")
                if self._navigate_to(step_pose):
                    robot_pose = self._wait_for_pose()
                    if robot_pose is None:
                        self._abort_run = True
                        self._handle_pose_failure()
                        return None
                    candidates = approach_candidates(
                        c_xy, robot_pose, self.get_costmap_fn(), self.standoff_dist
                    )
                    n_free = len(candidates)
                    candidates, validated = validate_with_planner(
                        candidates, self.plan_fn, self.clock_fn, budget_s=min(10.0, self.remaining_s())
                    )
                    if self.stopped or self._is_timeout():
                        return None

        if not candidates:
            self._set_state(
                SessionState.APPROACHING_CUSTOMER,
                {"customer_index": idx, "customer_xy": list(c_xy), "candidates": n_free, "reachable": 0},
            )
            self._speak("I could not reach the customer.")
            return None

        candidate_idx = 0
        approach_pose = candidates[candidate_idx]
        self._set_state(
            SessionState.APPROACHING_CUSTOMER,
            {
                "target": approach_pose,
                "customer_index": idx,
                "customer_xy": list(c_xy),
                "candidates": n_free,
                "reachable": len(candidates) if validated else None,
            },
        )
        self._speak("Moving to the customer table.")
        nav_ok = self._navigate_to(approach_pose)
        if (
            not nav_ok
            and not self.stopped
            and not self._is_timeout()
            and self.nav_aborted_fn()
            and len(candidates) > 1
        ):
            candidate_idx += 1
            approach_pose = candidates[candidate_idx]
            self._set_state(
                SessionState.APPROACHING_CUSTOMER,
                {"target": approach_pose, "customer_index": idx, "retry": 1},
            )
            self._speak("Moving to the customer table.")
            nav_ok = self._navigate_to(approach_pose)
        if self.stopped or self._is_timeout():
            return None
        if not nav_ok:
            self._speak("I could not reach the customer.")
            return None
        return approach_pose, candidates, candidate_idx

    def _listen(self, timeout_s: float = 15.0) -> str:
        if self.stopped or self._is_timeout():
            return ""
        speech = self.listen_fn(min(timeout_s, self.remaining_s())) or ""
        if is_stop_command(speech):
            self.stop()
        return speech if not self.stopped and not self._is_timeout() else ""

    def _ask_yes_no(self, prompt: str, retry_prompt: Optional[str] = None,
                    attempts: int = 3) -> Optional[bool]:
        for attempt in range(attempts):
            if self.stopped or self._is_timeout():
                return None
            self._speak(prompt if attempt == 0 else (retry_prompt or prompt))
            answer = rest.yes_no_answer(self._listen())
            if self.stopped or self._is_timeout():
                return None
            if answer is not None:
                return answer
        return None

    def _enroll_face(self, label: str, person: Optional[dict]) -> None:
        if self.enroll_face_fn is None or self.stopped or self._is_timeout():
            return
        self._set_state(SessionState.ENROLLING_FACE, {"label": label})
        if self.stopped or self._is_timeout():
            return
        try:
            ok = bool(self.enroll_face_fn(label, person))
        except Exception:
            ok = False
        self._set_state(SessionState.ENROLLING_FACE, {"label": label, "ok": ok})

    def _find_barman(self) -> None:
        # Dos intentos totales. Nunca se le pide al barman confirmar clientes.
        for _ in range(2):
            if self.stopped or self._is_timeout():
                return
            self._set_state(SessionState.FINDING_BARMAN)
            self._speak(rest.BARMAN_SEARCH)
            if self.stopped or self._is_timeout():
                return
            raw = self.detect_customers_fn(min(HEAD_LOOK_AROUND_TIMEOUT_S, self.remaining_s()), 3)
            if self.stopped or self._is_timeout():
                return
            people = (raw.get("customers", []) if raw.get("ok", True) else []) if isinstance(raw, dict) else raw
            if not people:
                continue
            def distance(person):
                xy = person.get("xy") or (person.get("x"), person.get("y"))
                try:
                    d = math.hypot(float(xy[0]) - self.bar_pose[0], float(xy[1]) - self.bar_pose[1])
                    return d if math.isfinite(d) else float("inf")
                except (TypeError, ValueError, IndexError):
                    return float("inf")
            person = min(people, key=distance)
            self._set_state(SessionState.CONFIRMING_BARMAN)
            answer = self._ask_yes_no("Are you the barman? Please answer robot yes or robot no.", attempts=1)
            if answer is True:
                self._enroll_face("barman", person)
                return

    def _eligible_customer(self, person: dict) -> bool:
        if self.stopped or self._is_timeout():
            return False
        label = None
        if self.identify_face_fn is not None:
            try:
                label = self.identify_face_fn(person)
            except Exception:
                label = None
            if label == "barman" or label in self.served_labels:
                return False
            if any(label == c.get("_face_label") for c in self._visited_customers if label):
                return False
        if label:
            return not self.stopped and not self._is_timeout()
        # Los ids del detector cambian entre barridos. Sin cara disponible,
        # conservar la asociacion espacial del barrido, no deduplicar por id.
        xy = person.get("xy") or (person.get("x"), person.get("y"))
        for previous in self._visited_customers:
            old = previous.get("xy") or (previous.get("x"), previous.get("y"))
            try:
                if math.hypot(float(xy[0]) - float(old[0]), float(xy[1]) - float(old[1])) < 0.8:
                    return False
            except (TypeError, ValueError, IndexError):
                pass
        return not self.stopped and not self._is_timeout()

    def _take_customer_order(self, idx: int, cust: Dict[str, Any]) -> List[str]:
        if self._is_timeout() or self.stopped:
            return []
        label = f"customer_{len(self._visited_customers) + 1}"
        cust["_face_label"] = label
        self._visited_customers.append(cust)
        self._set_state(SessionState.GREETING)
        self._speak(rest.greeting_phrase())
        self._enroll_face(label, cust)
        self._set_state(SessionState.CHECKING_ORDER)
        has_order = self._ask_yes_no(rest.HAS_ORDER, rest.REPEAT_HAS_ORDER)
        cust_id = str(cust.get("id", idx + 1))
        if cust_id in self.orders:
            cust_id = label
        self.orders[cust_id] = []
        if has_order is not True:
            return []

        for attempt in range(3):
            if self.stopped or self._is_timeout():
                return []
            self._set_state(SessionState.TAKING_ORDER, {"attempt": attempt + 1})
            self._speak(rest.ORDER_PROMPT)
            speech = self._listen()
            if self.stopped or self._is_timeout():
                return []
            items = self.extract_order(speech)
            if self.stopped or self._is_timeout():
                return []
            if items:
                self._set_state(SessionState.CONFIRMING_ORDER, {"items": items, "attempt": attempt + 1})
                # Una respuesta incomprensible consume el intento, igual que un no.
                accepted = self._ask_yes_no(rest.confirm_order_phrase(items), attempts=1)
                if self.stopped or self._is_timeout():
                    return []
                if accepted is True:
                    self.orders[cust_id] = items
                    self._speak(f"Thank you. I will ask the barman for: {rest.joined_items(items)}.")
                    return items
                if attempt < 2:
                    self._speak(rest.ORDER_CORRECTION)
            elif attempt < 2:
                self._speak(rest.repeat_order_phrase())
        self._speak(rest.cancel_order_phrase())
        return []

    def _arm_action(self, state: SessionState, callback: Optional[Callable],
                    *args, emit_state: bool = True) -> bool:
        if self.stopped or self._is_timeout():
            return False
        if emit_state:
            self._set_state(state, {"item": args[0]} if args else {})
        if self.stopped or self._is_timeout():
            return False
        try:
            return bool(callback(*args)) if callback is not None else False
        except Exception:
            # Una excepcion no garantiza brazo seguro, a diferencia de pick=False.
            self._abort_run = True
            self._set_state(SessionState.FAILED, {"reason": "arm_callback_error", "action": state.value})
            return False

    def _prepare_order(self, items: Sequence[str]) -> Tuple[Optional[str], List[str]]:
        held = None
        tray = list(items)
        if self.arm and self.pick_item_fn is not None and items:
            item = items[0]
            self._speak(rest.counter_request_phrase(item))
            detected = self.detect_item_fn is None  # adaptadores antiguos sin detector
            for attempt in range(3) if self.detect_item_fn is not None else ():
                if self.stopped or self._is_timeout():
                    return None, []
                self._set_state(SessionState.DETECTING_ITEM, {"item": item, "attempt": attempt + 1})
                if self.stopped or self._is_timeout():
                    return None, []
                try:
                    detected = bool(self.detect_item_fn(item, min(10.0, self.remaining_s())))
                except Exception:
                    detected = False
                if self.stopped or self._is_timeout():
                    return None, []
                if detected:
                    break
                self._speak(f"I cannot see the {item}. Can you please slightly move and rotate it?")
            if detected:
                for _ in range(2):
                    self._speak(f"I will now pick the {item}.")
                    if self._arm_action(SessionState.PICKING_ITEM, self.pick_item_fn, item):
                        held = item
                        self._holding_item = True
                        tray = list(items[1:])
                        break
                    if self._abort_run or self.stopped or self._is_timeout():
                        return None, []
            if held is None:
                self._speak(f"Please place the {item} on my tray instead.")
        for item in tray:
            if self.stopped or self._is_timeout() or self._abort_run:
                break
            # Tambien el fallback pide posicion acostada y confirmacion por item.
            self._speak(rest.tray_request_phrase(item))
            self._speak("Tell me robot yes when you are done.")
            self._set_state(SessionState.WAITING_BARMAN, {"item": item})
            self._wait_for_barman([item])
        return held, tray

    def _delivery_failed(self, reason: str) -> bool:
        self._abort_run = True
        self._set_state(SessionState.FAILED, {"reason": reason})
        return False

    def _fulfill_and_deliver_order(
        self, idx: int, cust: Dict[str, Any], order_items: Sequence[str],
        approach_pose: Tuple[float, float, float],
        candidates: Sequence[Tuple[float, float, float]], candidate_idx: int,
    ) -> bool:
        if self._is_timeout() or self.stopped:
            return False
        self._set_state(SessionState.RETURNING_TO_BAR)
        self._speak("Moving to the barman table.")
        nav_bar_ok = self._navigate_to(self.bar_pose)
        if self.stopped or self._is_timeout():
            return False
        if not nav_bar_ok:
            self._speak("I could not reach the bar.")
            return False
        self._set_state(SessionState.ORDERING_BARMAN)
        self._speak(rest.barman_request_phrase(order_items))
        held, tray = self._prepare_order(order_items)
        if self.stopped or self._is_timeout() or self._abort_run:
            return False
        # Nunca viajar con un objeto que no se puede entregar con este contrato.
        if held and not all((self.handover_fn, self.release_fn, self.stow_fn)):
            return self._delivery_failed("missing_delivery_callback")
        self._set_state(SessionState.DELIVERING_TO_CUSTOMER)
        self._speak("Moving to the customer table.")
        nav_back_ok = self._navigate_to(approach_pose)
        if (not nav_back_ok and not self.stopped and not self._is_timeout()
                and self.nav_aborted_fn() and candidate_idx + 1 < len(candidates)):
            candidate_idx += 1
            approach_pose = candidates[candidate_idx]
            self._set_state(SessionState.DELIVERING_TO_CUSTOMER,
                            {"target": approach_pose, "customer_index": idx, "retry": 1})
            self._speak("Moving to the customer table.")
            nav_back_ok = self._navigate_to(approach_pose)
        if self.stopped or self._is_timeout():
            return False
        if not nav_back_ok:
            self._speak("I could not return to the customer.")
            if held:
                return self._delivery_failed("held_item_customer_unreachable")
            return False
        self._speak(rest.deliver_phrase(order_items))
        if held:
            self._speak("Let me give you the object in my hand.")
            if not self._arm_action(SessionState.HANDING_OVER, self.handover_fn):
                if self.stopped or self._is_timeout() or self._abort_run:
                    return False
                return self._delivery_failed("handover_failed")
            # El tablero se actualiza antes de contar: despues de "one" solo
            # se comprueban stop/deadline y se llama al adaptador de apertura.
            self._set_state(SessionState.RELEASING_ITEM)
            self._speak("Please grab the object in my hand. I will open my hand in three,")
            self._speak("two,")
            self._speak("one.")
            if not self._arm_action(SessionState.RELEASING_ITEM, self.release_fn, emit_state=False):
                if self.stopped or self._is_timeout() or self._abort_run:
                    return False
                return self._delivery_failed("release_failed")
            self._holding_item = False
            self._speak("I will now stow my arm.")
            if not self._arm_action(SessionState.STOWING_ARM, self.stow_fn):
                if self.stopped or self._is_timeout() or self._abort_run:
                    return False
                return self._delivery_failed("stow_failed")
        if tray:
            self._set_state(SessionState.WAITING_CUSTOMER)
            taken = self._ask_yes_no("Please take the objects on my tray. Did you take your order? "
                                     "Please answer robot yes or robot no.")
            if taken is not True:
                if self.stopped or self._is_timeout():
                    return False
                # No llevarse la bandeja antes de que el cliente confirme la descarga.
                return self._delivery_failed("tray_not_taken")
        if self.stopped or self._is_timeout():
            return False
        self._speak("Enjoy your order. See you soon.")
        if self.stopped or self._is_timeout():
            return False
        self.served += 1
        self.served_labels.add(cust["_face_label"])
        return True

    def _interrupted(self) -> bool:
        if self.stopped:
            self._handle_stop()
            return True
        if self._is_timeout():
            self._handle_timeout()
            return True
        return self._abort_run

    def _return_to_bar(self, final: bool = False) -> bool:
        self._set_state(SessionState.RETURNING_TO_BAR_FINAL if final else SessionState.RETURNING_TO_BAR)
        self._speak("Moving to the barman table.")
        ok = self._navigate_to(self.bar_pose)
        if self._interrupted():
            return False
        if not ok:
            self._set_state(SessionState.FAILED, {"reason": "bar_unreachable"})
            self._speak("I could not reach the bar.")
        return ok

    def run(self) -> bool:
        """Atencion secuencial o por lotes; vuelve a buscar hasta max_customers/deadline."""
        self.start_time = self.clock_fn()
        self.deadline_s = self.start_time + self.max_time_s
        self.stopped = False
        self._abort_run = False
        self.served = 0
        self.served_labels.clear()
        self._visited_customers = []
        self._holding_item = False
        self.bar_pose = None
        self.customers = []
        self.orders = {}
        self.current_customer_idx = 0
        self._set_state(SessionState.STARTING)
        self.bar_pose = self._wait_for_pose()
        if self.bar_pose is None:
            return self._handle_pose_failure()
        self._find_barman()
        if self._interrupted():
            return False
        self._set_state(SessionState.CLEARING_COUNTER)
        self._speak("Please remove all the objects from the counter. When I have the orders, "
                    "I will ask you to place each object.")

        while self.served < self.max_customers:
            if self._interrupted():
                return False
            self._speak(rest.CUSTOMER_SEARCH)
            customers = self._look_around()
            if self._interrupted():
                return False
            if not customers:
                reason = getattr(self, "detection_error", "")
                phrases = {
                    "unlocalized_raised_hand": "I see your raised hand, but I cannot measure your distance. Please move closer.",
                    "unconfirmed_raised_hand": "I see your hand. Please hold it up a little longer.",
                    "timeout": "The customer detector did not respond in time.",
                }
                if reason in phrases:
                    self._speak(phrases[reason])
                self._speak(rest.SEARCH_AGAIN)
                # Evitar bucle ocupado con detectores/fakes que devuelven inmediatamente.
                self.sleep_fn(min(0.2, self.remaining_s()))
                continue
            self.customers = rank_customers(customers, self.bar_pose)
            self._speak(f"I have found {len(self.customers)} customers.")
            pending = []
            for idx, cust in enumerate(self.customers):
                if self.served + len(pending) >= self.max_customers:
                    break
                self.current_customer_idx = idx
                reached = self._approach_customer(idx, cust)
                if self._interrupted():
                    return False
                if reached is None:
                    continue
                items = self._take_customer_order(idx, cust)
                if self._interrupted():
                    return False
                if not items:
                    continue
                if self.batch_orders:
                    pending.append((idx, cust, items, *reached))
                else:
                    self._fulfill_and_deliver_order(idx, cust, items, *reached)
                    if self._interrupted():
                        return False
                    if not self._return_to_bar(final=self.served >= self.max_customers):
                        return False
            for idx, cust, items, pose, candidates, candidate_idx in pending:
                self.current_customer_idx = idx
                self._fulfill_and_deliver_order(idx, cust, items, pose, candidates, candidate_idx)
                if self._interrupted():
                    return False
            # El modo por lotes retorna una vez tras entregar el lote. Sin pedido
            # tambien regresa a la barra antes de volver a buscar clientes nuevos.
            if self.batch_orders or self.served < self.max_customers:
                pose = self._wait_for_pose()
                if pose is None:
                    return self._handle_pose_failure()
                if pose != self.bar_pose and not self._return_to_bar(final=self.served >= self.max_customers):
                    return False
        if self._interrupted():
            return False
        self._speak("Mission complete. I am back at the kitchen bar.")
        if self._interrupted():
            return False
        self._set_state(SessionState.SUCCEEDED, {"served": self.served})
        return True

    def start(self) -> bool:
        """Alias para iniciar la ejecucion."""
        return self.run()

    def _handle_stop(self) -> bool:
        self.stopped = True
        try:
            self.cancel_nav_fn()
        except Exception:
            pass
        self._speak("Stopping mission.", terminal=True)
        self._set_state(SessionState.ABORTED)
        return False

    def _handle_timeout(self) -> bool:
        try:
            self.cancel_nav_fn()
        except Exception:
            pass
        self._speak("Time limit reached. Stopping mission.", terminal=True)
        self._set_state(SessionState.TIMEOUT)
        return False
