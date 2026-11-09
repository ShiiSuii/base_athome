"""Tests unitarios para RestaurantSession (HOME-105).

Cubre:
  1. Dos clientes atendidos en orden estricto (visita c1, bar, entrega c1; luego c2, bar, entrega c2; fin en bar).
  2. Cliente no entendido dos veces: repite una vez, cancela atencion y sigue con el siguiente.
  3. 'stop' por voz cancela navegacion y aborta la sesion inmediatamente.
  4. Nadie levanta la mano: aviso breve y finalizacion limpia sin navegar.
  5. LLM caida: extraccion por palabras clave del menu, o pide repetir si no hay items validos.
  6. Tope global: cancela y termina sin iniciar un retorno a la barra.
  7. Frases breves: todas las locuciones tienen <= 15 palabras y terminan con puntuacion.
  8. Jamas anuncia el pedido antes de escucharlo.
"""

import ast
from concurrent.futures import Future
import math
from pathlib import Path
from types import SimpleNamespace as NS
from typing import Any, Dict, List, Optional, Sequence, Tuple
import pytest

from base_athome_skills.mission import restaurant as rest
from base_athome_skills.mission.restaurant_navigation import (
    NavigationStopMonitor,
    RestaurantNavigation,
    is_stop_command,
)
from base_athome_skills.mission.restaurant_session import (
    RestaurantSession,
    SessionState,
    NAV2_INSCRIBED_VALUE,
    approach_candidates,
    compute_approach_pose,
    path_length,
    validate_with_planner,
)


class FakeRestaurantHarness:
    """Arnes de fakes controlable para testear RestaurantSession sin ROS."""

    def __init__(
        self,
        customers: Optional[List[Dict[str, Any]]] = None,
        speech_script: Optional[List[str]] = None,
        llm_script: Optional[Dict[str, List[str]]] = None,
        llm_down: bool = False,
        nav_success: bool = True,
        initial_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        self.customers = customers if customers is not None else [
            {"id": "1", "xy": [2.0, 1.0], "range_m": 2.23},
            {"id": "2", "xy": [3.0, -1.0], "range_m": 3.16},
        ]
        self.speech_script = list(speech_script or [])
        self.llm_script = dict(llm_script or {})
        self.llm_down = llm_down
        self.nav_success = nav_success

        self.robot_pose = initial_pose
        self.current_time = 100.0

        # Registros de eventos
        self.events: List[Tuple[str, Any]] = []
        self.spoken: List[str] = []
        self.nav_goals: List[Tuple[float, float, float]] = []
        self.nav_deadlines: List[float] = []
        self.nav_cancelled: bool = False
        self.states: List[Dict[str, Any]] = []

    def clock(self) -> float:
        return self.current_time

    def sleep(self, seconds: float) -> None:
        self.current_time += seconds

    def get_pose(self) -> Tuple[float, float, float]:
        return self.robot_pose

    def navigate(self, x: float, y: float, yaw: float, *, deadline_s: float) -> bool:
        assert self.current_time < deadline_s
        self.nav_deadlines.append(deadline_s)
        goal = (round(x, 3), round(y, 3), round(yaw, 4))
        self.nav_goals.append(goal)
        self.events.append(("navigate", goal))
        self.robot_pose = (x, y, yaw)
        self.current_time += 5.0
        return self.nav_success

    def cancel_nav(self) -> None:
        self.nav_cancelled = True
        self.events.append(("cancel_nav", None))

    def speak(self, text: str) -> None:
        self.spoken.append(text)
        self.events.append(("speak", text))
        self.current_time += 1.0

    def listen(self, timeout_s: float) -> Optional[str]:
        self.events.append(("listen", timeout_s))
        self.current_time += 2.0
        if self.speech_script:
            return self.speech_script.pop(0)
        return ""

    def detect_customers(self, timeout_s: float, min_frames: int) -> List[Dict[str, Any]]:
        self.events.append(("detect_customers", (timeout_s, min_frames)))
        self.current_time += 1.0
        return list(self.customers)

    def ask_order(self, text: str, menu: Sequence[str]) -> List[str]:
        if self.llm_down:
            return []
        if text in self.llm_script:
            return self.llm_script[text]
        return rest.to_order(text, menu)

    def on_state_change(self, state_dict: Dict[str, Any]) -> None:
        self.states.append(state_dict)

    def make_session(self, max_time_s: float = 900.0, **kwargs) -> RestaurantSession:
        return RestaurantSession(
            navigate_fn=self.navigate,
            speak_fn=self.speak,
            listen_fn=self.listen,
            detect_customers_fn=self.detect_customers,
            ask_order_fn=self.ask_order,
            get_pose_fn=self.get_pose,
            clock_fn=self.clock,
            sleep_fn=self.sleep,
            cancel_nav_fn=self.cancel_nav,
            on_state_change_fn=self.on_state_change,
            max_time_s=max_time_s,
            **kwargs,
        )


# ------------------------------------------------------------------------------
# 1. Calculo geometrico de aproximacion
# ------------------------------------------------------------------------------
def test_compute_approach_pose_geometry():
    # Robot en el origen (0, 0), cliente a 2.0 m sobre el eje X (2.0, 0.0)
    # Standoff 0.9 m -> objetivo en (1.1, 0.0) mirando hacia yaw=0.0
    pose = compute_approach_pose([2.0, 0.0], (0.0, 0.0, 0.0), standoff_dist=0.9)
    assert pytest.approx(pose[0], abs=1e-2) == 1.1
    assert pytest.approx(pose[1], abs=1e-2) == 0.0
    assert pytest.approx(pose[2], abs=1e-2) == 0.0

    # Distancia desde la pose objetivo al cliente debe ser exactamente 0.9 m
    dist = math.hypot(2.0 - pose[0], 0.0 - pose[1])
    assert pytest.approx(dist, abs=1e-2) == 0.9

    # Cliente a 45 grados: (2.0, 2.0)
    pose_diag = compute_approach_pose([2.0, 2.0], (0.0, 0.0, 0.0), standoff_dist=0.9)
    dist_diag = math.hypot(2.0 - pose_diag[0], 2.0 - pose_diag[1])
    assert pytest.approx(dist_diag, abs=1e-2) == 0.9
    expected_yaw = math.atan2(2.0 - pose_diag[1], 2.0 - pose_diag[0])
    assert pytest.approx(pose_diag[2], abs=1e-2) == expected_yaw


# ------------------------------------------------------------------------------
# 2. Dos clientes en orden estricto
# ------------------------------------------------------------------------------
def test_two_customers_served_in_strict_order():
    harness = FakeRestaurantHarness(
        customers=[
            {"id": "1", "xy": [2.0, 0.0]},
            {"id": "2", "xy": [3.0, 0.0]},
        ],
        speech_script=[
            "I want a coke and water",  # c1 pedido
            "ready",                    # barman c1
            "two beers please",         # c2 pedido
            "done",                     # barman c2
        ],
        llm_script={
            "I want a coke and water": ["coke", "water"],
            "two beers please": ["beer"],
        },
        initial_pose=(0.0, 0.0, 0.0),  # kitchen bar
    )
    session = harness.make_session()
    ok = session.run()

    assert ok is True
    assert session.state == SessionState.SUCCEEDED
    assert len(session.customers) == 2

    # Verificacion de locuciones clave
    assert "I see two customers." in harness.spoken
    assert any("coke and water" in p for p in harness.spoken)
    assert any("beer" in p for p in harness.spoken)
    assert "Mission complete. I am back at the kitchen bar." == harness.spoken[-1]

    # Orden estricto de eventos por cliente:
    # Para cliente 1: navega a c1 (~1.1, 0), habla saludo, escucha, confirma, vuelve al bar (0, 0), habla barman, vuelve a c1, entrega.
    # Recien despues inicia con cliente 2.
    c1_deliver_idx = next(i for i, (ev, val) in enumerate(harness.events) if ev == "speak" and "tray" in val)
    c2_greet_idx = next(i for i, (ev, val) in enumerate(harness.events[c1_deliver_idx:]) if ev == "speak" and "order today" in val) + c1_deliver_idx

    assert c1_deliver_idx < c2_greet_idx, "Cliente 2 no debe ser atendido antes de entregar la orden a Cliente 1"

    # Verificacion de regreso al bar
    final_nav = harness.nav_goals[-1]
    assert pytest.approx(final_nav[0], abs=1e-2) == 0.0
    assert pytest.approx(final_nav[1], abs=1e-2) == 0.0


# ------------------------------------------------------------------------------
# 3. Cliente ininteligible dos veces: cancela y atiende al siguiente
# ------------------------------------------------------------------------------
def test_customer_unintelligible_twice_cancels_and_proceeds_to_next():
    harness = FakeRestaurantHarness(
        customers=[
            {"id": "1", "xy": [2.0, 0.0]},
            {"id": "2", "xy": [3.0, 0.0]},
        ],
        speech_script=[
            "mumble mumble 123",       # c1 intento 1 (falla)
            "still noise error",       # c1 intento 2 (falla)
            "please bring me coffee",  # c2 intento 1 (exito)
            "ready",                   # barman c2
        ],
        llm_script={
            "please bring me coffee": ["coffee"],
        },
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session()
    ok = session.run()

    assert ok is True
    assert session.state == SessionState.SUCCEEDED

    # Cliente 1 pidio repetir y luego fue cancelado
    assert rest.repeat_order_phrase() in harness.spoken
    assert rest.cancel_order_phrase() in harness.spoken

    # Para cliente 1 NO se debio ir al bar a pedir items
    spoken_texts = " ".join(harness.spoken)
    assert "Please place the order on my tray." not in spoken_texts
    assert "Please place coffee on my tray." in harness.spoken

    # Cliente 2 si fue atendido y completado
    assert any("coffee" in p for p in harness.spoken)
    assert session.orders.get("1") == []
    assert session.orders.get("2") == ["coffee"]


# ------------------------------------------------------------------------------
# 4. 'stop' por voz cancela Nav2 y aborta la sesion inmediatamente
# ------------------------------------------------------------------------------
def test_voice_stop_cancels_nav_and_aborts():
    harness = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2.0, 0.0]}],
        speech_script=["stop please"],
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session()
    ok = session.run()

    assert ok is False
    assert session.state == SessionState.ABORTED
    assert harness.nav_cancelled is True
    assert "Stopping mission." in harness.spoken


# ------------------------------------------------------------------------------
# 5. Nadie levanta la mano: aviso breve y terminacion limpia sin navegar
# ------------------------------------------------------------------------------
def test_no_customers_detected():
    harness = FakeRestaurantHarness(
        customers=[],  # Vacio
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session()
    ok = session.run()

    assert ok is False                       # sin clientes no se hizo la prueba: no es un exito
    assert session.state == SessionState.FAILED
    assert "I do not see any customers calling." in harness.spoken
    # No debe haber navegado hacia ningun comensal
    assert len(harness.nav_goals) == 0


# ------------------------------------------------------------------------------
# 6. LLM caida: fallback a palabras clave del menu
# ------------------------------------------------------------------------------
def test_llm_down_fallback_to_menu_keywords():
    harness = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2.0, 0.0]}],
        speech_script=[
            "hello could you bring me a coffee and juice thanks",
            "ready",
        ],
        llm_down=True,  # LLM apagada / falla
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session()
    ok = session.run()

    assert ok is True
    assert session.state == SessionState.SUCCEEDED
    # El extractor debio rescatar 'coffee' y 'juice' por regex directo
    assert set(session.orders["1"]) == {"coffee", "juice"}
    assert any("coffee and juice" in p for p in harness.spoken)


# ------------------------------------------------------------------------------
# 7. Tope de tiempo global de 15 minutos (900 s)
# ------------------------------------------------------------------------------
def test_global_timeout_terminates_mission():
    harness = FakeRestaurantHarness(
        customers=[
            {"id": "1", "xy": [2.0, 0.0]},
            {"id": "2", "xy": [3.0, 0.0]},
        ],
        speech_script=[
            "coke",
            "ready",
        ],
        llm_script={"coke": ["coke"]},
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session(max_time_s=10.0)

    # Forzar que el reloj avance mas alla del limite antes de completar
    orig_listen = harness.listen
    def slow_listen(timeout_s):
        res = orig_listen(timeout_s)
        harness.current_time += 15.0  # Sobrepasa max_time_s (10 s)
        return res
    session.listen_fn = slow_listen

    ok = session.run()

    assert ok is False
    assert session.state == SessionState.TIMEOUT
    assert "Time limit reached. Stopping mission." in harness.spoken
    assert harness.nav_cancelled is True
    assert len(harness.nav_goals) == 1


# ------------------------------------------------------------------------------
# 8. Todas las frases habladas tienen <= 15 palabras y terminan con puntuacion
# ------------------------------------------------------------------------------
def test_all_spoken_phrases_are_at_most_15_words():
    harness = FakeRestaurantHarness(
        customers=[
            {"id": "1", "xy": [2.0, 0.0]},
            {"id": "2", "xy": [3.0, 0.0]},
        ],
        speech_script=[
            "water and beer",
            "ready",
            "mumble",
            "milk",
            "done",
        ],
        llm_script={
            "water and beer": ["water", "beer"],
            "milk": ["milk"],
        },
    )
    session = harness.make_session()
    session.run()

    assert len(harness.spoken) > 5
    for phrase in harness.spoken:
        words = phrase.strip().split()
        assert len(words) <= 15, f"Frase excede 15 palabras ({len(words)}): '{phrase}'"
        assert len(words) >= 2, f"Frase demasiado corta: '{phrase}'"
        assert phrase.endswith((".", "!", "?")), f"Frase no termina con puntuacion: '{phrase}'"


# ------------------------------------------------------------------------------
# 9. Jamas anuncia el pedido antes de escucharlo
# ------------------------------------------------------------------------------
def test_never_speaks_order_before_listening():
    harness = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2.0, 0.0]}],
        speech_script=[
            "juice and tea",
            "ready",
        ],
        llm_script={"juice and tea": ["juice", "tea"]},
    )
    session = harness.make_session()
    session.run()

    # Revisar la cronologia de eventos
    first_listen_idx = next(i for i, (ev, _) in enumerate(harness.events) if ev == "listen")
    spoken_before_listen = [val for ev, val in harness.events[:first_listen_idx] if ev == "speak"]

    for phrase in spoken_before_listen:
        assert "juice" not in phrase.lower(), f"Anuncio 'juice' antes de escuchar: '{phrase}'"
        assert "tea" not in phrase.lower(), f"Anuncio 'tea' antes de escuchar: '{phrase}'"
        assert "order" not in phrase.lower() or "what would you like" in phrase.lower(), (
            f"Frase sospechosa previa al listen: '{phrase}'"
        )


def make_grid(value=0):
    from types import SimpleNamespace as NS
    return NS(info=NS(width=80, height=80, resolution=0.1,
                     origin=NS(position=NS(x=-2., y=-4.),
                               orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[value] * 6400)


def test_approach_avoids_table_and_faces_customer():
    from base_athome_skills.mission.restaurant_session import approach_candidates
    grid = make_grid()
    # Table across the straight approach at x=1.1, including footprint margins.
    for iy in range(35, 46):
        for ix in range(26, 35):
            grid.data[iy*80+ix] = 100
    poses = approach_candidates((2., 0.), (0., 0., 0.), grid)
    assert poses[0] != compute_approach_pose((2., 0.), (0., 0., 0.))
    x, y, yaw = poses[0]
    assert abs(y) > 0.7
    assert yaw == pytest.approx(math.atan2(-y, 2-x))
    assert [math.hypot(p[0], p[1]) for p in poses] == sorted(math.hypot(p[0], p[1]) for p in poses)


@pytest.mark.parametrize('customer,table', [
    ((1.5, 2.3), (1.1, 1.9, 0.9, 2.1)),
    ((2.5, -2.4), (2.1, 2.9, -2.2, -0.8)),
])
def test_seated_customer_approach_clears_restaurant_table(customer, table):
    grid = make_grid()
    xmin, xmax, ymin, ymax = table
    for iy in range(grid.info.height):
        y = grid.info.origin.position.y + (iy + 0.5) * grid.info.resolution
        for ix in range(grid.info.width):
            x = grid.info.origin.position.x + (ix + 0.5) * grid.info.resolution
            if xmin <= x <= xmax and ymin <= y <= ymax:
                grid.data[iy * grid.info.width + ix] = 100
    poses = approach_candidates(customer, (0., 0., 0.), grid)
    for x, y, yaw in poses:
        # Distance from the robot center to the rectangular tabletop footprint.
        assert math.hypot(max(xmin - x, 0., x - xmax),
                          max(ymin - y, 0., y - ymax)) >= 0.3
        assert 0.8 - 1e-6 <= math.hypot(x - customer[0], y - customer[1]) <= 1.1 + 1e-6
        assert yaw == pytest.approx(math.atan2(customer[1] - y, customer[0] - x))


@pytest.mark.parametrize('value,allowed', [(98, True), (99, True), (100, False), (-1, False)])
def test_entire_robot_radius_must_be_known_and_not_lethal(value, allowed):
    grid = make_grid()
    # Center of nearest candidate (0.9, 0) stays free; an adjacent cell intersects its radius.
    grid.data[42 * grid.info.width + 29] = value
    nearest = approach_candidates((2., 0.), (0., 0., 0.), grid)[0]
    assert (math.hypot(nearest[0] - 0.9, nearest[1]) < 1e-6) == allowed


@pytest.mark.parametrize('value,allowed', [(98, True), (99, False), (100, False), (-1, False)])
def test_center_cell_must_be_below_nav2_inscribed_cost(value, allowed):
    grid = make_grid()
    for iy, ix in ((39, 28), (39, 29), (40, 28), (40, 29)):   # celdas en torno al centro de (0.9, 0)
        grid.data[iy * grid.info.width + ix] = value
    poses = approach_candidates((2., 0.), (0., 0., 0.), grid)
    assert any(math.hypot(x - 0.9, y) < 1e-6 for x, y, _ in poses) == allowed
    assert math.hypot(poses[0][0] - 0.9, poses[0][1]) > 1e-6    # costo alto: nunca la preferida


def test_rotated_costmap_origin_preserves_candidate_geometry():
    grid = make_grid()
    expected = approach_candidates((2., 0.), (0., 0., 0.), grid)
    # Rotate the map and the scene 90 degrees, then translate by (5, 6).
    grid.info.origin.position.x, grid.info.origin.position.y = 9., 4.
    grid.info.origin.orientation.z = math.sin(math.pi / 4)
    grid.info.origin.orientation.w = math.cos(math.pi / 4)
    poses = approach_candidates((5., 8.), (5., 6., math.pi / 2), grid)
    assert len(poses) == len(expected)
    assert poses[0][:2] == pytest.approx((5. - expected[0][1], 6. + expected[0][0]))


@pytest.mark.parametrize('value', [-1, 99, 100])
def test_no_free_candidate_falls_back(value):
    from base_athome_skills.mission.restaurant_session import approach_candidates
    assert approach_candidates((2., 0.), (0., 0., 0.), make_grid(value)) == [compute_approach_pose((2., 0.), (0., 0., 0.))]


def test_aborted_navigation_retries_next_candidate_once():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: True)
    original = h.navigate
    def navigate(*pose, **kwargs):
        original(*pose, **kwargs)
        return len(h.nav_goals) != 1
    session.navigate_fn = navigate
    assert session.run()
    assert h.nav_goals[0] != h.nav_goals[1]
    assert h.nav_goals[3] == h.nav_goals[1]  # Delivery uses successful approach.


def test_non_aborted_navigation_does_not_retry():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], nav_success=False)
    session = h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: False)
    assert not session.run()
    assert len(h.nav_goals) == 2  # Approach and final bar only.


def test_unknown_navigation_failure_does_not_assume_abort():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], nav_success=False)
    assert not h.make_session(get_costmap_fn=make_grid).run()
    assert len(h.nav_goals) == 2


def test_aborted_navigation_is_retried_at_most_once():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], nav_success=False)
    assert not h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: True).run()
    assert len(h.nav_goals) == 3  # Two candidates, then final bar.


def test_aborted_fallback_has_no_free_candidate_to_retry():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], nav_success=False)
    assert not h.make_session(get_costmap_fn=lambda: make_grid(100), nav_aborted_fn=lambda: True).run()
    assert len(h.nav_goals) == 2


def test_delivery_aborted_retries_other_free_candidate():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: True)
    original = h.navigate
    def navigate(*pose, **kwargs):
        original(*pose, **kwargs)
        return len(h.nav_goals) != 3
    session.navigate_fn = navigate
    assert session.run()
    assert h.nav_goals[2] != h.nav_goals[3]
    assert len(h.nav_goals) == 5


def test_delivery_abort_does_not_retry_an_unchecked_fallback():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(nav_aborted_fn=lambda: True)
    original = h.navigate

    def navigate(*pose, **kwargs):
        original(*pose, **kwargs)
        if len(h.nav_goals) == 3:
            h.robot_pose = (1., 1., 0.)  # Nav2 aborted partway to the customer.
            return False
        return True

    session.navigate_fn = navigate
    assert not session.run()
    assert len(h.nav_goals) == 4  # No free alternatives without a costmap.


class FakeActionHarness:
    """Futuros reales, reloj virtual y respuestas programadas; no ROS ni sockets."""

    def __init__(self, accept_after=0., finish_after=1., status=4, cancel_delay=0.4,
                 cancel_status=5, accepted=True):
        self.now = 100.
        self.events = []
        self.waits = 0
        self.sent = []
        self.cancels = []
        self.warnings = []
        self.accept_after = accept_after
        self.finish_after = finish_after
        self.status = status
        self.cancel_delay = cancel_delay
        self.cancel_status = cancel_status
        self.on_send = lambda: None
        self.on_cancel = lambda: None
        self.acceptance = Future()
        self.result = Future()
        self.goal = NS(accepted=accepted, get_result_async=lambda: self.result,
                       cancel_goal_async=self.cancel)
        self.client = NS(wait_for_server=lambda **_: True, send_goal_async=self.send)
        self.nav = RestaurantNavigation(self.client, warn_fn=self.warnings.append,
                                        clock_fn=lambda: self.now, wait_fn=self.wait)

    def at(self, when, callback):
        self.events.append((round(when, 8), callback))
        self.events.sort(key=lambda ev: ev[0])

    def wait(self, event, seconds):
        self.waits += 1
        assert self.waits < 10000, 'No se recibio el resultado terminal esperado por el fake'
        end = round(self.now + seconds, 8)
        while self.events and self.events[0][0] <= end:
            self.now, callback = self.events.pop(0)
            callback()
        self.now = end

    def send(self, goal):
        assert self.nav.pending  # Debe registrarse ANTES del envio.
        self.sent.append(goal)
        self.on_send()
        if self.accept_after == 0:
            self.acceptance.set_result(self.goal)
        else:
            self.at(self.now + self.accept_after, lambda: self.acceptance.set_result(self.goal))
        if self.finish_after is not None:
            self.at(self.now + self.accept_after + self.finish_after, lambda: self.finish(self.status))
        return self.acceptance

    def finish(self, status):
        if not self.result.done():
            self.result.set_result(NS(status=status))

    def cancel(self):
        assert self.nav.active_goal_handle is self.goal
        assert self.nav.pending
        self.cancels.append(self.now)
        self.on_cancel()
        if self.cancel_delay is not None:
            self.at(self.now + self.cancel_delay, lambda: self.finish(self.cancel_status))
        ack = Future()
        ack.set_result(NS(goals_canceling=[self.goal]))
        return ack

    def run(self, timeout_s=90., deadline_s=1000., stop_requested_fn=lambda: False,
            monitor_fn=lambda: None):
        return self.nav.navigate(object(), timeout_s=timeout_s, deadline_s=deadline_s,
                                 stop_requested_fn=stop_requested_fn, monitor_fn=monitor_fn)


@pytest.mark.parametrize('status,expected', [(4, True), (5, False), (6, False)])
def test_navigation_terminal_results(status, expected):
    h = FakeActionHarness(status=status)
    assert h.run() is expected
    assert h.nav.last_status == status
    assert not h.cancels
    assert not h.nav.pending
    assert h.nav.active_goal_handle is None


@pytest.mark.parametrize('timeout_s', [37., 90., 180.])
@pytest.mark.parametrize('terminal_status', [4, 5, 6])
def test_navigation_timeout_cancels_and_waits_for_terminal_result(timeout_s, terminal_status):
    h = FakeActionHarness(finish_after=None, cancel_status=terminal_status)
    observed = []

    def after_cancel_ack():
        # El ACK de cancelacion llego, pero aun NO hay resultado terminal.
        assert not h.result.done()
        assert h.nav.active_goal_handle is h.goal
        assert not h.nav.reset()
        assert not h.run()  # Tampoco se permite un goal nuevo.
        assert len(h.sent) == 1
        observed.append(True)

    h.at(100. + timeout_s + 0.2, after_cancel_ack)
    assert not h.run(timeout_s=timeout_s)
    assert observed
    assert h.cancels[0] == pytest.approx(100. + timeout_s)
    assert h.now >= 100. + timeout_s + h.cancel_delay
    assert h.result.done()
    assert h.nav.last_status is None  # Ni un ABORTED tardio habilita reintento.
    assert not h.nav.pending
    assert h.nav.active_goal_handle is None


def test_late_acceptance_is_tracked_cancelled_and_drained():
    h = FakeActionHarness(accept_after=10.3, finish_after=None)
    observations = []

    def still_pending():
        assert h.nav.pending
        assert h.nav.active_goal_handle is None
        assert not h.nav.reset()
        assert not h.run()
        observations.append(True)

    h.at(110.1, still_pending)
    assert not h.run()
    assert observations
    assert len(h.sent) == 1
    assert h.cancels[0] == pytest.approx(110.3)
    assert h.result.done()
    assert h.now >= 110.7
    assert not h.nav.pending


@pytest.mark.parametrize('stop_during_send', [False, True])
def test_stop_before_handle_assignment_cancels_acceptance(stop_during_send):
    h = FakeActionHarness(accept_after=0. if stop_during_send else 0.5, finish_after=None)
    if stop_during_send:
        h.on_send = h.nav.cancel
    else:
        h.at(100.2, h.nav.cancel)
    assert not h.run()
    assert len(h.sent) == 1
    assert h.cancels[0] == pytest.approx(100. if stop_during_send else 100.5)
    assert h.result.done()
    assert not h.nav.pending


def test_stop_before_navigation_prevents_sending_until_explicit_restart():
    h = FakeActionHarness()
    h.nav.cancel()
    assert not h.run()
    assert not h.sent
    assert h.nav.reset()
    assert h.run()


def test_stop_while_waiting_for_server_prevents_send():
    h = FakeActionHarness()
    def ready_after_stop(**_):
        h.nav.cancel()
        return True
    h.client.wait_for_server = ready_after_stop
    assert not h.run()
    assert not h.sent
    assert not h.nav.pending


def test_rejected_goal_finishes_without_handle_or_cancel():
    h = FakeActionHarness(accepted=False, finish_after=None)
    assert not h.run()
    assert not h.cancels
    assert not h.nav.pending
    assert h.nav.active_goal_handle is None


@pytest.mark.parametrize('failure', ['pending', 'exception', 'rejected'])
def test_unconfirmed_cancellation_retries_and_retains_handle(failure):
    h = FakeActionHarness(finish_after=None, cancel_delay=None)
    def cancel():
        assert h.nav.active_goal_handle is h.goal
        h.cancels.append(h.now)
        if len(h.cancels) == 2:
            h.at(h.now + 0.2, lambda: h.finish(5))
        elif failure == 'exception':
            raise OSError('cancel request failed')
        ack = Future()
        if failure != 'pending':
            ack.set_result(NS(goals_canceling=[]))
        return ack
    h.goal.cancel_goal_async = cancel
    assert not h.run(timeout_s=0.2)
    assert len(h.cancels) == 2
    assert h.cancels[1] - h.cancels[0] == pytest.approx(1.)
    assert h.result.done()
    assert not h.nav.pending


def test_result_error_is_not_terminal_and_handle_is_retained_until_recovery():
    h = FakeActionHarness(finish_after=None, cancel_delay=None)
    observed = []
    def result_error():
        h.result.set_exception(OSError('result transport failed'))
        h.result = Future()
    def still_owned():
        assert h.nav.active_goal_handle is h.goal
        assert h.nav.pending
        observed.append(True)
    h.at(100.2, result_error)
    h.at(100.4, still_owned)
    h.at(101.5, lambda: h.finish(5))
    assert not h.run()
    assert observed
    assert h.result.done()
    assert h.cancels
    assert not h.nav.pending


@pytest.mark.parametrize('accept_after', [0., 0.7])
def test_global_deadline_cancels_executing_or_pending_goal(accept_after):
    h = FakeActionHarness(accept_after=accept_after, finish_after=None)
    assert not h.run(deadline_s=100.3)
    assert h.cancels[0] == pytest.approx(100. + max(0.3, accept_after))
    assert h.result.done()
    assert h.nav.last_status is None


def test_expired_deadline_does_not_send_goal():
    h = FakeActionHarness()
    assert not h.run(deadline_s=100.)
    assert not h.sent
    assert not h.nav.pending


class FakeVoice:
    def __init__(self, action, utterances=()):
        self.action = action
        self.utterances = list(utterances)
        self.requests = []
        self.cancelled = []

    def request(self, **kwargs):
        rid = f'voice_{len(self.requests)}'
        self.requests.append((rid, kwargs))
        return rid

    def poll(self, rid):
        if self.utterances and self.action.now >= self.utterances[0][0]:
            _, text = self.utterances.pop(0)
            return NS(ok=True, text=text)
        return None

    def cancel(self, rid):
        self.cancelled.append(rid)


def node_methods(*names):
    # Ejecuta los adaptadores reales con fakes, sin importar ni iniciar ROS.
    path = Path(__file__).parents[1] / 'base_athome_skills/restaurant_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RestaurantNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    def message():
        return NS(pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))
    env = dict(math=math, Optional=Optional, Tuple=Tuple, List=List,
               NavigateToPose=NS(Goal=message), GoalStatus=NS(STATUS_ABORTED=6),
               Spin=NS(Goal=NS), Duration=lambda seconds: NS(to_msg=lambda: NS(seconds=seconds)),
               NavigationStopMonitor=NavigationStopMonitor, rclpy=NS(time=NS(Time=lambda: None)))
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    return env


@pytest.mark.parametrize('speech', ['STOP!', 'Stop.', 'Please, STOP!!!', '“sToP”'])
def test_voice_stop_during_node_navigation_cancels_and_aborts_session(speech):
    action = FakeActionHarness(finish_after=None)
    voice = FakeVoice(action, [(100.2, speech)])
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}])
    session = h.make_session()
    session.clock_fn = lambda: action.now
    methods = node_methods('_navigate', '_cancel_nav')
    node = NS(_navigation=action.nav, _spinning=NS(cancel=lambda: None),
              voice=voice, session=session, navigation_timeout_s=90.,
              get_logger=lambda: NS(info=lambda _: None),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
    session.navigate_fn = lambda *pose, **kw: methods['_navigate'](node, *pose, **kw)
    session.cancel_nav_fn = lambda: methods['_cancel_nav'](node)
    assert not session.run()
    assert session.state == SessionState.ABORTED
    assert len(action.sent) == 1  # Ningun regreso ni reintento tras stop.
    assert action.cancels[0] == pytest.approx(100.2)
    assert action.result.done()
    assert not node._nav_aborted
    assert len(voice.requests) == 1


def test_voice_monitor_rearms_and_closes_only_its_own_request():
    h = FakeActionHarness()
    voice = FakeVoice(h, [(100., 'hello')])
    monitor = NavigationStopMonitor(voice, h.nav.cancel)
    assert h.run(monitor_fn=monitor.poll)
    monitor.close()
    assert len(voice.requests) == 2
    assert voice.cancelled == ['voice_1']


@pytest.mark.parametrize('text', ['stopwatch', 'stopping', 'nonstop', 'water please', ''])
def test_stop_normalization_requires_whole_word(text):
    assert not is_stop_command(text)


@pytest.mark.parametrize('speech', ['STOP!', 'Stop.', 'Please, STOP!!!'])
@pytest.mark.parametrize('at_barman', [False, True])
def test_punctuated_stop_during_dialogue(speech, at_barman):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}],
                              speech_script=['water', speech] if at_barman else [speech])
    session = h.make_session()
    assert not session.run()
    assert session.state == SessionState.ABORTED
    assert h.nav_cancelled
    assert len(h.nav_goals) == (2 if at_barman else 1)


@pytest.mark.parametrize('navigation_number', [1, 2, 3, 4])
def test_deadline_during_each_navigation_is_terminal_without_return(navigation_number):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session()
    def navigate(*pose, deadline_s):
        result = h.navigate(*pose, deadline_s=deadline_s)
        if len(h.nav_goals) == navigation_number:
            h.current_time = deadline_s
            return False
        return result
    session.navigate_fn = navigate
    assert not session.run()
    assert session.state == SessionState.TIMEOUT
    assert h.nav_cancelled
    assert len(h.nav_goals) == navigation_number
    assert set(h.nav_deadlines) == {session.deadline_s}
    assert h.spoken[-1] == 'Time limit reached. Stopping mission.'


def test_deadline_in_detection_cannot_report_success_or_start_navigation():
    h = FakeRestaurantHarness(customers=[])
    session = h.make_session(max_time_s=0.5)
    assert not session.run()
    assert session.state == SessionState.TIMEOUT
    assert not h.nav_goals
    assert ('detect_customers', (0.5, 3)) in h.events


@pytest.mark.parametrize('pose', [None, (float('nan'), 0., 0.), (0., float('inf'), 0.)])
def test_missing_initial_tf_aborts_without_inventing_bar(pose):
    h = FakeRestaurantHarness()
    session = h.make_session(pose_timeout_s=0.3)
    session.get_pose_fn = lambda: pose
    assert not session.run()
    assert session.state == SessionState.FAILED
    assert session.bar_pose is None
    assert not h.nav_goals
    assert not any(event == 'detect_customers' for event, _ in h.events)
    assert h.current_time >= 100.3


def test_tf_recovery_within_deadline_records_only_measured_pose():
    h = FakeRestaurantHarness(customers=[])
    session = h.make_session()
    session.get_pose_fn = lambda: (4., 5., 0.7) if h.current_time >= 100.3 else None
    assert session.run() is False      # sin clientes la prueba no se hizo; lo que importa es la pose medida
    assert session.bar_pose == (4., 5., 0.7)
    assert not h.nav_goals


def test_tf_missing_at_approach_aborts_without_origin_based_goal():
    h = FakeRestaurantHarness()
    session = h.make_session(pose_timeout_s=0.3)
    poses = iter([(4., 5., 0.7)])
    session.get_pose_fn = lambda: next(poses, None)
    assert not session.run()
    assert session.state == SessionState.FAILED
    assert session.bar_pose == (4., 5., 0.7)
    assert not h.nav_goals


def test_tf_wait_respects_global_deadline_even_when_clock_starts_at_zero():
    h = FakeRestaurantHarness()
    h.current_time = 0.
    session = h.make_session(max_time_s=0.2)
    session.get_pose_fn = lambda: None
    assert not session.run()
    assert session.state == SessionState.TIMEOUT
    assert not h.nav_goals


def test_node_tf_error_returns_absence_and_measured_origin_is_valid():
    get_pose = node_methods('_get_pose')['_get_pose']
    def missing(*_):
        raise RuntimeError('TF unavailable')
    node = NS(tf_buffer=NS(lookup_transform=missing))
    assert get_pose(node) is None
    transform = NS(transform=NS(translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    node.tf_buffer.lookup_transform = lambda *_: transform
    assert get_pose(node) == (0., 0., 0.)
    transform.transform.rotation.w = 0.
    assert get_pose(node) is None


def test_node_never_opens_voice_listen_with_expired_deadline():
    listen = node_methods('_listen')['_listen']
    calls = []
    node = NS(session=NS(stopped=False, remaining_s=lambda: 0.),
              voice=NS(listen=lambda **kw: calls.append(kw)))
    assert listen(node, 0.) == ''
    assert not calls


def test_restaurant_script_never_enables_torso_motion():
    script = (Path(__file__).parents[1] / 'scripts/restaurant.sh').read_text()
    torso_starts = [line for line in script.splitlines() if 'session_start torso ' in line]
    assert len(torso_starts) == 1
    assert '-p allow_motion:=false' in torso_starts[0]
    assert 'allow_motion:=true' not in script


def test_restaurant_script_rejects_existing_navigation_before_starting_real_stack():
    script = (Path(__file__).parents[1] / 'scripts/restaurant.sh').read_text()
    real = script.split('export ROS_DOMAIN_ID=10', 1)[1]
    assert real.index('session_init ') < real.index('restaurant_graph_idle || exit 1')
    assert real.index('restaurant_graph_idle || exit 1') < real.index('session_start ')
    assert 'if nav_up; then' not in script
    assert 'amcl|map_server|slam_toolbox|' in script
    assert 'ros2 param get /slam_toolbox mode' in script
    assert 'ros2 lifecycle get /slam_toolbox' in script
    assert real.index('wait_restaurant_nav || exit 1') < real.index('ros2 run base_athome_skills restaurant_node')
    sim = script.split('if [ "${SIM:-0}" = 1 ]; then', 1)[1].split('export ROS_DOMAIN_ID=10', 1)[0]
    for mode in (sim, real):
        assert mode.index('session_init ') < mode.index('restaurant_graph_idle || exit 1')
        assert 'session_run_terminal ros2 run base_athome_skills restaurant_node' in mode


# ------------------------------------------------------------------------------
# Aproximacion contra un costmap como el de Nav2 en restaurant.sdf
# ------------------------------------------------------------------------------
# Rectangulos (xmin, xmax, ymin, ymax) que el LiDAR ve. NARROW: restaurant.sdf hasta 2026-11-09
# (8 x 6 m, pasillos de 0.25-0.65 m junto a los clientes); es el caso dificil que se conserva.
NARROW_RESTAURANT = [
    (-3.0, 5.0, 2.95, 3.05), (-3.0, 5.0, -3.05, -2.95), (4.95, 5.05, -3., 3.), (-3.05, -2.95, -3., 3.),
    (-2.3, -1.7, -1.5, 1.5),                                          # barra
    (1.1, 1.9, 0.9, 2.1), (3.1, 3.9, 0.9, 2.1), (2.1, 2.9, -2.2, -0.8),   # mesas 1, 2, 3
    (1.275, 1.725, 2.075, 2.525), (1.275, 1.725, 0.475, 0.925),       # sillas mesa 1
    (3.275, 3.725, 2.075, 2.525), (3.275, 3.725, 0.475, 0.925),       # sillas mesa 2
    (2.275, 2.725, -0.825, -0.375), (2.275, 2.725, -2.625, -2.175),   # sillas mesa 3
]
# LARGE: restaurant.sdf actual (12 x 9 m, pasillos >= 1.3 m).
LARGE_RESTAURANT = [
    (-3.0, 9.0, 4.45, 4.55), (-3.0, 9.0, -4.55, -4.45), (8.95, 9.05, -4.5, 4.5), (-3.05, -2.95, -4.5, 4.5),
    (-2.3, -1.7, -1.5, 1.5),                                                # barra
    (1.6, 2.4, 1.4, 2.6), (4.1, 4.9, 1.4, 2.6), (2.6, 3.4, -2.7, -1.3),     # mesas 1, 2, 3
    (1.775, 2.225, 2.575, 3.025), (1.775, 2.225, 0.975, 1.425),             # sillas mesa 1
    (4.275, 4.725, 2.575, 3.025), (4.275, 4.725, 0.975, 1.425),             # sillas mesa 2
    (2.775, 3.225, -1.325, -0.875), (2.775, 3.225, -3.125, -2.675),         # sillas mesa 3
]


def nav2_value(dist, inscribed=0.44, inflation=0.59, scaling=4.0):
    """Valor OccupancyGrid que publica Nav2 (inflado de nav2_params.yaml) a dist del obstaculo."""
    if dist <= 0.0:
        cost = 254
    elif dist <= inscribed:
        cost = 253
    elif dist <= inflation:
        cost = int(252 * math.exp(-scaling * (dist - inscribed)))
    else:
        cost = 0
    return {254: 100, 253: 99, 0: 0}.get(cost, 1 + (97 * (cost - 1)) // 251)


def rect_dist(x, y, r):
    return math.hypot(max(r[0] - x, 0., x - r[1]), max(r[2] - y, 0., y - r[3]))


def restaurant_costmap(obstacles, res=0.05):
    x0, y0 = min(r[0] for r in obstacles) - 0.05, min(r[2] for r in obstacles) - 0.05
    w = int((max(r[1] for r in obstacles) + 0.05 - x0) / res)
    h = int((max(r[3] for r in obstacles) + 0.05 - y0) / res)
    grid = NS(info=NS(width=w, height=h, resolution=res,
                      origin=NS(position=NS(x=x0, y=y0), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[0] * (w * h))
    for iy in range(h):
        y = y0 + (iy + 0.5) * res
        for ix in range(w):
            x = x0 + (ix + 0.5) * res
            grid.data[iy * w + ix] = nav2_value(min(rect_dist(x, y, r) for r in obstacles))
    return grid


def grid_value(grid, x, y):
    i = grid.info
    return grid.data[math.floor((y - i.origin.position.y) / i.resolution) * i.width
                     + math.floor((x - i.origin.position.x) / i.resolution)]


@pytest.fixture(scope="module")
def restaurant_grid():
    return restaurant_costmap(NARROW_RESTAURANT)


@pytest.fixture(scope="module")
def large_restaurant_grid():
    return restaurant_costmap(LARGE_RESTAURANT)


@pytest.mark.parametrize('customer,table,straight_blocked', [
    ((1.5, 2.3), (1.1, 1.9, 0.9, 2.1), True),      # mesa 1: detectado en Gazebo en (1.5, 2.3)
    ((2.5, -2.4), (2.1, 2.9, -2.2, -0.8), True),
    # Mesa 3 detectada en Gazebo en (2.25, -2.15): la recta pasaba por casualidad (por eso
    # el robot si llegaba a la mesa 3 y no a la 1).
    ((2.25, -2.15), (2.1, 2.9, -2.2, -0.8), False),
])
def test_restaurant_costmap_customers_get_nav2_valid_candidates(restaurant_grid, customer, table,
                                                                 straight_blocked):
    """Regresion de Restaurant en Gazebo: la pose rectilinea cae dentro del inflado inscrito
    (Nav2: 'no valid path found') y el filtro anterior (costo < 50 en un disco de 0.3 m) no dejaba
    ninguna pose junto a un cliente sentado contra la pared, asi que se usaba la rectilinea."""
    straight = compute_approach_pose(customer, (0., 0., 0.))
    assert (grid_value(restaurant_grid, *straight[:2]) >= NAV2_INSCRIBED_VALUE) == straight_blocked
    poses = approach_candidates(customer, (0., 0., 0.), restaurant_grid)
    assert straight not in poses and len(poses) >= 3
    for x, y, yaw in poses:
        assert grid_value(restaurant_grid, x, y) < NAV2_INSCRIBED_VALUE
        assert min(rect_dist(x, y, r) for r in NARROW_RESTAURANT) > 0.44 - 0.05
        assert rect_dist(x, y, table) > 0.44 - 0.05
        assert 0.8 - 1e-6 <= math.hypot(x - customer[0], y - customer[1]) <= 1.1 + 1e-6
        assert yaw == pytest.approx(math.atan2(customer[1] - y, customer[0] - x))


@pytest.mark.parametrize('customer,table', [
    ((2.0, 2.8), (1.6, 2.4, 1.4, 2.6)),
    ((3.0, -2.9), (2.6, 3.4, -2.7, -1.3)),
])
def test_large_restaurant_customers_get_clear_candidates(large_restaurant_grid, customer, table):
    """Con pasillos >= 1.3 m la mejor pose queda fuera de todo inflado (costo 0) y a 0.8-1.1 m."""
    poses = approach_candidates(customer, (0., 0., 0.), large_restaurant_grid)
    assert len(poses) >= 6
    x, y, _ = poses[0]
    assert grid_value(large_restaurant_grid, x, y) == 0
    assert min(rect_dist(x, y, r) for r in LARGE_RESTAURANT) > 0.59 - 0.05
    for x, y, _ in poses:
        assert grid_value(large_restaurant_grid, x, y) < NAV2_INSCRIBED_VALUE
        assert rect_dist(x, y, table) > 0.44 - 0.05


def test_candidates_prefer_lower_cost_at_similar_distance():
    grid = make_grid()
    # Costo alto (pero valido para Nav2) alrededor de la candidata recta (0.9, 0) del cliente (2, 0).
    for iy in range(37, 44):
        for ix in range(26, 33):
            grid.data[iy * grid.info.width + ix] = 98
    first = approach_candidates((2., 0.), (0., 0., 0.), grid)[0]
    assert grid_value(grid, *first[:2]) == 0


def test_path_length():
    assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)
    assert path_length([(1, 1)]) == 0.0


def test_planner_validation_keeps_reachable_sorted_by_path_length():
    plans = {(1, 0, 0): None, (2, 0, 0): 5.0, (3, 0, 0): 2.0, (4, 0, 0): 3.0, (5, 0, 0): 1.0}
    calls = []

    def plan(pose):
        calls.append(pose)
        return plans[pose]
    poses, validated = validate_with_planner(list(plans), plan, max_valid=3)
    assert validated and poses == [(3, 0, 0), (4, 0, 0), (2, 0, 0)]
    assert (5, 0, 0) not in calls      # se detiene al juntar max_valid


def test_planner_validation_none_reachable_returns_empty():
    poses, validated = validate_with_planner([(1, 0, 0), (2, 0, 0)], lambda p: None)
    assert validated and poses == []


def test_planner_unavailable_keeps_unvalidated_candidates():
    def plan(pose):
        raise RuntimeError("compute_path_to_pose no disponible")
    poses, validated = validate_with_planner([(1, 0, 0), (2, 0, 0)], plan)
    assert not validated and poses == [(1, 0, 0), (2, 0, 0)]
    assert validate_with_planner([(1, 0, 0)], None) == ([(1, 0, 0)], False)


def test_planner_failure_after_valid_keeps_validated_ones():
    answers = iter([None, 4.0])

    def plan(pose):
        try:
            return next(answers)
        except StopIteration:
            raise RuntimeError("timeout")
    poses, validated = validate_with_planner([(1, 0, 0), (2, 0, 0), (3, 0, 0)], plan)
    assert validated and poses == [(2, 0, 0)]


def test_planner_validation_respects_time_budget_and_check_limit():
    t = [0.0]

    def plan(pose):
        t[0] += 1.0
        return None
    validate_with_planner([(i, 0, 0) for i in range(30)], plan, clock_fn=lambda: t[0], budget_s=4.0)
    assert t[0] == 4.0
    t[0] = 0.0
    validate_with_planner([(i, 0, 0) for i in range(30)], plan, clock_fn=lambda: t[0], budget_s=100.)
    assert t[0] == 16.0


def test_session_navigates_only_to_planner_validated_candidate(restaurant_grid):
    h = FakeRestaurantHarness(customers=[{'xy': [1.5, 2.3]}], speech_script=['water', 'ready'])
    planned = []

    def plan(pose):
        planned.append(pose)
        return None if len(planned) == 1 else 3.0   # la primera candidata no tiene camino
    session = h.make_session(get_costmap_fn=lambda: restaurant_grid, plan_fn=plan)
    assert session.run()
    assert h.nav_goals[0] == tuple(round(v, n) for v, n in zip(planned[1], (3, 3, 4)))
    assert h.nav_goals[2] == h.nav_goals[0]   # la entrega vuelve a la pose validada
    approach = [s for s in h.states if s["state"] == "APPROACHING_CUSTOMER"][0]["info"]
    assert approach["customer_xy"] == [1.5, 2.3] and approach["reachable"] >= 1


def test_session_skips_customer_without_any_planned_approach(restaurant_grid):
    h = FakeRestaurantHarness(customers=[{'xy': [1.5, 2.3]}, {'xy': [2.5, -2.4]}],
                              speech_script=['water', 'ready'])

    def plan(pose):
        return None if pose[1] > 0 else 4.0      # el cliente 1 no tiene camino, el 2 si
    session = h.make_session(get_costmap_fn=lambda: restaurant_grid, plan_fn=plan)
    assert session.run()
    assert "I could not reach the customer." in h.spoken
    assert all(goal[1] <= 0 for goal in h.nav_goals)   # nunca un goal sin plan
    assert len(h.nav_goals) == 4                       # cliente 2, barra, entrega, barra final


def test_initial_spin_order_fixed_head_and_three_views_merge_in_map():
    h = FakeRestaurantHarness(speech_script=['water', 'ready', 'beer', 'ready'])
    views = iter([
        [{'id': '1', 'xy': [4., 2.]}],
        {'ok': True, 'customers': [{'id': '1', 'xy': [2., -1.]}]},
        [{'id': '1', 'xy': [4.1, 2.1]}, {'id': '2', 'xy': [2.1, -1.1]}],
    ])
    events = []

    def spin(angle):
        events.append(('spin', angle))
        return True

    def detect(timeout_s, min_frames):
        events.append(('look', (timeout_s, min_frames)))
        return next(views)

    session = h.make_session(spin_fn=spin, point_head_fn=lambda p, t: events.append(('head', (p, t))))
    session.detect_customers_fn = detect
    assert session.run()
    assert [v for e, v in events if e == 'spin'] == [math.pi / 2, -math.pi, math.pi / 2]
    assert [e for e, _ in events[:9]] == ['head', 'spin', 'look'] * 3
    assert all(v == (0., 0.) for e, v in events[:9] if e == 'head')
    assert len(session.customers) == 2
    assert [c['xy'] for c in session.customers] == [[2., -1.], [4., 2.]]
    assert len(set(c['id'] for c in session.customers)) == 2
    assert len(session.orders) == 2 and session.served == 2
    assert all(v == (8., 3) for e, v in events if e == 'look')


@pytest.mark.parametrize('failed_spin', [1, 2, 3])
def test_failed_spin_keeps_customers_and_continues_without_blind_relative_turns(failed_spin):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    turns = []

    def spin(angle):
        turns.append(angle)
        return len(turns) != failed_spin

    session = h.make_session(spin_fn=spin)
    assert session.run()
    assert len(turns) == failed_spin
    assert len([e for e, _ in h.events if e == 'detect_customers']) == failed_spin
    assert len(session.customers) == 1 and session.served == 1


def test_failed_second_spin_retains_customer_from_first_view_when_next_is_empty():
    h = FakeRestaurantHarness(speech_script=['water', 'ready'])
    results = iter([True, False])
    views = iter([[{'xy': [2., 1.]}], {'ok': False, 'customers': []}])
    session = h.make_session(spin_fn=lambda _: next(results))
    session.detect_customers_fn = lambda *_: next(views)
    assert session.run()
    assert session.customers[0]['xy'] == [2., 1.]


@pytest.mark.parametrize('interrupt', ['stop', 'timeout'])
def test_interrupt_during_initial_spin_prevents_detection_and_navigation(interrupt):
    h = FakeRestaurantHarness()
    turns = []

    def spin(angle):
        turns.append(angle)
        if interrupt == 'stop':
            session.stop()
        else:
            h.current_time = session.deadline_s
        return False

    session = h.make_session(spin_fn=spin)
    assert not session.run()
    assert session.state == (SessionState.ABORTED if interrupt == 'stop' else SessionState.TIMEOUT)
    assert turns == [math.pi / 2] and h.nav_cancelled
    assert not h.nav_goals
    assert not any(e == 'detect_customers' for e, _ in h.events)


@pytest.mark.parametrize('stop_at', [100.2, 101.0])
def test_node_spin_voice_stop_cancels_even_before_acceptance_and_drains(stop_at):
    action = FakeActionHarness(accept_after=0.5, finish_after=None)
    voice = FakeVoice(action, [(stop_at, 'STOP!')])
    h = FakeRestaurantHarness()
    session = h.make_session()
    session.clock_fn = lambda: action.now
    methods = node_methods('_spin', '_cancel_nav')
    nav_cancel = []
    node = NS(_navigation=NS(cancel=lambda: nav_cancel.append(True)), _spinning=action.nav,
              voice=voice, session=session, spin_timeout_s=40.)
    session.spin_fn = lambda angle: methods['_spin'](node, angle)
    session.cancel_nav_fn = lambda: methods['_cancel_nav'](node)
    assert not session.run()
    assert session.state == SessionState.ABORTED
    assert action.cancels and action.result.done() and not action.nav.pending
    assert len(action.sent) == 1 and nav_cancel
    assert action.sent[0].target_yaw == math.pi / 2
    assert not h.nav_goals
    assert not any(e == 'detect_customers' for e, _ in h.events)


@pytest.mark.parametrize('remaining,spin_timeout', [(900., 2.), (0.5, 40.)])
def test_node_spin_time_allowance_and_client_timeout_are_bounded(remaining, spin_timeout):
    action = FakeActionHarness(finish_after=None)
    h = FakeRestaurantHarness()
    session = h.make_session()
    session.clock_fn = lambda: action.now
    session.deadline_s = action.now + remaining
    method = node_methods('_spin')['_spin']
    node = NS(_spinning=action.nav, voice=FakeVoice(action, []), session=session,
              spin_timeout_s=spin_timeout)
    assert not method(node, -math.pi)
    assert action.sent[0].time_allowance.seconds == min(remaining, spin_timeout)
    assert action.sent[0].target_yaw == -math.pi
    assert action.cancels and action.result.done() and not action.nav.pending
    assert action.cancels[0] == pytest.approx(100. + min(remaining, spin_timeout))


def test_restaurant_rolling_overlay_preserves_slam_obstacles_and_unknown_space():
    import yaml

    root = Path(__file__).parents[1]
    overlay = yaml.safe_load((root / 'config/nav2_restaurant.yaml').read_text())
    base = yaml.safe_load((root.parent / 'base_athome_navigation/config/nav2_params.yaml').read_text())
    tree = ast.parse((root / 'launch/restaurant_nav.launch.py').read_text())
    merge = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_merge')
    scope = {}
    exec(compile(ast.Module(body=[merge], type_ignores=[]), 'restaurant_merge', 'exec'), scope)
    params = scope['_merge'](base, overlay)['global_costmap']['global_costmap']['ros__parameters']
    assert params['global_frame'] == 'map' and params['rolling_window']
    assert params['width'] == params['height'] == 20
    assert params['track_unknown_space']
    assert params['plugins'] == ['static_layer', 'obstacle_layer', 'inflation_layer']
    assert params['obstacle_layer']['scan']['topic'] == '/scan_nav'
