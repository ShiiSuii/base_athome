"""Tests del flujo Restaurant con reloj, voz, navegacion y brazo falsos.

Confirma barman/pedidos/descarga por voz, filtra clientes conocidos, reintenta
hasta los limites y demuestra audio antes de movimiento. Las regresiones de
navegacion mantienen cancelacion/deadlines y geometria sin ROS ni hardware.
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
        self.speech_script = list(speech_script if speech_script is not None else ["yes"])
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
        if self.states and self.states[-1]["state"] == "FINDING_BARMAN":
            rx, ry, yaw = self.robot_pose
            return [{"id": "barman", "xy": [rx + .1 * math.cos(yaw), ry + .1 * math.sin(yaw)]}] if self.customers else []
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
        kwargs.setdefault("max_customers", max(1, len(self.customers)))
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


def _spinning_harness(h, turns):
    """Spin falso que gira el yaw del robot, como Nav2."""
    def spin(angle):
        turns.append(angle)
        x, y, yaw = h.robot_pose
        h.robot_pose = (x, y, math.atan2(math.sin(yaw + angle), math.cos(yaw + angle)))
        return True
    return spin


def test_barman_full_turn_then_faces_nearest_person_and_asks():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns, tracking, enrolled, views = [], [], [], []
    far, near = [0., 3.], [-1., .5]
    def detect(timeout_s, min_frames):
        views.append(len(turns))
        h.events.append(('people_view', len(turns)))
        # La misma persona cercana aparece en dos vistas con 5 cm de diferencia.
        return {2: [{'xy': far}], 5: [{'xy': near}], 6: [{'xy': [-1.05, .5]}, {'xy': far}]}.get(len(turns), [])
    session = h.make_session(
        spin_fn=_spinning_harness(h, turns),
        detect_people_fn=detect,
        detect_barman_fn=lambda *_: pytest.fail('el barman ya no exige mano levantada'),
        track_person_fn=lambda p: tracking.append(p) or h.events.append(('track', p)),
        stop_tracking_fn=lambda: tracking.append(None),
        point_head_fn=lambda p, t: h.events.append(('head', (p, t))),
        enroll_face_fn=lambda label, p: enrolled.append((label, p)) or True,
    )
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert views == list(range(13))  # 12 vistas de la vuelta + 1 para corregir mirando
    assert turns[:11] == pytest.approx([math.pi / 6] * 11)
    want = math.atan2(near[1], near[0])
    assert len(turns) == 12 and h.robot_pose[2] == pytest.approx(want)  # de frente al mas cercano
    assert session.barman_position == (-1., .5)
    assert tracking == [{'xy': near}, None] and enrolled == [('barman', {'xy': near})]
    assert [v for k, v in h.events if k == 'head'][0] == (0., 0.)
    assert h.spoken == [rest.BARMAN_SEARCH, 'Are you the barman? Please answer robot yes or robot no.']
    assert h.events.index(('track', {'xy': near})) < h.events.index(('speak', h.spoken[-1]))


def test_barman_is_nearest_with_raised_hand_and_turn_ignores_map_yaw():
    """Prueba real 10/11: el yaw de SLAM se corrio al girar; el giro final sale de los giros contados."""
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns, tracking = [], []
    near_no_hand = {'xy': [1., 0.], 'range_m': 1.0, 'bearing_deg': 10., 'raised_frames': 0}
    barman = {'xy': [-1.5, .3], 'range_m': 1.5, 'bearing_deg': -5., 'raised_frames': 8}
    centered = dict(barman, bearing_deg=3.)
    def detect(*_):
        if len(turns) == 2:
            return [near_no_hand]
        if len(turns) == 5:
            return [barman]
        return [centered] if len(turns) == 12 else []
    # spin falso que NO toca la pose: el yaw del mapa queda en 0 todo el tiempo.
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True, detect_people_fn=detect,
                             track_person_fn=tracking.append, stop_tracking_fn=lambda: None)
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    # rumbo del barman: 5 giros x 30 - 5 = 145 grados; robot en 330 -> girar +175
    assert len(turns) == 12 and math.degrees(turns[-1]) == pytest.approx(175.)
    assert tracking == [centered] and session.barman_position == (-1.5, .3)
    assert rest.BARMAN_SEARCH in h.spoken and 'raised' in rest.BARMAN_SEARCH


def test_barman_alignment_turns_until_centered_in_camera():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns = []
    barman = {'xy': [0., 2.], 'range_m': 2.0, 'bearing_deg': 0., 'raised_frames': 5}
    aligned = iter([[dict(barman, bearing_deg=20.)], [dict(barman, bearing_deg=-2.)]])
    def detect(*_):
        if len(turns) == 3:
            return [barman]
        return next(aligned) if len(turns) >= 12 else []
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True, detect_people_fn=detect)
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert math.degrees(turns[11]) == pytest.approx(90. - 330. + 360.)
    assert len(turns) == 13 and math.degrees(turns[12]) == pytest.approx(20.)  # una correccion y ya centrado
    assert session.barman_position == (0., 2.)


def test_barman_without_raised_hands_falls_back_to_nearest_person():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns = []
    far = {'xy': [3., 0.], 'range_m': 3.0, 'bearing_deg': 0., 'raised_frames': 0}
    near = {'xy': [0., -1.], 'range_m': 1.0, 'bearing_deg': 0., 'raised_frames': 1}  # 1 cuadro no alcanza
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True,
                             detect_people_fn=lambda *_: [far, near] if not turns else [])
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert session.barman_position == (0., -1.)


def test_barman_nearest_already_in_front_does_not_turn_again():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns = []
    session = h.make_session(spin_fn=_spinning_harness(h, turns),
                             detect_people_fn=lambda *_: [{'xy': [2., -.05]}] if not turns else [])
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    h.robot_pose = (0., 0., math.pi / 6)  # 30 + 11 giros de 30 grados = vuelta completa: mira a +x
    session._find_barman()
    assert len(turns) == 11  # ya mira al barman (menos de 5 grados): no gira otra vez
    assert session.barman_position == (2., -.05)


def test_barman_full_circle_without_people_continues_with_initial_bar():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'water', 'yes', 'yes', 'yes'])
    turns = []
    session = h.make_session(spin_fn=lambda angle: turns.append(angle) or True,
                             detect_barman_fn=lambda *_: [])
    assert session.run()
    assert len(turns) == 11 and sum(turns) == pytest.approx(11 * math.pi / 6)
    assert session.barman_position is None and session.bar_pose == (0., 0., 0.)
    assert 'I could not find the barman. I will use my initial position as the bar.' in h.spoken


@pytest.mark.parametrize('label', [None, 'new_customer'])
def test_barman_position_filters_customers_even_without_matching_face(label):
    h = FakeRestaurantHarness()
    session = h.make_session(identify_face_fn=lambda _: label)
    session.barman_position = (2., 1.)
    assert not session._eligible_customer({'xy': [2.99, 1.]})  # 1 m alrededor del barman
    assert session._eligible_customer({'x': 3.01, 'y': 1.})


def test_customers_search_never_spins_base_and_announces_detection():
    h = FakeRestaurantHarness()
    turns = []
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True)
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    assert session._look_around()
    assert turns == []
    assert h.events[0] == ('speak', 'Searching for people.')
    assert h.events[1][0] == 'detect_customers'


@pytest.mark.parametrize('answer', ['robot no', 'unclear'])
def test_barman_rejected_cancels_tracking_and_retains_bar(answer):
    h = FakeRestaurantHarness(speech_script=[answer])
    turns, tracking, enrolled = [], [], []
    session = h.make_session(
        spin_fn=_spinning_harness(h, turns),
        detect_people_fn=lambda *_: [{'xy': [0., 2.]}] if turns else [],
        track_person_fn=lambda p: tracking.append(p),
        stop_tracking_fn=lambda: tracking.append(None),
        enroll_face_fn=lambda *a: enrolled.append(a) or True,
    )
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., .5)
    session._find_barman()
    assert len(turns) == 12 and tracking == [{'xy': [0., 2.]}, None]
    assert not enrolled and session.barman_position is None
    assert session._bar_target_pose() == (0., 0., .5)
    assert h.spoken[-1] == 'The barman was not confirmed. I will use my initial position as the bar.'


def test_barman_spin_failure_stops_turning_and_uses_people_already_seen():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns = []
    def spin(angle):
        turns.append(angle)
        return len(turns) < 3
    session = h.make_session(spin_fn=spin, detect_people_fn=lambda *_: [{'xy': [1., 0.]}])
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert len(turns) == 3  # 2 giros bien, el tercero falla: no se encadenan mas
    assert session.barman_position == (1., 0.)


def test_camera_looks_forward_and_stops_tracking_before_navigating():
    h = FakeRestaurantHarness()
    session = h.make_session(point_head_fn=lambda p, t: h.events.append(('head', (p, t))),
                             stop_tracking_fn=lambda: h.events.append(('stop_tracking', None)))
    session.deadline_s = h.clock() + 900.
    session._set_state(SessionState.APPROACHING_CUSTOMER)
    assert session._navigate_to((1., 0., 0.))
    nav = h.events.index(('navigate', (1., 0., 0.)))
    assert ('head', (0., 0.)) in h.events[:nav] and ('stop_tracking', None) in h.events[:nav]


@pytest.mark.parametrize('batch', [False, True])
def test_confirmed_barman_position_orients_every_bar_return_and_filters_scan(batch):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol())
    enrolled = []
    session = h.make_session(batch_orders=batch, detect_barman_fn=lambda *_: [{'xy': [0., 2.]}],
                             enroll_face_fn=lambda label, p: enrolled.append(label) or False)
    views = iter([[], [{'xy': [0., 2.1]}, {'id': '1', 'xy': [2., 0.]}]])
    def detect(*args):
        h.events.append(('customer_search', args))
        return next(views)
    session.detect_customers_fn = detect
    assert session.run()
    assert session.barman_position == (0., 2.)  # memoria espacial aunque falle el enrolamiento
    assert [c['id'] for c in session.customers] == ['1']
    assert enrolled == ['barman', 'customer_1']
    returns = [goal for goal in h.nav_goals if goal[:2] == (0., 0.)]
    assert len(returns) == 2
    assert all(goal[2] == pytest.approx(math.pi / 2, abs=1e-4) for goal in returns)
    assert session.bar_pose == (0., 0., 0.)
    assert h.spoken.count(rest.CUSTOMER_SEARCH) == 2
    for idx, (event, _) in enumerate(h.events):
        if event == 'customer_search':
            assert h.events[idx-1] == ('speak', 'Searching for people.')


@pytest.mark.parametrize('interrupt', ['stop', 'timeout'])
def test_barman_tracking_stops_on_interruption_during_answer(interrupt):
    h = FakeRestaurantHarness()
    tracking = []
    session = h.make_session(detect_barman_fn=lambda *_: [{'xy': [0., 2.]}],
                             track_person_fn=lambda p: tracking.append(p),
                             stop_tracking_fn=lambda: tracking.append(None))
    def listen(_):
        if interrupt == 'stop':
            session.stop()
        else:
            h.current_time = session.deadline_s
        return 'robot yes'
    session.listen_fn = listen
    assert not session.run()
    assert tracking[0] == {'xy': [0., 2.]} and tracking[-1] is None
    assert session.barman_position is None and not h.nav_goals


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
            "yes",  # barman
            "yes", "I want a coke and water", "robot yes", "yes", "yes", "yes",
            "yes", "two beers please", "yes", "yes", "robot yes",
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
    assert "I have found 2 customers." in harness.spoken
    assert any("coke and water" in p for p in harness.spoken)
    assert any("beer" in p for p in harness.spoken)
    assert "Mission complete. I am back at the kitchen bar." == harness.spoken[-1]

    # Orden estricto de eventos por cliente:
    # Para cliente 1: navega a c1 (~1.1, 0), habla saludo, escucha, confirma, vuelve al bar (0, 0), habla barman, vuelve a c1, entrega.
    # Recien despues inicia con cliente 2.
    c1_deliver_idx = next(i for i, (ev, val) in enumerate(harness.events) if ev == "speak" and "Enjoy your order" in val)
    c2_greet_idx = next(i for i, (ev, val) in enumerate(harness.events[c1_deliver_idx:]) if ev == "speak" and "Hello, my name" in val) + c1_deliver_idx

    assert c1_deliver_idx < c2_greet_idx, "Cliente 2 no debe ser atendido antes de entregar la orden a Cliente 1"

    # Verificacion de regreso al bar
    final_nav = harness.nav_goals[-1]
    assert pytest.approx(final_nav[0], abs=1e-2) == 0.0
    assert pytest.approx(final_nav[1], abs=1e-2) == 0.0


# ------------------------------------------------------------------------------
# 3. Cliente ininteligible tres veces: cancela y atiende al siguiente
# ------------------------------------------------------------------------------
def test_customer_unintelligible_three_times_cancels_and_proceeds_to_next():
    harness = FakeRestaurantHarness(
        customers=[
            {"id": "1", "xy": [2.0, 0.0]},
            {"id": "2", "xy": [3.0, 0.0]},
        ],
        speech_script=[
            "yes", "yes",  # barman y tiene pedido
            "mumble mumble 123", "still noise error", "third noise",
            "yes", "please bring me coffee", "yes", "yes", "yes",
        ],
        llm_script={
            "please bring me coffee": ["coffee"],
        },
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session(max_customers=1)
    ok = session.run()

    assert ok is True
    assert session.state == SessionState.SUCCEEDED

    # Cliente 1 pidio repetir y luego fue cancelado
    assert rest.repeat_order_phrase() in harness.spoken
    assert rest.cancel_order_phrase() in harness.spoken

    # Para cliente 1 NO se debio ir al bar a pedir items
    spoken_texts = " ".join(harness.spoken)
    assert "Please place the order on my tray." not in spoken_texts
    assert rest.tray_request_phrase("coffee") in harness.spoken

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
        speech_script=["yes", "stop please"],
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
    assert session.state == SessionState.TIMEOUT
    assert rest.SEARCH_AGAIN in harness.spoken
    # No debe haber navegado hacia ningun comensal
    assert len(harness.nav_goals) == 0


# ------------------------------------------------------------------------------
# 6. LLM caida: fallback a palabras clave del menu
# ------------------------------------------------------------------------------
def test_llm_down_fallback_to_menu_keywords():
    harness = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2.0, 0.0]}],
        speech_script=[
            "yes", "yes", "hello could you bring me a coffee and juice thanks",
            "yes", "yes", "yes", "yes",
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
            "yes",
        ],
        llm_script={"coke": ["coke"]},
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = harness.make_session(max_time_s=20.0)

    # Forzar que el reloj avance mas alla del limite antes de completar
    orig_listen = harness.listen
    def slow_listen(timeout_s):
        res = orig_listen(timeout_s)
        harness.current_time += 15.0  # Sobrepasa el tiempo disponible
        return res
    session.listen_fn = slow_listen

    ok = session.run()

    assert ok is False
    assert session.state == SessionState.TIMEOUT
    assert "Time limit reached. Stopping mission." in harness.spoken
    assert harness.nav_cancelled is True
    assert len(harness.nav_goals) == 0  # vence durante confirmacion del barman


# ------------------------------------------------------------------------------
# 8. Las frases del protocolo no se truncan
# ------------------------------------------------------------------------------
def test_protocol_speech_is_complete_and_not_truncated():
    h = FakeRestaurantHarness(customers=[{"id": "1", "xy": [2., 0.]}],
                              speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
    assert h.make_session().run()
    assert rest.BARMAN_SEARCH in h.spoken
    assert rest.CUSTOMER_SEARCH in h.spoken
    assert rest.HAS_ORDER in h.spoken
    assert rest.confirm_order_phrase(["water"]) in h.spoken
    assert any(len(phrase.split()) > 15 for phrase in h.spoken)


# ------------------------------------------------------------------------------
# 9. Jamas anuncia el pedido antes de escucharlo
# ------------------------------------------------------------------------------
def test_never_speaks_order_before_listening():
    harness = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2.0, 0.0]}],
        speech_script=[
            "yes", "yes", "juice and tea", "yes", "yes", "yes", "yes",
        ],
        llm_script={"juice and tea": ["juice", "tea"]},
    )
    session = harness.make_session()
    session.run()

    # Revisar la cronologia de eventos
    # Barman y tiene-pedido se escuchan primero; el tercer listen es el pedido.
    order_listen_idx = [i for i, (ev, _) in enumerate(harness.events) if ev == "listen"][2]
    spoken_before_listen = [val for ev, val in harness.events[:order_listen_idx] if ev == "speak"]

    for phrase in spoken_before_listen:
        assert "juice" not in phrase.lower(), f"Anuncio 'juice' antes de escuchar: '{phrase}'"
        assert "tea" not in phrase.lower(), f"Anuncio 'tea' antes de escuchar: '{phrase}'"



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
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
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
    assert len(h.nav_goals) == 4  # Two customer candidates and one retry to the bar.
    assert h.nav_goals[-2:] == [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


def test_aborted_fallback_has_no_free_candidate_to_retry():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], nav_success=False)
    assert not h.make_session(get_costmap_fn=lambda: make_grid(100), nav_aborted_fn=lambda: True).run()
    assert len(h.nav_goals) == 3  # Failed approach and bounded final-bar retry.


def test_delivery_aborted_retries_other_free_candidate():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
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
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
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
    assert len(h.nav_goals) == 3  # No further search/return after incomplete delivery.
    assert session.state == SessionState.FAILED


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
    methods = node_methods('_navigate', '_navigate_goal', '_cancel_nav')
    node = NS(_navigation=action.nav, _spinning=NS(cancel=lambda: None),
              voice=voice, session=session, navigation_timeout_s=90.,
              get_logger=lambda: NS(info=lambda _: None),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
    node._navigate_goal = lambda x, y, yaw, *, deadline_s: methods['_navigate_goal'](
        node, x, y, yaw, deadline_s=deadline_s)
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
                              speech_script=['yes', 'yes', 'water', 'yes', speech] if at_barman else ['yes', speech])
    session = h.make_session()
    assert not session.run()
    assert session.state == SessionState.ABORTED
    assert h.nav_cancelled
    assert len(h.nav_goals) == (2 if at_barman else 1)


@pytest.mark.parametrize('navigation_number', [1, 2, 3, 4])
def test_deadline_during_each_navigation_is_terminal_without_return(navigation_number):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
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
    assert not any(event == 'detect_customers' for event, _ in h.events)  # audio ya agoto el limite


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
    h = FakeRestaurantHarness(customers=[{'xy': [1.5, 2.3]}], speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
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
                              speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])

    def plan(pose):
        return None if pose[1] > 0 else 4.0      # el cliente 1 no tiene camino, el 2 si
    session = h.make_session(get_costmap_fn=lambda: restaurant_grid, plan_fn=plan, max_customers=1)
    assert session.run()
    assert "I could not reach the customer." in h.spoken
    assert all(goal[1] <= 0 for goal in h.nav_goals)   # nunca un goal sin plan
    assert len(h.nav_goals) == 4                       # cliente 2, barra, entrega, barra final


def _phase_detector(session_ref, barman_view, customer_views):
    """Detector de personas falso: el barman en su busqueda; clientes por giro durante DETECTING_CUSTOMERS."""
    turns = {'customers': 0}
    def detect(*_):
        if session_ref[0].state == SessionState.DETECTING_CUSTOMERS:
            return customer_views.get(turns['customers'], [])
        return barman_view
    def spin(_):
        if session_ref[0].state == SessionState.DETECTING_CUSTOMERS:
            turns['customers'] += 1
        return True
    return detect, spin, turns


def test_customers_found_by_full_base_turn_with_raised_hand_and_ranked():
    """Pedido del usuario 10/11: tras el barman, cabeza al frente y vuelta con la base buscando manos levantadas."""
    h = FakeRestaurantHarness(speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes', 'yes', 'beer', 'yes', 'yes', 'yes'])
    ref = [None]
    hand = {'raised_frames': 4}
    detect, spin, turns = _phase_detector(ref, [{'xy': [0., 1.], 'raised_frames': 5}], {
        3: [dict(hand, id='1', xy=[4., 2.])],
        7: [dict(hand, id='2', xy=[2., -1.]), {'id': 'no_hand', 'xy': [1.5, 1.5], 'raised_frames': 0}],
    })
    session = h.make_session(spin_fn=spin, detect_people_fn=detect,
                             point_head_fn=lambda p, t: h.events.append(('head', (p, t))))
    ref[0] = session
    session.detect_customers_fn = lambda *_: pytest.fail('sin barrido del pan-tilt')
    assert session.run()
    assert turns['customers'] == 11  # vuelta completa: 12 vistas
    assert [c['xy'] for c in session.customers] == [[2., -1.], [4., 2.]]
    assert len(session.orders) == 2 and session.served == 2
    assert {v for k, v in h.events if k == 'head'} == {(0., 0.)}  # la cabeza solo mira al frente


@pytest.mark.parametrize('failed_spin', [1, 2, 3])
def test_failed_barman_spin_stops_turns_and_continues_with_customer_orders(failed_spin):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=one_customer_protocol()[1:])
    turns, looks = [], []
    def spin(angle):
        turns.append(angle)
        return len(turns) != failed_spin
    def barman(*args):
        looks.append(args)
        return []
    session = h.make_session(spin_fn=spin, detect_barman_fn=barman)
    assert session.run()
    assert turns == pytest.approx([math.pi / 6] * failed_spin)
    assert len(looks) == failed_spin + 1  # vista frontal mas una por giro completado/fallido
    assert len(session.customers) == 1 and session.served == 1


def test_failed_second_barman_spin_can_still_use_person_in_last_view():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 1.]}], speech_script=one_customer_protocol())
    results = iter([True, False])
    views = iter([[], [], [{'xy': [0., 1.]}]])
    session = h.make_session(spin_fn=lambda _: next(results),  # sin tercer giro: ni de frente tras el fallo
                             detect_barman_fn=lambda *_: next(views))
    assert session.run()
    assert session.barman_position == (0., 1.)
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

    session = h.make_session(spin_fn=spin, detect_barman_fn=lambda *_: h.detect_customers(14., 3) and [])
    assert not session.run()
    assert session.state == (SessionState.ABORTED if interrupt == 'stop' else SessionState.TIMEOUT)
    assert turns == [math.pi / 6] and h.nav_cancelled
    assert not h.nav_goals
    assert len([e for e, _ in h.events if e == 'detect_customers']) == 1  # barman antes del giro


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
    session.detect_barman_fn = lambda *_: h.detect_customers(14., 3) and []
    session.cancel_nav_fn = lambda: methods['_cancel_nav'](node)
    assert not session.run()
    assert session.state == SessionState.ABORTED
    assert action.cancels and action.result.done() and not action.nav.pending
    assert len(action.sent) == 1 and nav_cancel
    assert action.sent[0].target_yaw == math.pi / 6
    assert not h.nav_goals
    assert len([e for e, _ in h.events if e == 'detect_customers']) == 1  # barman antes del giro


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



def test_customer_scan_without_raised_hands_searches_again():
    h = FakeRestaurantHarness(speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
    ref = [None]
    detect, spin, turns = _phase_detector(ref, [], {5: [{'id': 'quiet', 'xy': [2., 0.], 'raised_frames': 0}]})
    session = h.make_session(spin_fn=spin, detect_people_fn=detect, max_customers=1, max_time_s=200.)
    ref[0] = session
    assert not session.run()  # nadie levanta la mano: busca de nuevo hasta el tiempo limite
    assert h.spoken.count(rest.SEARCH_AGAIN) >= 1 and not session.customers


def test_default_head_scan_has_eighty_second_budget():
    h = FakeRestaurantHarness()
    session = h.make_session()
    assert session.spin_fn is None
    session.deadline_s = h.clock() + 900.0
    session._look_around()
    assert ('detect_customers', (80.0, 3)) in h.events


def test_node_tracking_lease_follows_customer_states_and_stops():
    import json
    path = Path(__file__).parents[1] / 'base_athome_skills/restaurant_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'RestaurantNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_on_session_state_change', '_refresh_customer_tracking')]
    env = dict(Dict=Dict, Any=Any, String=NS, json=json)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), env)
    node = type('TrackingNode', (), {n.name: env[n.name] for n in methods})()
    requests = []
    node.session = NS(customers=[{'xy': [2., 3.]}], stopped=False, remaining_s=lambda: 50.)
    node.pub_bag_request = NS(publish=lambda msg: requests.append(json.loads(msg.data)))
    node.pub_state = NS(publish=lambda msg: None)
    for state in ('APPROACHING_CUSTOMER', 'GREETING', 'TAKING_ORDER', 'CONFIRMING_ORDER', 'DELIVERING_TO_CUSTOMER'):
        node._on_session_state_change({'state': state, 'customer_idx': 0})
        assert requests[-1] == {'mode': 'track_customer', 'xy': [2., 3.]}
    for state in ('RETURNING_TO_BAR', 'FAILED', 'COMPLETED'):
        node._on_session_state_change({'state': state, 'customer_idx': 0})
        assert requests[-1]['xy'] is None
    node._head_customer_xy = [2., 3.]
    node.session.stopped = True
    node._refresh_customer_tracking()
    assert requests[-1]['xy'] is None


@pytest.mark.parametrize('reason, expected', [
    ('unlocalized_raised_hand', 'cannot measure your distance'),
    ('unconfirmed_raised_hand', 'hold it up'),
    ('timeout', 'did not respond'),
])
def test_detection_failure_explains_actual_reason_without_navigation(reason, expected):
    h = FakeRestaurantHarness()
    session = h.make_session()
    session.detect_customers_fn = lambda *args: {'ok': False, 'error': reason, 'customers': []}
    assert session.run() is False
    assert any(expected in phrase for phrase in h.spoken)
    assert not any('do not see any customers' in phrase for phrase in h.spoken)
    assert not h.nav_goals


def test_batch_orders_takes_both_orders_first_then_delivers_both():
    """Task planning fuerte para 2 personas: decide a cual ir primero (mas cercana/medida),
    toma el pedido de la primera, va directo a la segunda a tomarle el pedido, y luego va al bar."""
    h = FakeRestaurantHarness(
        customers=[
            {"id": "far", "xy": [4.0, 0.0]},
            {"id": "near", "xy": [2.0, 0.0]},
        ],
        speech_script=[
            "yes",  # barman
            "yes", "I want a coke and water", "yes",
            "yes", "please bring me coffee", "yes",
            "yes", "yes", "yes",  # bandeja cliente 1 y entrega
            "yes", "yes",             # bandeja cliente 2 y entrega
        ],
        initial_pose=(0.0, 0.0, 0.0),
    )
    session = h.make_session(batch_orders=True)
    assert session.run() is True
    assert session.state == SessionState.SUCCEEDED
    assert session.served == 2
    # El planner ordeno primero al mas cercano (2.0, 0.0) y segundo al mas lejano (4.0, 0.0)
    assert [c["xy"] for c in session.customers] == [[2.0, 0.0], [4.0, 0.0]]

    # Secuencia de navegacion:
    # 0: mesa cliente 1 (~1.1, 0) -> 1: mesa cliente 2 (~3.1, 0) ->
    # 2: bar (0, 0) -> 3: mesa cliente 1 (~1.1, 0) ->
    # 4: bar (0, 0) -> 5: mesa cliente 2 (~3.1, 0) -> 6: bar final (0, 0)
    assert len(h.nav_goals) == 7
    assert h.nav_goals[0][0] == pytest.approx(1.1, abs=0.05)
    assert h.nav_goals[1][0] == pytest.approx(3.1, abs=0.05)
    assert h.nav_goals[2][:2] == pytest.approx((0.0, 0.0), abs=0.05)
    assert h.nav_goals[3][0] == pytest.approx(1.1, abs=0.05)
    assert h.nav_goals[4][:2] == pytest.approx((0.0, 0.0), abs=0.05)
    assert h.nav_goals[5][0] == pytest.approx(3.1, abs=0.05)
    assert h.nav_goals[6][:2] == pytest.approx((0.0, 0.0), abs=0.05)


def test_batch_delivery_failure_stops_instead_of_restarting_customer_search():
    h = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [2., 0.]}, {"id": "2", "xy": [3., 0.]}],
        speech_script=["yes", "yes", "I want water", "yes", "yes", "I want juice", "yes"],
    )
    session = h.make_session(batch_orders=True, max_customers=2)
    attempts = []

    def deliver(idx, *_args):
        attempts.append(idx)
        return idx == 0

    session._fulfill_and_deliver_order = deliver
    assert session.run() is False
    assert attempts == [0, 1]
    assert session.state == SessionState.FAILED
    assert h.states[-1]["info"]["reason"] == "order_delivery_incomplete"
    assert sum(s["state"] == "DETECTING_CUSTOMERS" for s in h.states) == 1


def test_bar_navigation_retries_once_after_nav2_abort():
    h = FakeRestaurantHarness(customers=[{"id": "1", "xy": [2., 0.]}])
    session = h.make_session(nav_aborted_fn=lambda: len(h.nav_goals) == 1)
    original_navigate = h.navigate

    def abort_first_goal(x, y, yaw, *, deadline_s):
        original_navigate(x, y, yaw, deadline_s=deadline_s)
        return len(h.nav_goals) > 1

    session.navigate_fn = abort_first_goal
    session.start_time = h.clock()
    session.deadline_s = h.clock() + 900.
    session._abort_run = False
    session.bar_pose = (0., 0., 0.)

    assert session._return_to_bar(final=True)
    assert len(h.nav_goals) == 2
    assert h.states[-1]["state"] == "RETURNING_TO_BAR_FINAL"
    assert "The path to the bar was blocked. I will retry once." in h.spoken


def test_terminal_success_is_not_changed_to_timeout_by_final_announcement():
    h = FakeRestaurantHarness(customers=[{"id": "1", "xy": [2., 0.]}],
                              speech_script=one_customer_protocol())
    session = h.make_session(max_customers=1)
    original_speak = h.speak

    def speak(text):
        original_speak(text)
        if text == "Mission complete. I am back at the kitchen bar.":
            h.current_time = session.deadline_s + 1.0

    session.speak_fn = speak
    assert session.run() is True
    assert session.state == SessionState.SUCCEEDED
    assert h.states[-1]["state"] == "SUCCEEDED"
    assert h.spoken[-1] == "Mission complete. I am back at the kitchen bar."


def test_estimated_far_customer_triggers_progressive_advance_and_relocalization():
    """Cuando un cliente lejano fue localizado por estimacion sin depth (estimated=True),
    el robot avanza primero un tramo intermedio con navegacion sin mapa, re-mide de cerca y se aproxima."""
    h = FakeRestaurantHarness(
        customers=[{"id": "1", "xy": [4.5, 0.0], "estimated": True}],
        speech_script=["yes", "yes", "water please", "yes", "yes", "yes"],
        initial_pose=(0.0, 0.0, 0.0),
    )
    detect_calls = []

    def detect_customers(timeout_s, min_frames):
        detect_calls.append((timeout_s, min_frames))
        if len(detect_calls) == 1:  # barman separado del cliente
            return [{"xy": [0., .1]}]
        if len(detect_calls) == 2:
            return [{"id": "1", "xy": [4.5, 0.0], "estimated": True}]
        # Al haberse acercado, la RealSense ya mide profundidad real en (4.0, 0.2)
        return [{"id": "1", "xy": [4.0, 0.2], "estimated": False}]

    session = h.make_session(get_costmap_fn=make_grid)
    session.detect_customers_fn = detect_customers
    assert session.run() is True
    assert session.served == 1
    assert len(detect_calls) == 3
    # Goal 0: avance progresivo intermedio (< 2.5 m); Goal 1: aproximacion final a ~0.9 m de (4.0, 0.2)
    assert 0.6 <= h.nav_goals[0][0] <= 2.5
    assert math.hypot(h.nav_goals[1][0] - 4.0, h.nav_goals[1][1] - 0.2) == pytest.approx(0.9, abs=0.25)
    assert session.customers[0]["estimated"] is False


def test_protocol_requires_explicit_order_confirmation():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'yes', 'water', 'no',
                                             'coffee', 'yes', 'yes', 'yes'])
    session = h.make_session(max_customers=1)
    assert session.run()
    assert session.orders['1'] == ['coffee']
    assert "Sorry for my mistake. Let's try again." in h.spoken


def test_new_callbacks_are_optional_and_arm_pick_failure_uses_tray():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes'])
    picks = []
    session = h.make_session(max_customers=1, detect_item_fn=lambda *_: True,
                             pick_item_fn=lambda item: picks.append(item) or False,
                             handover_fn=lambda: True, release_fn=lambda: True,
                             stow_fn=lambda: True)
    assert session.run()
    assert picks == ['water', 'water']
    assert 'I could not pick it. Please place everything on my tray instead.' in h.spoken


def one_customer_protocol(order='water', tray_items=1):
    # barman, tiene pedido, items, confirmacion, carga por item, descarga.
    return ['robot yes', 'robot yes', order, 'robot yes'] + ['robot yes'] * (tray_items + 1)


def manipulation_callbacks(h, **overrides):
    def action(name, result=True):
        def callback(*args):
            h.events.append((name, args))
            return result
        return callback
    callbacks = dict(detect_item_fn=action('detect_item'), pick_item_fn=action('pick'),
                     handover_fn=action('handover'), release_fn=action('release'),
                     stow_fn=action('stow'))
    callbacks.update(overrides)
    return callbacks


@pytest.mark.parametrize('answers', [['robot no'], ['maybe']])
def test_barman_not_confirmed_does_not_block_customer_orders(answers):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'x': 2., 'y': 0.}],
                              speech_script=answers + one_customer_protocol()[1:])
    enrollments = []
    session = h.make_session(enroll_face_fn=lambda label, person: enrollments.append((label, person)) or True)
    assert session.run()
    assert h.spoken.count(rest.BARMAN_SEARCH) == 1
    assert [label for label, _ in enrollments] == ['customer_1']
    assert not any('customer' in phrase.lower() and 'barman' in phrase.lower() for phrase in h.spoken)
    assert session.bar_pose == (0., 0., 0.)


def test_barman_missing_continues_and_closest_person_is_selected_when_present():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol()[1:])
    session = h.make_session()
    calls = []
    def detect(*args):
        calls.append(args)
        return [] if len(calls) == 1 else h.customers
    session.detect_customers_fn = detect
    assert session.run()
    assert len(calls) == 2

    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol())
    enrolled = []
    session = h.make_session(enroll_face_fn=lambda label, person: enrolled.append((label, person)) or True)
    def detect_nearest(*args):
        if session.state == SessionState.FINDING_BARMAN:
            return [{'id': 'far', 'xy': [5., 0.]}, {'id': 'near', 'x': 0.2, 'y': 0.}]
        return h.customers
    session.detect_customers_fn = detect_nearest
    assert session.run()
    assert enrolled[0][0] == 'barman' and enrolled[0][1]['id'] == 'near'


def test_face_filter_discards_barman_and_served_customer_after_detector_ids_change():
    h = FakeRestaurantHarness(customers=[{'id': 'one', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol() + one_customer_protocol()[1:])
    session = h.make_session(max_customers=2, identify_face_fn=lambda p: p.get('label'))
    views = iter([
        [{'id': 'bar', 'xy': [0., 1.]}],
        [{'id': 'one', 'xy': [2., 0.]}],
        [{'id': 'renumbered_bar', 'xy': [0., 1.], 'label': 'barman'},
         {'id': 'renumbered_one', 'xy': [7., 3.], 'label': 'customer_1'},
         {'id': 'two', 'xy': [3., -1.]}],
    ])
    session.detect_customers_fn = lambda *_: next(views)
    assert session.run()
    assert session.served_labels == {'customer_1', 'customer_2'}
    assert set(session.orders) == {'one', 'two'}
    assert len(h.nav_goals) == 8  # dos ciclos completos y regreso antes de nueva busqueda


def test_barman_with_raised_hand_is_not_taken_as_customer():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=one_customer_protocol())
    ref = [None]
    barman = {'xy': [0., 1.], 'raised_frames': 5}
    detect, spin, turns = _phase_detector(ref, [barman], {0: [barman], 4: [{'xy': [2., 0.], 'raised_frames': 3}]})
    session = h.make_session(spin_fn=spin, detect_people_fn=detect)
    ref[0] = session
    assert session.run()
    assert session.barman_position == (0., 1.)
    assert [c['xy'] for c in session.customers] == [[2., 0.]] and session.served == 1


def test_no_customers_retries_then_serves_without_exceeding_deadline():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    session = h.make_session()
    views = iter([[{'xy': [0., .1]}], [], h.customers])
    session.detect_customers_fn = lambda *_: next(views)
    assert session.run()
    assert h.spoken.count(rest.SEARCH_AGAIN) == 1
    assert h.spoken.count(rest.CUSTOMER_SEARCH) == 2
    assert h.current_time < session.deadline_s


@pytest.mark.parametrize('speech_script, repeats', [
    (['yes', 'no'], 0),
    (['yes', 'noise', 'more noise', 'still noise'], 2),
    (['yes', 'noise', 'robot yes', 'water', 'yes', 'yes', 'yes'], 1),
])
def test_has_order_no_and_three_unrecognized_answers(speech_script, repeats):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=speech_script)
    session = h.make_session(max_time_s=100.)
    ok = session.run()
    assert h.spoken.count(rest.REPEAT_HAS_ORDER) == repeats
    if repeats == 1:
        assert ok and session.served == 1
    else:
        assert not ok and session.state == SessionState.TIMEOUT
        assert session.orders == {'1': []}
        assert not any('Dear barman, I have received' in p for p in h.spoken)


@pytest.mark.parametrize('answers', [
    ['gibberish', 'gibberish', 'gibberish'],
    ['water', 'no', 'coffee', 'no', 'tea', 'no'],
    ['water', 'maybe', 'coffee', '', 'tea', 'unclear'],
])
def test_order_attempts_are_limited_to_three_and_never_deliver_unconfirmed_items(answers):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'yes'] + answers)
    session = h.make_session(max_time_s=100.)
    assert not session.run()
    assert h.spoken.count(rest.ORDER_PROMPT) == 3
    assert h.spoken.count(rest.cancel_order_phrase()) == 1
    assert session.orders == {'1': []} and session.served == 0
    assert not any('Dear barman, I have received' in p for p in h.spoken)


def test_arm_delivery_countdown_immediately_precedes_release_and_then_stows():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol('water and beer', tray_items=1))
    session = h.make_session(**manipulation_callbacks(h))
    assert session.run()
    assert [e for e, _ in h.events if e in ('pick', 'handover', 'release', 'stow')] == [
        'pick', 'handover', 'release', 'stow']
    release_idx = next(i for i, (e, _) in enumerate(h.events) if e == 'release')
    assert h.events[release_idx-3:release_idx] == [
        ('speak', 'Please grab the object in my hand. I will open my hand in three,'),
        ('speak', 'two,'), ('speak', 'one.')]
    pick_idx = next(i for i, (e, _) in enumerate(h.events) if e == 'pick')
    assert h.events[pick_idx-1] == ('speak', 'I will now pick the water.')
    assert rest.tray_request_phrase('beer') in h.spoken
    assert rest.tray_request_phrase('water') not in h.spoken
    for i, (event, _) in enumerate(h.events):
        if event == 'navigate':
            assert h.events[i-1][0] == 'speak' and h.events[i-1][1].startswith('Moving to ')
    states = {s['state'] for s in h.states}
    assert {'FINDING_BARMAN', 'CONFIRMING_BARMAN', 'CLEARING_COUNTER', 'CHECKING_ORDER',
            'DETECTING_ITEM', 'PICKING_ITEM', 'HANDING_OVER', 'RELEASING_ITEM',
            'STOWING_ARM', 'WAITING_CUSTOMER'} <= states
    assert all('customer_idx' in payload and 'info' in payload for payload in h.states)


@pytest.mark.parametrize('arm, pick_present', [(False, True), (True, False)])
def test_without_arm_every_item_goes_on_tray(arm, pick_present):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol('water and beer', tray_items=2))
    callbacks = manipulation_callbacks(h)
    if not pick_present:
        callbacks['pick_item_fn'] = None
    session = h.make_session(arm=arm, use_tray=False, **callbacks)
    assert session.run()
    assert not any(e in ('detect_item', 'pick', 'handover', 'release', 'stow') for e, _ in h.events)
    assert rest.tray_request_phrase('water') in h.spoken
    assert rest.tray_request_phrase('beer') in h.spoken
    assert h.spoken.count('Tell me robot yes when you are done.') == 2


@pytest.mark.parametrize('detections, expected_picks', [([False, False, False], 0), ([False, False, True], 1)])
def test_counter_detection_retries_three_times_and_falls_back_to_tray(detections, expected_picks):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    detections = iter(detections)
    seen = []
    def detect(item, timeout):
        assert item == 'water' and 0 < timeout <= 10.
        seen.append(item)
        return next(detections)
    session = h.make_session(**manipulation_callbacks(h, detect_item_fn=detect))
    assert session.run()
    assert len(seen) == 3
    assert sum(e == 'pick' for e, _ in h.events) == expected_picks
    assert h.spoken.count('I cannot see the water. Can you please slightly move and rotate it?') == (3 if expected_picks == 0 else 2)


def test_pick_can_succeed_on_second_attempt_without_requesting_tray():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    results = iter([False, True])
    session = h.make_session(**manipulation_callbacks(h, pick_item_fn=lambda _: next(results)))
    assert session.run()
    assert h.spoken.count('I will now pick the water.') == 2
    assert rest.tray_request_phrase('water') not in h.spoken


@pytest.mark.parametrize('failed_action', ['handover_fn', 'release_fn', 'stow_fn'])
def test_failed_delivery_action_prevents_later_actions_and_return_navigation(failed_action):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    callbacks = manipulation_callbacks(h, **{failed_action: lambda: False})
    session = h.make_session(**callbacks)
    assert not session.run()
    assert session.state == SessionState.FAILED and session.served == 0
    assert len(h.nav_goals) == 3
    events = [event for event, _ in h.events]
    if failed_action == 'handover_fn':
        assert 'release' not in events and 'stow' not in events
    if failed_action == 'release_fn':
        assert 'stow' not in events


@pytest.mark.parametrize('interrupt', ['stop', 'timeout'])
@pytest.mark.parametrize('phrase', [rest.counter_request_phrase('water'), 'I will now pick the water.',
                                   'Let me give you the object in my hand.', 'one.',
                                   'I will now stow my arm.'])
def test_interrupt_during_blocking_speech_prevents_following_movement(interrupt, phrase):
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    session = h.make_session(**manipulation_callbacks(h))
    def speak(text):
        h.speak(text)
        if text == phrase:
            if interrupt == 'stop':
                session.stop()
            else:
                h.current_time = session.deadline_s
    session.speak_fn = speak
    assert not session.run()
    assert session.state == (SessionState.ABORTED if interrupt == 'stop' else SessionState.TIMEOUT)
    i = next(i for i, event in enumerate(h.events) if event == ('speak', phrase))
    assert not any(e in ('navigate', 'detect_item', 'pick', 'handover', 'release', 'stow') for e, _ in h.events[i+1:])
    assert h.nav_cancelled


def test_tray_barman_confirmation_can_timeout_without_waiting_past_global_limit():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'yes', 'water', 'yes'])
    session = h.make_session(max_time_s=30.)
    assert not session.run()
    assert session.state == SessionState.TIMEOUT and h.nav_cancelled
    assert len(h.nav_goals) == 2
    assert all(timeout <= 30. for e, timeout in h.events if e == 'listen')


def test_tray_customer_no_never_counts_as_served_or_departs_with_tray():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'robot no'])
    session = h.make_session()
    assert not session.run()
    assert session.state == SessionState.FAILED and session.served == 0
    assert len(h.nav_goals) == 3
    assert h.states[-1]['info']['reason'] == 'tray_not_taken'


def test_optional_face_failures_do_not_prevent_tray_service():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    def unavailable(*_):
        raise RuntimeError('fake unavailable')
    session = h.make_session(enroll_face_fn=unavailable, identify_face_fn=unavailable)
    assert session.run()
    enrollment = [s['info'] for s in h.states if s['state'] == 'ENROLLING_FACE' and 'ok' in s['info']]
    assert enrollment == [{'label': 'barman', 'ok': False}, {'label': 'customer_1', 'ok': False}]


def test_callback_exception_is_not_assumed_to_leave_arm_safe():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    def broken_pick(_):
        raise RuntimeError('fake pick failed with unknown state')
    session = h.make_session(**manipulation_callbacks(h, pick_item_fn=broken_pick))
    assert not session.run()
    assert session.state == SessionState.FAILED
    assert h.states[-1]['info']['reason'] == 'arm_callback_error'
    assert len(h.nav_goals) == 2
    assert not any(e == 'release' for e, _ in h.events)


def test_no_state_callback_runs_between_last_countdown_number_and_release():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}], speech_script=one_customer_protocol())
    session = h.make_session(**manipulation_callbacks(h))
    def changed(payload):
        h.on_state_change(payload)
        h.events.append(('state', payload['state']))
    session.on_state_change_fn = changed
    assert session.run()
    idx = next(i for i, (event, _) in enumerate(h.events) if event == 'release')
    assert h.events[idx-3:idx] == [
        ('speak', 'Please grab the object in my hand. I will open my hand in three,'),
        ('speak', 'two,'), ('speak', 'one.')]


def test_known_new_face_is_not_discarded_by_previous_customer_position():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=one_customer_protocol() + one_customer_protocol()[1:])
    session = h.make_session(max_customers=2, identify_face_fn=lambda person: person.get('label'))
    views = iter([[{'xy': [0., .1]}], h.customers, [{'id': '2', 'xy': [2., 0.], 'label': 'new_customer'}]])
    session.detect_customers_fn = lambda *_: next(views)
    assert session.run()
    assert session.served == 2 and set(session.orders) == {'1', '2'}


def test_barman_negative_answer_does_not_restart_barman_search():
    h = FakeRestaurantHarness(customers=[{'id': '1', 'xy': [2., 0.]}],
                              speech_script=['no'] + one_customer_protocol()[1:])
    enrolled = []
    session = h.make_session(enroll_face_fn=lambda label, person: enrolled.append(label) or True)
    assert session.run()
    assert enrolled == ['customer_1']
    assert h.spoken.count(rest.BARMAN_SEARCH) == 1


def test_barman_wait_uses_yes_and_timeout_and_never_accepts_no_as_done():
    h = FakeRestaurantHarness(speech_script=['robot no', 'robot yes'])
    session = h.make_session()
    session.deadline_s = h.clock() + 100.
    session._wait_for_barman(['water'], max_wait_s=30.)
    assert not h.speech_script and h.clock() < 130.

    h.speech_script = ['no'] * 100
    start = h.clock()
    session._wait_for_barman(['water'], max_wait_s=30.)
    # El fake consume dos segundos por listen incluso si queda menos tiempo.
    assert 30. <= h.clock() - start < 32.
    assert not h.nav_goals


def test_picks_second_item_when_first_is_not_visible_and_rest_goes_to_tray():
    h = FakeRestaurantHarness(speech_script=['yes'] * 10)
    picked = []
    session = h.make_session(arm=True, detect_item_fn=lambda item, _t: item == 'coke',
                             pick_item_fn=lambda item: picked.append(item) or True)
    session.deadline_s = h.clock() + 900.
    session._abort_run = False
    held, tray = session._prepare_order(["chips", "coke"])
    assert held == 'coke' and picked == ['coke'] and tray == ['chips']


def test_barman_puts_first_item_in_gripper_and_second_on_tray():
    h = FakeRestaurantHarness(speech_script=['robot yes', 'robot yes'])
    arm = []
    session = h.make_session(arm=True, present_gripper_fn=lambda: arm.append('present') or True,
                             grip_item_fn=lambda: arm.append('grip') or True,
                             pick_item_fn=lambda _i: pytest.fail('sin agarre por vision'))
    session.deadline_s = h.clock() + 900.
    session._abort_run = False
    held, tray = session._prepare_order(['coke', 'chips'])
    assert (held, tray, arm) == ('coke', ['chips'], ['present', 'grip'])
    assert 'Dear barman, please put the coke in my gripper. Tell me robot yes when you are done.' in h.spoken
    assert rest.tray_request_phrase('chips') in h.spoken


def test_continuous_turn_faces_barman_by_odom_heading_not_map_yaw():
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns, calls = [], []
    barman = {'xy': [-1.5, .3], 'range_m': 1.5, 'heading_deg': 170., 'raised_frames': 6}
    other = {'xy': [1., 0.], 'range_m': 1.0, 'heading_deg': 0., 'raised_frames': 0}
    def scan(on_first_step=None):
        calls.append(on_first_step)
        return [other, barman], True
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True, scan_turn_fn=scan,
                             get_odom_yaw_fn=lambda: math.radians(-10.),  # odom tras la vuelta
                             detect_people_fn=lambda *_: [dict(barman, bearing_deg=2.)])
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert calls == [None] and len(turns) == 1
    assert math.degrees(turns[0]) == pytest.approx(180.)  # 170 - (-10)
    assert session.barman_position == (-1.5, .3)


def test_continuous_customer_turn_announces_after_first_step_and_keeps_raised_hands():
    h = FakeRestaurantHarness()
    def scan(on_first_step=None):
        h.events.append(('turn_started', None))
        on_first_step()
        return [{'xy': [2., 0.], 'raised_frames': 4}, {'xy': [0., 3.], 'raised_frames': 0}], True
    session = h.make_session(spin_fn=lambda a: True, scan_turn_fn=scan, get_odom_yaw_fn=lambda: 0.)
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    found = session._look_around()
    assert [c['xy'] for c in found] == [[2., 0.]]
    started = h.events.index(('turn_started', None))
    assert h.events.index(('speak', rest.CUSTOMER_TURN)) > started


def test_blocked_turn_falls_back_to_head_sweep_instead_of_looping():
    """Prueba real 10/11: pegado a la barra, Nav2 corta el Spin y la busqueda se repetia sin fin."""
    h = FakeRestaurantHarness()
    sweeps = []
    session = h.make_session(spin_fn=lambda a: False, scan_turn_fn=lambda cb=None: ([], False),
                             get_odom_yaw_fn=lambda: 0.)
    session.detect_customers_fn = lambda *a: sweeps.append(a) or [{'id': 'c', 'xy': [2., 1.]}]
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    found = session._look_around()
    assert len(sweeps) == 1 and [c['xy'] for c in found] == [[2., 1.]]
    assert rest.CUSTOMER_TURN in h.spoken


def test_barman_is_faced_even_when_the_scan_turn_was_cut_short():
    """Prueba real 10/11: Spin corto con 'Collision Ahead' y el robot le hablaba a otro lado."""
    h = FakeRestaurantHarness(speech_script=['robot yes'])
    turns, tracked = [], []
    barman = {'xy': [9., 9.], 'range_m': 1.5, 'heading_deg': 90., 'raised_frames': 6}  # xy del SLAM corrido
    fresh = {'xy': [0., 1.5], 'range_m': 1.5, 'bearing_deg': 1., 'raised_frames': 4}
    session = h.make_session(spin_fn=lambda a: turns.append(a) or True,
                             scan_turn_fn=lambda: ([barman], False), get_odom_yaw_fn=lambda: 0.,
                             detect_people_fn=lambda *_: [fresh], track_person_fn=tracked.append)
    session.deadline_s = h.clock() + 900.
    session.bar_pose = (0., 0., 0.)
    session._find_barman()
    assert math.degrees(turns[0]) == pytest.approx(90.)
    assert tracked == [fresh] and session.barman_position == (0., 1.5)  # posicion medida de frente


def test_cut_short_scan_uses_enough_confirmed_customers_without_head_rescan():
    h = FakeRestaurantHarness()
    people = [
        {'xy': [-2.6, 0.], 'raised_frames': 3},
        {'xy': [2., 2.8], 'raised_frames': 3},
        {'xy': [3., -2.9], 'raised_frames': 3},
        {'xy': [2.1, 2.8], 'raised_frames': 3},
    ]
    session = h.make_session(spin_fn=lambda _: False,
                             scan_turn_fn=lambda cb=None: (people, False))
    session.barman_position = (-2.6, 0.)
    rescans = []
    session.detect_customers_fn = lambda *args: rescans.append(args) or []
    session.start_time = h.clock()
    session.deadline_s = h.clock() + 900.
    found = session._look_around()
    assert len(found) == 2
    assert rescans == []


@pytest.mark.parametrize('people', [
    [{'xy': [2., 0.], 'raised_frames': 3}, {'xy': [2.1, 0.], 'raised_frames': 3}],
    [{'xy': [2., 0.], 'raised_frames': 3}, {'xy': [-2.6, 0.], 'raised_frames': 3}],
    [{'xy': [2., 0.], 'raised_frames': 3}, {'xy': [3., 2.], 'raised_frames': 0}],
    [{'xy': [float('nan'), 0.], 'raised_frames': 3}],
])
def test_cut_short_scan_still_sweeps_when_confirmed_customers_are_insufficient(people):
    h = FakeRestaurantHarness()
    session = h.make_session(spin_fn=lambda _: False,
                             scan_turn_fn=lambda cb=None: (people, False))
    session.barman_position = (-2.6, 0.)
    rescans = []
    session.detect_customers_fn = lambda *args: rescans.append(args) or []
    session.deadline_s = h.clock() + 900.
    session._look_around()
    assert len(rescans) == 1


def test_cut_short_scan_needs_only_one_confirmed_customer_after_first_delivery():
    h = FakeRestaurantHarness()
    person = {'xy': [2., 0.], 'raised_frames': 3}
    session = h.make_session(spin_fn=lambda _: False,
                             scan_turn_fn=lambda cb=None: ([person], False))
    session.served = 1
    session.deadline_s = h.clock() + 900.
    session.detect_customers_fn = lambda *args: pytest.fail('already found remaining customer')
    assert len(session._look_around()) == 1


@pytest.mark.parametrize('limit', [1, 3])
@pytest.mark.parametrize('unreachable', [False, True])
def test_search_stops_after_bounded_rounds_without_serving_customer(limit, unreachable):
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}] if unreachable else [],
                              nav_success=not unreachable)
    session = h.make_session(max_no_progress_rounds=limit)
    if unreachable:
        # El regreso a la barra sigue disponible; solo fallan las mesas.
        session.navigate_fn = lambda x, y, yaw, **kw: (
            h.navigate(x, y, yaw, **kw) or (x, y) == (0., 0.))
    assert not session.run()
    assert session.state == SessionState.FAILED
    assert h.states[-1]['info'] == {'reason': 'no_customer_progress', 'rounds': limit, 'served': 0}
    assert h.spoken.count(rest.CUSTOMER_SEARCH) == limit
    assert h.clock() < session.deadline_s


@pytest.mark.parametrize('limit', [0, -1, 1.5, '3'])
def test_invalid_no_progress_round_limit_is_rejected(limit):
    with pytest.raises(ValueError, match='max_no_progress_rounds'):
        FakeRestaurantHarness().make_session(max_no_progress_rounds=limit)


def test_no_progress_round_limit_resets_after_a_customer_is_served():
    h = FakeRestaurantHarness(speech_script=['yes', 'yes', 'water', 'yes', 'yes', 'yes',
                                            'yes', 'water', 'yes', 'yes', 'yes'])
    views = iter([[], [], [h.customers[0]], [], [], [h.customers[1]]])
    session = h.make_session(max_no_progress_rounds=3,
                             detect_barman_fn=lambda *_: [{'xy': [-2.6, 0.]}])
    session.detect_customers_fn = lambda *_: next(views, [])
    assert session.run()
    assert session.served == 2
    assert h.spoken.count(rest.CUSTOMER_SEARCH) == 6


@pytest.mark.parametrize('remaining', [900., .2])
def test_simulated_table_retreat_is_slow_bounded_and_announced(remaining):
    action = FakeActionHarness(finish_after=.05)
    voice = FakeVoice(action)
    methods = node_methods('_back_away_from_table')
    methods['_back_away_from_table'].__globals__['BackUp'] = NS(Goal=lambda: NS(target=NS()))
    spoken = []
    node = NS(_backing=action.nav, voice=voice, _speak=spoken.append,
              session=NS(stopped=False, remaining_s=lambda: remaining, deadline_s=1000., stop=lambda: None))
    assert methods['_back_away_from_table'](node)
    assert spoken and action.sent[0].target.x == -.45
    assert action.sent[0].speed == .05
    assert action.sent[0].time_allowance.seconds == min(30., remaining)
    assert voice.cancelled


def test_failed_table_retreat_stops_before_sending_another_navigation_goal():
    events = []
    methods = node_methods('_navigate')
    node = NS(_needs_table_retreat=True, _back_away_from_table=lambda: False,
              session=NS(stop=lambda: events.append('stop')))
    assert not methods['_navigate'](node, 1., 2., 0., deadline_s=1000.)
    assert events == ['stop'] and node._needs_table_retreat


def test_bar_return_keeps_retreat_armed_until_both_waypoints_succeed():
    methods = node_methods('_navigate')
    methods['_navigate'].__globals__.update(math=math)
    goals = []
    node = NS(_needs_table_retreat=True, _nav_aborted=False,
              _back_away_from_table=lambda: True,
              _navigate_goal=lambda x, y, yaw, *, deadline_s: goals.append((x, y, yaw)) or len(goals) > 1,
              session=NS(stopped=False, stop=lambda: None),
              get_logger=lambda: NS(info=lambda _: None))
    assert not methods['_navigate'](node, 0., 0., -math.pi, deadline_s=100.)
    assert node._needs_table_retreat
    goals.clear()
    node._navigate_goal = lambda x, y, yaw, *, deadline_s: goals.append((x, y, yaw)) or True
    assert methods['_navigate'](node, 0., 0., -math.pi, deadline_s=100.)
    assert goals[0][:2] == (1.79, -.10) and goals[1][:2] == (0., 0.)
    assert not node._needs_table_retreat
