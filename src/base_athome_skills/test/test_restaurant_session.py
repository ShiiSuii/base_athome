"""Tests unitarios para RestaurantSession (HOME-105).

Cubre:
  1. Dos clientes atendidos en orden estricto (visita c1, bar, entrega c1; luego c2, bar, entrega c2; fin en bar).
  2. Cliente no entendido dos veces: repite una vez, cancela atencion y sigue con el siguiente.
  3. 'stop' por voz cancela navegacion y aborta la sesion inmediatamente.
  4. Nadie levanta la mano: aviso breve y finalizacion limpia sin navegar.
  5. LLM caida: extraccion por palabras clave del menu, o pide repetir si no hay items validos.
  6. Tope de tiempo global de 15 minutos (900 s): aborta y regresa al bar.
  7. Frases breves: todas las locuciones tienen <= 15 palabras y terminan con puntuacion.
  8. Jamas anuncia el pedido antes de escucharlo.
"""

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple
import pytest

from base_athome_skills.mission import restaurant as rest
from base_athome_skills.mission.restaurant_session import (
    RestaurantSession,
    SessionState,
    approach_candidates,
    compute_approach_pose,
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
        self.nav_cancelled: bool = False
        self.states: List[Dict[str, Any]] = []

    def clock(self) -> float:
        return self.current_time

    def sleep(self, seconds: float) -> None:
        self.current_time += seconds

    def get_pose(self) -> Tuple[float, float, float]:
        return self.robot_pose

    def navigate(self, x: float, y: float, yaw: float) -> bool:
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

    assert ok is True
    assert session.state == SessionState.SUCCEEDED
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
    assert "Time limit reached. Returning to the kitchen bar." in harness.spoken
    assert harness.nav_cancelled is True


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


@pytest.mark.parametrize('value,allowed', [(49, True), (50, False), (-1, False)])
def test_entire_robot_radius_must_have_known_cost_below_50(value, allowed):
    grid = make_grid()
    # Center of nearest candidate (0.9, 0) stays free; an adjacent cell intersects its radius.
    grid.data[42 * grid.info.width + 29] = value
    nearest = approach_candidates((2., 0.), (0., 0., 0.), grid)[0]
    assert (math.hypot(nearest[0] - 0.9, nearest[1]) < 1e-6) == allowed


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


@pytest.mark.parametrize('value', [-1, 50, 100])
def test_no_free_candidate_falls_back(value):
    from base_athome_skills.mission.restaurant_session import approach_candidates
    assert approach_candidates((2., 0.), (0., 0., 0.), make_grid(value)) == [compute_approach_pose((2., 0.), (0., 0., 0.))]


def test_aborted_navigation_retries_next_candidate_once():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: True)
    original = h.navigate
    def navigate(*pose):
        original(*pose)
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


@pytest.mark.parametrize('timeout_s', [37., 90.])
@pytest.mark.parametrize('completed,status,expected,cancelled', [
    (False, None, False, True), (False, 4, False, True), (False, 6, False, True),
    (True, 4, True, False), (True, 6, False, False), (True, 5, False, False),
])
def test_node_navigation_timeout_and_abort(completed, status, expected, cancelled, timeout_s):
    # Compile the actual method with fake action messages/events, without starting ROS.
    import ast
    from pathlib import Path
    from types import SimpleNamespace as NS
    path = Path(__file__).parents[1] / 'base_athome_skills/restaurant_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RestaurantNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_navigate')
    waits, cancels, pending = [], [], []

    def deliver_result():
        pending.pop()(Future(NS(status=status)))

    class Event:
        def set(self): pass
        def wait(self, timeout):
            waits.append(timeout)
            if len(waits) == 1:
                return True
            if completed:
                deliver_result()
            return completed
    class Future:
        def __init__(self, result, delayed=False):
            self.value, self.delayed = result, delayed
        def result(self): return self.value
        def add_done_callback(self, cb):
            if self.delayed:
                pending.append(cb)
            else:
                cb(self)

    def cancel():
        cancels.append(True)
        # Model a result racing with the cancellation after the wait expired.
        deliver_result()

    goal = NS(accepted=True, get_result_async=lambda: Future(NS(status=status), delayed=True),
              cancel_goal_async=cancel)
    def message():
        return NS(pose=NS(header=NS(), pose=NS(position=NS(), orientation=NS())))
    import threading
    env = dict(math=math, threading=NS(Event=Event), NavigateToPose=NS(Goal=message),
               GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_ABORTED=6))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
    node = NS(get_logger=lambda: NS(info=lambda _: None, warn=lambda _: None),
              nav_client=NS(wait_for_server=lambda **_: True, send_goal_async=lambda _: Future(goal)),
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)),
              _nav_lock=threading.Lock(), navigation_timeout_s=timeout_s)
    assert env['_navigate'](node, 1., 2., 0.) == expected
    assert waits == [10., timeout_s]
    assert bool(cancels) == cancelled
    assert node._nav_aborted == (completed and status == 6)


def test_delivery_aborted_retries_other_free_candidate():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(get_costmap_fn=make_grid, nav_aborted_fn=lambda: True)
    original = h.navigate
    def navigate(*pose):
        original(*pose)
        return len(h.nav_goals) != 3
    session.navigate_fn = navigate
    assert session.run()
    assert h.nav_goals[2] != h.nav_goals[3]
    assert len(h.nav_goals) == 5


def test_delivery_abort_does_not_retry_an_unchecked_fallback():
    h = FakeRestaurantHarness(customers=[{'xy': [2., 0.]}], speech_script=['water', 'ready'])
    session = h.make_session(nav_aborted_fn=lambda: True)
    original = h.navigate

    def navigate(*pose):
        original(*pose)
        if len(h.nav_goals) == 3:
            h.robot_pose = (1., 1., 0.)  # Nav2 aborted partway to the customer.
            return False
        return True

    session.navigate_fn = navigate
    assert not session.run()
    assert len(h.nav_goals) == 4  # No free alternatives without a costmap.
