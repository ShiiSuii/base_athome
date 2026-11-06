"""Tests de la lógica pura de seguimiento de personas (HOME-92).

Valida trayectorias sintéticas sin ROS ni Nav2:
1. persona quieta
2. caminando (recta y giro)
3. cruza otra persona
4. se oculta 3 s
5. desaparece
6. llegada a destino ('llegó')
7. constantes y límites de calibración
8. desempaquetado e interfaces heterogéneas
"""

import math
import pytest

from base_athome_skills.mission.follow_logic import (
    DEFAULT_FOLLOW_DISTANCE_M,
    FOLLOW_DISTANCE_TOLERANCE_M,
    GATE_RADIUS_M,
    HEAD_PAN_DEADBAND_DEG,
    LEAD_TIME_S,
    LOST_GRACE_TIME_S,
    MAX_HEAD_PAN_DEG,
    MAX_LINEAR_VEL_MPS,
    MIN_HEAD_PAN_DEG,
    MIN_LINEAR_VEL_MPS,
    PERSON_STATIONARY_SPEED_MPS,
    SEARCH_TIMEOUT_S,
    DetectedPerson,
    FollowOutput,
    FollowState,
    PersonFollowerLogic,
    RobotPose,
    follow_person_step,
)


def test_persona_quieta():
    """Escenario 1: Persona quieta a 2.5 m al frente.

    El robot debe mantener estado 'siguiendo', fijar el objetivo a 1.1 m
    de la persona (x=1.4 m), mantener la cabeza al centro (pan ~ 0°)
    y estimar velocidad cercana a cero.
    """
    follower = PersonFollowerLogic(target_color="blue")
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # Simular 30 ciclos a 10 Hz (3.0 segundos)
    dt = 0.10
    out = None
    for step in range(30):
        t = step * dt
        person = DetectedPerson(x=2.5, y=0.0, clothing_color="blue")
        out = follower.step([person], robot, timestamp=t, dt=dt)

        assert out.state == FollowState.SIGUIENDO
        assert out.state == "siguiendo"
        assert out.nav_goal is not None

    # Verificaciones finales tras convergencia del filtro
    assert out is not None
    goal_x, goal_y, goal_yaw = out.nav_goal
    # Meta a 1.1 m de la persona (2.5 - 1.1 = 1.4 m)
    assert abs(goal_x - 1.40) < 0.05
    assert abs(goal_y - 0.0) < 0.05
    assert abs(goal_yaw - 0.0) < 0.05

    # Cabezal apuntando al frente
    assert abs(out.head_pan_deg) < 3.0

    # Velocidad estimada de la persona quieta < umbral estacionario
    vx, vy = out.target_vel
    assert math.hypot(vx, vy) < PERSON_STATIONARY_SPEED_MPS


def test_caminando():
    """Escenario 2: Persona caminando en línea recta a 0.30 m/s.

    El robot debe seguir a la persona manteniendo la distancia objetivo,
    estimar la velocidad forward (~0.30 m/s) con anticipación predictiva,
    y luego girar la cabeza si la persona realiza un desvío lateral.
    """
    follower = PersonFollowerLogic(target_color="blue")
    dt = 0.10
    v_person = 0.30  # m/s
    v_robot = 0.25  # m/s (robot avanza siguiendo)

    robot_x = 0.0
    out = None

    # Tramo 1: Caminata recta a lo largo del eje X (40 pasos, 4.0 segundos)
    for step in range(40):
        t = step * dt
        px = 1.5 + v_person * t
        robot_x = min(px - DEFAULT_FOLLOW_DISTANCE_M, v_robot * t)
        robot = RobotPose(x=robot_x, y=0.0, yaw=0.0)

        out = follower.step([DetectedPerson(x=px, y=0.0, clothing_color="blue")], robot, timestamp=t, dt=dt)
        assert out.state == FollowState.SIGUIENDO

    assert out is not None
    # Velocidad estimada debe haber convergido cerca de 0.30 m/s
    vx, vy = out.target_vel
    assert abs(vx - 0.30) < 0.05
    assert abs(vy) < 0.05

    # La meta debe estar adelantada con lead_time respecto a la posición actual
    current_px = 1.5 + v_person * (39 * dt)
    goal_x, goal_y, _ = out.nav_goal
    # pred_x = current_px + 0.30 * 0.40 = current_px + 0.12
    # goal_x = pred_x - 1.10
    expected_goal_x = (current_px + 0.30 * LEAD_TIME_S) - DEFAULT_FOLLOW_DISTANCE_M
    assert abs(goal_x - expected_goal_x) < 0.08

    # Tramo 2: Giro lateral (persona empieza a desplazarse hacia +Y a 0.25 m/s)
    # El cabezal debe girar a la izquierda (pan positivo)
    py = 0.0
    for step in range(40, 55):
        t = step * dt
        py += 0.25 * dt
        px = 1.5 + v_person * t
        robot = RobotPose(x=robot_x, y=0.0, yaw=0.0)

        out = follower.step([DetectedPerson(x=px, y=py, clothing_color="blue")], robot, timestamp=t, dt=dt)
        assert out.state == FollowState.SIGUIENDO

    # Pan debe haber respondido girando hacia +Y (pan > 0)
    assert out.head_pan_deg > 5.0
    assert out.head_pan_deg <= MAX_HEAD_PAN_DEG


def test_cruza_otra_persona():
    """Escenario 3: Otra persona cruza la trayectoria del objetivo.

    El anfitrión viste ropa azul y camina sobre X.
    Un transeúnte viste ropa roja y cruza transversalmente sobre Y
    pasando exactamente por el mismo punto donde se encuentra el anfitrión.
    El seguidor no debe confundirse ni cambiar de objetivo hacia el intruso.
    """
    follower = PersonFollowerLogic(target_color="blue")
    dt = 0.10
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # 50 pasos (5.0 segundos)
    # Host camina de x=2.0 a x=3.5 sobre y=0.0 (vx = 0.30 m/s)
    # Stranger cruza de y=-1.5 a y=+1.5 sobre x=2.6 (vy = 0.60 m/s)
    # En t ~ 2.0 s (paso 20), ambos coinciden en (2.6, 0.0)!
    for step in range(50):
        t = step * dt
        host_x = 2.0 + 0.30 * (t if t < 2.0 else 2.0 + 0.10 * (t - 2.0))
        host_y = 0.0
        stranger_x = 2.6
        stranger_y = -1.5 + 0.60 * t

        host = DetectedPerson(x=host_x, y=host_y, clothing_color="blue", track_id=1)
        stranger = DetectedPerson(x=stranger_x, y=stranger_y, clothing_color="red", track_id=2)

        # Alternar el orden en la lista para evitar sesgos de índice
        people = [stranger, host] if step % 2 == 0 else [host, stranger]

        out = follower.step(people, robot, timestamp=t, dt=dt)

        assert out.state == FollowState.SIGUIENDO
        # La posición rastreada debe pertenecer siempre al anfitrión (y ~ 0.0)
        assert out.target_pos is not None
        assert abs(out.target_pos[1]) < 0.20, f"Se desvió hacia el intruso en paso {step}: y={out.target_pos[1]}"

    # Al final, el intruso está en y=+1.5, pero el anfitrión sigue en y=0.0
    assert abs(out.target_pos[1]) < 0.10
    assert out.target_pos[0] > 2.2


def test_se_oculta_3_segundos():
    """Escenario 4: La persona se oculta durante exactamente 3.0 segundos y reaparece.

    Fase 1 (t=0.0 a 1.0s): Detección activa -> estado 'siguiendo'.
    Fase 2 (t=1.0 a 4.0s, 3s de oclusión):
      - Primeros 0.6s: tiempo de gracia / inercial.
      - 0.6s a 3.0s: estado 'buscando' (NO 'perdida', ya que 3s < SEARCH_TIMEOUT 5s).
      - El robot navega hacia la última posición conocida y barre el cabezal.
    Fase 3 (t=4.1s en adelante): La persona reaparece -> vuelve a 'siguiendo'.
    """
    follower = PersonFollowerLogic(target_color="blue", search_timeout_s=5.0, lost_grace_time_s=0.6)
    dt = 0.10
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # 1. Fase visible (1.0 s)
    last_visible_x = 3.0
    for step in range(10):
        t = step * dt
        person = DetectedPerson(x=last_visible_x, y=0.0, clothing_color="blue")
        out = follower.step([person], robot, timestamp=t, dt=dt)
        assert out.state == FollowState.SIGUIENDO

    # 2. Fase de oclusión (30 pasos = 3.0 segundos, de t=1.0 a t=4.0)
    for step in range(10, 40):
        t = step * dt
        out = follower.step([], robot, timestamp=t, dt=dt)

        t_lost = t - 0.90  # tiempo transcurrido desde la última detección
        if t_lost > LOST_GRACE_TIME_S:
            assert out.state == FollowState.BUSCANDO, f"En t={t:.1f} (t_lost={t_lost:.2f}) debía ser 'buscando'"
        # Bajo ninguna circunstancia debe pasar a 'perdida' antes de los 5.0 s
        assert out.state != FollowState.PERDIDA, f"Marcó 'perdida' prematuramente a los {t_lost:.2f} s"

    # En el estado 'buscando', la meta apunta a la última posición conocida
    assert out.state == FollowState.BUSCANDO
    assert out.nav_goal is not None
    goal_x, goal_y, _ = out.nav_goal
    assert abs(goal_x - last_visible_x) < 0.10
    assert abs(goal_y - 0.0) < 0.10

    # 3. Fase de reaparición en t = 4.1 s
    reappear_person = DetectedPerson(x=3.15, y=0.05, clothing_color="blue")
    out_reappear = follower.step([reappear_person], robot, timestamp=4.1, dt=dt)

    assert out_reappear.state == FollowState.SIGUIENDO
    assert out_reappear.target_pos is not None
    assert abs(out_reappear.target_pos[0] - 3.15) < 0.15


def test_desaparece():
    """Escenario 5: La persona desaparece definitivamente.

    Tras superar SEARCH_TIMEOUT_S (5.0 s sin detección), el estado debe
    transicionar irreversiblemente a 'perdida', cancelando el nav_goal (None)
    y regresando el pan a neutro (0°).
    """
    follower = PersonFollowerLogic(target_color="blue", search_timeout_s=5.0, lost_grace_time_s=0.6)
    dt = 0.10
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # 1. Detección inicial breve (0.5 s)
    for step in range(5):
        t = step * dt
        person = DetectedPerson(x=2.0, y=0.0, clothing_color="blue")
        follower.step([person], robot, timestamp=t, dt=dt)

    # 2. Desaparición prolongada: simular hasta 7.0 s (6.5 s sin detección)
    out = None
    for step in range(5, 75):
        t = step * dt
        out = follower.step([], robot, timestamp=t, dt=dt)

        t_lost = t - 0.40
        if t_lost >= 5.0:
            assert out.state == FollowState.PERDIDA
            assert out.state == "perdida"
            # Sin objetivo de navegación
            assert out.nav_goal is None
            # Cabezal en neutro
            assert abs(out.head_pan_deg) < 1e-3

    assert out is not None
    assert out.state == FollowState.PERDIDA


def test_llego_destino():
    """Escenario 6: Llegada exitosa al destino ('llegó').

    Cuando se fija un destino (destination), la persona se aproxima a él
    (distancia <= ARRIVAL_DISTANCE_TOLERANCE_M) y se detiene durante al menos
    ARRIVAL_STOP_TIME_S (2.0 s), el estado debe cambiar a 'llegó'.
    """
    dest_x, dest_y = 5.0, 0.0
    follower = PersonFollowerLogic(
        target_color="blue",
        destination=(dest_x, dest_y),
        arrival_stop_time_s=2.0,
        arrival_distance_tolerance_m=1.2,
    )
    dt = 0.10
    robot = RobotPose(x=3.5, y=0.0, yaw=0.0)

    # 1. Persona se desplaza hacia el destino
    for step in range(20):
        t = step * dt
        px = 3.0 + 0.10 * step  # llega a 4.9 m
        out = follower.step([DetectedPerson(x=px, y=0.0, clothing_color="blue")], robot, timestamp=t, dt=dt)
        assert out.state == FollowState.SIGUIENDO

    # 2. Persona se detiene en (4.9, 0.0) durante 2.5 segundos (25 pasos)
    out_arrived = None
    for step in range(20, 50):
        t = step * dt
        # Persona quieta en x=4.9
        out_arrived = follower.step([DetectedPerson(x=4.9, y=0.0, clothing_color="blue")], robot, timestamp=t, dt=dt)

        time_stopped = (step - 20) * dt
        if time_stopped >= 2.0:
            assert out_arrived.state == FollowState.LLEGO
            assert out_arrived.state == "llegó"
            assert out_arrived.state == "llego"  # Comprobación de robustez sin acento

    assert out_arrived is not None
    assert out_arrived.state == FollowState.LLEGO
    # Robot mantiene posición encarando a la persona
    goal_x, goal_y, goal_yaw = out_arrived.nav_goal
    assert abs(goal_x - robot.x) < 1e-3
    assert abs(goal_y - robot.y) < 1e-3


def test_constantes_y_calibracion():
    """Escenario 7: Validación de constantes marcadas de calibración y límites físicos."""
    # Límites de velocidad según especificación de ROBOT.md y Nav2
    assert MAX_LINEAR_VEL_MPS == 0.38
    assert MIN_LINEAR_VEL_MPS == 0.08
    assert DEFAULT_FOLLOW_DISTANCE_M == 1.10
    assert MAX_HEAD_PAN_DEG == 65.0
    assert MIN_HEAD_PAN_DEG == -65.0
    assert LEAD_TIME_S == 0.40
    assert LOST_GRACE_TIME_S == 0.60
    assert SEARCH_TIMEOUT_S == 5.00

    follower = PersonFollowerLogic()
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # 1. Clamping del cabezal ante objetivo a 90° (debe acotarse a MAX_HEAD_PAN_DEG 65°)
    person_side = DetectedPerson(x=0.0, y=3.0, clothing_color="blue")
    out = follower.step([person_side], robot, timestamp=0.0, dt=0.1)
    assert out.head_pan_deg == MAX_HEAD_PAN_DEG

    # Objetivo a -90° (debe acotarse a MIN_HEAD_PAN_DEG -65°)
    follower.reset()
    person_other_side = DetectedPerson(x=0.0, y=-3.0, clothing_color="blue")
    out_neg = follower.step([person_other_side], robot, timestamp=0.0, dt=0.1)
    assert out_neg.head_pan_deg == MIN_HEAD_PAN_DEG

    # 2. Zona muerta (deadband): ángulo pequeño (< 2°) debe dar 0.0°
    follower.reset()
    # x=10.0, y=0.1 -> ángulo = atan2(0.1, 10.0) = 0.01 rad = ~0.57° < 2.0°
    person_deadband = DetectedPerson(x=10.0, y=0.1, clothing_color="blue")
    out_db = follower.step([person_deadband], robot, timestamp=0.0, dt=0.1)
    assert out_db.head_pan_deg == 0.0


def test_desempaquetado_e_interfaces():
    """Escenario 8: Interfaces heterogéneas (dict, tuple, dataclass) y desempaquetado."""
    follower = PersonFollowerLogic(target_color="green")

    # 1. Desempaquetado directo como tupla de 3 elementos
    people_dict = [{"x": 2.0, "y": 0.0, "color": "green"}]
    robot_tuple = (0.0, 0.0, 0.0)

    nav_goal, head_pan, state = follower.step(people_dict, robot_tuple, timestamp=0.0, dt=0.1)

    assert nav_goal is not None
    assert isinstance(head_pan, float)
    assert state == "siguiendo"

    # 2. Tuplas en personas y dicts en robot
    follower.reset()
    people_tuple = [(2.2, 0.5, "green")]
    robot_dict = {"x": 0.1, "y": 0.0, "yaw": 0.1}

    out = follower.step(people_tuple, robot_dict, timestamp=0.1, dt=0.1)
    assert out.state == FollowState.SIGUIENDO
    assert out.target_pos is not None
    assert abs(out.target_pos[0] - 2.2) < 1e-3

    # 3. Función de conveniencia follow_person_step
    goal, pan, st = follow_person_step(people_tuple, robot_dict, target_color="green")
    assert goal is not None
    assert st == "siguiendo"


def test_inicio_sin_detecciones_y_barrido():
    """Escenario 9: Inicio sin detecciones previas.

    Debe arrancar en 'buscando', ejecutar barrido del cabezal, y pasar a 'perdida'
    si se agota SEARCH_TIMEOUT_S sin detectar a nadie.
    """
    follower = PersonFollowerLogic(search_timeout_s=3.0)
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # Paso 0: Sin detecciones -> buscando con barrido
    out0 = follower.step([], robot, timestamp=0.0, dt=0.1)
    assert out0.state == FollowState.BUSCANDO
    assert out0.nav_goal is None

    # Avanzar hasta 3.5 s sin detecciones
    out_timeout = None
    for step in range(1, 35):
        t = step * 0.10
        out_timeout = follower.step([], robot, timestamp=t, dt=0.10)

    assert out_timeout is not None
    assert out_timeout.state == FollowState.PERDIDA
    assert out_timeout.nav_goal is None


def test_distancia_seguridad_proximidad():
    """Escenario 10: Persona demasiado cerca (dentro de distancia de seguimiento).

    El robot no debe avanzar hacia la persona si la distancia es menor a follow_distance_m;
    debe mantener su posición actual para evitar colisión con los talones.
    """
    follower = PersonFollowerLogic(target_color="blue", follow_distance_m=1.10)
    # Persona a solo 0.70 m al frente
    person = DetectedPerson(x=0.70, y=0.0, clothing_color="blue")
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    out = follower.step([person], robot, timestamp=0.0, dt=0.1)
    assert out.state == FollowState.SIGUIENDO
    assert out.nav_goal is not None

    goal_x, goal_y, _ = out.nav_goal
    # Meta debe mantenerse en la posición actual del robot (no avanzar)
    assert abs(goal_x - robot.x) < 1e-3
    assert abs(goal_y - robot.y) < 1e-3


def test_robustez_color_ropa():
    """Escenario 11: Robustez ante mayúsculas, espacios y subtipos de color."""
    follower = PersonFollowerLogic(target_color="blue")
    robot = RobotPose(x=0.0, y=0.0, yaw=0.0)

    # Coincidencia con "Dark Blue" y espacios
    person_dark = DetectedPerson(x=2.0, y=0.0, clothing_color=" Dark Blue ")
    out = follower.step([person_dark], robot, timestamp=0.0, dt=0.1)
    assert out.state == FollowState.SIGUIENDO

    # Intruso con "Red" no es seleccionado aunque esté más cerca
    follower.reset()
    follower.target_color = "blue"
    person_red = DetectedPerson(x=1.2, y=0.0, clothing_color="RED")
    person_blue = DetectedPerson(x=2.0, y=0.0, clothing_color="light blue")

    out_dual = follower.step([person_red, person_blue], robot, timestamp=0.0, dt=0.1)
    assert out_dual.state == FollowState.SIGUIENDO
    assert abs(out_dual.target_pos[0] - 2.0) < 0.1
