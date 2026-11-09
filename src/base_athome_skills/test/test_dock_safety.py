"""Frescura pura y handlers con reloj/publicador falsos, sin iniciar ROS."""

import threading
from types import MethodType, SimpleNamespace

import pytest

from base_athome_skills.mission.dock_safety import sensor_freshness_error


@pytest.mark.parametrize("now,scan_t,odom_t,expected", [
    (0.0, 0.0, 0.0, None),
    (0.3, 0.0, 0.0, None),
    (10.0, 9.8, 9.9, None),
    (10.0, None, 10.0, "no_scan"),
    (10.0, 10.0, None, "no_odom"),
    (10.0, None, None, "no_scan"),
    (0.300001, 0.0, 0.3, "stale_scan"),
    (0.300001, 0.3, 0.0, "stale_odom"),
    (0.300001, 0.0, 0.0, "stale_scan"),
    (10.0, 10.1, 10.0, "stale_scan"),
    (10.0, 10.0, 10.1, "stale_odom"),
    (10.0, float("nan"), 10.0, "stale_scan"),
    (10.0, 10.0, float("inf"), "stale_odom"),
    (float("nan"), 10.0, 10.0, "stale_scan"),
])
def test_sensor_freshness_error(now, scan_t, odom_t, expected):
    assert sensor_freshness_error(now, scan_t, odom_t) == expected


@pytest.fixture
def docking(monkeypatch):
    # Solo se importan las dependencias: nunca rclpy.init() ni Node.__init__().
    module = pytest.importorskip("base_athome_skills.mission.node")
    cls = module.MissionExecutorNode
    clock = SimpleNamespace(now=0.0)
    commands, frozen = [], set()
    robot = SimpleNamespace(
        last_scan=None, odom_xy=None, scan_received_at=None, odom_received_at=None,
        _dock_sensor_lock=threading.Lock(), table_docked_m=0.0,
        pub_cmd_vel=SimpleNamespace(publish=lambda msg: commands.append(
            (clock.now, msg.linear.x, msg.angular.z))),
    )
    robot._dock_sensor_snapshot = MethodType(cls._dock_sensor_snapshot, robot)
    robot._stop_base = MethodType(cls._stop_base, robot)
    scan = SimpleNamespace(
        angle_min=0.0, angle_increment=0.1, ranges=[1.2], range_min=0.06, range_max=5.0,
    )
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0))))

    def receive(sensor):
        if sensor == "scan":
            cls._scan(robot, scan)
        else:
            cls._odom(robot, odom)

    def advance(seconds):
        clock.now += seconds
        for sensor in {"scan", "odom"} - frozen:
            receive(sensor)

    monkeypatch.setattr(module, "time", SimpleNamespace(
        monotonic=lambda: clock.now, time=lambda: clock.now, sleep=advance,
    ))
    monkeypatch.setattr(module.front_dock, "front_dock_step", lambda *a, **kw: ("go", 1.0))
    monkeypatch.setattr(module.table_dock, "table_dock_step", lambda *a, **kw: ("go", 1.0))
    receive("scan")
    receive("odom")
    ctx = SimpleNamespace(args={}, cancel=threading.Event(), remaining=lambda: 10.0 - clock.now)
    return SimpleNamespace(
        module=module, cls=cls, robot=robot, ctx=ctx, clock=clock,
        commands=commands, frozen=frozen, advance=advance,
    )


@pytest.mark.parametrize("handler", ["h_dock_front", "h_dock_table", "h_undock_table"])
@pytest.mark.parametrize("frozen,expected", [
    ({"scan"}, "stale_scan"),
    ({"odom"}, "stale_odom"),
    ({"scan", "odom"}, "stale_scan"),
])
def test_handlers_frenan_al_congelarse_cualquier_sensor(docking, handler, frozen, expected):
    docking.frozen.update(frozen)

    result = getattr(docking.cls, handler)(docking.robot, docking.ctx)

    assert not result.ok
    assert result.error == expected
    assert expected in result.message
    assert len(docking.commands) > 1
    assert all(vx != 0.0 for _, vx, _ in docking.commands[:-1])
    stop_at, vx, wz = docking.commands[-1]
    assert (vx, wz) == (0.0, 0.0)
    # El control de 50 ms frena en el primer ciclo vencido, sin esperar al timeout.
    assert 0.3 < stop_at <= 0.35 + 1e-9
    assert stop_at - 0.3 < 0.3


@pytest.mark.parametrize("handler", ["h_dock_front", "h_dock_table", "h_undock_table"])
@pytest.mark.parametrize("sensor", ["scan", "odom"])
def test_handlers_sin_sensor_frenan_sin_mover(docking, handler, sensor):
    setattr(docking.robot, f"{sensor}_received_at", None)
    setattr(docking.robot, "last_scan" if sensor == "scan" else "odom_xy", None)

    result = getattr(docking.cls, handler)(docking.robot, docking.ctx)

    assert result.error == f"no_{sensor}"
    assert docking.commands == [(0.0, 0.0, 0.0)]


@pytest.mark.parametrize("handler,algorithm", [
    ("h_dock_front", "front_dock"), ("h_dock_table", "table_dock"),
])
def test_decision_que_envejece_durante_el_calculo_no_mueve(docking, monkeypatch, handler, algorithm):
    def slow_step(*args, **kwargs):
        # Incluso si entran mensajes nuevos, la decisión usa el snapshot anterior.
        docking.advance(0.31)
        return "go", 1.0

    monkeypatch.setattr(getattr(docking.module, algorithm), f"{algorithm}_step", slow_step)

    result = getattr(docking.cls, handler)(docking.robot, docking.ctx)

    assert result.error == "stale_scan"
    assert docking.commands == [(0.31, 0.0, 0.0)]
