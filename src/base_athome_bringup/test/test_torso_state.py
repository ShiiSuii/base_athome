"""HOME-43: parseo del estado del elevador y nodo de solo lectura con un puerto falso."""
import json
import os
import time

import pytest

from base_athome_bringup.torso_state import parse_status

CAL = ("[STATUS] Modo: IDLE | Accion: SUBIENDO (Hacia Pin 7) | Retraso: 250 us (4000 pasos/s | ~150.0 RPM) | "
       "Limites: [TOP(Pin7)=LIBRE, BOT(Pin8)=LIBRE] | Restante: 0 | Pos: 75000 / 150000 [CALIBRADO]")
UNCAL = ("[STATUS] Modo: IDLE | Accion: BAJANDO (Hacia Pin 8) | Retraso: 250 us (4000 pasos/s | ~150.0 RPM) | "
         "Limites: [TOP(Pin7)=PULSADO, BOT(Pin8)=LIBRE] | Restante: 0 | Pos: 0")


def test_parse_calibrated_maps_to_urdf_joint():
    st = parse_status(CAL)
    assert st.calibrated and st.steps == 75000 and st.max_steps == 150000
    assert st.joint_position(0.60) == pytest.approx(-0.30)
    top = parse_status(CAL.replace("Pos: 75000", "Pos: 150000"))
    assert top.joint_position() == pytest.approx(0.0)
    assert parse_status(CAL.replace("Pos: 75000", "Pos: 0")).joint_position() == pytest.approx(-0.60)


def test_uncalibrated_has_no_position():
    st = parse_status(UNCAL)
    assert st is not None and not st.calibrated and st.top_hit
    assert st.joint_position() is None


def test_garbage_is_ignored():
    assert parse_status("[OK] Velocidad fijada: 250 us") is None


class FakePort:
    def __init__(self, line):
        self.line, self.written = line, []

    def write(self, b):
        self.written.append(b)

    def read(self, n):
        return (self.line + "\n").encode() if self.written else b""

    def close(self):
        pass


@pytest.mark.parametrize("line,expect_js", [(CAL, True), (UNCAL, False)])
def test_node_publishes_only_measured_position_and_only_queries(line, expect_js):
    rclpy = pytest.importorskip("rclpy")
    assert os.environ.get("ROS_DOMAIN_ID") != "10"
    from base_athome_bringup.torso_state_node import TorsoStateNode
    rclpy.init()
    try:
        port = FakePort(line)
        node = TorsoStateNode(port_factory=lambda _p: port)
        js, st = [], []
        node.pub_js.publish = js.append
        node.pub_st.publish = st.append
        node._tick()
        node._tick()
        assert set(port.written) == {b"?\n"}                 # solo consultas, nunca movimiento
        status = json.loads(st[-1].data)
        assert status["connected"] and status["fresh"]
        assert bool(js) is expect_js
        if expect_js:
            assert js[-1].name == ["torso_lift_joint"] and js[-1].position[0] == pytest.approx(-0.30)
        else:
            assert status["calibrated"] is False and status["position_m"] is None
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_node_without_port_reports_disconnected():
    rclpy = pytest.importorskip("rclpy")
    from base_athome_bringup.torso_state_node import TorsoStateNode
    rclpy.init()
    try:
        def boom(_p):
            raise OSError("no existe")
        node = TorsoStateNode(port_factory=boom)
        st, js = [], []
        node.pub_st.publish = st.append
        node.pub_js.publish = js.append
        node._tick()
        assert json.loads(st[-1].data)["connected"] is False and js == []
        node.destroy_node()
    finally:
        rclpy.shutdown()
