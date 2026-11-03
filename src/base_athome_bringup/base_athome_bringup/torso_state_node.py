#!/usr/bin/env python3
"""Nodo `torso_state`: publica la posicion MEDIDA del elevador. Solo LEE: envia unicamente `?`
(consulta de estado) al ESP32; nunca comandos de movimiento.

  pub /torso/joint_states  sensor_msgs/JointState  torso_lift_joint (solo si el firmware esta CALIBRADO)
  pub /torso/status        std_msgs/String JSON    {connected, calibrated, position_m, steps, max_steps,
                                                    top_hit, bottom_hit, moving, age_s}
Sin calibracion no publica joint_states (no hay posicion inventada). El puerto se abre sin tocar DTR/RTS
para no resetear el ESP32 (perderia la calibracion). Un solo dueno del puerto: no correr junto con
stepper_controller (CLI).
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from base_athome_bringup.torso_state import parse_status

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CBD013609-if00"


def open_port_no_reset(port: str, baud: int = 115200):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 0.05
    s.dtr = False   # antes de open(): evita el pulso de reset del ESP32
    s.rts = False
    s.open()
    return s


class TorsoStateNode(Node):
    def __init__(self, port_factory=open_port_no_reset):
        super().__init__("torso_state")
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("stroke_m", 0.60)
        self.declare_parameter("poll_hz", 5.0)
        self.declare_parameter("stale_s", 1.5)
        self.port_factory = port_factory
        self.ser = None
        self.buf = ""
        self.last = None
        self.last_t = None
        self.pub_js = self.create_publisher(JointState, "/torso/joint_states", 10)
        self.pub_st = self.create_publisher(String, "/torso/status", 10)
        self.create_timer(1.0 / float(self.get_parameter("poll_hz").value), self._tick)

    def _ensure_port(self) -> bool:
        if self.ser is not None:
            return True
        try:
            self.ser = self.port_factory(str(self.get_parameter("port").value))
            self.get_logger().info(f"Elevador conectado en {self.get_parameter('port').value} (solo lectura)")
            return True
        except Exception as e:  # noqa: BLE001 - sin puerto: se informa desconectado
            self.get_logger().warn(f"Elevador sin conexion: {e}", throttle_duration_sec=30.0)
            self.ser = None
            return False

    def _tick(self):
        if self._ensure_port():
            try:
                self.ser.write(b"?\n")
                data = self.ser.read(4096)
                self.buf += data.decode("utf-8", "replace")
                *lines, self.buf = self.buf.split("\n")
                for line in lines:
                    st = parse_status(line)
                    if st is not None:
                        self.last, self.last_t = st, time.monotonic()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Elevador: error de lectura ({e}); reconectando")
                try:
                    self.ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self.ser = None
        self._publish()

    def _publish(self):
        age = None if self.last_t is None else time.monotonic() - self.last_t
        fresh = age is not None and age < float(self.get_parameter("stale_s").value)
        st = self.last if fresh else None
        pos = st.joint_position(float(self.get_parameter("stroke_m").value)) if st else None
        self.pub_st.publish(String(data=json.dumps(dict(
            connected=self.ser is not None, fresh=fresh, calibrated=bool(st and st.calibrated),
            position_m=None if pos is None else round(pos, 4), steps=st.steps if st else None,
            max_steps=st.max_steps if st else None, top_hit=st.top_hit if st else None,
            bottom_hit=st.bottom_hit if st else None, moving=st.moving if st else None,
            age_s=None if age is None else round(age, 2)))))
        if pos is not None:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name, js.position = ["torso_lift_joint"], [float(pos)]
            self.pub_js.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = TorsoStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
