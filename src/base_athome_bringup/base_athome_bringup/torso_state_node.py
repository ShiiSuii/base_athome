#!/usr/bin/env python3
"""Nodo `torso_state`: publica la posicion MEDIDA del elevador. Por defecto solo LEE (`?`); con allow_motion:=true
tambien lo mueve, siempre con la posicion medida, a traves de /torso/goal y de los sensores del firmware.

  sub /torso/goal          std_msgs/Float64      torso_lift_joint objetivo (0 arriba, negativo abajo); NaN = parar
  srv /torso/calibrate, /torso/go_top            calibracion completa; subir hasta el sensor superior
La carrera medida queda en ~/.config/base_athome/elevador.json. Con ella, tocar el sensor superior ya ubica al elevador.

  pub /torso/joint_states  sensor_msgs/JointState  torso_lift_joint (solo si el firmware esta CALIBRADO)
  pub /torso/status        std_msgs/String JSON    {connected, calibrated, position_m, steps, max_steps,
                                                    top_hit, bottom_hit, moving, age_s}
Sin calibracion no publica joint_states (no hay posicion inventada). El puerto se abre sin tocar DTR/RTS
para no resetear el ESP32 (perderia la calibracion). Un solo dueno del puerto: no correr junto con
stepper_controller (CLI).
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from base_athome_bringup.torso_state import TorsoReference, parse_status

MEMORY_FILE = os.environ.get("ATHOME_TORSO_MEMORY") or os.path.expanduser("~/.config/base_athome/elevador.json")
TOP_GOAL_M = -0.001        # pedir "arriba" es ir hasta el sensor: asi la proxima vez se sabe la posicion sin moverse
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
        self.declare_parameter("allow_motion", False)
        self.port_factory = port_factory
        self.memory_file = MEMORY_FILE
        self.ref = TorsoReference(self._load_travel())
        self.ser = None
        self.buf = ""
        self.last = None
        self.last_t = None
        self.pub_js = self.create_publisher(JointState, "/torso/joint_states", 10)
        self.pub_st = self.create_publisher(String, "/torso/status", 10)
        self.create_subscription(Float64, "/torso/goal", self._goal, 5)
        self.create_service(Trigger, "/torso/calibrate", self._calibrate)
        self.create_service(Trigger, "/torso/go_top", self._go_top)
        self.create_timer(1.0 / float(self.get_parameter("poll_hz").value), self._tick)

    def _load_travel(self):
        try:
            with open(self.memory_file, encoding="utf-8") as f:
                return int(json.load(f)["max_steps"])
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _save_travel(self):
        os.makedirs(os.path.dirname(self.memory_file), exist_ok=True)
        tmp = self.memory_file + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(dict(max_steps=self.ref.max_steps, measured=time.strftime("%Y-%m-%dT%H:%M:%S")), f)
        os.replace(tmp, self.memory_file)
        self.get_logger().info(f"Carrera del elevador guardada: {self.ref.max_steps} pasos")

    def _calibrate(self, _req, res):
        """Calibracion completa del firmware: recorre hasta los dos sensores. El puerto es de este nodo."""
        res.success = bool(self.get_parameter("allow_motion").value) and self.ser is not None
        if res.success:
            self.ser.write(b"CALIBRATE\n")
            res.message = "calibrando: el elevador recorre hasta los dos sensores"
        else:
            res.message = "no se calibra: allow_motion=false o sin conexion"
        return res

    def _go_top(self, _req, res):
        """Sube hasta el sensor superior, donde el firmware frena solo. Con la carrera guardada, alcanza para ubicarse."""
        res.success = bool(self.get_parameter("allow_motion").value) and self.ser is not None
        if res.success:
            self.ser.write(b"F\n")
            res.message = "subiendo hasta el sensor superior"
        else:
            res.message = "no se mueve: allow_motion=false o sin conexion"
        return res

    def _goal(self, msg: Float64):
        if not bool(self.get_parameter("allow_motion").value):
            self.get_logger().warn("Pedido de mover el elevador ignorado: allow_motion=false")
            return
        if self.ser is None:
            self.get_logger().warn("Pedido de mover el elevador ignorado: sin conexion")
            return
        if math.isnan(msg.data):          # parar siempre se manda, este como este el estado
            self.ser.write(b"S\n")
            return
        fresh = self.last_t is not None and time.monotonic() - self.last_t < float(self.get_parameter("stale_s").value)
        steps = self.ref.steps_to(self.last, msg.data, float(self.get_parameter("stroke_m").value)) if fresh else None
        if steps is None:
            self.get_logger().warn("Pedido de mover el elevador ignorado: sin estado reciente o sin ubicar")
            return
        if self.last.moving:
            self.ser.write(b"S\n")
            time.sleep(0.05)
        if msg.data >= TOP_GOAL_M:
            if not self.last.top_hit:
                self.get_logger().info("Elevador arriba: hasta el sensor")
                self.ser.write(b"F\n")
            return
        if steps:
            self.get_logger().info(f"Elevador a {msg.data:+.3f} m: M {steps}")
            self.ser.write(f"M {steps}\n".encode())

    def _ensure_port(self) -> bool:
        if self.ser is not None:
            return True
        try:
            self.ser = self.port_factory(str(self.get_parameter("port").value))
            self.get_logger().info(f"Elevador conectado en {self.get_parameter('port').value}; "
                                   f"movimiento {'PERMITIDO' if self.get_parameter('allow_motion').value else 'desactivado'}")
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
                        if self.ref.observe(st):
                            self._save_travel()
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
        pos = self.ref.joint_position(st, float(self.get_parameter("stroke_m").value)) if st else None
        self.pub_st.publish(String(data=json.dumps(dict(
            connected=self.ser is not None, fresh=fresh, calibrated=pos is not None,
            travel_steps=self.ref.max_steps,
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
