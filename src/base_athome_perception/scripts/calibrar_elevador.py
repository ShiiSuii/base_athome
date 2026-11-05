#!/usr/bin/env python3
"""Ubica el elevador (alias `calibrar_elevador`). Abre torso_state si hace falta.

Primera vez, sin carrera guardada: calibracion completa, BAJA hasta el sensor inferior. Exige el brazo en la pose de piso
(colgando chocaria con la base) y confirmacion. La carrera queda guardada en ~/.config/base_athome/elevador.json.
Despues: solo sube hasta el sensor superior. Si ya esta apoyado ahi, no se mueve.  --completa fuerza la calibracion completa.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from base_athome_perception import pose_store

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config", "arm_poses_and_limits.yaml")
POSE_TOLERANCE_DEG = 12.0
FULL_TIMEOUT_S = 150.0
TOP_TIMEOUT_S = 90.0


def arm_near_pose(joints_deg, pose_deg, tolerance=POSE_TOLERANCE_DEG):
    """(cerca, peor diferencia en grados)."""
    worst = max(abs(a - b) for a, b in zip(joints_deg, pose_deg))
    return worst <= tolerance, worst


class Client(Node):
    def __init__(self):
        super().__init__("calibrar_elevador")
        self.status, self.status_t, self.joints = {}, 0.0, None
        self.create_subscription(String, "/torso/status", self._status, 10)
        self.create_subscription(JointState, "/arm/joint_states", self._joints, 10)
        self.pub_goal = self.create_publisher(Float64, "/torso/goal", 5)
        self.cli_full = self.create_client(Trigger, "/torso/calibrate")
        self.cli_top = self.create_client(Trigger, "/torso/go_top")

    def _status(self, m):
        self.status, self.status_t = json.loads(m.data), time.time()

    def _joints(self, m):
        d = dict(zip(m.name, m.position))
        if all(f"joint{i}" in d for i in range(1, 7)):
            self.joints = [math.degrees(d[f"joint{i}"]) for i in range(1, 7)]

    def spin_for(self, seconds, until=lambda: False):
        end = time.time() + seconds
        while time.time() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.1)
        return until()

    def call(self, client):
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "servicio no disponible"
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        return (res.success, res.message) if res else (False, "sin respuesta")


def main():
    full = "--completa" in sys.argv
    if subprocess.run(["pgrep", "-f", "stepper_controller"], capture_output=True).returncode == 0:
        print("Cerra `stepper` primero (exit): usan el mismo puerto.")
        return 1
    rclpy.init()
    node = Client()
    own = None
    if not node.spin_for(2.5, lambda: node.status):
        print("Abriendo torso_state...")
        own = subprocess.Popen(["ros2", "run", "base_athome_bringup", "torso_state", "--ros-args", "-p", "allow_motion:=true"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        if not node.spin_for(15.0, lambda: node.status.get("fresh")):
            print("El elevador no responde: revisar USB y energia.")
            return 1
        st = node.status
        if st.get("calibrated") and not full:
            print(f"Listo: elevador en {st['position_m']:+.3f} m, carrera guardada {st.get('travel_steps')} pasos. No hizo falta moverlo.")
            return 0
        if st.get("travel_steps") and not full:
            print("Carrera guardada: subo hasta el sensor superior.")
            ok, msg = node.call(node.cli_top)
            if not ok:
                print(f"No pude: {msg}")
                return 1
            if not node.spin_for(TOP_TIMEOUT_S, lambda: node.status.get("calibrated") and not node.status.get("moving")):
                print("No llego al sensor superior a tiempo.")
                return 1
        else:
            pose = pose_store.load_named_poses(CONFIG).get("pre_grasp_floor")
            node.spin_for(3.0, lambda: node.joints is not None)
            if not pose or node.joints is None:
                print("No puedo verificar el brazo o falta enseñar la pose (ensenar_pregrasp). No bajo.")
                return 1
            near, worst = arm_near_pose(node.joints, pose["joints_deg"])
            if not near:
                print(f"El brazo no esta en la pose de piso (hasta {worst:.0f} grados de diferencia). Corre ensenar_pregrasp. No bajo.")
                return 1
            print("Calibracion completa: BAJA hasta el sensor inferior y vuelve a subir. Piso despejado, corte de energia a mano.")
            if input("Escribi si para empezar: ").strip().lower() not in ("si", "sí", "s"):
                print("Cancelado.")
                return 1
            ok, msg = node.call(node.cli_full)
            if not ok:
                print(f"No pude: {msg}")
                return 1
            time.sleep(1.0)
            done = node.spin_for(FULL_TIMEOUT_S, lambda: node.status.get("calibrated") and not node.status.get("moving"))
            if not done:
                print("La calibracion no termino a tiempo.")
                return 1
            node.pub_goal.publish(Float64(data=0.0))   # que quede apoyado en el sensor: la proxima vez no se mueve
            node.spin_for(5.0)
        st = node.status
        print(f"Listo: elevador en {st.get('position_m')} m, carrera guardada {st.get('travel_steps')} pasos.")
        return 0
    except KeyboardInterrupt:
        node.pub_goal.publish(Float64(data=float("nan")))
        print("Detenido.")
        return 1
    finally:
        if own is not None:
            try:
                os.killpg(own.pid, signal.SIGTERM)
            except OSError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
