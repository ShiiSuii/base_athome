#!/usr/bin/env python3
"""Ensenar la pose de pre-grasp de piso (alias `ensenar_pregrasp`). Se mueve el brazo A MANO y se guarda.

Es la pose desde la que el robot baja el elevador para agarrar la bolsa, asi que se ensena con el ELEVADOR ARRIBA.
Tiene que quedar: pinza abierta, adelante y hacia abajo, lejos de la base. De esta pose salen donde se para el robot y
cuanto baja el elevador, y el programa rechaza una pose que deje la bolsa demasiado cerca del chasis.

  T  soltar el brazo (SE CAE: sostenelo con la otra mano)      ENTER  guardar y volver a trabarlo
  C  cancelar y trabar                                         Q      salir
Guarda en arm_poses_and_limits.yaml; la pose anterior queda como pre_grasp_floor_previous.
"""
import json
import math
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from base_athome_perception import pose_store
from base_athome_perception.arm_manipulation import CONFIG, GRIPPER_OPEN_MM, SHOULDER_IN_BASE
from base_athome_perception.piper_kinematics import PiperKinematicsSolver
from base_athome_perception.teach_wizard import FLOOR_POSE, TeachWizard
from base_athome_skills.mission import bag_task


def describe(joints_deg, kin, max_advance_m=0.26):
    """(geometria, texto, aceptable) de una pose."""
    tcp_mm, R = kin.forward_kinematics(np.radians(joints_deg))
    tcp = np.asarray(SHOULDER_IN_BASE) + np.asarray(tcp_mm) / 1000.0
    axis = R[:, 2]
    grasp = bag_task.floor_grasp_from_tcp(tcp, axis)
    clearance = bag_task.base_clearance(grasp=grasp)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(axis[2])))))
    bag_z = bag_task.grasp_height(0.30)      # se agarra debajo de la parte mas alta de una bolsa de 30 cm
    torso = bag_task.torso_for_object(bag_z, grasp=grasp)
    advance = bag_task.grasp_advance(tcp, axis, bag_z, grasp)
    problems = []
    if advance > max_advance_m:
        problems.append(f"la pinza tendria que avanzar {advance * 100:.0f} cm y el tope es {max_advance_m * 100:.0f}: bajala")
    if tcp[2] + torso < bag_z - bag_task.GROUND_TO_BASE_LINK_M + 0.05:
        problems.append("con el elevador abajo la pinza quedaria a la altura de la bolsa: subila")
    if clearance < bag_task.MIN_BASE_CLEARANCE_M:
        problems.append(f"la bolsa quedaria a {clearance * 100:.0f} cm del chasis: llevala mas adelante")
    if axis[2] > -0.3:
        problems.append("la pinza no apunta hacia abajo")
    if tcp[0] < bag_task.CHASSIS_FRONT_M:
        problems.append("la pinza queda sobre la base: al bajar chocaria")
    text = (f"pinza a {tcp[0]:.2f} m adelante y {tcp[2]:.2f} m de alto, {tilt:.0f} grados del vertical; bolsa a "
            f"{grasp.reach_x:.2f} m, {clearance * 100:.0f} cm libres del chasis; bolsa de 30 cm: elevador a {torso:+.2f} m y {advance * 100:.0f} cm de avance")
    return grasp, text, problems


class Teacher(Node):
    def __init__(self, out=print, config=CONFIG):
        super().__init__("ensenar_pregrasp")
        self.out, self.config = out, config
        self.joints, self.joints_t, self.gripper, self.feedback = None, 0.0, GRIPPER_OPEN_MM, False
        self.torso = None
        self.kin = PiperKinematicsSolver()
        envelope = pose_store.load_pose_config(config).get("safety_envelope", {})
        self.max_advance_m = float(envelope.get("max_advance_cm", 26.0)) / 100.0
        self.create_subscription(JointState, "/arm/joint_states", self._js, 20)
        self.create_subscription(String, "/arm/status", self._status, 10)
        self.create_subscription(String, "/torso/status", self._torso, 10)
        self.cli = self.create_client(SetBool, "/arm/set_enable")
        self.wizard = TeachWizard(self._set_torque, self._save)

    def _js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(f"joint{i}" in d for i in range(1, 7)):
            self.joints = [math.degrees(d[f"joint{i}"]) for i in range(1, 7)]
            self.joints_t = time.time()

    def _status(self, msg):
        try:
            d = json.loads(msg.data)
            self.feedback = bool(d.get("feedback_ok"))
            self.gripper = float(d.get("gripper_mm", self.gripper))
        except ValueError:
            pass

    def _torso(self, msg):
        try:
            self.torso = json.loads(msg.data).get("position_m")
        except ValueError:
            pass

    def _set_torque(self, on):
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.out("Servicio /arm/set_enable no disponible.")
            return False
        fut = self.cli.call_async(SetBool.Request(data=bool(on)))
        end = time.time() + 5.0
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        ok = fut.done() and fut.result() is not None and fut.result().success
        if not ok:
            self.out("No pude cambiar el estado del brazo.")
        return ok

    def _save(self, name, joints, gripper):
        _, text, problems = describe(joints, self.kin, self.max_advance_m)
        self.out(text)
        if problems:
            return "; ".join(problems)
        res = pose_store.save_named_pose(name, joints, GRIPPER_OPEN_MM, path=self.config, source="ensenar_pregrasp")
        return None if res.valid else (res.error or "pose invalida")

    def fresh(self):
        return self.joints is not None and time.time() - self.joints_t < 1.0

    def start(self):
        if self.torso is not None and self.torso < -0.03:
            self.out(f"El elevador esta en {self.torso:+.2f} m: subilo antes. Esta pose se ensena con el elevador arriba.")
            return False
        if self.torso is None:
            self.out("AVISO: no se la altura del elevador. Comproba a ojo que este ARRIBA.")
        self.wizard.start(only=FLOOR_POSE)
        self.out(self.wizard.message)
        return True

    def confirm(self):
        self.wizard.confirm(self.joints or [], self.gripper, feedback_ok=self.feedback and self.fresh())
        self.out(self.wizard.message)

    def status(self):
        if not self.fresh():
            return "sin datos del brazo"
        _, text, problems = describe(self.joints, self.kin, self.max_advance_m)
        return f"[{'!! ' + problems[0] if problems else 'pose aceptable'}] {text}"


def main():
    rclpy.init()

    def out(text):
        sys.stdout.write("\r\033[K" + str(text).replace("\n", "\r\n") + "\r\n")
        sys.stdout.flush()
    node = Teacher(out=out)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        out(__doc__.split("\n\n", 1)[1])
        while True:
            sys.stdout.write("\r\033[K" + node.status()[:150])
            sys.stdout.flush()
            if not select.select([sys.stdin], [], [], 0.3)[0]:
                continue
            ch = sys.stdin.read(1).lower()
            if ch == "q":
                break
            if ch == "t" and not node.wizard.active:
                node.start()
            elif ch in "\r\n" and node.wizard.active:
                node.confirm()
            elif ch == "c" and node.wizard.active:
                node.wizard.cancel()
                out(node.wizard.message)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        if node.wizard.active:
            print("Sigue en ensenanza: trabando el brazo.")
            node.wizard.cancel()
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
