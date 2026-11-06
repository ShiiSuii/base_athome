#!/usr/bin/env python3
"""Ventana para ensenar y calibrar la apertura de la puerta (alias `abrir_puerta`).

MUEVE EL BRAZO. Solo se mueve al apretar una tecla. Sostene el brazo al ensenar: sin torque se cae.
Deja el robot a mano frente a la puerta. Todavia no mueve la base ni mide la fuerza: mirala con el E-stop a mano.

  1-5  elegir paso             T  soltar el brazo y ensenar ese paso      ENTER  guardar la pose ensenada
  N    hacer el paso actual    R  hacer lo que queda de la rutina         B      volver al primer paso
  O/C  abrir o cerrar la pinza H  ir a HOME                               S      guardar parametros
  ESC  cancelar o detener      Q  salir (el brazo queda trabado)
"""
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy

from base_athome_perception import door_open, pose_store
from base_athome_perception.arm_manipulation import CONFIG, GRIPPER_CLOSE_MM, GRIPPER_OPEN_MM, Manipulacion

WINDOW = "Abrir puerta"
PARAMS = Path(CONFIG).with_name("door_open.yaml")
GREEN, YELLOW, WHITE, GRAY = (90, 220, 90), (60, 210, 240), (235, 235, 235), (150, 150, 150)


class AbrirPuerta(Manipulacion):
    def __init__(self):
        self.params = door_open.load_params(PARAMS)
        super().__init__()
        self.speed_pct = self.params["speed_pct"]
        self.selected = 0
        self.teaching = False
        self.routine = door_open.DoorRoutine(self.go_to_blocking, self.gripper, self._pause, self.params)
        self.status = "Listo. Ensena las poses con T antes de mover."

    def reload_poses(self):
        super().reload_poses()
        names = {s.pose for s in door_open.STEPS}
        self.poses.update({n: p for n, p in pose_store.load_named_poses(CONFIG).items() if n in names})

    def _pause(self, seconds):
        cv2.waitKey(1)
        threading.Event().wait(seconds)

    def set_param(self, key, value):
        self.params[key] = door_open.clamp_params({key: value})[key]
        if key == "speed_pct":
            self.speed_pct = self.params[key]

    def _background(self, work):
        if self.busy or self.teaching:
            return

        def run():
            self.busy, self.abort, self.routine.abort = True, False, False
            try:
                self.status = work()[1]
            finally:
                self.busy = False
        threading.Thread(target=run, daemon=True).start()

    def _start_teach(self):
        if self.busy:
            return
        if self.set_torque(False):
            self.teaching = True
            step = door_open.STEPS[self.selected]
            self.status = f"BRAZO LIBRE, SOSTENELO. {step.hint} ENTER guarda, ESC cancela."

    def _finish_teach(self, keep):
        with self.lock:
            q, g, ok = list(self.joints_deg), self.gripper_mm, self.feedback_ok
        step = door_open.STEPS[self.selected]
        message = f"Cancelado: {step.label} no cambio."
        if keep:
            error = self.save_pose(step.pose, q, g) if ok else "sin datos reales del brazo"
            message = f"No se guardo: {error}." if error else f"{step.label} guardada."
        self.teaching = False
        self.set_torque(True)
        self.status = message

    def handle(self, key):
        if key in (ord("q"), ord("Q")):
            return False
        if key == 27:
            if self.teaching:
                self._finish_teach(False)
            elif self.busy:
                self.abort = self.routine.abort = True
            return True
        if self.teaching:
            if key in (10, 13):
                self._finish_teach(True)
            return True
        if ord("1") <= key < ord("1") + len(door_open.STEPS):
            self.selected = self.routine.index = key - ord("1")
        elif key in (ord("t"), ord("T")):
            self._start_teach()
        elif key in (ord("n"), ord("N")):
            self._background(self.routine.run_step)
        elif key in (ord("r"), ord("R")):
            missing = door_open.DoorRoutine.missing(self.poses)
            if missing:
                self.status = "Faltan poses: " + ", ".join(missing)
            else:
                self._background(self.routine.run_all)
        elif key in (ord("b"), ord("B")):
            self.routine.restart()
            self.selected = 0
            self.status = "De vuelta en el primer paso."
        elif key in (ord("o"), ord("O")):
            self.gripper(GRIPPER_OPEN_MM)
        elif key in (ord("c"), ord("C")):
            self.gripper(GRIPPER_CLOSE_MM)
        elif key in (ord("h"), ord("H")):
            self._background(lambda: self.go_to_blocking("home"))
        elif key in (ord("s"), ord("S")):
            door_open.save_params(PARAMS, self.params)
            self.status = f"Parametros guardados en {PARAMS.name}."
        return True

    def panel(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            fb, g = self.feedback_ok, self.gripper_mm
        if frame is None:
            frame = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(frame, "Sin imagen de la camara de muneca", (90, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2)
        frame = cv2.resize(frame, (640, 480))
        side = np.full((480, 560, 3), 28, np.uint8)
        put = lambda text, y, color=WHITE, size=0.5: cv2.putText(side, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)
        put("Abrir puerta: mueve el brazo. E-stop a mano.", 26, YELLOW, 0.55)
        y = 58
        for i, step in enumerate(door_open.STEPS):
            done = step.pose in self.poses
            mark = ">" if i == self.routine.index else " "
            put(f"{mark} {i + 1} {step.label}: {'ensenada' if done else 'FALTA'}", y, GREEN if done else YELLOW)
            y += 24
        step = door_open.STEPS[min(self.selected, len(door_open.STEPS) - 1)]
        put(step.hint[:66], y + 6, GRAY)
        put(f"Brazo: {'con datos' if fb else 'SIN DATOS'}   pinza {g:.0f} mm   {'MOVIENDO' if self.busy else 'quieto'}", y + 34)
        p = self.params
        put(f"Velocidad {p['speed_pct']:.0f} %   Agarre {p['grip_mm']:.0f} mm   Pausa {p['pause_s']:.1f} s", y + 58)
        y += 92
        for line in ("1-5 elegir   T ensenar   ENTER guardar   ESC cancelar", "N paso   R rutina   B al principio   H HOME",
                     "O/C pinza   S guardar parametros   Q salir"):
            put(line, y, GRAY)
            y += 22
        status = self.status or ""
        put(status[:68], y + 14, YELLOW)
        put(status[68:136], y + 36, YELLOW)
        return np.hstack([frame, side])


def main():
    rclpy.init()
    node = AbrirPuerta()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1200, 480)
    p = node.params
    cv2.createTrackbar("Velocidad_%", WINDOW, int(p["speed_pct"]), int(door_open.LIMITS["speed_pct"][1]), lambda v: node.set_param("speed_pct", v))
    cv2.createTrackbar("Agarre_mm", WINDOW, int(p["grip_mm"]), int(door_open.LIMITS["grip_mm"][1]), lambda v: node.set_param("grip_mm", v))
    cv2.createTrackbar("Pausa_ds", WINDOW, int(p["pause_s"] * 10), 50, lambda v: node.set_param("pause_s", v / 10.0))
    try:
        while rclpy.ok():
            cv2.imshow(WINDOW, node.panel())
            key = cv2.waitKey(30) & 0xFF
            if key != 255 and not node.handle(key):
                break
    finally:
        if node.teaching:
            node._finish_teach(False)
        node.abort = node.routine.abort = True
        cv2.destroyAllWindows()
        node.stop_move_group()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
