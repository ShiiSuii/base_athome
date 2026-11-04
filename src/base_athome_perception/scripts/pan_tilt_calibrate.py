#!/usr/bin/env python3
"""Calibrador y prueba del pan-tilt (alias `calibrar_pantilt`). Corre en la NUC, en paralelo a lo demas del robot.

Usa las interfaces de pan_tilt_node (el puerto serie es de ese nodo). La cabeza solo se mueve al tocar una tecla.
Si el receptionist esta abierto, queda en modo lectura para no pelearse por la cabeza.
  Flechas/WASD  mover (A = izquierda DEL ROBOT: de frente a el, se ve al reves)
  +/-  paso      0  ir al centro       T  torque on/off (sin torque se mueve a mano)
  C  marcar centro aca   1/2  pan izquierda/derecha   3/4  tilt arriba/abajo (marcar limites, mejor a mano)
  V  verificar el sentido con la camara  M  prueba completa  G  guardar  H  ayuda  Q  salir (no cambia el torque)
Guardar deja la calibracion en la Jetson (con copia de respaldo) y en el repo. El nodo la lee al arrancar.
"""
import argparse
import math
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from base_athome_perception import pan_tilt_calibration as cal

JETSON = "ubuntu@10.42.0.12"
CAMERA_TOPIC = "/camera/camera/color/image_raw"
REMOTE_YAML = "~/athome_ws/src/pan_tilt_controller/config/pan_tilt_params.yaml"
REPO_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "jetson_head", "src",
                         "pan_tilt_controller", "config", "pan_tilt_params.yaml")
RATE_HZ = 20.0
JOG_DPS = 30.0
SCAN_DPS = 40.0            # velocidad del barrido del receptionist
SCAN_DWELL_S = 0.8
DEFAULTS = dict(pan_center_tick=2059, tilt_center_tick=2048, pan_min_rad=-1.57, pan_max_rad=1.57, tilt_min_rad=-0.4878,
                tilt_max_rad=0.615, ticks_per_rad=651.8986)


class Aborted(Exception):
    pass


class PanTiltCalibrator(Node):
    def __init__(self, use_ssh=True, read_only=False, out=print):
        super().__init__("pan_tilt_calibrator")
        self.use_ssh, self.read_only, self.out = use_ssh, read_only, out
        self.connected, self.drops, self.meas, self.meas_t = None, 0, None, 0.0
        self.torque, self.marks, self.step_deg, self.saved = False, {}, 5.0, False
        self.pub = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 10)
        self.cli = self.create_client(SetBool, "/pan_tilt/set_torque")
        self.create_subscription(JointState, "/pan_tilt/joint_states", self._joints, 20)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/pan_tilt/connected", self._link, latched)
        self.params = dict(DEFAULTS)
        self.params.update(self._read_params())
        self.pan_dir = self._node_direction("pan", 1.0)
        self.tilt_dir = self._node_direction("tilt", -1.0)
        self.limits = cal.logical_limits(self.params, self.pan_dir, self.tilt_dir)
        self.goal = (0.0, 0.0)

    # ------------------------------------------------------------------ estado
    def _joints(self, msg):
        d = dict(zip(msg.name, msg.position))
        if "pan_joint" in d and "tilt_joint" in d:
            self.meas = (math.degrees(d["pan_joint"]), math.degrees(d["tilt_joint"]))
            self.meas_t = time.time()

    def _link(self, msg):
        if self.connected is True and not msg.data:
            self.drops += 1
        self.connected = bool(msg.data)

    def fresh(self):
        return self.meas is not None and time.time() - self.meas_t < 1.0

    def status(self):
        p = f"pan {self.meas[0]:+6.1f} tilt {self.meas[1]:+6.1f}" if self.fresh() else "sin posicion"
        link = {True: "enlace OK", False: "ENLACE CAIDO", None: "sin enlace"}[self.connected]
        return f"{p} | {link} | torque {'ON' if self.torque else 'off'} | paso {self.step_deg:.0f} | caidas {self.drops}"

    # ------------------------------------------------------------------ jetson
    def _ssh(self, command, stdin=None, timeout=10):
        if not self.use_ssh:
            return None
        try:
            r = subprocess.run(["ssh", "-o", "ConnectTimeout=4", JETSON, command], input=stdin, capture_output=True,
                               text=True, timeout=timeout)
            return r.stdout if r.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None

    def _read_params(self):
        # Sin archivo en la Jetson el nodo usa sus valores por defecto: son los que valen, no los del repo
        text = self._ssh(f"cat {REMOTE_YAML} 2>/dev/null || true")
        if text is None and os.path.exists(REPO_YAML):
            with open(REPO_YAML) as f:
                text = f.read()
        try:
            return cal.params_from_yaml(text) if text else {}
        except (KeyError, TypeError, ValueError):
            return {}

    def _node_direction(self, axis, default):
        """El sentido que esta usando el nodo: define como se pasan las marcas a ticks."""
        if not self.use_ssh:
            return default
        try:
            out = subprocess.run(["ros2", "param", "get", "/pan_tilt_node", f"{axis}_direction"], capture_output=True,
                                 text=True, timeout=8).stdout
            return 1.0 if float(out.split()[-1]) >= 0 else -1.0
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            self.out(f"AVISO: no pude leer {axis}_direction del nodo; uso {default:+.0f}.")
            return default

    def usb_drops(self):
        """Desconexiones del FTDI que registro el kernel de la Jetson (None si no se puede leer)."""
        text = self._ssh("sudo -n dmesg | grep -c 'FTDI USB Serial Device converter now attached'")
        try:
            return int(text.strip()) if text is not None else None
        except ValueError:
            return None

    # ------------------------------------------------------------------ movimiento
    def _can_move(self):
        if self.read_only:
            self.out("Modo lectura: el receptionist esta usando la cabeza.")
            return False
        if self.connected is not True or not self.fresh():
            self.out("No hay enlace con los servos: no se manda nada.")
            return False
        if not self.torque:
            self.out("El torque esta apagado: T para encenderlo.")
            return False
        return True

    def set_torque(self, on):
        if self.read_only:
            self.out("Modo lectura: el receptionist esta usando la cabeza.")
            return False
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.out("Servicio /pan_tilt/set_torque no disponible.")
            return False
        fut = self.cli.call_async(SetBool.Request(data=bool(on)))
        end = time.time() + 5.0
        while not fut.done() and time.time() < end:
            time.sleep(0.02)
        res = fut.result() if fut.done() else None
        if res is None or not res.success:
            self.out(f"No se pudo cambiar el torque: {res.message if res else 'sin respuesta'}")
            return False
        self.torque = bool(on)
        if on and self.fresh():
            self.goal = self.meas         # no saltar: el objetivo arranca donde esta
            self._send(self.goal)
        return True

    def _send(self, goal):
        self.pub.publish(Vector3(x=float(goal[0]), y=float(goal[1]), z=0.0))

    def _clip(self, goal):
        return (max(self.limits["pan"][0], min(self.limits["pan"][1], goal[0])),
                max(self.limits["tilt"][0], min(self.limits["tilt"][1], goal[1])))

    def _follow(self, goal, speed, samples=None):
        """Lleva el objetivo a goal a velocidad limitada. Corta si se cae el enlace."""
        goal = self._clip(goal)
        for step in cal.ramp(self.goal, goal, speed, RATE_HZ):
            if self.connected is not True:
                raise Aborted()
            self.goal = step
            self._send(step)
            time.sleep(1.0 / RATE_HZ)
            if samples is not None and self.fresh():
                samples.append((time.time(), step[0], step[1], self.meas[0], self.meas[1]))

    def jog(self, dpan, dtilt):
        if not self._can_move():
            return
        try:
            self._follow((self.goal[0] + dpan, self.goal[1] + dtilt), JOG_DPS)
        except Aborted:
            self.out("Se cayo el enlace: movimiento cortado.")

    def go_center(self):
        if not self._can_move():
            return
        try:
            self._follow((0.0, 0.0), JOG_DPS)
        except Aborted:
            self.out("Se cayo el enlace: movimiento cortado.")

    # ------------------------------------------------------------------ marcas y guardado
    def mark(self, name):
        if not self.fresh():
            self.out("Sin posicion medida: no se puede marcar.")
            return
        if name == "center":
            self.marks["pan_center"], self.marks["tilt_center"] = self.meas
            self.out(f"Centro marcado en pan {self.meas[0]:+.1f}, tilt {self.meas[1]:+.1f}. G guarda.")
        else:
            axis = 0 if name.startswith("pan") else 1
            self.marks[name] = self.meas[axis]
            self.out(f"{name} marcado en {self.meas[axis]:+.1f} grados. G guarda.")

    def new_params(self):
        return cal.updated_params(self.params, self.marks, self.pan_dir, self.tilt_dir)

    def save(self):
        if not self.marks:
            self.out("No hay nada marcado para guardar.")
            return False
        try:
            new = self.new_params()
        except ValueError as e:
            self.out(f"No guardo: {e}")
            return False
        text = cal.params_to_yaml(new)
        self.out("Parametros nuevos: " + ", ".join(f"{k}={new[k]}" for k in sorted(new) if new[k] != self.params.get(k)))
        stamp = time.strftime("%Y%m%d_%H%M%S")
        cmd = (f"mkdir -p $(dirname {REMOTE_YAML}) && {{ [ ! -f {REMOTE_YAML} ] || cp -p {REMOTE_YAML} "
               f"{REMOTE_YAML}.bak_{stamp}; }} && cat > {REMOTE_YAML}")
        if self.use_ssh and self._ssh(cmd, stdin=text) is None:
            self.out("No pude escribir en la Jetson: no se guardo nada.")
            return False
        with open(REPO_YAML, "w") as f:
            f.write(text)
        self.saved = True
        self.out(f"Guardado (respaldo .bak_{stamp}). El nodo lo lee al arrancar: se aplica al reiniciar jetson-head. "
                 "Antes, sostene o baja el brazo: se deshabilita al reiniciar.")
        return True

    # ------------------------------------------------------------------ prueba
    def full_test(self, ask=None):
        """Sentido, escalones, onda (como el seguimiento) y el barrido del receptionist. Devuelve el reporte."""
        if not self._can_move():
            return None
        usb0, drops0 = self.usb_drops(), self.drops
        steps, wrong, samples, aborted = [], [], [], False
        try:
            self.out("1/3 Escalones y sentido...")
            self._follow((0.0, 0.0), JOG_DPS)
            for goal in ((15.0, 0.0), (-15.0, 0.0), (0.0, 0.0), (0.0, 10.0), (0.0, -10.0), (0.0, 0.0)):
                start = self.meas
                self._follow(goal, JOG_DPS)
                time.sleep(0.8)
                if not self.fresh():
                    raise Aborted()
                steps.append(max(abs(self.goal[0] - self.meas[0]), abs(self.goal[1] - self.meas[1])))
                moved = (self.meas[0] - start[0], self.meas[1] - start[1])
                want = (self.goal[0] - start[0], self.goal[1] - start[1])
                for i, axis in enumerate(("pan", "tilt")):
                    if abs(want[i]) > 5.0 and moved[i] * want[i] < 0 and axis not in wrong:
                        wrong.append(axis)
            self.out("2/3 Onda lenta, como el seguimiento de rostro (10 s)...")
            t0 = time.time()
            while time.time() - t0 < 10.0:
                t = time.time() - t0
                g = self._clip((25.0 * math.sin(2 * math.pi * 0.2 * t), 8.0 * math.sin(2 * math.pi * 0.1 * t)))
                self.goal = g
                if self.connected is not True:
                    raise Aborted()
                self._send(g)
                time.sleep(1.0 / RATE_HZ)
                if self.fresh():
                    samples.append((time.time(), g[0], g[1], self.meas[0], self.meas[1]))
            self.out("3/3 Barrido del receptionist (~20 s)...")
            for view in cal.scan_path(self.limits):
                self._follow(view, SCAN_DPS, samples=samples)
                time.sleep(SCAN_DWELL_S)
        except Aborted:
            aborted = True
            self.out("Se cayo el enlace: prueba cortada.")
        usb1 = self.usb_drops()
        usb = max(0, usb1 - usb0) if usb0 is not None and usb1 is not None else 0
        stats = cal.tracking_stats(samples)
        lines = cal.verdict(steps, stats, self.drops - drops0, usb, wrong)
        report = dict(aborted=aborted, steps=steps, stats=stats, link_drops=self.drops - drops0, usb_drops=usb,
                      usb_known=usb0 is not None and usb1 is not None, wrong=wrong, verdict=lines)
        self.out(f"Resultado: error final max {max(steps) if steps else float('nan'):.1f} grados | seguimiento rms "
                 f"{stats['rms_deg']:.1f}, retardo {stats['lag_s']:.2f} s | caidas enlace {report['link_drops']}, "
                 f"USB {usb if report['usb_known'] else 'sin dato'}")
        for line in lines:
            self.out("  -> " + line)
        return report

    # ------------------------------------------------------------------ sentido real, medido con la camara
    def _image(self, timeout=3.0):
        """Una imagen tomada DESPUES de este llamado (None si no llega)."""
        got = []

        def on_image(msg):
            if not got and msg.encoding in ("rgb8", "bgr8", "mono8"):
                got.append(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1).squeeze().copy())
        sub = self.create_subscription(Image, CAMERA_TOPIC, on_image, qos_profile_sensor_data)
        end = time.time() + timeout
        while not got and time.time() < end:
            time.sleep(0.02)
        self.destroy_subscription(sub)
        return got[0] if got else None

    def _measure_sense(self, axis, amount):
        """Mueve un eje, compara la imagen y vuelve. Mantener la escena quieta. Devuelve ok, inverted o unknown."""
        start = self.goal
        results = []
        for delta in (amount, -amount):
            target = self._clip((start[0] + delta, start[1]) if axis == "pan" else (start[0], start[1] + delta))
            command = target[0] - start[0] if axis == "pan" else target[1] - start[1]
            if abs(command) < 3.0:
                continue
            time.sleep(0.5)
            before = self._image()
            self._follow(target, JOG_DPS)
            time.sleep(0.8)
            after = self._image()
            self._follow(start, JOG_DPS)
            if before is None or after is None:
                results.append("unknown")
                continue
            dx, dy, conf = cal.image_shift(before, after)
            shift = dx if axis == "pan" else dy
            results.append(cal.sense_from_shift(command, shift, conf, axis))
            self.out(f"  {axis} {command:+.0f} grados: imagen {shift:+.0f} px (confianza {conf:.2f}) -> {results[-1]}")
        known = [r for r in results if r != "unknown"]
        if len(known) == len(results) and known and len(set(known)) == 1:
            return known[0]
        return "unknown"

    def verify_sense(self):
        """El sentido real respecto del robot, sin depender de desde donde se mire."""
        if not self._can_move():
            return None
        self.out("Verificando el sentido con la camara: que nadie se mueva delante (unos 15 s)...")
        result = {}
        try:
            for axis, amount in (("pan", 10.0), ("tilt", 6.0)):
                result[axis] = self._measure_sense(axis, amount)
        except Aborted:
            self.out("Se cayo el enlace: verificacion cortada.")
            return None
        text = {"ok": "CORRECTO", "inverted": "INVERTIDO", "unknown": "sin dato (repetir con la escena quieta y con textura)"}
        for axis in result:
            self.out(f"{axis}: {text[result[axis]]}")
        if result.get("pan") == "ok":
            self.out("Pan correcto: A gira a la izquierda del robot. Si lo mirabas de frente, se ve al reves; no hay que cambiar nada.")
        if "inverted" in result.values():
            self.out("Hay que cambiar pan_direction/tilt_direction del nodo: el seguimiento de rostro iria al reves.")
        return result

    # ------------------------------------------------------------------ teclado
    def key(self, ch):
        s = self.step_deg
        moves = {"a": (s, 0), "d": (-s, 0), "w": (0, s), "s": (0, -s), "\x1b[D": (s, 0), "\x1b[C": (-s, 0),
                 "\x1b[A": (0, s), "\x1b[B": (0, -s)}
        marks = {"1": "pan_left", "2": "pan_right", "3": "tilt_up", "4": "tilt_down", "c": "center"}
        if ch in moves:
            self.jog(*moves[ch])
        elif ch in marks:
            self.mark(marks[ch])
        elif ch == "t":
            if self.torque:
                self.out("Apagando el torque: sostene la cabeza.")
            if self.set_torque(not self.torque):
                self.out("Torque " + ("encendido." if self.torque else "apagado."))
        elif ch == "0":
            self.go_center()
        elif ch in "+=":
            self.step_deg = min(20.0, s + 1.0)
        elif ch == "-":
            self.step_deg = max(1.0, s - 1.0)
        elif ch == "m":
            self.full_test()
        elif ch == "v":
            self.verify_sense()
        elif ch == "g":
            self.save()
        elif ch == "h":
            self.out(__doc__.split("\n\n", 1)[1])


def _read_key():
    ch = sys.stdin.read(1)
    if ch == "\x1b" and select.select([sys.stdin], [], [], 0.02)[0]:
        ch += sys.stdin.read(2)
    return ch.lower() if len(ch) == 1 else ch


def main():
    ap = argparse.ArgumentParser(description="Calibrador y prueba del pan-tilt")
    ap.parse_args()
    receptionist = subprocess.run(["pgrep", "-f", "hri_receptionist_challenge"], capture_output=True).returncode == 0

    def out(text):
        sys.stdout.write("\r\033[K" + text.replace("\n", "\r\n") + "\r\n")
        sys.stdout.flush()

    rclpy.init()
    node = PanTiltCalibrator(read_only=receptionist, out=out)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    old = termios.tcgetattr(sys.stdin)
    worker = None
    try:
        tty.setcbreak(sys.stdin.fileno())
        out(__doc__.split("\n\n", 1)[1])
        time.sleep(1.0)
        if node.pub.get_subscription_count() == 0:
            out("AVISO: pan_tilt_node no esta escuchando (revisar jetson-head).")
        while True:
            sys.stdout.write("\r\033[K" + node.status())
            sys.stdout.flush()
            if select.select([sys.stdin], [], [], 0.2)[0]:
                ch = _read_key()
                if ch == "q":
                    break
                if worker is not None and worker.is_alive():
                    continue                # una accion por vez; la prueba completa no se interrumpe con otra tecla
                worker = threading.Thread(target=node.key, args=(ch,), daemon=True)
                worker.start()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
        if node.marks and not node.saved:
            print("Quedaron marcas sin guardar.")
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
