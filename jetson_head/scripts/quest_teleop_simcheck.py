#!/usr/bin/env python3
"""Validacion de punta a punta del modo real de quest_teleop_node contra el BRAZO SIMULADO.

Se niega a correr fuera del dominio 42: manda mensajes como los del Quest y verifica las 4 garantias:
sin X no se mueve; con X va a HOME; con X sigue la mano en la direccion correcta; al soltar X frena.
  (Jetson) ROS_DOMAIN_ID=42 python3 quest_teleop_node.py --ros-args -p real:=true ...
           ROS_DOMAIN_ID=42 python3 quest_teleop_simcheck.py        # codigo de salida 0 = todo OK
"""
import json
import os
import sys
import threading
import time
import urllib.request

import numpy as np
import rclpy
from std_msgs.msg import String

import quest_cartesian_core as qc

HOME = [73.87, 169.72, -138.02, 26.69, -15.7, -117.17]
URL = "http://127.0.0.1:8765/state"


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        print("simcheck: solo en ROS_DOMAIN_ID=42 (nunca contra el robot real)")
        return 2
    rclpy.init()
    n = rclpy.create_node("quest_simcheck")
    arm, stat = {}, {}
    n.create_subscription(String, "/arm/status", lambda m: arm.update(json.loads(m.data)), 10)
    n.create_subscription(String, "/quest_teleop/status", lambda m: stat.update(json.loads(m.data)), 10)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    time.sleep(1.5)
    if not arm.get("sim"):
        print("simcheck: /arm/status no viene del simulador: abortado")
        return 2

    def post(x, xyz=(0.0, 1.0, 0.0), grip=0.0):
        d = json.dumps({"v": 7, "xr": True, "hand": "left", "axes": [0, 0, 0, 0], "buttons": [0, grip, 0, 0, x, 0, 0, 0],
                        "pose": {"p": list(xyz), "q": [0, 0, 0, 1]}}).encode()
        urllib.request.urlopen(urllib.request.Request(URL, d, {"Content-Type": "application/json"}), timeout=2).read()

    def hold(sec, *a, **k):
        t = time.time()
        while time.time() - t < sec:
            post(*a, **k)
            time.sleep(0.02)

    dist = lambda: max(abs(a - b) for a, b in zip(arm["joints_deg"], HOME))
    moved = lambda ref: max(abs(a - b) for a, b in zip(arm["joints_deg"], ref))
    tcp = lambda: qc.fk(np.radians(arm["joints_deg"]))[0]
    ok = True
    j0 = list(arm["joints_deg"])
    hold(1.5, 0.0)
    r = moved(j0) < 0.5
    ok &= r
    print(f"1) sin X no se mueve: {'OK' if r else 'FALLA'}")
    t = time.time()
    while time.time() - t < 120 and dist() > 1.5:
        hold(0.1, 1.0)
    hold(0.5, 1.0)
    r = dist() < 2.0
    ok &= r
    print(f"2) con X va a HOME: {'OK' if r else 'FALLA'} ({time.time() - t:.1f} s, {dist():.1f} grados)")
    hold(0.3, 0.0)
    p0 = tcp()
    hold(0.4, 1.0)
    for k in range(100):                    # 4 cm adelante en 2 s (2 cm/s)
        hold(0.02, 1.0, (0.0, 1.0, -0.04 * (k + 1) / 100.0))
    hold(1.0, 1.0, (0.0, 1.0, -0.04))
    d = tcp() - p0
    print(f"   pinza: x {d[0]:+.1f} y {d[1]:+.1f} z {d[2]:+.1f} mm | track {stat.get('track_mm')} mm | {str(stat.get('reason'))[:45]}")
    r = 30.0 < d[0] < 50.0 and abs(d[1]) < 8.0 and abs(d[2]) < 8.0
    ok &= r
    print(f"3) con X sigue la mano (40 mm adelante): {'OK' if r else 'FALLA'}")
    hold(0.15, 0.0, (0.0, 1.0, -0.10))      # soltar y seguir moviendo la mano
    jr = list(arm["joints_deg"])
    hold(1.0, 0.0, (0.0, 1.0, -0.10))
    r = moved(jr) < 0.5
    ok &= r
    print(f"4) al soltar X frena: {'OK' if r else 'FALLA'}")
    print("RESULTADO:", "TODO OK" if ok else "HAY FALLAS")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
