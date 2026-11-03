#!/usr/bin/env python3
"""`mission_run`: envia un plan a /mission/run, sigue /mission/status y muestra cada paso.

  mission_run plan.yaml [--cancel-after S] [--timeout S] [--json]
Codigo de salida: 0 SUCCEEDED, 1 FAILED/rechazado, 2 CANCELED, 3 sin respuesta.
Se niega a correr en ROS_DOMAIN_ID=10 salvo --real (los planes con navegacion mueven la base).
"""
import argparse
import json
import os
import sys
import time

import yaml


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--cancel-after", type=float, default=None)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--json", action="store_true", help="imprimir el estado final completo en JSON")
    ap.add_argument("--real", action="store_true", help="permitir el dominio 10 (supervision humana)")
    a = ap.parse_args(argv)
    if os.environ.get("ROS_DOMAIN_ID", "0") == "10" and not a.real:
        print("mission_run: dominio 10 (robot real) sin --real: rechazado")
        return 1
    with open(a.plan) as f:
        plan = yaml.safe_load(f)
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init()
    n = rclpy.create_node("mission_run")
    last = {}
    n.create_subscription(String, "/mission/status", lambda m: last.update(s=json.loads(m.data)),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = n.create_publisher(String, "/mission/run", 5)
    cli = n.create_client(Trigger, "/mission/cancel")

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.05)
    t_wait = time.time() + 10
    while pub.get_subscription_count() == 0 and time.time() < t_wait:
        spin(0.1)
    if pub.get_subscription_count() == 0:
        print("mission_run: no hay mission_executor escuchando /mission/run")
        return 3
    prev_id = (last.get("s") or {}).get("mission_id")
    pub.publish(String(data=json.dumps(plan)))
    t0 = time.time()
    shown = {}
    while time.time() - t0 < a.timeout:
        spin(0.2)
        s = last.get("s") or {}
        new = s.get("mission_id") != prev_id or s.get("message", "").startswith("plan rechazado")
        if not new:
            continue
        for st in s.get("steps", []):
            key = (st["index"], st["state"], st["attempts"])
            if shown.get(st["index"]) != key and st["state"] != "PENDING":
                shown[st["index"]] = key
                print(f"  [{st['index']}] {st['id']:<12} {st['skill']:<17} {st['state']:<9} "
                      f"intentos={st['attempts']} dur={st['duration_s']} {st['error'] or ''} {st['message']}")
        if a.cancel_after is not None and time.time() - t0 > a.cancel_after and s.get("state") == "RUNNING":
            f = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=3)
            print("mission_run: cancelacion ->", f.result().message if f.result() else "sin respuesta")
            a.cancel_after = None
        if s.get("state") in ("SUCCEEDED", "FAILED", "CANCELED") and s.get("finished_at"):
            print(f"ESTADO: {s['state']} | {s['message']} | {time.time() - t0:.1f} s")
            if a.json:
                print(json.dumps(s, indent=1))
            return {"SUCCEEDED": 0, "FAILED": 1, "CANCELED": 2}[s["state"]]
    print("mission_run: timeout sin estado final")
    return 3


if __name__ == "__main__":
    sys.exit(main())
