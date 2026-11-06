#!/usr/bin/env python3
"""Corre pedidos por la API web del commander y espera a que terminen. rc=0 solo si todos dan SUCCEEDED."""
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8770"
FINAL = ("SUCCEEDED", "FAILED", "CANCELED", "ABORTED")


def post(path, **body):
    req = urllib.request.Request(BASE + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(req, timeout=5))


def status():
    return json.load(urllib.request.urlopen(BASE + "/api/status", timeout=5))


def run(text, timeout_s=300.0):
    print(f"\n=== {text}")
    plan = post("/api/plan", text=text)
    if not plan["ok"]:
        print("  RECHAZADO:", plan["errors"])
        return False
    for i, step in enumerate(plan["steps"], 1):
        print(f"  {i}. {step}")
    before = status().get("mission", {}).get("mission_id")
    res = post("/api/run", text=text)
    if not res.get("executed"):
        print("  NO ENVIADO:", res.get("errors"))
        return False
    t0, last = time.time(), None
    while time.time() - t0 < timeout_s:
        m = status().get("mission", {})
        if m.get("mission_id") == before:      # todavia el estado de la mision anterior
            time.sleep(0.1)
            continue
        now = [(s["id"], s["state"]) for s in m.get("steps", []) if s["state"] != "PENDING"]
        if now != last:
            for step, state in now:
                if last is None or (step, state) not in last:
                    done = next(x for x in m["steps"] if x["id"] == step)
                    print(f"  [{time.time() - t0:5.1f}s] {step}: {state} {done.get('message', '')}")
            last = now
        if m.get("state") in FINAL:
            print(f"  FIN: {m['state']} {m.get('message', '')}")
            return m["state"] == "SUCCEEDED"
        time.sleep(0.25)
    print("  TIMEOUT")
    return False


def main():
    t0 = time.time()
    while True:
        try:
            info = status()["info"]
            if "ejecutor conectado" in info:
                break
        except OSError:
            pass
        if time.time() - t0 > 60:
            print("commander/ejecutor no responden")
            return 1
        time.sleep(1)
    results = [run(t) for t in sys.argv[1:]]
    print(f"\n{sum(results)}/{len(results)} pedidos SUCCEEDED")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
