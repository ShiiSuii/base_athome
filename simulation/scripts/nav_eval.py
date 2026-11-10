#!/usr/bin/env python3
"""Compara perfiles de Nav2 en Gazebo (mundo nav_eval: puerta de 0.85 m). SOLO SIMULACION, ROS_DOMAIN_ID=42.

  simulation/scripts/nav_eval.py                 # todos los escenarios con el perfil del robot y con 'eval'
  simulation/scripts/nav_eval.py persona eval    # un escenario y un perfil ('robot' = nav2_params.yaml sin cambios)
  ATHOME_SIM_ARENA=1 simulation/scripts/nav_eval.py puerta robot
      los mismos tres escenarios en la casa completa (worlds/arena.sdf): puerta living-comedor de 0.90 m y camino
      entre las mesas hasta la barra.

Por corrida: si llego, cuanto tardo y lo mas cerca que paso del centro de la persona. Guarda ~/.local/share/base_athome/nav_eval.json
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
OUT = Path.home() / ".local" / "share" / "base_athome" / "nav_eval.json"
FAR = "4.6,2.6"          # persona fuera del camino
SCENARIOS = {
    # cruzar la puerta de 0.85 m sin nadie
    "puerta": dict(command="go to afuera", person=FAR),
    # una persona quieta en el medio del camino, con lugar para rodearla
    "persona": dict(command="go to este", person="2.0,0.0"),
    # una persona parada en la puerta, que se corre a los 12 s
    "se_corre": dict(command="go to afuera", person="-3.0,0.0", walk="-2.2,1.8", walk_after_s="12"),
}
PROFILES = ("robot", "altura", "eval")
ARENA = os.environ.get("ATHOME_SIM_ARENA") == "1"
ARENA_FAR = "-2.5,0.5"   # en el hall, fuera del camino
ARENA_SCENARIOS = {
    # del living a la barra del comedor: puerta de 0.90 m y pasillo entre las mesas, sin nadie
    "puerta": dict(command="go to the bar", person=ARENA_FAR),
    # una persona quieta en el medio del living, con lugar para rodearla
    "persona": dict(command="go to the dining room", person="2.5,0.2"),
    # una persona parada en la puerta living-comedor, que entra al comedor a los 12 s
    # (5.9,1.4) pisa el inflado de la silla 1; el waypoint dining_room queda libre.
    "se_corre": dict(command="go to the bar", person="5.0,0.5", walk="5.8,1.3", walk_after_s="12"),
}
if ARENA:
    SCENARIOS, FAR = ARENA_SCENARIOS, ARENA_FAR


class Recorder:
    """Lee /odom y /sim/people en el dominio 42 y guarda la menor distancia robot-persona."""

    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        rclpy.init()
        self.rclpy, self.node = rclpy, rclpy.create_node("nav_eval_recorder")
        self.robot, self.person, self.min_d = None, None, None
        self.node.create_subscription(Odometry, "/odom", self._odom, 20)
        self.node.create_subscription(String, "/sim/people", self._people, 5)
        self.stop = False
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        while not self.stop:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def _odom(self, m):
        self.robot = (m.pose.pose.position.x, m.pose.pose.position.y)
        self._update()

    def _people(self, m):
        people = json.loads(m.data).get("people", [])
        self.person = tuple(people[0]["xy"][:2]) if people else None

    def _update(self):
        if self.robot and self.person:
            d = math.hypot(self.robot[0] - self.person[0], self.robot[1] - self.person[1])
            self.min_d = d if self.min_d is None else min(self.min_d, d)

    def close(self):
        self.stop = True
        time.sleep(0.3)
        self.node.destroy_node()
        self.rclpy.shutdown()


def run(name, profile):
    sc = SCENARIOS[name]
    world = dict(ATHOME_SIM_WORLD="arena.sdf", ATHOME_SIM_MAP=str(WS / "simulation/config/arena.yaml"),
                 ATHOME_SIM_SEMANTIC_MAP=str(WS / "simulation/base_athome_sim/config/arena_semantic_map.yaml"),
                 ATHOME_SIM_BAG="-2.8,1.5,0.30") if ARENA else \
        dict(ATHOME_SIM_WORLD="nav_eval.sdf", ATHOME_SIM_MAP=str(WS / "simulation/config/nav_eval.yaml"),
             ATHOME_SIM_SEMANTIC_MAP=str(WS / "simulation/base_athome_sim/config/nav_eval_semantic_map.yaml"),
             ATHOME_SIM_BAG="4.6,-2.6,0.30")       # la bolsa, lejos: sin ella el simulador no publica /sim/people
    env = dict(os.environ, GAZEBO="1", NAV2="1", SIM_DOMAIN="42", ROS_DOMAIN_ID="42", **world,
               ATHOME_SIM_PERSON=sc["person"], ATHOME_SIM_PERSON_POINTS="0",
               ATHOME_SIM_PERSON_WALK=sc.get("walk", ""), ATHOME_SIM_PERSON_WALK_AFTER_S=sc.get("walk_after_s", "0"),
               ATHOME_NAV_PROFILE="" if profile == "robot" else profile)
    os.environ["ROS_DOMAIN_ID"] = "42"
    rec = Recorder()
    t0 = time.time()
    proc = subprocess.run(["nice", "-n", "5", str(HERE / "sim_commander.sh"), sc["command"]], env=env,
                          capture_output=True, text=True, timeout=900)
    rec.close()
    tail = [l for l in proc.stdout.splitlines() if "FIN:" in l or "FAILED" in l or "SUCCEEDED" in l][-3:]
    return dict(escenario=name, perfil=profile, mundo="arena" if ARENA else "nav_eval", ok=proc.returncode == 0,
                segundos=round(time.time() - t0),
                min_persona_m=None if rec.min_d is None or sc["person"] == FAR else round(rec.min_d, 2), detalle=tail)


def main(argv):
    names = [a for a in argv if a in SCENARIOS] or list(SCENARIOS)
    profiles = [a for a in argv if a in PROFILES] or list(PROFILES)
    results = []
    for name in names:
        for profile in profiles:
            print(f">>> {name} con perfil {profile}", flush=True)
            r = run(name, profile)
            results.append(r)
            print(f"    {'LLEGO' if r['ok'] else 'NO LLEGO'} en {r['segundos']} s; persona a {r['min_persona_m']} m; {r['detalle']}",
                  flush=True)
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=WS, capture_output=True, text=True).stdout.strip()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(dict(commit=head, cuando=time.strftime("%Y-%m-%d %H:%M"), filas=results), indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
