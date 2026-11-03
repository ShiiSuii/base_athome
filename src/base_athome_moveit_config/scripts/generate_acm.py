#!/usr/bin/env python3
"""Genera la matriz de colisiones permitidas (ACM) del SRDF.

Criterio (como el MoveIt Setup Assistant, HOME-46):
  - Pares brazo-movil <-> cuerpo del robot (base, torso, cubo, pantalla, cabeza): se CHEQUEAN, salvo los
    que ya colisionan en la pose de reposo medida (solapamiento de mallas en el montaje).
  - Pares INTERNOS del brazo: se deshabilitan solo los adyacentes (padre-hijo, incluidas uniones fijas),
    los que colisionan en >= 95 % de las muestras ("Default") y los que nunca colisionan en N muestras
    aleatorias dentro de los limites ("Never"). El resto (p. ej. pinza contra link2/link3) se CHEQUEA.
  - Pares del cuerpo entre si: deshabilitados (no se mueven entre ellos).
Uso:  python3 generate_acm.py robot.urdf base_athome.srdf.in base_athome.srdf [n_muestras]
"""
import itertools
import math
import sys
import xml.etree.ElementTree as ET

from moveit.core import collision_detection
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

ARM_MOVING = ["link2", "link3", "link4", "link5", "link6", "gripper_base", "link7", "link8", "arm_camera_link"]
REST_DEG = [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]


def arm_pairs_by_sampling(urdf, srdf_tmp, arm_links, n_samples):
    """Clasifica los pares internos del brazo por muestreo: (adyacentes, siempre, nunca)."""
    import random
    root = ET.parse(urdf).getroot()
    adjacent = set()
    for j in root.findall("joint"):
        a, b = j.find("parent").get("link"), j.find("child").get("link")
        if a in arm_links and b in arm_links:
            adjacent.add(tuple(sorted((a, b))))
    # hermanos colgados de un mismo link por uniones (gripper_base, link7, link8, camara) tambien son adyacentes
    model = RobotModel(urdf_xml_path=urdf, srdf_xml_path=srdf_tmp)
    scene = PlanningScene(model)
    state = RobotState(model)
    jmg = model.get_joint_model_group("arm")
    limits = []
    for j in root.findall("joint"):
        if j.get("name") in [f"joint{i}" for i in range(1, 7)]:
            lim = j.find("limit")
            limits.append((j.get("name"), float(lim.get("lower")), float(lim.get("upper"))))
    pairs = [tuple(sorted(p)) for p in itertools.combinations(arm_links, 2)]
    counts = {p: 0 for p in pairs}
    acm = scene.allowed_collision_matrix
    all_links = [l.get("name") for l in root.findall("link")]
    for x, y in itertools.combinations(all_links, 2):   # todo permitido; se habilita un par por vez
        acm.set_entry(x, y, True)
    rng = random.Random(0)
    req = collision_detection.CollisionRequest()
    for _ in range(n_samples):
        q = {name: rng.uniform(lo, hi) for name, lo, hi in limits}
        q["joint7"] = rng.uniform(0.0, 0.05)
        q["joint8"] = -q["joint7"]
        state.joint_positions = q
        state.update()
        for p in pairs:
            acm.set_entry(p[0], p[1], False)
            res = collision_detection.CollisionResult()
            scene.check_self_collision(req, res, state, acm)   # devuelve None: el resultado va en res
            if res.collision:
                counts[p] += 1
            acm.set_entry(p[0], p[1], True)
    always = {p for p, c in counts.items() if c >= 0.95 * n_samples}
    never = {p for p, c in counts.items() if c == 0}
    return adjacent, always, never, counts


def main():
    urdf, srdf_in, srdf_out = sys.argv[1:4]
    n_samples = int(sys.argv[4]) if len(sys.argv) > 4 else 300
    root = ET.parse(urdf).getroot()
    robot_name = root.get("name")
    links = [l.get("name") for l in root.findall("link") if l.find("collision") is not None]
    body = [l for l in links if l not in ARM_MOVING and l not in ("arm_base_link", "link1")]
    checked = {tuple(sorted((a, b))) for a in ARM_MOVING for b in body}
    with open(srdf_in) as f:
        base = f.read().replace('<robot name="base_athome">', f'<robot name="{robot_name}">')

    def write(disabled):
        lines = [f'  <disable_collisions link1="{a}" link2="{b}" reason="{r}"/>' for (a, b), r in sorted(disabled.items())]
        with open(srdf_out, "w") as f:
            f.write(base.replace("<!--ACM-->", "\n".join(lines)))

    arm_links = [l for l in ["arm_base_link", "link1"] + ARM_MOVING if l in links]
    arm_internal = {tuple(sorted(p)) for p in itertools.combinations(arm_links, 2)}
    disabled = {tuple(sorted(p)): "Never" for p in itertools.combinations(links, 2)
                if tuple(sorted(p)) not in checked}
    write(disabled)
    adjacent, always, never, counts = arm_pairs_by_sampling(urdf, srdf_out, arm_links, n_samples)
    for p in arm_internal:
        disabled.pop(p, None)
        if p in adjacent:
            disabled[p] = "Adjacent"
        elif p in always:
            disabled[p] = "Default"
        elif p in never:
            disabled[p] = "Never"
    write(disabled)
    checked_internal = sorted(p for p in arm_internal if p not in disabled)
    print(f"pares internos del brazo: {len(arm_internal)}; adyacentes {len(adjacent)}, siempre {len(always - adjacent)}, "
          f"nunca {len(never - adjacent - always)}; CHEQUEADOS {len(checked_internal)}: {checked_internal}")
    model = RobotModel(urdf_xml_path=urdf, srdf_xml_path=srdf_out)
    scene = PlanningScene(model)
    state = RobotState(model)
    state.set_joint_group_positions("arm", [math.radians(d) for d in REST_DEG])
    state.update()

    acm = scene.allowed_collision_matrix
    for x, y in itertools.combinations(links, 2):   # ACM explicito: todo permitido salvo el par evaluado
        acm.set_entry(x, y, True)
    req = collision_detection.CollisionRequest()
    to_test = sorted(set(checked) | set(checked_internal))
    colliding_at_rest = []
    for pair in to_test:
        acm.set_entry(pair[0], pair[1], False)
        res = collision_detection.CollisionResult()
        scene.check_collision(req, res, state, acm)
        if res.collision:
            colliding_at_rest.append(pair)
        acm.set_entry(pair[0], pair[1], True)
    for pair in colliding_at_rest:
        disabled[pair] = "Default"
    write(disabled)
    enabled = sorted(set(to_test) - set(colliding_at_rest))
    body_enabled = [p for p in enabled if p in checked]
    internal_enabled = [p for p in enabled if p in arm_internal]
    print(f"robot {robot_name}: {len(links)} links; CHEQUEADOS brazo-cuerpo: {len(body_enabled)}, "
          f"internos del brazo: {len(internal_enabled)}; en colision en la pose de reposo medida "
          f"(deshabilitados, imprecision del modelo): {colliding_at_rest}")


if __name__ == "__main__":
    main()
