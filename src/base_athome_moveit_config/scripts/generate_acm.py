#!/usr/bin/env python3
"""Genera la matriz de colisiones permitidas (ACM) del SRDF.

Criterio (conservador y simple):
  - Se CHEQUEAN solo los pares brazo-movil <-> cuerpo del robot (base, torso, cubo, pantalla,
    cabeza): es lo que protege al robot de pegarse a si mismo.
  - Se deshabilitan todos los pares internos del brazo (la cinematica/limites del Piper ya
    evitan autocolisiones) y los del cuerpo entre si (no se mueven).
  - De los pares brazo-cuerpo, se deshabilitan los que YA colisionan en la pose de reposo
    medida (solapamiento de mallas en el montaje), probando par por par.
Uso:  python3 generate_acm.py robot.urdf base_athome.srdf.in base_athome.srdf
"""
import itertools
import math
import sys
import xml.etree.ElementTree as ET

from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

ARM_MOVING = ["link2", "link3", "link4", "link5", "link6", "gripper_base", "link7", "link8", "arm_camera_link"]
REST_DEG = [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]


def main():
    urdf, srdf_in, srdf_out = sys.argv[1:4]
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

    disabled = {tuple(sorted(p)): "Never" for p in itertools.combinations(links, 2)
                if tuple(sorted(p)) not in checked}
    write(disabled)
    model = RobotModel(urdf_xml_path=urdf, srdf_xml_path=srdf_out)
    scene = PlanningScene(model)
    state = RobotState(model)
    state.set_joint_group_positions("arm", [math.radians(d) for d in REST_DEG])
    state.update()

    acm = scene.allowed_collision_matrix
    colliding_at_rest = []
    for pair in sorted(checked):
        for other in checked:  # permitir todo menos este par
            acm.set_entry(other[0], other[1], other != pair)
        if scene.is_state_colliding(robot_state=state, joint_model_group_name="arm"):
            colliding_at_rest.append(pair)
    for pair in colliding_at_rest:
        disabled[pair] = "Default"
    write(disabled)
    enabled = sorted(checked - set(colliding_at_rest))
    print(f"robot {robot_name}: {len(links)} links; chequeados brazo-cuerpo: {len(enabled)}; "
          f"en colision en reposo (deshabilitados): {colliding_at_rest}")


if __name__ == "__main__":
    main()
