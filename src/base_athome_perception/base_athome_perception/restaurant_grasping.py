"""Guardar las poses del brazo de Restaurant leyendo donde esta el brazo ahora. No mueve nada.

Uso (alias restaurantgrasping):
    restaurantgrasping              muestra las 3 poses y la posicion actual
    restaurantgrasping pregrasp     guarda la posicion actual como restaurant_pregrasp
    restaurantgrasping postgrasp    ... restaurant_postgrasp
    restaurantgrasping handover     ... restaurant_handover

Llevar el brazo a la posicion (p. ej. con el boton de ensenanza del Piper) y despues
guardar. Guardar es la decision humana de usar esa pose: queda calibrated: true.
"""

import json
import sys
import time
from pathlib import Path

from . import pose_store

POSES = {"pregrasp": "restaurant_pregrasp", "postgrasp": "restaurant_postgrasp",
         "handover": "restaurant_handover"}
SRC_CONFIG = Path(__file__).resolve().parent.parent / "config"


def _config_dir():
    """config/ del codigo fuente (el install apunta ahi con symlink-install)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("base_athome_perception")) / "config"
    except Exception:  # noqa: BLE001 - fuera de ROS: el arbol fuente
        return SRC_CONFIG


def read_arm(timeout_s=5.0):
    """(joints_deg, gripper_mm) del ultimo /arm/status con feedback valido, o None."""
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("restaurant_grasping")
    got = {}

    def cb(msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get("feedback_ok") and len(st.get("joints_deg", [])) >= 6:
            got["arm"] = ([float(v) for v in st["joints_deg"][:6]], float(st.get("gripper_mm", 0.0)))

    node.create_subscription(String, "/arm/status", cb, 10)
    end = time.time() + timeout_s
    while "arm" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    return got.get("arm")


def save(short, joints_deg, gripper_mm, config_dir):
    """Valida contra los limites del brazo y guarda en restaurant_arm_poses.yaml. Devuelve error o None."""
    limits = config_dir / "arm_poses_and_limits.yaml"
    valid = pose_store.validate_joints(joints_deg, path=limits)
    if not valid:
        return valid.error
    result = pose_store.save_named_pose(
        POSES[short], joints_deg, gripper_mm, path=config_dir / "restaurant_arm_poses.yaml",
        source="restaurantgrasping", calibrated=True,
    )
    return None if result else result.error


def show(config_dir, arm):
    poses = pose_store.load_named_poses(config_dir / "restaurant_arm_poses.yaml")
    for short, name in POSES.items():
        p = poses.get(name, {})
        mark = "LISTA" if p.get("calibrated") is True else "falta"
        print(f"  {short:10s} {mark:6s} {p.get('joints_deg')}  pinza {p.get('gripper_mm')} mm")
    if arm:
        print(f"  ahora      brazo {[round(v, 1) for v in arm[0]]}  pinza {arm[1]:.0f} mm")
    else:
        print("  ahora      sin datos del brazo (/arm/status sin feedback)")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    config_dir = _config_dir()
    if argv and argv[0] not in POSES:
        print(f"Pose desconocida: {argv[0]}. Usar: {', '.join(POSES)}")
        return 2
    arm = read_arm()
    if not argv:
        show(config_dir, arm)
        return 0
    if arm is None:
        print("No llega /arm/status con feedback: ¿esta corriendo el controlador del brazo en la Jetson?")
        return 1
    err = save(argv[0], arm[0], arm[1], config_dir)
    if err:
        print(f"No se guardo: {err}")
        return 1
    print(f"Guardada {POSES[argv[0]]}: {[round(v, 1) for v in arm[0]]}, pinza {arm[1]:.0f} mm")
    show(config_dir, arm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
