"""Ensenanza Restaurant con TeachPanel y save_pose existentes; SOLO LAB_SUPERVISADO.

Ejecutar con Restaurant/commander/manipulacion cerrados. GRABAR libera el brazo:
sostenerlo antes de tocar el boton. GUARDAR lee feedback real y escribe solamente
el archivo extra. calibrated queda false hasta validar los recorridos completos.
"""

import argparse
from pathlib import Path
import threading

from . import pose_store
from .restaurant_arm import RESTAURANT_POSES, merge_restaurant_poses


def save_pose(name, joints_deg, gripper_mm, *, extra_path, limits_path):
    """Callback del panel; nunca escribe la configuracion principal ni habilita poses."""
    if name not in RESTAURANT_POSES:
        return "Nombre de pose Restaurant invalido"
    if Path(extra_path).resolve() == Path(limits_path).resolve():
        return "El archivo extra no puede ser la configuracion principal del brazo"
    valid = pose_store.validate_joints(joints_deg, path=limits_path)
    if not valid:
        return valid.error
    result = pose_store.save_named_pose(
        name, joints_deg, gripper_mm, path=extra_path,
        source="restaurant_teach_panel", calibrated=False,
    )
    return None if result else result.error


def main():
    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from .arm_manipulation import CONFIG, Manipulacion
    from .teach_panel import TeachPanel

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses-file", default=str(
        Path(get_package_share_directory("base_athome_perception")) / "config/restaurant_arm_poses.yaml"))
    args, ros_args = parser.parse_known_args()

    class Teacher(Manipulacion):
        def save_pose(self, name, joints_deg, gripper_mm):
            return save_pose(name, joints_deg, gripper_mm, extra_path=args.poses_file, limits_path=CONFIG)

    rclpy.init(args=ros_args)
    node = Teacher()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    panel = TeachPanel(width_px=580, poses=RESTAURANT_POSES)
    teaching = None
    window = "Restaurant: ensenar poses (LAB_SUPERVISADO)"

    def action(kind, name):
        nonlocal teaching
        if kind == "teach" and node.feedback_ok:
            if node.set_torque(False):
                teaching = name
        elif kind == "save" and teaching and node.feedback_ok:
            with node.lock:
                q, g = list(node.joints_deg), node.gripper_mm
            err = node.save_pose(teaching, q, g)
            node.status = err or f"{teaching} guardada; calibrated: false"
        elif kind in ("cancel", "torque_on"):
            node.set_torque(True)
            teaching = None
        # IR/atajos de movimiento no se usan para validar poses aun no calibradas.

    def click(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected = panel.handle_click(x, y)
            if selected:
                action(*selected)

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, click)
    try:
        while rclpy.ok():
            poses = merge_restaurant_poses({}, args.poses_file, limits_path=CONFIG)
            canvas = np.zeros((820, 600, 3), dtype=np.uint8)
            panel.render(canvas, 5, 5, dict(
                feedback_ok=node.feedback_ok, torque_on=node.enabled, teaching=teaching,
                current_joints_deg=list(node.joints_deg), current_gripper_mm=node.gripper_mm,
                saved_poses=poses,
            ))
            cv2.putText(canvas, node.status[:85], (10, 800), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (10, 13):
                action("save", None)
            elif key == 27:
                action("cancel", None)
    finally:
        if teaching:
            node.set_torque(True)
        node.stop_move_group()
        rclpy.try_shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
