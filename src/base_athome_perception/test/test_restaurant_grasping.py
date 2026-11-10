"""restaurantgrasping: guarda la posicion actual como pose Restaurant calibrada, validando limites."""

import shutil
from pathlib import Path

from base_athome_perception import pose_store
from base_athome_perception.restaurant_grasping import save

CONFIG = Path(__file__).resolve().parent.parent / "config"


def _copy(tmp_path):
    for name in ("arm_poses_and_limits.yaml", "restaurant_arm_poses.yaml"):
        shutil.copy(CONFIG / name, tmp_path / name)
    return tmp_path


def test_saves_current_pose_as_calibrated_only_in_restaurant_file(tmp_path):
    cfg = _copy(tmp_path)
    main_before = (cfg / "arm_poses_and_limits.yaml").read_text()
    assert save("pregrasp", [10.0, 150.0, -90.0, 0.0, 20.0, -40.0], 97.0, cfg) is None
    pose = pose_store.load_named_poses(cfg / "restaurant_arm_poses.yaml")["restaurant_pregrasp"]
    assert pose["calibrated"] is True and pose["joints_deg"] == [10.0, 150.0, -90.0, 0.0, 20.0, -40.0]
    assert (cfg / "arm_poses_and_limits.yaml").read_text() == main_before


def test_rejects_pose_outside_arm_limits(tmp_path):
    cfg = _copy(tmp_path)
    assert save("handover", [999.0, 0, 0, 0, 0, 0], 50.0, cfg)
