"""Rendered Restaurant participants and honest scripted dialogue share one layout."""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
import numpy as np
from base_athome_sim.restaurant_humans import CUSTOMERS, reply_for_state

ROOT = Path(__file__).parents[1]


def test_world_matches_default_actor_coordinates_and_meshes():
    world = ET.parse(ROOT / 'worlds/restaurant.sdf').getroot()
    humans = [m for m in world.findall('./world/model') if m.get('name', '').startswith('human_')]
    assert len(humans) == len(CUSTOMERS) == 3
    for model, expected in zip(humans, CUSTOMERS):
        assert [float(v) for v in model.findtext('pose').split()[:2]] == list(expected[:2])
        assert model.findtext('static') == 'true'
        assert not model.findall('.//collision')  # no new navigation obstacles
        uri = model.findtext('.//mesh/uri')
        assert (ROOT / 'models/humans' / uri.removeprefix('model://')).is_file()


def test_diner_does_not_impersonate_barman():
    for xy in ([2., 2.8], [3., -2.9], None):
        assert reply_for_state(dict(state='CONFIRMING_BARMAN', info=dict(xy=xy))) == 'robot no'
    assert reply_for_state(dict(state='CONFIRMING_BARMAN', info=dict(xy=[-2.55, .1]))) == 'robot yes'
    assert reply_for_state(dict(state='TAKING_ORDER', customer_idx=0)) == 'my order is coke and water'
    assert reply_for_state(dict(state='TAKING_ORDER', customer_idx=1)) == 'my order is juice and tea'


def test_baked_skin_is_finite_human_scale_and_has_seated_legs(tmp_path):
    path = ROOT / 'models/humans/download_restaurant.py'
    spec = importlib.util.spec_from_file_location('restaurant_rig', path)
    rig = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rig)
    src = path.parent / 'male_visitor/meshes/MaleVisitorWalk.dae'
    standing = rig.bake_pose(src, tmp_path / 'standing.dae', False, 'right')
    seated = rig.bake_pose(src, tmp_path / 'seated.dae', True, 'left')
    for points in (standing, seated):
        assert np.isfinite(points).all()
        assert np.ptp(points[:, 0]) < .9 and np.ptp(points[:, 1]) < 1.0
        assert -.02 < points[:, 2].min() < .08
    assert 1.9 < standing[:, 2].max() < 2.2  # raised hand, not a stretched mesh
    assert 1.5 < seated[:, 2].max() < 1.7
    tree = ET.parse(tmp_path / 'seated.dae')
    assert tree.find('.//{*}instance_controller') is None  # static skin, no animation drift


def test_measured_head_never_reports_targets_or_stale_feedback(monkeypatch):
    from types import SimpleNamespace as NS
    from base_athome_sim.restaurant_humans import apply_measured_head
    import time
    monkeypatch.setattr(time, 'monotonic', lambda: 10.)
    def messages():
        return NS(name=['joint1', 'pan_joint', 'tilt_joint'], position=[.5, 1., 2.]), NS(name=['pan_joint', 'tilt_joint'], position=[1., 2.])
    full, head = messages()
    apply_measured_head(NS(rendered_head=(9.8, dict(pan_joint=.1, tilt_joint=.2))), full, head)
    assert full.position == [.5, .1, .2] and head.position == [.1, .2]
    for sample in (None, (8., dict(pan_joint=.1, tilt_joint=.2))):
        full, head = messages()
        apply_measured_head(NS(rendered_head=sample), full, head)
        assert full.name == ['joint1'] and head.name == []
