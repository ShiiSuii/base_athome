"""Depth holes on a person's clothes must not discard valid body/face measurements."""
import numpy as np
import pytest
from base_athome_perception import bag_vision as bv


def person():
    kp = np.zeros((17, 2), float)
    kp[[5, 6, 11, 12]] = [(70, 70), (130, 70), (70, 150), (130, 150)]
    kp[[0, 1, 2]] = [(100, 40), (92, 35), (108, 35)]
    return ([60, 20, 140, 190], kp, np.ones(17))


def locate(depth, p=None):
    return bv.person_map_position(p or person(), depth, [0, 0, 1.5],
                                  np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
                                  (200., 200., 100., 100.))


def test_torso_center_hole_uses_other_measured_torso_pixels():
    d = np.zeros((200, 200), np.uint16)
    d[80:140, 80:120] = 2500
    d[100:121, 90:111] = 0
    assert locate(d) == pytest.approx((2.5, 0.), abs=.02)


def test_black_clothes_with_valid_face_can_be_localized():
    d = np.zeros((200, 200), np.uint16)
    d[30:45, 88:113] = 2200
    assert locate(d) == pytest.approx((2.2, 0.), abs=.02)


def test_background_outside_body_is_not_used():
    d = np.full((200, 200), 4000, np.uint16)
    d[20:155, 60:141] = 0
    assert locate(d) is None


@pytest.mark.parametrize('value', [0, 200, 7000])
def test_missing_or_out_of_range_depth_is_rejected(value):
    assert locate(np.full((200, 200), value, np.uint16)) is None


def test_two_competing_depth_surfaces_are_not_averaged():
    d = np.zeros((200, 200), np.uint16)
    d[70:151, 70:100] = 2000
    d[70:151, 100:131] = 4000
    assert locate(d) is None


def test_a_few_isolated_depth_pixels_are_not_enough():
    d = np.zeros((200, 200), np.uint16)
    d[110, 100] = 2500
    assert locate(d) is None


def test_unreliable_face_landmarks_do_not_use_face_depth():
    d = np.zeros((200, 200), np.uint16)
    d[30:45, 88:113] = 2200
    p = person()
    p[2][:3] = 0.
    assert locate(d, p) is None


def test_cropped_hips_keep_supported_shoulder_center_measurement():
    p = person()
    p[2][[11, 12]] = 0.
    d = np.zeros((200, 200), np.uint16)
    d[66:75, 96:105] = 2500
    assert locate(d, p) == pytest.approx((2.5, 0.))


def test_holey_depth_confirms_customer_and_ends_sweep():
    from test_raised_hands_mode import SweepHarness
    h = SweepHarness(always_visible=True)
    h.node._sweep_people.__func__.__globals__['bv'] = bv
    p = person()
    p[1][9] = [70, 15]  # left wrist above shoulder and nose
    d = np.zeros((200, 200), np.uint16)
    d[80:140, 80:120] = 2500
    d[100:121, 90:111] = 0
    h.node._sim_person_spot = lambda *args: None
    h.node._cam = lambda *args: ([0, 0, 1.5], np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
    h.node.K = (200., 200., 100., 100.)
    h.node._poses = lambda *args: [p]
    h.node._frames = lambda with_stamp=False: (np.zeros((200, 200, 3)), d, h.node.rgb_stamp)
    result = h.node._find_raised_hands(dict(timeout_s=80., min_frames=3, stop_on_first=True))
    assert result['ok'] and len(result['customers']) == 1
    assert result['customers'][0]['xy'] == pytest.approx([2.5, 0.], abs=.02)
    assert h.commands == [0.]


def test_distant_person_with_zero_depth_is_estimated_from_apparent_scale():
    """Cuando la RealSense no devuelve profundidad por distancia, estima posicion por escala aparente y rayo TF."""
    kp = np.zeros((17, 2), float)
    # Persona a ~4.0 m: torso vertical de 25 px (0.50 * 200 / 25 = 4.0 m), centrada en (100, 100)
    kp[[5, 6, 11, 12]] = [(90, 88), (110, 88), (90, 113), (110, 113)]
    kp[[0, 1, 2]] = [(100, 76), (97, 74), (103, 74)]
    far_person = ([85, 70, 115, 140], kp, np.ones(17))
    zero_depth = np.zeros((200, 200), np.uint16)
    T = [0., 0., 1.5]
    R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    K = (200., 200., 100., 100.)

    assert bv.person_map_position(far_person, zero_depth, T, R, K) is None
    est = bv.person_estimated_map_position(far_person, zero_depth, T, R, K)
    assert est is not None
    assert est[0] == pytest.approx(4.0, abs=0.2)
    assert est[1] == pytest.approx(0.0, abs=0.05)


def test_sweep_falls_back_to_estimated_position_when_depth_fails_at_distance():
    from test_raised_hands_mode import SweepHarness
    h = SweepHarness(always_visible=True)
    h.node._sweep_people.__func__.__globals__['bv'] = bv
    kp = np.zeros((17, 2), float)
    kp[[5, 6, 11, 12]] = [(90, 88), (110, 88), (90, 113), (110, 113)]
    kp[[0, 1, 2]] = [(100, 76), (97, 74), (103, 74)]
    kp[9] = [90, 60]  # mano izquierda levantada
    far_person = ([85, 70, 115, 140], kp, np.ones(17))
    zero_depth = np.zeros((200, 200), np.uint16)
    h.node._sim_person_spot = lambda *args: None
    h.node._cam = lambda *args: ([0, 0, 1.5], np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
    h.node.K = (200., 200., 100., 100.)
    h.node._poses = lambda *args: [far_person]
    h.node._frames = lambda with_stamp=False: (np.zeros((200, 200, 3)), zero_depth, h.node.rgb_stamp)
    result = h.node._find_raised_hands(dict(timeout_s=80., min_frames=3, stop_on_first=True))
    assert result['ok'] and len(result['customers']) == 1
    assert result['customers'][0]['estimated'] is True
    assert result['customers'][0]['xy'][0] == pytest.approx(4.0, abs=0.2)
