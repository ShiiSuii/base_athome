"""Regresiones de autofiltro del Hokuyo, sin importar ni iniciar ROS."""

import math

import pytest

from base_athome_bringup.scan_self_filter import filter_scan_ranges


@pytest.mark.parametrize("angle_deg", [116, 123, 130])
@pytest.mark.parametrize("distance", [0.30, 0.32, 0.34])
def test_measured_left_body_returns_are_filtered(angle_deg, distance):
    # Barrido de 270 grados: el indice no es el angulo del rayo.
    ranges = [math.inf] * 271
    index = angle_deg + 135
    ranges[index] = distance

    filtered = filter_scan_ranges(ranges, math.radians(-135), math.radians(1))

    assert filtered[index] == math.inf
    assert len(filtered) == len(ranges)
    assert ranges[index] == distance  # No modificar el scan original.


def test_real_obstacle_half_a_meter_ahead_is_kept():
    assert filter_scan_ranges([0.5], 0.0, 0.0) == [0.5]


def test_leg_thirty_centimeters_in_front_of_chassis_is_kept():
    # Frente del chasis x=0.26; pierna x=0.56; laser x=0.22.
    distance_from_laser = 0.26 + 0.30 - 0.22
    assert filter_scan_ranges([distance_from_laser], 0.0, 0.0) == [distance_from_laser]


def test_invalid_and_nonfinite_ranges_are_unchanged():
    ranges = [math.nan, math.inf, -math.inf, 0.0, -0.1]
    filtered = filter_scan_ranges(ranges, -1.0, 0.1)
    assert math.isnan(filtered[0])
    assert filtered[1:] == ranges[1:]


def test_laser_translation_and_yaw_are_applied_in_base_link():
    # El mismo rayo apunta afuera con el montaje por defecto y adentro con este.
    assert filter_scan_ranges([0.32], 0.0, 0.0) == [0.32]
    assert filter_scan_ranges(
        [0.32], 0.0, 0.0, laser_x=0.10, laser_y=-0.05, laser_yaw=math.pi / 2
    ) == [math.inf]
    assert filter_scan_ranges(
        [0.32], math.radians(123), 0.0, laser_x=1.0
    ) == [0.32]


def test_body_box_is_configurable():
    assert filter_scan_ranges([0.32], math.radians(123), 0.0) == [math.inf]
    assert filter_scan_ranges(
        [0.32], math.radians(123), 0.0, box_max_y=0.20
    ) == [0.32]


@pytest.mark.parametrize(
    "angle,distance,inside",
    [
        (0.0, 0.26, True),
        (0.0, 0.2601, False),
        (math.pi, 0.30, True),
        (math.pi, 0.3001, False),
        (math.pi / 2, 0.36, True),
        (math.pi / 2, 0.3601, False),
        (-math.pi / 2, 0.36, True),
        (-math.pi / 2, 0.3601, False),
    ],
)
def test_box_includes_its_boundary_without_expanding_it(angle, distance, inside):
    filtered = filter_scan_ranges([distance], angle, 0.0, laser_x=0.0)
    assert filtered == [math.inf if inside else distance]


@pytest.mark.parametrize(
    "geometry",
    [
        {"box_min_x": 0.27},
        {"box_min_y": 0.37},
        {"laser_x": math.nan},
        {"laser_yaw": math.inf},
        {"box_max_x": math.inf},
    ],
)
def test_invalid_geometry_is_rejected(geometry):
    with pytest.raises(ValueError):
        filter_scan_ranges([0.32], 0.0, 0.0, **geometry)


def test_front_only_drops_side_and_rear_beams():
    """Hokuyo de 270 grados: con +-100 grados, un objeto lejano a 120 grados no llega a Nav2; uno adelante si."""
    import math as _m
    from base_athome_bringup.scan_self_filter import filter_scan_ranges
    inc = _m.radians(1.0)
    amin = -_m.radians(135.0)
    ranges = [2.0] * 271
    out = filter_scan_ranges(ranges, amin, inc, max_abs_angle=_m.radians(100.0))
    assert _m.isinf(out[135 + 120]) and _m.isinf(out[135 - 120])     # costados de atras
    assert out[135] == 2.0 and out[135 + 90] == 2.0                   # adelante y 90 grados


def test_scan_nav_offers_reliable_and_short_queue_while_input_stays_sensor_qos():
    """Constructor real con Node/QoS falsos, sin ROS. RELIABLE satisface ambos lectores."""
    import ast
    from pathlib import Path
    from types import SimpleNamespace as NS
    ReliabilityPolicy = NS(RELIABLE='RELIABLE')
    qos_profile_sensor_data = object()

    path = Path(__file__).parents[1] / 'base_athome_bringup/scan_self_filter_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ScanSelfFilterNode')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    published, subscribed = [], []

    class FakeNode:
        def __init__(self, name):
            pass

        def declare_parameter(self, name, value, descriptor):
            return NS(value=value)

        def create_publisher(self, msg, topic, qos):
            published.append((topic, qos))

        def create_subscription(self, msg, topic, callback, qos):
            subscribed.append((topic, qos))

        def _scan_callback(self, msg):
            pass

    fake_class = ast.ClassDef(name='ScanSelfFilterNode', bases=[ast.Name(id='FakeNode', ctx=ast.Load())],
                             keywords=[], body=[init], decorator_list=[])
    scope = dict(FakeNode=FakeNode, ParameterDescriptor=NS, math=math, LaserScan=object,
                 filter_scan_ranges=filter_scan_ranges, QoSProfile=NS,
                 ReliabilityPolicy=ReliabilityPolicy, qos_profile_sensor_data=qos_profile_sensor_data)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fake_class], type_ignores=[])), str(path), 'exec'), scope)
    scope['ScanSelfFilterNode']()
    topic, offered = published[0]
    assert topic == '/scan_nav' and offered.depth == 2
    assert offered.reliability == ReliabilityPolicy.RELIABLE
    assert subscribed == [('/scan', qos_profile_sensor_data)]
