"""Ocupacion de asientos (geometria pura)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception import seat_occupancy as so  # noqa: E402

SOFA = {'name': 'Sofa', 'capacity': 2, 'polygon': [[3.48, -8.06], [4.09, -8.06], [4.09, -6.41], [3.48, -6.41]]}
CHAIR = {'name': 'Chair', 'capacity': 1, 'x': 1.0, 'y': 1.0}
SEATS = {'sofa': SOFA, 'chair': CHAIR}


def test_person_straight_ahead_with_head_centered():
    x, y = so.person_map_position(u=320, depth_z=2.0, fx=600, cx=320, robot_pose=(0, 0, 0), head_pan_rad=0.0)
    assert x == pytest.approx(2.0) and y == pytest.approx(0.0)


def test_head_pan_left_and_pixel_right_offsets():
    x, y = so.person_map_position(320, 2.0, 600, 320, (0, 0, 0), math.radians(90))
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(2.0)
    # pixel a la derecha del centro -> yaw negativo
    x, y = so.person_map_position(920, 1.0, 600, 320, (0, 0, 0), 0.0)
    assert y < 0 and math.degrees(math.atan2(y, x)) == pytest.approx(-45.0)


def test_occupancy_counts_people_in_polygon_and_near_point():
    persons = [(3.8, -7.5), (3.7, -6.9), (1.2, 1.1), (10, 10)]
    assert so.seat_occupancy(SEATS, persons) == {'sofa': 2, 'chair': 1}


def test_margin_allows_person_sitting_on_the_edge():
    assert so.seat_occupancy(SEATS, [(4.30, -7.0)])['sofa'] == 1
    assert so.seat_occupancy(SEATS, [(4.80, -7.0)])['sofa'] == 0


def test_choose_free_seat_respects_capacity_and_reservations():
    assert so.choose_free_seat(SEATS, {'sofa': 1, 'chair': 0}) in ('sofa', 'chair')
    assert so.choose_free_seat(SEATS, {'sofa': 2, 'chair': 0}) == 'chair'
    assert so.choose_free_seat(SEATS, {'sofa': 1, 'chair': 1}) == 'sofa'
    assert so.choose_free_seat(SEATS, {'sofa': 1, 'chair': 0}, reserved={'sofa': 1}) == 'chair'
    assert so.choose_free_seat(SEATS, {'sofa': 2, 'chair': 1}) is None


def test_merge_repeated_detections_of_same_person():
    assert len(so.merge_nearby([(1.0, 1.0), (1.1, 1.05), (3.0, 3.0)])) == 2


def test_depth_at_bbox_median_and_invalid():
    depth = np.full((480, 640), 2.5, np.float32)
    assert so.depth_at_bbox(depth, (200, 100, 300, 400)) == pytest.approx(2.5)
    assert so.depth_at_bbox(np.zeros((480, 640), np.float32), (200, 100, 300, 400)) is None


def test_image_fallback_counts_only_centered_sitting_people():
    boxes = [(280, 0, 360, 400), (0, 0, 80, 400), (300, 0, 340, 400)]
    assert so.image_fallback_occupied(boxes, ['SENTADO', 'SENTADO', 'DE_PIE'], 640) == 1
