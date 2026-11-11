"""Offline adapter tests: no ROS node initialization or network discovery."""
import math
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
from base_athome_skills.restaurant_docking import RestaurantDocking
from base_athome_skills.mission.manipulation_dock import ManipulationDock


def adapter():
    d = RestaurantDocking.__new__(RestaurantDocking)
    d.node = Mock()
    d.node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
    d.lock = threading.RLock()
    d.scan = d.rear_scan = d.pose = d.odom_at = None
    d.controller = None
    d.advance_m = 0.
    d.pub = Mock()
    d.active = lambda: True
    d.permitted = lambda: True
    return d


def scan(stamp=1., frame='laser'):
    return NS(header=NS(frame_id=frame, stamp=NS(sec=int(stamp),nanosec=int(stamp%1*1e9))),
              angle_min=-math.pi/2, angle_increment=math.pi/360,
              ranges=[2.]*361, range_min=.06, range_max=8.)


@pytest.mark.parametrize('stamp,frame', [(0.,'laser'),(2.,'laser'),(1.,'base_link')])
def test_bad_header_invalidates_previous_scan(stamp,frame):
    d = adapter()
    d._scan(scan(), False)
    assert d.scan is not None
    d._scan(scan(stamp,frame), False)
    assert d.scan is None


def test_rear_does_not_accept_front_frame():
    d = adapter()
    d._scan(scan(), True)
    assert d.rear_scan is None
    d._scan(scan(frame='restaurant_rear_laser'), True)
    assert d.rear_scan is not None


def test_timer_zero_on_stale_sensor_and_cancel():
    d = adapter()
    d.controller = ManipulationDock(1.,(0.,0.,0.),enabled=True)
    d._tick()
    assert d.controller.result == 'no_scan'
    assert d.pub.publish.call_args.args[0].linear.x == 0.
    d.controller = ManipulationDock(1.,(0.,0.,0.),enabled=True)
    d.cancel()
    assert d.controller.result == 'cancelled'
    assert d.pub.publish.call_args.args[0].linear.x == 0.


def test_preflight_rejects_approach_without_rear_scan():
    d = adapter()
    d.pose = (0.,0.,0.)
    assert not d.run()
    assert d.controller is None
    d.pub.publish.assert_not_called()


def test_exception_in_watchdog_publishes_zero():
    d = adapter()
    d.controller = Mock(result=None)
    d.controller.step.side_effect = RuntimeError('bad data')
    d._tick()
    assert d.controller.result == 'adapter_error'
    assert d.pub.publish.call_args.args[0].linear.x == 0.
