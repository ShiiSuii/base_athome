"""ROS adapter for the opt-in pure manipulation dock (HOME-58).

Scan must be in `laser` (x=0.22m, forward). Rear input is optional but REQUIRED
for retreat: /restaurant/rear_scan in `restaurant_rear_laser`, located at the
rear bumper and facing backwards, with +/-90 degree coverage. The real robot
currently has no such sensor: retreat fails closed. This is not clearance
certification at other heights. Never enable on hardware without Level C review.
A 20Hz timer owns velocity output and stops on stale headers AND stale arrivals;
run() blocks only the mission thread, leaving sensor and watchdog callbacks live.
"""
import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from .mission.manipulation_dock import ManipulationDock, Scan


class RestaurantDocking:
    def __init__(self, node, active_fn, permitted_fn):
        self.node, self.active, self.permitted = node, active_fn, permitted_fn
        self.lock = threading.RLock()
        self.scan = self.rear_scan = self.pose = self.odom_at = None
        self.controller = None
        self.advance_m = 0.0
        self.pub = node.create_publisher(Twist, '/cmd_vel', 1)
        for topic, reverse in (('/scan', False), ('/restaurant/rear_scan', True)):
            node.create_subscription(LaserScan, topic, lambda m, r=reverse: self._scan(m, r),
                                     qos_profile_sensor_data, callback_group=node.cb_group)
        node.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data,
                                 callback_group=node.cb_group)
        node.create_timer(.05, self._tick, callback_group=node.cb_group)

    def _header_valid(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        return math.isfinite(age) and 0 <= age <= .3

    def _scan(self, msg, reverse):
        frame = 'restaurant_rear_laser' if reverse else 'laser'
        with self.lock:
            value = None
            if msg.header.frame_id == frame and self._header_valid(msg) and msg.angle_increment > 0:
                angles = tuple(msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges)))
                value = Scan(angles, tuple(msg.ranges), time.monotonic() - max(0., self.node.get_clock().now().nanoseconds * 1e-9
                             - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9), msg.range_min, msg.range_max)
            if reverse:
                self.rear_scan = value
            else:
                self.scan = value

    def _odom(self, msg):
        with self.lock:
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if (not self._header_valid(msg) or msg.header.frame_id != 'odom'
                    or not math.isfinite(norm) or abs(norm-1) > .01):
                self.odom_at = None
                return
            self.pose = (p.x, p.y, math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            self.odom_at = time.monotonic() - max(0., self.node.get_clock().now().nanoseconds * 1e-9
                - msg.header.stamp.sec - msg.header.stamp.nanosec * 1e-9)

    def _zero(self):
        self.pub.publish(Twist())

    def _tick(self):
        with self.lock:
            c = self.controller
            if c is None or c.result is not None:
                return
            try:
                speed = c.step(time.monotonic(), self.pose, self.odom_at,
                               self.rear_scan if c.retreat_m is not None else self.scan,
                               active=self.active() and self.permitted())
                msg = Twist()
                msg.linear.x = speed
                self.pub.publish(msg)
            except Exception:
                c.result = 'adapter_error'
                self._zero()

    def cancel(self):
        with self.lock:
            if self.controller is not None and self.controller.result is None:
                self.controller.result = 'cancelled'
                self._zero()

    def run(self, *, reverse=False):
        with self.lock:
            if not self.active() or not self.permitted() or self.pose is None:
                return False
            if self.controller is not None and self.controller.result is None:
                return False
            # Check exit sensing BEFORE entering the counter gap.
            from .mission.manipulation_dock import corridor_error, DockConfig
            from .mission.dock_safety import sensor_freshness_error
            now = time.monotonic()
            rear_error = (sensor_freshness_error(now,
                    self.rear_scan.received_at if self.rear_scan else None, self.odom_at)
                    or corridor_error(self.rear_scan, DockConfig(), reverse=True))
            if not reverse and rear_error:
                self.node.get_logger().error(f'Dock rejected: rear clearance: {rear_error}')
                return False
            if reverse and self.advance_m <= .005:
                return False
            c = ManipulationDock(now, self.pose, enabled=True,
                                 retreat_m=self.advance_m if reverse else None)
            self.controller = c
        try:
            # Independent wall-clock cap also bounds a stalled ROS timer.
            end = time.monotonic() + c.cfg.timeout_s + .2
            while c.result is None and time.monotonic() < end and self.active() and self.permitted():
                time.sleep(.02)
        finally:
            with self.lock:
                if c.result is None:
                    c.result = 'cancelled'
                self._zero()
                if not reverse:
                    self.advance_m = c.travel_m
                elif c.result == 'done':
                    self.advance_m = 0.0
            self.node.get_logger().info(f'Manipulation dock: reverse={reverse}, result={c.result}, travel_m={c.travel_m:.3f}')
        return c.result == 'done'
