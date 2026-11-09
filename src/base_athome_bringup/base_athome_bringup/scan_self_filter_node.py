"""Nodo /scan -> /scan_nav: solo cambia ranges, conserva header y metadatos.

La pose laser_x/laser_y/laser_yaw y box_min_x/box_max_x/box_min_y/box_max_y
se configuran al arrancar (m, rad, caja en base_link). laser_frame debe
coincidir con el frame del Hokuyo para aplicar esa geometria fija.
"""
import math

from copy import deepcopy

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from base_athome_bringup.scan_self_filter import filter_scan_ranges


class ScanSelfFilterNode(Node):
    """Filtra retornos del chasis sin depender de TF ni abrir hardware."""

    def __init__(self):
        super().__init__("scan_self_filter")
        descriptor = ParameterDescriptor(read_only=True)
        defaults = {
            "laser_x": 0.22,
            "laser_y": 0.0,
            "laser_yaw": 0.0,
            "box_min_x": -0.30,
            "box_max_x": 0.26,
            "box_min_y": -0.36,
            "box_max_y": 0.36,
            "max_abs_angle_deg": 100.0,   # Hokuyo de 270 grados: Nav2 usa solo +-100 (adelante y costados delanteros)
        }
        self.filter_parameters = {
            name: self.declare_parameter(name, value, descriptor).value
            for name, value in defaults.items()
        }
        self.filter_parameters["max_abs_angle"] = math.radians(self.filter_parameters.pop("max_abs_angle_deg"))
        self.laser_frame = self.declare_parameter("laser_frame", "laser", descriptor).value
        # Validar antes de suscribir: una caja mal configurada impide el arranque.
        filter_scan_ranges([], 0.0, 0.0, **self.filter_parameters)
        if not self.laser_frame:
            raise ValueError("laser_frame no puede estar vacio")

        self.publisher = self.create_publisher(LaserScan, "/scan_nav", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )

    def _scan_callback(self, scan: LaserScan):
        if scan.header.frame_id != self.laser_frame:
            self.get_logger().warn(
                f"Scan ignorado: frame {scan.header.frame_id!r}, esperado {self.laser_frame!r}",
                throttle_duration_sec=5.0,
            )
            return
        filtered_scan = deepcopy(scan)
        filtered_scan.ranges = filter_scan_ranges(
            scan.ranges, scan.angle_min, scan.angle_increment, **self.filter_parameters
        )
        self.publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
