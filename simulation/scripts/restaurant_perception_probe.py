#!/usr/bin/env python3
"""Collect real Restaurant perception rates without saving camera/person imagery.

Run with the simulation environment sourced, ROS_DOMAIN_ID=42, and an output
JSONL path. Subscribing debug_image enables bag_finder's periodic inference.
"""
import json
import os
from pathlib import Path
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise SystemExit('Restaurant probe requires isolated ROS_DOMAIN_ID=42')
    path = Path(sys.argv[1])
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('restaurant_perception_probe')
    with path.open('a', buffering=1) as out:
        def metric(msg):
            row = json.loads(msg.data)
            out.write(json.dumps(row) + '\n')
        node.create_subscription(String, '/bag_finder/sim_metrics', metric, 10)
        node.create_subscription(Image, '/bag_finder/debug_image', lambda msg: None, qos_profile_sensor_data)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
