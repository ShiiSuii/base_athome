"""Nodo ROS 2: publica la senal de inicio cuando se abre la puerta. Solo lee /scan.

Publica (latched, transient_local):
  /start_signal/door_open  std_msgs/Bool   True una sola vez al abrirse la puerta
  /start_signal/status     std_msgs/String estado del detector (CALIBRATING, WAITING_CLOSED, OPEN, NO_DOOR)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from base_athome_skills.door_detector import DoorDetectorConfig, DoorOpenDetector, window_distance


class DoorOpenNode(Node):

    def __init__(self):
        super().__init__('door_open_detector')
        defaults = DoorDetectorConfig()
        cfg = DoorDetectorConfig(**{
            name: type(value)(self.declare_parameter(name, value).value)
            for name, value in vars(defaults).items()
        })
        scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.detector = DoorOpenDetector(cfg)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_open = self.create_publisher(Bool, '/start_signal/door_open', latched)
        self.pub_status = self.create_publisher(String, '/start_signal/status', latched)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.last_state = None
        self.get_logger().info(f'Esperando puerta en {scan_topic} (ventana {cfg.window_deg} deg)')

    def on_scan(self, msg: LaserScan):
        d = window_distance(msg.ranges, msg.angle_min, msg.angle_increment,
                            msg.range_min, msg.range_max, self.detector.cfg)
        state = self.detector.update(d)
        if state == self.last_state:
            return
        self.last_state = state
        self.pub_status.publish(String(data=state))
        ref = self.detector.reference_m
        self.get_logger().info(f'Estado puerta: {state}' + (f' (referencia {ref:.2f} m)' if ref else ''))
        if state == 'OPEN':
            self.pub_open.publish(Bool(data=True))
        elif state == 'NO_DOOR':
            self.get_logger().warn('No se detecta puerta cerrada al frente: usar senal manual de inicio')


def main(args=None):
    rclpy.init(args=args)
    node = DoorOpenNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
