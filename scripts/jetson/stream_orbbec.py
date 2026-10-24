#!/usr/bin/env python3
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class OrbbecStreamer(Node):
    def __init__(self, device_id=6):
        super().__init__('orbbec_streamer_node')
        self.pub = self.create_publisher(Image, '/arm_camera/image_raw', 10)
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir /dev/video{device_id}')
            return
        
        self.timer = self.create_timer(0.033, self.timer_cb) # ~30 FPS
        self.get_logger().info(f'Streaming Orbbec DaBai (/dev/video{device_id}) -> /arm_camera/image_raw')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_camera_optical_frame'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = OrbbecStreamer(device_id=6)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
