#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def find_dabai_video_device(default_id: int = 6) -> int:
    """Detecta automáticamente el índice /dev/videoX correspondiente a la cámara Orbbec DaBai."""
    try:
        out = subprocess.check_output("v4l2-ctl --list-devices 2>/dev/null", shell=True).decode()
        for block in out.split("\n\n"):
            if "dabai" in block.lower() or "0557" in block:
                for line in block.split("\n"):
                    line = line.strip()
                    if line.startswith("/dev/video"):
                        dev_id = int(line.replace("/dev/video", ""))
                        return dev_id
    except Exception:
        pass
    return default_id


class OrbbecStreamer(Node):
    def __init__(self, device_id: int = 6):
        super().__init__('orbbec_streamer_node')
        self.pub = self.create_publisher(Image, '/arm_camera/image_raw', 10)
        self.device_id = device_id
        self.cap = None
        self._init_camera()
        self.timer = self.create_timer(0.033, self.timer_cb)  # ~30 FPS

    def _init_camera(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.device_id = find_dabai_video_device(self.device_id)
        self.cap = cv2.VideoCapture(self.device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().warn(f'No se pudo abrir camara DaBai en /dev/video{self.device_id}. Reintentando...')
        else:
            self.get_logger().info(f'Streaming Orbbec DaBai (/dev/video{self.device_id}) -> /arm_camera/image_raw')

    def timer_cb(self):
        if self.cap is None or not self.cap.isOpened():
            self._init_camera()
            return

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
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    dev = find_dabai_video_device(6)
    node = OrbbecStreamer(device_id=dev)
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
