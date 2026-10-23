#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Prueba Interactiva de Deteccion de Poses Humanas en Tiempo Real
# Subscribidor ROS 2 a la camara RealSense D435 con visualizacion directa en pantalla
# ==============================================================================

import argparse
import os
import sys
import time

import cv2
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(current_dir)
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from base_athome_perception.person_pose_detector import PersonPoseDetector


class HumanPoseViewer(Node if ROS2_AVAILABLE else object):
    def __init__(
        self,
        topic: str = "/camera/camera/color/image_raw",
        use_camera_id: int = -1,
        model_path: str = "",
    ):
        if ROS2_AVAILABLE and use_camera_id == -1:
            super().__init__("human_pose_viewer")
            self.get_logger().info(f"Iniciando detector de poses sobre: {topic}")
            self.bridge = CvBridge()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sub = self.create_subscription(Image, topic, self.image_callback, qos)
        else:
            self.sub = None

        self.use_camera_id = use_camera_id
        self.cap = None
        if self.use_camera_id >= 0:
            print(f"Abriendo camara local /dev/video{use_camera_id}...")
            self.cap = cv2.VideoCapture(self.use_camera_id)

        self.detector = PersonPoseDetector(model_path=model_path, conf_thresh=0.50)
        self.latest_frame = None
        self.fps = 0.0
        self.running = True

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            if hasattr(self, "get_logger"):
                self.get_logger().error(f"Error convirtiendo imagen: {e}")

    def run(self):
        window_name = "Base@Home - Deteccion de Poses Humanas (YOLOv8-Pose)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)

        print("==================================================")
        print(" Detector de Poses Humanas Iniciado")
        print(" Clasifica: DE PIE, SENTADO, SALUDANDO / SENALANDO")
        print(" Presiona [Q] para salir")
        print("==================================================")

        last_fps_calc = time.time()
        fps_counter = 0

        while self.running:
            if ROS2_AVAILABLE and self.sub:
                rclpy.spin_once(self, timeout_sec=0.01)
                frame = self.latest_frame
            elif self.cap:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.02)
                    continue
            else:
                frame = None

            if frame is None:
                wait_screen = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    wait_screen,
                    "Esperando se;al de camara RealSense D435...",
                    (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 200, 255),
                    2,
                )
                cv2.imshow(window_name, wait_screen)
                key = cv2.waitKey(30) & 0xFF
                if key in [ord("q"), ord("Q"), 27]:
                    break
                continue

            # Inferencia de poses
            poses = self.detector.detect_poses(frame)

            # Calculo de FPS
            fps_counter += 1
            now = time.time()
            if now - last_fps_calc >= 0.5:
                self.fps = fps_counter / (now - last_fps_calc)
                fps_counter = 0
                last_fps_calc = now

            # Dibujar
            vis = self.detector.draw_poses(frame, poses)

            # Barra superior
            h, w = vis.shape[:2]
            cv2.rectangle(vis, (0, 0), (w, 35), (20, 24, 30), -1)
            cv2.putText(vis, f"ROBOCUP @HOME - POSES HUMANAS | Personas: {len(poses)} | FPS: {self.fps:.1f}", (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow(window_name, vis)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                break

        cv2.destroyAllWindows()
        if self.cap:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Deteccion de Poses Humanas para RoboCup @Home")
    parser.add_argument("--topic", default="/camera/camera/color/image_raw", help="Topico ROS 2 de imagen RGB")
    parser.add_argument("--webcam", type=int, default=-1, help="Usar camara USB local /dev/videoX")
    args = parser.parse_args()

    pkg_dir = "/home/athome/base_athome_ws/src/base_athome_perception"
    pose_model = os.path.join(pkg_dir, "models", "yolov8n-pose.pt")

    if ROS2_AVAILABLE and args.webcam == -1:
        rclpy.init()
        viewer = HumanPoseViewer(topic=args.topic, use_camera_id=-1, model_path=pose_model)
        try:
            viewer.run()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                viewer.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
    else:
        viewer = HumanPoseViewer(use_camera_id=args.webcam if args.webcam >= 0 else 0, model_path=pose_model)
        try:
            viewer.run()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

