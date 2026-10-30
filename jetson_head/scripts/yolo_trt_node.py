#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nodo ROS 2: detecciones YOLOv8 con TensorRT en la GPU de la Jetson.

  SUB  <image_topic>                (sensor_msgs/Image bgr8, default /arm_camera/image_raw)
  PUB  <image_topic base>/detections (vision_msgs/Detection2DArray, class_id = nombre COCO)

Solo percepcion: no comanda actuadores. Procesa el frame mas reciente (descarta
los atrasados) para no acumular latencia.
    python3 ~/yolo_trt_node.py --ros-args -p engine_path:=$HOME/models/yolo/yolov8n_fp16.engine
"""
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_trt import COCO_CLASSES, TRTEngine, letterbox, postprocess  # noqa: E402


class YoloTrtNode(Node):
    def __init__(self):
        super().__init__('yolo_trt_node')
        self.declare_parameter('engine_path', os.path.expanduser('~/models/yolo/yolov8n_fp16.engine'))
        self.declare_parameter('image_topic', '/arm_camera/image_raw')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('iou_threshold', 0.5)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.conf, self.iou = float(gp('conf_threshold')), float(gp('iou_threshold'))
        topic = gp('image_topic')
        out_topic = topic.rsplit('/', 1)[0] + '/detections'

        self.engine = TRTEngine(gp('engine_path'))
        self.pub = self.create_publisher(Detection2DArray, out_topic, 10)
        self.create_subscription(Image, topic, self.image_cb, qos_profile_sensor_data)
        self.latest = None
        self.cv = threading.Condition()
        self.n, self.t_acc, self.t_log = 0, 0.0, time.monotonic()
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'YOLO TensorRT {gp("engine_path")}: {topic} -> {out_topic}')

    def image_cb(self, msg: Image):
        with self.cv:
            self.latest = msg
            self.cv.notify()

    def worker(self):
        while rclpy.ok():
            with self.cv:
                while self.latest is None and rclpy.ok():
                    self.cv.wait(0.5)
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            if msg.encoding not in ('bgr8', 'rgb8'):
                self.get_logger().warn(f'encoding no soportado: {msg.encoding}', throttle_duration_sec=5.0)
                continue
            t0 = time.monotonic()
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                img = img[:, :, ::-1]
            x, scale, pad = letterbox(img)
            dets = postprocess(self.engine.infer(x), scale, pad, img.shape[:2], self.conf, self.iou)
            self.pub.publish(self.to_msg(msg.header, dets))
            self.n += 1
            self.t_acc += time.monotonic() - t0
            if time.monotonic() - self.t_log > 10.0 and self.n:
                self.get_logger().info(f'{self.n / (time.monotonic() - self.t_log):.1f} Hz, '
                                       f'{1000 * self.t_acc / self.n:.1f} ms/frame')
                self.n, self.t_acc, self.t_log = 0, 0.0, time.monotonic()

    @staticmethod
    def to_msg(header, dets) -> Detection2DArray:
        out = Detection2DArray()
        out.header = header
        for x1, y1, x2, y2, score, cls in dets:
            d = Detection2D()
            d.header = header
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = COCO_CLASSES[cls] if cls < len(COCO_CLASSES) else str(cls)
            hyp.hypothesis.score = score
            d.results.append(hyp)
            out.detections.append(d)
        return out

    def destroy_node(self):
        self.engine.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloTrtNode()
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
