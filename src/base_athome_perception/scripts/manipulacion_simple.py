#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ventana de manipulacion (alias `manipulacion`). La logica esta en base_athome_perception.arm_manipulation."""
import threading

import cv2
import rclpy

from base_athome_perception.arm_manipulation import WINDOW, Manipulacion

def main():
    rclpy.init()
    node = Manipulacion()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1160, 520)
    cv2.createTrackbar("Medido_cm", WINDOW, 24, 80, lambda v: setattr(node, "measured_cm", float(max(3, v))))
    cv2.createTrackbar("Avance_max_cm", WINDOW, int(node.max_advance_cm), 50, lambda v: node.set_advance_limits(max_cm=v))
    cv2.createTrackbar("Velocidad_%", WINDOW, int(node.speed_pct), 100,
                       lambda v: setattr(node, "speed_pct", float(max(5, v))))
    cv2.createTrackbar("Ajuste_cm", WINDOW, int(node.advance_adjust_cm) + 10, 20,
                       lambda v: node.set_advance_limits(adjust_cm=v - 10))
    try:
        while rclpy.ok():
            cv2.imshow(WINDOW, node.draw())
            key = cv2.waitKey(30) & 0xFF
            if key != 255 and not node.handle_key(key):
                break
    finally:
        if node.wizard.active:
            node.wizard.cancel()
        node.stop_move_group()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
