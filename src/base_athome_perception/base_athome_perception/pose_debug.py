"""Dibujo de poses COCO-17 estilo OpenPose. Sin ROS ni inferencia de modelos."""

import cv2
import numpy as np

from .raised_hand import classify_raised_hand, normalize_keypoints

# Pares COCO (indices desde cero): piernas, cadera, torso, brazos y rostro.
COCO_LIMBS = (
    (15, 13), (13, 11), (16, 14), (14, 12), (11, 12),
    (5, 11), (6, 12), (5, 6), (5, 7), (6, 8), (7, 9), (8, 10),
    (1, 2), (0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (4, 6),
)
# BGR, color fijo por articulacion y miembro.
JOINT_COLORS = (
    (0, 0, 255), (0, 85, 255), (0, 170, 255), (0, 255, 255), (0, 255, 170),
    (0, 255, 85), (0, 255, 0), (85, 255, 0), (170, 255, 0), (255, 255, 0),
    (255, 170, 0), (255, 85, 0), (255, 0, 0), (255, 0, 85), (255, 0, 170),
    (255, 0, 255), (170, 0, 255),
)
RAISED_COLOR = (0, 200, 0)
LOWERED_COLOR = (0, 160, 255)


def draw_pose_debug(bgr, people, head=None, conf_thresh=0.4):
    """Copia BGR con miembros, articulaciones, caja y MANO left/right/both/none.

    `people`: (box_xyxy, keypoints, confidence), como bag_vision.yolo_people.
    Omite puntos no finitos, fuera de imagen o de baja confianza.
    """
    img = bgr.copy()
    height, width = img.shape[:2]
    for box, keypoints, conf in people:
        points, scores = normalize_keypoints(keypoints, conf)
        valid = [np.isfinite((x, y, c)).all() and c >= conf_thresh
                 and 0 <= x < width and 0 <= y < height
                 for (x, y), c in zip(points, scores)]
        scores = [c if ok else 0.0 for c, ok in zip(scores, valid)]
        hand = classify_raised_hand(points, scores, conf_thresh=conf_thresh)
        color = RAISED_COLOR if hand.is_raised else LOWERED_COLOR
        if np.isfinite(box).all():
            x0, y0, x1, y1 = [int(v) for v in box]
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
            cv2.putText(img, f"MANO {hand.state}", (x0, max(15, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        for a, b in COCO_LIMBS:
            if valid[a] and valid[b]:
                cv2.line(img, tuple(map(int, points[a])), tuple(map(int, points[b])),
                         JOINT_COLORS[a], 2)
        for i, (point, ok) in enumerate(zip(points, valid)):
            if ok:
                cv2.circle(img, tuple(map(int, point)), 3, JOINT_COLORS[i], -1)
    if head is not None:
        cv2.putText(img, f"personas: {len(people)}  cabeza pan={head[0]:.0f} tilt={head[1]:.0f}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img
