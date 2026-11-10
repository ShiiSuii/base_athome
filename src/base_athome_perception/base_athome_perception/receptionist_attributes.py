"""Pure, conservative guest descriptions for the receptionist.

The current extractor supplies four fields: upper/lower garment and their colors.
It does not measure glasses, age or height. Missing fields are never filled in.
"""

import math

import numpy as np

from .person_attributes import (
    ALL_COLORS, DEFAULT_MIN_CONFIDENCE, PersonAttributesExtractor, get_english_article,
    parse_coco_keypoints,
)


def confident_attributes(result, min_confidence=DEFAULT_MIN_CONFIDENCE):
    """Copy trusted extractor fields into a JSON-serializable guest snapshot."""
    attributes = {}
    if result is None:
        return attributes
    for region, item in (("torso", "shirt"), ("legs", "trousers")):
        garment = getattr(result, region, None)
        if (garment is None or not math.isfinite(garment.confidence)
                or not min_confidence <= garment.confidence <= 1.0
                or garment.color not in ALL_COLORS or garment.item != item):
            continue
        attributes[region] = {
            "color": garment.color, "item": garment.item,
            "confidence": float(garment.confidence),
        }
    return attributes


def extract_guest_attributes(frame, poses, faces):
    """Use same-frame poses and faces; abstain when association is ambiguous.

    Face boxes are xywh, pose boxes xyxy. The largest face is the interlocutor,
    as in face enrollment. Its center must belong to exactly one body's head area.
    """
    if frame is None or not faces or not poses:
        return {}
    face = max(faces, key=lambda f: f["bbox"][2] * f["bbox"][3])
    x, y, w, h = face["bbox"]
    cx, cy = x + w / 2, y + h / 2
    candidates = []
    for pose in poses:
        x1, y1, x2, y2 = pose["bbox"]
        if x1 <= cx <= x2 and y1 <= cy <= y1 + 0.45 * (y2 - y1):
            candidates.append(pose)
    if len(candidates) != 1:
        return {}
    pose = candidates[0]
    if pose.get("keypoints_conf") is None:
        return {}  # Do not let the extractor synthesize keypoint confidence.
    points, confidence = parse_coco_keypoints(pose)
    if (points is None or confidence is None or confidence.shape != (17,)
            or not np.isfinite(points).all() or not np.isfinite(confidence).all()
            or np.any(confidence < 0) or np.any(confidence > 1)):
        return {}  # Invalid pose evidence must not become a confident color result.
    return confident_attributes(PersonAttributesExtractor().extract(frame, points, confidence))


def prefer_attributes(previous, candidate):
    """Keep one coherent observation, preferring coverage and then confidence."""
    def quality(attributes):
        return len(attributes), sum(a["confidence"] for a in attributes.values())
    selected = candidate if quality(candidate) > quality(previous) else previous
    return {region: dict(garment) for region, garment in selected.items()}


def describe_guest_attributes(attributes):
    """One English sentence of at most 20 words, with up to four trusted fields."""
    garments = []
    for region, item in (("torso", "shirt"), ("legs", "trousers")):
        garment = attributes.get(region)
        if not garment:
            continue
        confidence = garment.get("confidence", 0.0)
        if (not math.isfinite(confidence) or not DEFAULT_MIN_CONFIDENCE <= confidence <= 1.0
                or garment.get("color") not in ALL_COLORS or garment.get("item") != item):
            continue
        label = f"{garment['color']} {item}"
        garments.append(f"{get_english_article(label)} {label}" if region == "torso" else label)
    if not garments:
        return ""
    return "Our first guest is wearing " + " and ".join(garments) + "."
