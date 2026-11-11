"""Tests del benchmark con imagenes y keypoints sinteticos. No requieren ROS ni los pesos YOLO."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import bench  # noqa: E402


def _keypoints(y_nose=100.0, y_l_shoulder=200.0, y_l_wrist=260.0,
               y_r_shoulder=200.0, y_r_wrist=260.0, conf=0.9):
    points = np.zeros((17, 2), dtype=float)
    confidence = np.full(17, conf, dtype=float)
    points[0] = (100.0, y_nose)
    points[5], points[9] = (80.0, y_l_shoulder), (80.0, y_l_wrist)
    points[6], points[10] = (120.0, y_r_shoulder), (120.0, y_r_wrist)
    points[11], points[12] = (85.0, 300.0), (115.0, 300.0)
    points[13], points[14] = (85.0, 350.0), (115.0, 350.0)
    points[15], points[16] = (85.0, 400.0), (115.0, 400.0)
    return points, confidence


def _person(kp, conf, score=0.9, box=(40.0, 40.0, 160.0, 420.0)):
    return bench.PersonDetection(
        box=np.asarray(box, dtype=float),
        score=score,
        keypoints=np.asarray(kp, dtype=float),
        keypoints_conf=np.asarray(conf, dtype=float),
    )


def _synthetic_image(width=64, height=48, value=(30, 60, 90)):
    image = np.zeros((height, width, 3), np.uint8)
    image[:] = value
    return image


def test_visible_keypoints_uses_confidence_threshold():
    kp, conf = _keypoints()
    conf[:7] = 0.1
    person = _person(kp, conf)
    assert person.visible_keypoints(0.4) == 10
    assert person.visible_keypoints(0.9) == 10


def test_frame_record_counts_persons_confidence_and_visible_keypoints():
    kp, conf = _keypoints()
    frames = bench.frame_record(0, "a.png", [_person(kp, conf, 0.8), _person(kp, conf, 0.6)], faces=[(1, 2, 3, 4, 0.9)])
    assert frames["persons"] == 2
    assert frames["mean_conf"] == pytest.approx(0.7)
    assert frames["visible_keypoints"] == 34
    assert frames["faces"] == 1


def test_classify_raised_hand_left_is_reused_by_benchmark():
    kp, conf = _keypoints(y_l_wrist=120.0, y_r_wrist=260.0)
    record = bench.frame_record(0, "raised.png", [_person(kp, conf)])
    assert record["raised_hands"] == 1
    assert record["raised_left"] == 1
    assert record["raised_right"] == 0
    assert record["people"][0]["hand"] == "left"
    assert record["people"][0]["raised"] is True


def test_classify_no_raised_hand():
    kp, conf = _keypoints(y_l_wrist=260.0, y_r_wrist=260.0)
    record = bench.frame_record(0, "rest.png", [_person(kp, conf)])
    assert record["raised_hands"] == 0
    assert record["people"][0]["hand"] == "none"


def test_classify_both_hands():
    kp, conf = _keypoints(y_l_wrist=120.0, y_r_wrist=120.0)
    record = bench.frame_record(0, "both.png", [_person(kp, conf)])
    assert record["raised_both"] == 1
    assert record["raised_left"] == 0 and record["raised_right"] == 0


def test_summarize_aggregates_frames():
    kp, conf = _keypoints()
    raised_kp, raised_conf = _keypoints(y_l_wrist=120.0)
    frames = [
        bench.frame_record(0, "a", [_person(kp, conf, 0.9)]),
        bench.frame_record(1, "b", [_person(kp, conf, 0.5), _person(raised_kp, raised_conf, 0.5)], faces=[(0, 0, 4, 4, 0.5)]),
        bench.frame_record(2, "c", []),
    ]
    summary = bench.summarize(frames)
    assert summary["frames"] == 3
    assert summary["persons_total"] == 3
    assert summary["persons_per_frame_mean"] == pytest.approx(1.0)
    assert summary["mean_conf"] == pytest.approx((0.9 + 0.5 + 0.5) / 3, abs=1e-3)
    assert summary["frames_with_person"] == 2
    assert summary["frames_with_raised_hand"] == 1
    assert summary["frames_with_face"] == 1
    assert summary["faces_total"] == 1


def test_summarize_empty():
    summary = bench.summarize([])
    assert summary["frames"] == 0
    assert summary["persons_per_frame_mean"] == 0.0
    assert summary["mean_conf"] == 0.0


def test_write_tsv_and_json_roundtrip(tmp_path):
    kp, conf = _keypoints()
    frames = [bench.frame_record(0, "img.png", [_person(kp, conf, 0.75)])]
    summary = bench.summarize(frames)
    tsv = tmp_path / "out.tsv"
    js = tmp_path / "out.json"
    bench.write_tsv(frames, str(tsv))
    bench.write_json({"mode": "offline"}, frames, summary, str(js))

    lines = tsv.read_text(encoding="utf-8").strip().splitlines()
    assert lines[0].split("\t") == list(bench.TSV_COLUMNS)
    assert len(lines) == 2
    payload = json.loads(js.read_text(encoding="utf-8"))
    assert payload["meta"]["mode"] == "offline"
    assert payload["frames"][0]["persons"] == 1
    assert payload["summary"]["persons_total"] == 1


def test_image_to_bgr_rgb8_message():
    rgb = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]]] * 2, dtype=np.uint8)
    message = SimpleNamespace(encoding="rgb8", height=2, width=3, step=9, data=rgb.tobytes())
    out = bench.image_to_bgr(message)
    assert out is not None
    assert out.shape == (2, 3, 3)
    assert tuple(out[0, 0]) == (0, 0, 255)
    assert tuple(out[0, 2]) == (255, 0, 0)


def test_image_to_bgr_handles_padded_rows_and_unknown_encoding():
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    padded = np.zeros((2, 12), dtype=np.uint8)
    padded[:, :9] = rgb.reshape(2, 9)
    message = SimpleNamespace(encoding="bgr8", height=2, width=3, step=12, data=padded.tobytes())
    out = bench.image_to_bgr(message)
    assert out is not None and out.shape == (2, 3, 3)
    unknown = SimpleNamespace(encoding="yuv422", height=2, width=3, step=0, data=b"")
    assert bench.image_to_bgr(unknown) is None


def test_load_images_reads_synthetic_folder(tmp_path):
    for name, value in (("b.png", (1, 2, 3)), ("a.png", (4, 5, 6))):
        assert cv2.imwrite(str(tmp_path / name), _synthetic_image(value=value))
    (tmp_path / "nota.txt").write_text("ignorar", encoding="utf-8")
    images = bench.load_images(str(tmp_path))
    assert [Path(source).name for source, _ in images] == ["a.png", "b.png"]
    assert images[0][1].shape == (48, 64, 3)


def test_face_detector_disabled_reports_unavailable():
    detector = bench.FaceDetector(enabled=False)
    assert detector.available is False
    assert detector.detect(_synthetic_image()) == []


def test_face_detector_blank_image_returns_list():
    detector = bench.FaceDetector()
    assert isinstance(detector.detect(_synthetic_image(width=640, height=480)), list)
