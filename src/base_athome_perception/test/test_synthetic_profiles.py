"""Tests offline: perfiles con embeddings sinteticos no se reconocen ni contaminan perfiles reales.

    python3 -m pytest -q src/base_athome_perception/test/test_synthetic_profiles.py
"""
import os
import sys
import threading
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception.face_recognizer import FaceRecognizer  # noqa: E402


def unit(seed):
    v = np.random.default_rng(seed).standard_normal((1, 128)).astype(np.float32)
    return v / np.linalg.norm(v)


def make_recognizer():
    rec = FaceRecognizer.__new__(FaceRecognizer)
    rec.profiles = {}
    rec.cosine_thresh = 0.52
    rec.lock = threading.RLock()
    rec.session_id = 'test_session'
    rec.db_dir = '/nonexistent'
    rec.sface = mock.Mock()
    rec.sface.match.side_effect = lambda a, b, _: (a @ b.T).item()
    rec.save_database = mock.Mock()
    return rec


def test_synthetic_profile_is_not_recognized():
    rec = make_recognizer()
    fake = unit(1)
    rec.enroll_person('Charlie', [fake], attributes={'synthetic': True})
    assert rec.recognize_face(fake)[0] == 'Desconocido'


def test_real_samples_replace_synthetic_ones():
    rec = make_recognizer()
    rec.enroll_person('Charlie', [unit(1)], attributes={'synthetic': True})
    real = unit(2)
    rec.enroll_person('Charlie', [real], attributes={'role': 'host'})
    profile = next(iter(rec.profiles.values()))
    assert profile.attributes['synthetic'] is False
    assert len(profile.features) == 1
    assert rec.recognize_face(real)[0] == 'Charlie'


def test_synthetic_samples_do_not_pollute_real_profile():
    rec = make_recognizer()
    real = unit(3)
    rec.enroll_person('Alex', [real])
    rec.enroll_person('Alex', [unit(4)], attributes={'synthetic': True})
    profile = next(iter(rec.profiles.values()))
    assert profile.attributes.get('synthetic') is False
    assert len(profile.features) == 1
    assert rec.recognize_face(real)[0] == 'Alex'
