"""Memoria de personas: sesion temporal vs persistente, homonimos, FIFO, limpieza.

    python3 -m pytest -q src/base_athome_perception/test/test_face_memory.py
"""
import os
import sys
import threading
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception import face_recognizer as fr  # noqa: E402
from base_athome_perception.face_recognizer import FaceRecognizer  # noqa: E402


def unit(seed, noise_of=None, eps=0.15):
    rng = np.random.default_rng(seed)
    v = rng.standard_normal((1, 128)).astype(np.float32)
    if noise_of is not None:
        v = noise_of + eps * v / np.linalg.norm(v)
    return (v / np.linalg.norm(v)).astype(np.float32)


def make_recognizer(tmp_path, session='s1'):
    rec = FaceRecognizer.__new__(FaceRecognizer)
    rec.profiles = {}
    rec.cosine_thresh = 0.40
    rec.lock = threading.RLock()
    rec.session_id = session
    rec.db_dir = str(tmp_path)
    rec.thumbnails_dir = os.path.join(rec.db_dir, 'thumbnails')
    os.makedirs(rec.thumbnails_dir, exist_ok=True)
    rec.metadata_file = os.path.join(rec.db_dir, 'profiles.json')
    rec.features_file = os.path.join(rec.db_dir, 'features.pkl')
    rec.legacy_pkl_path = os.path.join(rec.db_dir, 'faces_db.pkl')
    rec.sface = mock.Mock()
    rec.sface.match.side_effect = lambda a, b, _: float((a @ b.T).item())
    return rec


def reload(rec, session):
    new = make_recognizer(rec.db_dir, session)
    new.load_database()
    return new


def test_guest_from_previous_session_is_not_recognized(tmp_path):
    rec = make_recognizer(tmp_path, 's1')
    alex = unit(1)
    rec.enroll_person('Alex', [alex])
    assert rec.recognize_face(alex)[0] == 'Alex'
    rec2 = reload(rec, 's2')
    assert rec2.recognize_face(alex)[0] == 'Desconocido'


def test_persistent_profile_survives_sessions(tmp_path):
    rec = make_recognizer(tmp_path, 's1')
    host = unit(2)
    rec.enroll_person('Charlie', [host], persistent=True)
    assert reload(rec, 's2').recognize_face(host)[0] == 'Charlie'


def test_same_name_new_session_creates_fresh_profile(tmp_path):
    rec = make_recognizer(tmp_path, 's1')
    rec.enroll_person('Alex', [unit(1)])
    rec2 = reload(rec, 's2')
    new_alex = unit(5)
    pid, is_new = rec2.enroll_person('Alex', [new_alex])
    assert is_new
    assert rec2.recognize_face(new_alex) == ('Alex', rec2.recognize_face(new_alex)[1], pid)


def test_same_name_different_face_is_not_merged(tmp_path):
    rec = make_recognizer(tmp_path)
    pid1, _ = rec.enroll_person('Alex', [unit(1)])
    pid2, is_new = rec.enroll_person('Alex', [unit(9)])
    assert is_new and pid1 != pid2
    assert len(rec.profiles[pid1].features) == 1


def test_same_person_more_samples_merges_and_caps_fifo(tmp_path):
    rec = make_recognizer(tmp_path)
    base = unit(3)
    pid, _ = rec.enroll_person('Sam', [unit(100 + i, base) for i in range(10)])
    pid2, is_new = rec.enroll_person('Sam', [unit(200 + i, base) for i in range(10)])
    assert pid2 == pid and not is_new
    assert len(rec.profiles[pid].features) == fr.MAX_SAMPLES


def test_placeholder_names_are_not_enrolled(tmp_path):
    rec = make_recognizer(tmp_path)
    for name in ['', 'Guest', 'unknown', 'Desconocido']:
        assert rec.enroll_person(name, [unit(1)]) == ('', False)
    assert rec.profiles == {}


def test_clear_session_keeps_persistent(tmp_path):
    rec = make_recognizer(tmp_path)
    rec.enroll_person('Charlie', [unit(2)], persistent=True)
    rec.enroll_person('Alex', [unit(1)])
    rec.enroll_person('Taylor', [unit(4)])
    assert rec.clear_session() == 2
    assert [p.name for p in rec.profiles.values()] == ['Charlie']
    assert [p.name for p in reload(rec, 's1').profiles.values()] == ['Charlie']


def test_save_is_atomic_and_reloadable(tmp_path):
    rec = make_recognizer(tmp_path)
    alex = unit(1)
    rec.enroll_person('Alex', [alex], thumbnail_img=np.zeros((60, 60, 3), np.uint8))
    assert not any(f.endswith('.tmp') for f in os.listdir(tmp_path))
    again = reload(rec, 's1')
    p = next(iter(again.profiles.values()))
    assert p.name == 'Alex' and p.thumbnail_img is not None
    assert again.recognize_face(alex)[0] == 'Alex'
