"""Real simulation uses rendered sensors and never privileged world coordinates."""
import ast
from pathlib import Path
import numpy as np
import pytest
from base_athome_perception.sim_perception import decode_depth


def test_gazebo_depth_invalid_ranges_are_unknown():
    values = np.array([[1.25, np.nan, np.inf, -1, 100]], '<f4')
    assert decode_depth(values.tobytes(), 1, 5, '32FC1').tolist() == [[1250, 0, 0, 0, 0]]


def test_padded_big_endian_depth():
    assert decode_depth(bytes.fromhex('03e807d000000bb80fa00000'), 2, 2, '16UC1', 6, True).tolist() == [[1000, 2000], [3000, 4000]]


def test_unknown_depth_encoding_rejected():
    with pytest.raises(ValueError):
        decode_depth(b'', 0, 0, 'mono8')


@pytest.mark.parametrize('real', [False, True])
def test_pose_backend_and_privileged_position_guards(real):
    path = Path(__file__).parents[1] / 'scripts/bag_finder.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BagFinder')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_poses')
    from types import SimpleNamespace
    import threading
    calls = []
    bv = SimpleNamespace(project_people=lambda *a: calls.append('project') or ['projected'], yolo_people=lambda r: ['inferred'])
    ns = dict(SIM=True, REAL_SIM=real, bv=bv)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), ns)
    obj = SimpleNamespace(_cam=lambda: (0, 0), K=(1, 1, 1, 1), sim_people=[], pose_lock=threading.Lock(),
                          _record_sim_metrics=lambda p: None, pose_model=SimpleNamespace(predict=lambda *a, **kw: calls.append('yolo') or [None]))
    assert ns['_poses'](obj, np.zeros((2, 2, 3))) == (['inferred'] if real else ['projected'])
    assert calls == (['yolo'] if real else ['project'])
    guards = [n for n in ast.walk(cls) if isinstance(n, ast.IfExp) and isinstance(n.body, ast.Call)
              and isinstance(n.body.func, ast.Attribute) and n.body.func.attr == '_sim_person_spot']
    assert len(guards) == 2
    for guard in guards:
        assert eval(compile(ast.Expression(guard.test), str(path), 'eval'), ns) == (not real)


def test_metrics_denominator_includes_missed_frames_and_deduplicates():
    from base_athome_perception.sim_perception import FrameMetrics
    m = FrameMetrics()
    m.add((1, 0), 'north', True, True, True)
    assert m.add((1, 0), 'north', True, True, True) is None
    row = m.add((2, 0), 'north', False, False, False)
    assert row['frames'] == 2
    assert row['percent'] == dict(person=50., raised=50., valid_depth=50.)
    assert m.add((3, 0), 'bar', True, False, True)['frames'] == 1
