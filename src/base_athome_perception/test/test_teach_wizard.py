"""Asistente de ensenanza HOME -> PRE-GRASP -> POST-GRASP."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception.teach_wizard import TeachWizard  # noqa: E402

Q = [86.9, 172.8, -162.6, -99.8, -0.6, 45.0]


def make(fail_on=None):
    log = {'torque': [], 'saved': []}

    def save(name, q, g):
        if name == fail_on:
            return 'fuera de limites'
        log['saved'].append((name, q, g))
        return None
    w = TeachWizard(lambda on: log['torque'].append(on) or True, save)
    return w, log


def test_three_enters_save_three_poses_and_lock_at_end():
    w, log = make()
    w.start()
    assert log['torque'] == [False] and w.current == 'home'
    for g in (10.0, 105.0, 50.0):
        w.confirm(Q, g)
    assert [s[0] for s in log['saved']] == ['home', 'pre_grasp', 'post_grasp']
    assert log['torque'] == [False, True] and not w.active
    assert 'Listo' in w.message and 'POST-GRASP' in w.message


def test_arm_stays_free_between_poses():
    w, log = make()
    w.start()
    w.confirm(Q, 0.0)
    assert log['torque'] == [False] and w.current == 'pre_grasp'


def test_no_feedback_does_not_advance():
    w, log = make()
    w.start()
    w.confirm(Q, 0.0, feedback_ok=False)
    assert w.current == 'home' and log['saved'] == []


def test_save_error_keeps_same_pose():
    w, log = make(fail_on='pre_grasp')
    w.start()
    w.confirm(Q, 0.0)
    w.confirm(Q, 0.0)
    assert w.current == 'pre_grasp' and 'fuera de limites' in w.message


def test_cancel_locks_and_reports_saved():
    w, log = make()
    w.start()
    w.confirm(Q, 0.0)
    w.cancel()
    assert log['torque'] == [False, True] and not w.active and 'HOME' in w.message


def test_teach_only_post_grasp():
    w, log = make()
    w.start(only='post_grasp')
    assert w.current == 'post_grasp' and log['torque'] == [False]
    w.confirm(Q, 50.0)
    assert [s[0] for s in log['saved']] == ['post_grasp'] and log['torque'] == [False, True] and not w.active
