"""Exercise the simulator's real listening method without ROS or devices."""
import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def listening_node():
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_open_listen')
    constants = [n for n in tree.body if isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == 'AUTO_LINES' for t in n.targets)]
    timers = []
    ns = {'threading': SimpleNamespace(Timer=lambda delay, cb: SimpleNamespace(start=lambda: timers.append(cb))),
          'String': lambda **kw: SimpleNamespace(**kw)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=constants + [method], type_ignores=[])), str(source), 'exec'), ns)
    lines = []
    node = SimpleNamespace(auto_guest=True, auto_idx=0, prompt='',
        mic=SimpleNamespace(request=lambda *a: None, start_timeout=8),
        _voice_cancelled=lambda *a: None, _voice_event=lambda *a: None,
        get_logger=lambda: SimpleNamespace(info=lambda *a: None),
        _speech=lambda msg: lines.append(msg.data))
    def listen(prompt):
        ns['_open_listen'](node, 'test', prompt)
        timers.pop(0)()
        return lines[-1]
    return node, listen


@pytest.mark.parametrize('confirm', [False, True])
def test_two_guests_with_optional_confirmation(confirm):
    node, listen = listening_node()
    first = listen('Guests introduce themselves at a party. Names: Robin, Morgan. Drinks: iced tea, coffee.')
    assert 'Robin' in first
    if confirm:
        assert listen('Answer yes or no. Yes. No.') == 'yes'
    second = listen('Guests introduce themselves at a party. Names: Robin, Morgan. Drinks: iced tea, coffee.')
    assert 'Morgan' in second
    assert 'coffee' in second.lower()
