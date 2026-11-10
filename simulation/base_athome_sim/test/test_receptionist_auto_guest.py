"""Exercise the simulator's real listening method without ROS or devices."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace

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


@pytest.mark.parametrize('side,xy', [('left', [3.5, 1.5]), ('right', [3.5, -1.5])])
def test_reception_guest_sits_at_the_offered_gazebo_chair(side, xy):
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    methods = [next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
               for name in ('_heard_robot', '_record_seated_guest', '_publish_receptionist_state')]
    string = lambda data: SimpleNamespace(data=data)
    published = []
    ns = {'String': string, 'json': __import__('json')}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), ns)
    node = SimpleNamespace(guest_present=True, guest_idx=0, guest_seated=False,
        person_walk=[], walk_start=None, person_points=False, bag_map=None,
        bag_in_hand=False, bag_delivered=False, obj_attached=False, person_hands_over=True,
        gz=False, receptionist_seated_guests=[], receptionist_seat_xy=None,
        pub_receptionist_state=SimpleNamespace(publish=published.append))
    node._record_seated_guest = MethodType(ns['_record_seated_guest'], node)
    ns['_heard_robot'](node, string(f"Robin, the chair on the {side} is free. Please take a seat there."))
    ns['_publish_receptionist_state'](node)
    assert node.receptionist_seat_xy == xy
    assert published[0].data == __import__('json').dumps({
        'guest_index': 1, 'seated_guests': [{'guest_index': 1, 'xy': xy}]})


def test_gazebo_guest_walks_before_the_seated_state_is_published():
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    methods = [next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
               for name in ('_gz_seat_guest', '_record_seated_guest')]
    calls = []
    fake_gz = SimpleNamespace(
        set_pose_command=lambda world, name, xyz: ('move', xyz),
        remove_command=lambda world, name: ('remove', name),
        customer_sdf=lambda name, xy, hand, seated: ('seated', xy, seated),
        create_command=lambda world, model: ('create', model),
    )
    ns = {'gz_model': fake_gz, 'GZ_WORLD': 'receplab', 'time': SimpleNamespace(sleep=lambda _: None)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), ns)
    node = SimpleNamespace(person_xy=[0.5, 1.8], receptionist_seated_guests=[],
                           _gz_run=calls.append)
    node._record_seated_guest = MethodType(ns['_record_seated_guest'], node)
    ns['_gz_seat_guest'](node, [3.5, 1.5], 2)
    moves = [c for c in calls if c[0] == 'move']
    assert len(moves) == 8
    assert moves[0][1] != [0.5, 1.8]
    assert list(moves[-1][1][:2]) == [3.5, 1.5]
    assert calls[-1][0:2] == ('create', ('seated', [3.5, 1.5], True))
    assert node.receptionist_seated_guests == [{'guest_index': 2, 'xy': [3.5, 1.5]}]


def test_second_guest_returns_to_reception_before_arriving():
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_guest')
    string = lambda data: SimpleNamespace(data=data)
    started = []
    fake_threading = SimpleNamespace(Thread=lambda **kw: SimpleNamespace(start=lambda: started.append(kw['target'])))
    ns = {'threading': fake_threading}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), 'exec'), ns)
    reset = lambda: None
    node = SimpleNamespace(guest_idx=0, guest_present=False, gz=True,
                           _gz_reset_reception_guest=reset,
                           get_logger=lambda: SimpleNamespace(info=lambda *a: None))
    ns['_guest'](node, string('next'))
    assert node.guest_idx == 1 and node.guest_present
    assert started == [reset]
