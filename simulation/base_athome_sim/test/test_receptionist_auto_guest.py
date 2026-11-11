"""Exercise the simulator's real listening method without ROS or devices."""
import ast
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from base_athome_sim import gz_model


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
                           _gz_run=calls.append,
                           _publish_receptionist_state=lambda: calls.append(('state', True)))
    node._record_seated_guest = MethodType(ns['_record_seated_guest'], node)
    ns['_gz_seat_guest'](node, [3.5, 1.5], 2)
    moves = [c for c in calls if c[0] == 'move']
    assert len(moves) == 8
    assert moves[0][1] != [0.5, 1.8]
    assert list(moves[-1][1][:2]) == [3.5, 1.5]
    assert calls[-2][0:2] == ('create', ('seated', [3.5, 1.5], True))
    assert calls[-1] == ('state', True)
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


def test_receptionist_bag_and_drop_marker_have_visible_models():
    bag = ET.fromstring(gz_model.receptionist_bag_sdf((1.0, 2.0, 0.95)))
    drop = ET.fromstring(gz_model.receptionist_drop_sdf((4.15, 0.55)))
    assert bag.find('./model').attrib['name'] == 'recepcionista_bolsa'
    assert bag.find('.//box/size').text == '0.3 0.22 0.24'
    assert drop.find('./model').attrib['name'] == 'recepcionista_drop'


def test_receptionist_host_walks_to_drop_path_and_finishes_there():
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    methods = [next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
               for name in ('_gz_receptionist_follow_host', '_gz_move_receptionist_base')]
    calls = []
    ns = {'time': SimpleNamespace(sleep=lambda _: None), 'math': __import__('math'),
          'gz_model': SimpleNamespace(MODEL='base_athome',
              set_pose_command=lambda world, name, xyz, yaw: (name, xyz, yaw)),
          'GZ_WORLD': 'casa_simple'}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), ns)
    node = SimpleNamespace(receptionist_bag_state='received', receptionist_follow_active=False,
        receptionist_host_path_done=False, receptionist_host_xy=[2.0, 1.9], receptionist_host_yaw=0.0,
        receptionist_drop_xy=[4.15, 0.55], receptionist_bag_held=False,
        base=SimpleNamespace(goal=(1.0, 1.0), x=0.0, y=0.0, yaw=0.0),
        pub_receptionist_task=SimpleNamespace(publish=lambda msg: None),
        _publish_receptionist_task_state=lambda: None, _gz_run=calls.append,
        get_logger=lambda: SimpleNamespace(info=lambda *args: None))
    node._gz_move_receptionist_base = MethodType(ns['_gz_move_receptionist_base'], node)
    node._gz_move_receptionist_bag_to_gripper = lambda: None
    ns['_gz_receptionist_follow_host'](node)
    assert node.base.goal is None
    assert node.receptionist_host_path_done
    assert node.receptionist_host_xy == [4.15, 0.55]
    assert node.receptionist_bag_state == 'host_at_drop'
    host_calls = [call for call in calls if call[0] == 'recep_host']
    assert host_calls[0][1][:2] != (2.0, 1.9)
    assert host_calls[-1][1][:2] == (4.15, 0.55)
    assert __import__('math').hypot(node.base.x - 4.15, node.base.y - 0.55) == pytest.approx(0.72, abs=0.02)


def test_receptionist_handover_moves_guest_bag_to_simulated_gripper():
    source = Path(__file__).resolve().parents[1] / 'base_athome_sim/sim_robot.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimRobot')
    methods = [next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
               for name in ('_gz_receptionist_handover', '_gz_move_receptionist_bag_to_gripper')]
    calls = []
    ns = {'time': SimpleNamespace(sleep=lambda _: None, time=lambda: 12.0),
          'gz_model': SimpleNamespace(set_pose_command=lambda world, name, xyz: (name, xyz)),
          'GZ_WORLD': 'casa_simple'}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), ns)
    node = SimpleNamespace(receptionist_bag_state='ready', receptionist_bag_held=False,
        receptionist_bag_model='recepcionista_bolsa', receptionist_seated_guests=[
            {'guest_index': 2, 'xy': [3.5, -1.5]}], person_xy=[0.5, 1.8],
        receptionist_bag_pose_update=0.0,
        _publish_receptionist_task_state=lambda: None,
        _gz_run=calls.append, _lookup=lambda *args: ('rotation', [0.3, 0.0, 0.7]),
        get_logger=lambda: SimpleNamespace(info=lambda *args: None))
    node._gz_move_receptionist_bag_to_gripper = MethodType(
        ns['_gz_move_receptionist_bag_to_gripper'], node)
    ns['_gz_receptionist_handover'](node)
    assert calls[0] == ('recepcionista_bolsa', [3.15, -1.5, 0.95])
    assert calls[-1][1] == (0.3, 0.0, 0.86)
    assert node.receptionist_bag_held
    assert node.receptionist_bag_state == 'received'


def test_hri_requests_second_guest_bag_only_in_simulation():
    source = Path(__file__).resolve().parents[3] / 'src/base_athome_perception/scripts/hri_receptionist_challenge.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'HriReceptionistChallengeNode')
    methods = [next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
               for name in ('_after_guest_seated', '_sim_receptionist_task_command')]
    published = []
    subtitles = []
    ns = {'os': SimpleNamespace(environ={'ATHOME_SIM': '1'}),
          'String': lambda **kw: SimpleNamespace(**kw), 'time': SimpleNamespace(time=lambda: 12.0)}
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), 'exec'), ns)
    node = SimpleNamespace(guest_index=2, current_name='Morgan', state=15,
        STATE_SIM_BAG_HANDOVER=20, STATE_CHECK_NEXT_GUEST=12,
        pub_sim_receptionist_task=SimpleNamespace(publish=published.append),
        say=lambda text, **kw: subtitles.append(text),
        _introduce_guests_to_each_other=lambda: subtitles.append('introductions'))
    node._sim_receptionist_task_command = MethodType(ns['_sim_receptionist_task_command'], node)
    ns['_after_guest_seated'](node)
    assert subtitles == ['introductions', 'Morgan, may I take your bag, please?']
    assert published[0].data == 'handover'
    assert node.state == 20 and node.state_start_time == 12.0
