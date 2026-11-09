"""GPSR visual: comandos oficiales, confirmacion y handler sin nodo ni DDS."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from base_athome_skills.mission.command_parser import compile_gpsr_actions
from base_athome_skills.mission.gpsr_plan import gpsr_actions
from base_athome_skills.mission.skills import default_registry, SkillResult
from base_athome_skills.mission.voice_dialog import plan_to_speech, needs_confirmation

ROOT = Path(__file__).resolve().parents[3]
ROWS = [json.loads(line) for line in (ROOT / 'tools/gpsr_parser/data/commands.jsonl').read_text().splitlines()]


@pytest.mark.parametrize('command,descriptor,following', [
    ('go to the trashbin then find the person raising their left arm and follow them to the armchair',
     {'gesture': 'raising their left arm'}, ['speak', 'follow_person']),
    ('go to the shelf then find the person raising their right arm and follow them to the bathroom',
     {'gesture': 'raising their right arm'}, ['speak', 'follow_person']),
    ('look for a person pointing to the left in the bathroom and lead them to the kitchen',
     {'gesture': 'pointing to the left'}, ['speak', 'navigate_to', 'speak']),
    ('get a cup from the storage rack and bring it to the person pointing to the right in the office',
     {'gesture': 'pointing to the right'}, ['speak', 'give_object']),
    ('find an orange in the bathroom then grasp it and give it to the waving person in the living room',
     {'gesture': 'waving'}, ['speak', 'give_object']),
    ('fetch a snack from the shelf and bring it to the sitting person in the office',
     {'posture': 'sitting'}, ['speak', 'give_object']),
    ('take the lying person from the refrigerator to the bathroom',
     {'posture': 'lying'}, ['speak', 'navigate_to', 'speak']),
    ('navigate to the sofa then look for an iced tea and get it and give it to the standing person in the kitchen',
     {'posture': 'standing'}, ['speak', 'give_object']),
    ('take the person wearing a blue sweater from the cabinet to the refrigerator',
     {'clothing': 'blue sweater'}, ['speak', 'navigate_to', 'speak']),
    ('take the person wearing a red jacket from the refrigerator to the side tables',
     {'clothing': 'red jacket'}, ['speak', 'navigate_to', 'speak']),
    ('lead the person wearing an orange jacket from the trashbin to the office',
     {'clothing': 'orange jacket'}, ['speak', 'navigate_to', 'speak']),
])
def test_official_commands_preserve_descriptor_and_following_steps(command, descriptor, following):
    assert command in {row['command'] for row in ROWS}
    actions = gpsr_actions(command)
    places = {str(a['params'][k]).replace(' ', '_'): (0., 0., 0.)
              for a in actions for k in ('location', 'destination') if a.get('params', {}).get(k)}
    result = compile_gpsr_actions(actions, places)
    assert result.ok, result.errors
    steps = result.plan['steps']
    target = next(i for i, step in enumerate(steps) if step['skill'] == 'find_person')
    assert steps[target]['args'] == descriptor
    assert [step['skill'] for step in steps[target + 1:]] == following
    assert needs_confirmation(result.plan, default_registry())
    assert all(value in plan_to_speech(result.plan) for value in descriptor.values())


def test_name_remains_explicitly_unsupported():
    res = compile_gpsr_actions([dict(action='find_person', params=dict(name='Angel', gesture='waving'))], {})
    assert not res.ok
    assert 'dialogo de confirmacion por voz' in res.errors[0]


def test_find_person_speech():
    assert plan_to_speech(dict(steps=[dict(skill='find_person', args=dict(gesture='raising their left arm'))])) == 'I will find the person raising their left arm.'


def test_find_person_registry_and_combined_descriptor():
    registry = default_registry()
    spec = registry.get('find_person')
    assert spec.status == 'available' and spec.moves_hardware
    assert spec.check_args({'name': 'Angel'})
    descriptor = dict(gesture='waving', posture='standing', clothing='red jacket')
    result = compile_gpsr_actions([dict(action='find_person', params=descriptor)], {})
    assert result.ok and result.plan['steps'][0]['args'] == descriptor
    assert needs_confirmation(result.plan, registry)
    assert plan_to_speech(result.plan) == 'I will find the person waving and standing and wearing red jacket.'


def test_unqualified_person_keeps_existing_approach():
    result = compile_gpsr_actions([dict(action='find_person', params={})], {})
    assert result.ok and result.plan['steps'][0]['skill'] == 'approach_person'


def test_delivery_keeps_the_person_selected_by_descriptor():
    actions = gpsr_actions('fetch a snack from the shelf and bring it to the sitting person in the office')
    result = compile_gpsr_actions(actions, dict(shelf=(0., 0., 0.), office=(2., 0., 0.)))
    assert result.ok, result.errors
    steps = result.plan['steps']
    target = next(i for i, step in enumerate(steps) if step['skill'] == 'find_person')
    assert [step['skill'] for step in steps[target:]] == ['find_person', 'speak', 'give_object']


@pytest.mark.parametrize('value', [42, ['waving'], {'gesture': 'waving'}, '   '])
def test_handler_rejects_invalid_descriptor_before_request(value):
    from base_athome_skills.mission.node import MissionExecutorNode

    node = SimpleNamespace(_ask_finder=lambda *args, **kwargs: pytest.fail('invalid descriptor sent'))
    ctx = SimpleNamespace(args=dict(gesture=value), remaining=lambda: 60.)
    result = MissionExecutorNode.h_find_person(node, ctx)
    assert not result.ok and result.error == 'invalid_descriptor'


@pytest.mark.parametrize('person', [None, {}, {'xy': [float('nan'), 0.]}, {'xy': ['bad', 0.]}, {'xy': [1.]}])
def test_handler_rejects_invalid_result_before_navigation(person):
    from base_athome_skills.mission.node import MissionExecutorNode

    node = SimpleNamespace(
        _ask_finder=lambda *args, **kwargs: (dict(person=person), None),
        robot_pose=(0., 0., 0.), _pose_now=lambda: (0., 0., 0.),
        _navigate=lambda *args: pytest.fail('invalid position sent to navigation'))
    ctx = SimpleNamespace(args=dict(gesture='waving'), remaining=lambda: 60.)
    result = MissionExecutorNode.h_find_person(node, ctx)
    assert not result.ok and result.error == 'no_person'


def test_finder_reason_only_failure_is_preserved():
    import threading
    from base_athome_skills.mission.node import MissionExecutorNode

    results = {}
    def publish(msg):
        request = json.loads(msg.data)
        results[request['id']] = dict(ok=False, reason='no_matching_person')

    node = SimpleNamespace(bag_results=results, pub_bag_req=SimpleNamespace(
        get_subscription_count=lambda: 1, publish=publish))
    response, error = MissionExecutorNode._ask_finder(
        node, SimpleNamespace(cancel=threading.Event()), 'find_person', 1.)
    assert response is None and error.error == 'no_matching_person'


@pytest.mark.parametrize('failure,pose', [(True, (0., 0., 0.)), (False, None), (False, (0., 0., 0.))])
def test_handler_uses_selected_position_or_propagates_failure(failure, pose):
    from base_athome_skills.mission.node import MissionExecutorNode
    from base_athome_skills.mission import bag_task
    calls = []
    err = SkillResult.failure('no_matching_person', 'no match') if failure else None
    def ask(ctx, mode, timeout, **extra):
        calls.append((mode, extra))
        return dict(person=dict(xy=[3., 2.])), err
    node = SimpleNamespace(_ask_finder=ask, robot_pose=pose, _pose_now=lambda: pose,
                           _navigate=lambda ctx, label, goal: calls.append(goal) or SkillResult.success())
    ctx = SimpleNamespace(args=dict(gesture='waving'), remaining=lambda: 60.)
    res = MissionExecutorNode.h_find_person(node, ctx)
    assert calls[0] == ('find_person', dict(descriptor=dict(gesture='waving')))
    if failure:
        assert res is err and len(calls) == 1
    elif pose is None:
        assert not res.ok and len(calls) == 1
    else:
        assert res.ok and calls[1] == bag_task.person_stand_pose([3., 2.], pose)


def test_handler_does_not_navigate_when_selected_person_is_close():
    from base_athome_skills.mission.node import MissionExecutorNode

    node = SimpleNamespace(
        _ask_finder=lambda *args, **kwargs: (dict(person=dict(xy=[1., 0.])), None),
        robot_pose=(0., 0., 0.), _pose_now=lambda: (0., 0., 0.),
        _navigate=lambda *args: pytest.fail('already close'))
    result = MissionExecutorNode.h_find_person(node, SimpleNamespace(
        args=dict(posture='sitting'), remaining=lambda: 60.))
    assert result.ok and result.data['person'] == [1., 0.]
