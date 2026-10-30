"""Interpretacion de pedidos para el secuenciador de misiones."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception.mission_intents import describe, parse_commands  # noqa: E402

OBJ = {'obj_1': {'number': 1, 'name': 'pringles', 'class': 'can'}, 'obj_2': {'number': 2, 'name': 'coke', 'class': 'can'}}
WPS = ['door', 'living_room']


def test_go_to_door_and_receive():
    assert parse_commands('Go to the door and receive the guest', OBJ, WPS) == [('go_to', 'door'), ('receive_guest', None)]


def test_spanish_receive():
    assert parse_commands('andá a la puerta y recibí a la persona', OBJ, WPS) == [('go_to', 'door'), ('receive_guest', None)]


def test_fetch_by_name_number_and_class():
    assert parse_commands('take the pringles from the table', OBJ, WPS) == [('fetch', 'obj_1')]
    assert parse_commands('bring me object number 2', OBJ, WPS) == [('fetch', 'obj_2')]
    assert parse_commands('agarrá el objeto 1', OBJ, WPS) == [('fetch', 'obj_1')]


def test_guide_then_place():
    steps = parse_commands('take the guest to the living room and leave the pringles there', OBJ, WPS)
    assert steps == [('guide', 'living_room'), ('place', None)]
    steps = parse_commands('llevá a la persona hasta la sala de estar y soltá el pringles ahí', OBJ, WPS)
    assert steps == [('guide', 'living_room'), ('place', None)]


def test_full_mission_keyword():
    steps = parse_commands('do the mission', OBJ, WPS)
    assert steps == [('go_to', 'door'), ('receive_guest', None), ('fetch', 'obj_1'), ('guide', 'living_room'),
                     ('place', None), ('go_home', None)]


def test_unknown_text_gives_no_steps_and_describe():
    assert parse_commands('what a nice day', OBJ, WPS) == []
    assert describe([('go_to', 'door'), ('fetch', 'obj_1')], OBJ) == 'go to the door and take the pringles'
