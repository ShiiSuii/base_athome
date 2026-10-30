"""Pantalla tactil del invitado (sin ROS)."""
import json
import os
import sys
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception.guest_input_server import GuestInputServer  # noqa: E402


def test_page_question_and_input_roundtrip():
    got = []
    srv = GuestInputServer(['Robin', 'Jules'], ['Iced Tea'], got.append, host='127.0.0.1', port=0)
    try:
        base = f'http://127.0.0.1:{srv.port}'
        html = urllib.request.urlopen(base + '/').read().decode()
        assert '"Robin"' in html and '"Iced Tea"' in html
        srv.question = 'What is your name?'
        assert urllib.request.urlopen(base + '/question').read().decode() == 'What is your name?'
        req = urllib.request.Request(base + '/input', data=json.dumps({'type': 'name', 'value': 'Robin'}).encode(),
                                     headers={'Content-Type': 'application/json'})
        assert urllib.request.urlopen(req).read() == b'ok'
        assert got == [{'type': 'name', 'value': 'Robin'}]
    finally:
        srv.stop()
