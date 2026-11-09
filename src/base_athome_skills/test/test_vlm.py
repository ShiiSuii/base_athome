"""Tests unitarios del cliente VLM. Sin modelo real: servidor simulado con http.server."""
import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from base_athome_skills.mission import vlm


class MockVLMHandler(BaseHTTPRequestHandler):
    answer = "a red cup on the kitchen table"
    malformed = False

    def log_message(self, *_):
        pass

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        assert "image" in req and "question" in req
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        if self.malformed:
            self.wfile.write(b"not-json")
        else:
            self.wfile.write(json.dumps(dict(answer=self.answer)).encode("utf-8"))


@pytest.fixture
def mock_server():
    server = HTTPServer(("127.0.0.1", 0), MockVLMHandler)
    port = server.server_port
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{port}"
    MockVLMHandler.malformed = False
    MockVLMHandler.answer = "a red cup on the kitchen table"
    yield url
    server.shutdown()
    server.server_close()


def test_encode_image_bytes():
    raw = b"\xff\xd8\xff\xe0testjpeg"
    encoded = vlm.encode_image_bytes(raw)
    assert base64.b64decode(encoded.encode("ascii")) == raw


def test_ask_vlm_empty_returns_none():
    assert vlm.ask_vlm(b"", "what is this?") is None
    assert vlm.ask_vlm(b"dummy", "") is None


def test_ask_vlm_offline_server_returns_none():
    # Puerto inactivo sin lanzar excepcion
    assert vlm.ask_vlm(b"dummy", "what is this?", url="http://127.0.0.1:9/none", timeout_s=0.5) is None


def test_ask_vlm_success(mock_server):
    res = vlm.ask_vlm(b"sample_jpeg", "what do you see?", url=mock_server, timeout_s=2.0)
    assert res == "a red cup on the kitchen table"


def test_ask_vlm_malformed_response(mock_server):
    MockVLMHandler.malformed = True
    res = vlm.ask_vlm(b"sample_jpeg", "what do you see?", url=mock_server, timeout_s=2.0)
    assert res is None


def test_ask_vlm_cv_with_mock(mock_server):
    try:
        import numpy as np
        img = np.zeros((64, 64, 3), dtype=np.uint8)
        res = vlm.ask_vlm_cv(img, "what color?", url=mock_server, timeout_s=2.0)
        assert res == "a red cup on the kitchen table"
    except ImportError:
        pytest.skip("numpy / cv2 no disponibles")


def test_short_answer_keeps_two_sentences():
    from base_athome_skills.mission import vlm
    assert vlm.short_answer("A cup. A table!  A dog? A cat.") == "A cup. A table."
    assert vlm.short_answer("") == ""


def test_questions_from_official_gpsr_params():
    from base_athome_skills.mission import vlm
    q = vlm.count_question(dict(target="people", clothing="gray jackets"))
    assert q == "How many people wearing gray jackets do you see? Answer with one number."
    assert "largest drink" in vlm.describe_question(dict(property="largest", target="drink"))
    assert vlm.describe_question({}) == vlm.DEFAULT_QUESTION
