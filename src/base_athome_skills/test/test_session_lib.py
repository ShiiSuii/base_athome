"""El lanzador no deja procesos vivos. Con `sleep` marcados: sin ROS y sin tocar el robot."""
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

LIB = Path(__file__).resolve().parents[1] / "scripts" / "session_lib.sh"


def _alive(token):
    return subprocess.run(["pgrep", "-f", f"^sleep {token}$"], capture_output=True).returncode == 0


def _until(cond, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def _session(tmp_path, token):
    script = f'. "{LIB}"; session_init "{tmp_path}/log"; session_start a sleep {token}; wait'
    p = subprocess.Popen(["bash", "-c", script], stdout=subprocess.PIPE, text=True)
    assert _until(lambda: _alive(token))
    return p


@pytest.mark.parametrize("sig,token", [(signal.SIGHUP, 4801.5), (signal.SIGINT, 4802.5), (signal.SIGTERM, 4803.5)])
def test_closing_the_session_closes_its_processes(tmp_path, sig, token):
    p = _session(tmp_path, token)
    p.send_signal(sig)
    p.wait(timeout=15)
    assert not _alive(token)


def test_next_session_closes_what_a_killed_session_left(tmp_path):
    token = 4804.5
    p = _session(tmp_path, token)
    p.kill()                              # ni la trampa llega a correr
    p.wait(timeout=5)
    assert _until(lambda: _alive(token), 1)
    q = _session(tmp_path, 4805.5)
    assert _until(lambda: not _alive(token))
    assert _alive(4805.5), "no tiene que cerrar lo propio"
    q.send_signal(signal.SIGTERM)
    q.wait(timeout=15)


def test_does_not_touch_processes_without_the_mark(tmp_path):
    env = {k: v for k, v in os.environ.items() if k != "BASE_ATHOME_SESSION"}
    other = subprocess.Popen(["sleep", "4806.5"], env=env)
    try:
        p = _session(tmp_path, 4807.5)
        p.send_signal(signal.SIGTERM)
        p.wait(timeout=15)
        assert other.poll() is None
    finally:
        other.terminate()
