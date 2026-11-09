"""El lanzador no deja procesos vivos. Con `sleep` marcados: sin ROS y sin tocar el robot."""
import os
import fcntl
import pty
import signal
import shlex
import subprocess
import time
import termios
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


def test_cleanup_escalates_only_marked_processes_that_ignore_term(tmp_path):
    ready = tmp_path / 'ready'
    worker = ('import os,signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
              f'open({str(ready)!r}, "w").write(str(os.getpid())); time.sleep(4808)')
    script = (f'. "{LIB}"; session_init "{tmp_path}/log"; '
              f'session_start stubborn python3 -c {shlex.quote(worker)}; wait')
    other = subprocess.Popen(['sleep', '4809'])
    p = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE, text=True)
    pid = None
    try:
        assert _until(ready.exists)
        pid = int(ready.read_text())
        started = time.monotonic()
        p.send_signal(signal.SIGINT)
        p.wait(timeout=14)
        assert p.returncode == 130
        assert not Path(f'/proc/{pid}').exists()
        assert time.monotonic() - started < 13
        assert other.poll() is None
    finally:
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=15)
        if pid and Path(f'/proc/{pid}').exists():
            os.kill(pid, signal.SIGKILL)  # solo el doble de este test
        other.terminate()
        other.wait(timeout=5)


@pytest.mark.parametrize('hangup', [False, True], ids=['ctrl-c', 'terminal-close'])
def test_visible_node_keeps_terminal_input_and_does_not_delay_traps(tmp_path, hangup):
    ready, entered = tmp_path / 'ready', tmp_path / 'entered'
    worker = ('import os,sys,time; '
              f'open({str(ready)!r}, "w").write(str(os.getpid())); '
              'assert sys.stdin.isatty(); line=sys.stdin.readline(); '
              f'open({str(entered)!r}, "w").write(line); time.sleep(4810)')
    script = (f'. "{LIB}"; session_init "{tmp_path}/log"; '
              f'session_run_terminal python3 -c {shlex.quote(worker)}')
    master, slave = pty.openpty()
    def terminal_session():
        os.setsid()
        fcntl.ioctl(0, termios.TIOCSCTTY, 0)
    p = subprocess.Popen(['bash', '-c', script], stdin=slave, stdout=slave,
                         stderr=slave, preexec_fn=terminal_session)
    os.close(slave)
    try:
        assert _until(ready.exists)
        os.write(master, b' \n')
        assert _until(entered.exists)
        assert entered.read_text() == ' \n'
        if hangup:
            os.close(master)
            master = None
        else:
            os.write(master, b'\x03')  # Ctrl-C al grupo con foco, no al nodo con setsid
        p.wait(timeout=3)
        assert p.returncode == (129 if hangup else 130)
        assert not Path(f'/proc/{ready.read_text()}').exists()
    finally:
        if master is not None:
            os.close(master)
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=15)


def _running(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def test_orphan_cleanup_waits_and_kills_detached_marked_descendants(tmp_path):
    ready = tmp_path / 'ready'
    worker = ('import os,signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); '
              f'open({str(ready)!r}, "w").write(str(os.getpid())); time.sleep(4811)')
    spawner = ('import subprocess,sys,time; '
               f'subprocess.Popen([sys.executable, "-c", {worker!r}], start_new_session=True); '
               'time.sleep(4812)')
    script = (f'. "{LIB}"; session_init "{tmp_path}/old"; '
              f'session_start a python3 -c {shlex.quote(spawner)}; wait')
    p = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE, text=True)
    pid = None
    try:
        assert _until(ready.exists)
        pid = int(ready.read_text())
        p.kill()
        p.wait(timeout=5)
        assert _running(pid)
        q = subprocess.run(['bash', '-c', f'. "{LIB}"; SESSION_TERM_TIMEOUT=0.3; '
                            f'session_init "{tmp_path}/new"'], capture_output=True, text=True, timeout=5)
        assert q.returncode == 0, q.stderr
        assert 'Cerre 2 procesos' in q.stdout
        assert not _running(pid)
    finally:
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=15)
        if pid and _running(pid):
            os.kill(pid, signal.SIGKILL)


def _graph_check(tmp_path, snapshot, orphans=0, prelude=''):
    """Solo define funciones extraidas; nunca ejecuta el lanzador ni un CLI ROS."""
    script = LIB.with_name('restaurant.sh').read_text()
    functions = script[script.index('restaurant_graph_snapshot()'):script.index('restaurant_nav_ready()')]
    fake = tmp_path / 'python3'
    fake.write_text('#!/bin/bash\n' + snapshot)
    fake.chmod(0o755)
    env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}')
    return subprocess.run(['bash', '-c', functions +
                           f'\n{prelude}\nSESSION_ORPHANS_STOPPED={orphans}; restaurant_graph_idle'],
                          env=env, capture_output=True, text=True, timeout=24)


def test_graph_waits_for_orphan_nodes_and_actions_to_disappear(tmp_path):
    count = tmp_path / 'count'
    snapshot = (f'if [ ! -e "{count}" ]; then touch "{count}"; '
                'echo "node /slam_toolbox"; echo "action /navigate_to_pose"; fi\n')
    result = _graph_check(tmp_path, snapshot, orphans=2)
    assert result.returncode == 0, result.stdout + result.stderr
    assert count.exists()


def test_graph_rejection_names_both_conflicting_nodes_and_actions(tmp_path):
    result = _graph_check(tmp_path, 'echo "node /other/amcl"; echo "action /navigate_to_pose"\n')
    assert result.returncode == 1
    assert 'node /other/amcl' in result.stdout
    assert 'action /navigate_to_pose' in result.stdout


def test_graph_query_failure_rejects_start(tmp_path):
    result = _graph_check(tmp_path, 'exit 1\n', orphans=1)
    assert result.returncode == 1
    assert 'No se pudo comprobar' in result.stdout


def test_graph_wait_is_bounded_and_reports_remaining_conflicts(tmp_path):
    count = tmp_path / 'count'
    snapshot = (f'echo query >> "{count}"\n'
                'echo "node /slam_toolbox"; echo "action /navigate_to_pose"\n')
    # Reloj de Bash adelantado: ejercita el limite real de 20 s sin esperar 20 s.
    result = _graph_check(tmp_path, snapshot, orphans=2,
                          prelude='sleep() { SECONDS=$((SECONDS + 10)); }')
    assert result.returncode == 1
    assert count.read_text().splitlines() == ['query', 'query']
    assert 'node /slam_toolbox' in result.stdout
    assert 'action /navigate_to_pose' in result.stdout


def test_kill_never_targets_unmarked_sibling_in_same_group(tmp_path):
    ready, other_ready = tmp_path / 'ready', tmp_path / 'other'
    other_code = ('import os,time; '
                  f'open({str(other_ready)!r}, "w").write(str(os.getpid())); time.sleep(4813)')
    worker = ('import os,signal,subprocess,sys,time; '
              'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
              'env=dict(os.environ); env.pop("BASE_ATHOME_SESSION", None); '
              f'subprocess.Popen([sys.executable, "-c", {other_code!r}], env=env); '
              f'open({str(ready)!r}, "w").write(str(os.getpid())); time.sleep(4814)')
    script = (f'. "{LIB}"; SESSION_TERM_TIMEOUT=0.3; session_init "{tmp_path}/log"; '
              f'session_start stubborn python3 -c {shlex.quote(worker)}; wait')
    p = subprocess.Popen(['bash', '-c', script], stdout=subprocess.PIPE, text=True)
    other_pid = None
    try:
        assert _until(lambda: ready.exists() and other_ready.exists())
        pid, other_pid = int(ready.read_text()), int(other_ready.read_text())
        assert os.getpgid(pid) == os.getpgid(other_pid)
        p.send_signal(signal.SIGINT)
        p.wait(timeout=5)
        assert not _running(pid)
        assert _running(other_pid), 'ni TERM ni KILL pueden alcanzar un proceso sin la marca'
    finally:
        if p.poll() is None:
            p.terminate()
            p.wait(timeout=15)
        if other_pid and _running(other_pid):
            os.kill(other_pid, signal.SIGTERM)
