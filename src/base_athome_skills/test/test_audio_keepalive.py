"""Keepalive y exclusion TTS con reloj/subprocesos falsos; nunca abre audio."""

from contextlib import contextmanager
import importlib.util
import json
from pathlib import Path
import queue
import struct
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

# Probar las bibliotecas de este worktree, aunque install apunte al checkout de Claude.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'base_athome_perception'))
from base_athome_perception.audio_lock import playback_lock
from base_athome_perception import speaker

path = Path(__file__).parents[1] / 'scripts/audio_keepalive.py'
spec = importlib.util.spec_from_file_location('audio_keepalive', path)
keepalive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(keepalive)


class FakeAudio:
    def __init__(self, sinks=None, default='alsa_output.default'):
        self.sinks = sinks if sinks is not None else [
            {'name': 'alsa_output.default', 'state': 'IDLE'},
            {'name': 'bluez_output.speaker', 'state': 'SUSPENDED'},
        ]
        self.default = default
        self.calls = []

    def run(self, args, **kwargs):
        self.calls.append((args, kwargs))
        if args[0] == 'paplay':
            return NS(returncode=0)
        return NS(stdout=self.default if args[1] == 'get-default-sink' else json.dumps(self.sinks))


@contextmanager
def free_lock(**kwargs):
    assert kwargs == {'blocking': False}
    yield True


def test_default_interval_waits_5_seconds_and_plays_once_per_cycle():
    """El usuario pidio un sonido cada 5 s: el parlante Bluetooth se apagaba antes."""
    now, played = [0.], []
    timer = keepalive.AudioKeepalive(clock_fn=lambda: now[0], play_fn=lambda: played.append(now[0]) or True)
    assert not timer.tick()
    now[0] = 4.999
    assert not timer.tick()
    now[0] = 5.
    assert timer.tick() and played == [5.]
    assert not timer.tick()
    now[0] = 10.
    assert timer.tick() and played == [5., 10.]


def test_configurable_interval_skips_busy_cycle_without_catchup():
    now, attempts = [0.], []
    answers = iter([False, True])

    def play():
        attempts.append(now[0])
        return next(answers)

    timer = keepalive.AudioKeepalive(3., clock_fn=lambda: now[0], play_fn=play)
    now[0] = 3.
    assert not timer.tick()
    now[0] = 5.99
    assert not timer.tick() and attempts == [3.]
    now[0] = 6.
    assert timer.tick() and attempts == [3., 6.]


@pytest.mark.parametrize('interval', [0., -1., float('inf'), float('nan')])
def test_invalid_interval_rejected(interval):
    with pytest.raises(ValueError):
        keepalive.AudioKeepalive(interval)


def test_tone_is_150_ms_soft_but_audible_pcm_with_smooth_endpoints():
    tone = keepalive.quiet_tone()
    samples = struct.unpack('<' + 'h' * (len(tone) // 2), tone)
    assert len(samples) == 2400
    assert samples[0] == samples[-1] == 0
    assert 1000 < max(abs(v) for v in samples) <= 2000      # ~6 %: al 0.3 % no se oia


def test_prefers_bluetooth_and_plays_generated_tone():
    audio = FakeAudio()
    assert keepalive.play_if_idle(audio.run, free_lock)
    args, kwargs = audio.calls[-1]
    assert args[:3] == ['paplay', '--device', 'bluez_output.speaker']
    assert kwargs['input'] == keepalive.quiet_tone()
    assert kwargs['timeout'] == 5.          # un parlante recien despierto tarda


def test_no_beep_on_a_wired_speaker(monkeypatch):
    """Con el parlante por cable no hay nada que mantener despierto: no se bipea."""
    monkeypatch.delenv('AUDIO_SPEAKER', raising=False)
    audio = FakeAudio([{'name': 'alsa_output.default', 'state': 'IDLE'}])
    assert not keepalive.play_if_idle(audio.run, free_lock)
    assert not any(c[0][0] == 'paplay' for c in audio.calls)


def test_bluetooth_mode_reconnects_then_falls_back_to_default(monkeypatch):
    monkeypatch.setenv('AUDIO_SPEAKER', 'bluetooth')
    monkeypatch.setattr(keepalive, 'reconnect_bluetooth_speaker', lambda: False)
    audio = FakeAudio([{'name': 'alsa_output.default', 'state': 'IDLE'}])
    assert keepalive.play_if_idle(audio.run, free_lock)
    assert audio.calls[-1][0][:3] == ['paplay', '--device', audio.default]


@pytest.mark.parametrize('bluetooth', [True, False])
@pytest.mark.parametrize('state', ['RUNNING', 'UNKNOWN'])
def test_running_or_unknown_sink_skips_audio(bluetooth, state):
    name = 'bluez_output.speaker' if bluetooth else 'alsa_output.default'
    audio = FakeAudio([{'name': name, 'state': state}])
    assert not keepalive.play_if_idle(audio.run, free_lock)
    assert all(args[0] != 'paplay' for args, _ in audio.calls)


def test_no_sink_skips_audio():
    audio = FakeAudio([])
    assert not keepalive.play_if_idle(audio.run, free_lock)
    assert all(args[0] != 'paplay' for args, _ in audio.calls)


def test_shared_lock_skips_even_when_sink_idle_during_tts_synthesis(tmp_path):
    path = tmp_path / 'audio.lock'
    audio = FakeAudio()
    with playback_lock(path=path) as acquired:
        assert acquired
        assert not keepalive.play_if_idle(audio.run, lambda **kw: playback_lock(path=path, **kw))
    assert audio.calls == []
    assert keepalive.play_if_idle(audio.run, lambda **kw: playback_lock(path=path, **kw))


def test_audio_errors_skip_cycle_and_preserve_interval():
    now = [0.]

    def failed():
        raise subprocess.TimeoutExpired('pactl', 1.)

    timer = keepalive.AudioKeepalive(2., clock_fn=lambda: now[0], play_fn=failed)
    now[0] = 2.
    assert not timer.tick() and timer.next_due == 4.


def test_playback_exception_releases_lock(tmp_path):
    path = tmp_path / 'audio.lock'

    def failed(*_, **kw):
        raise FileNotFoundError('pactl')

    with pytest.raises(FileNotFoundError):
        keepalive.play_if_idle(failed, lambda **kw: playback_lock(path=path, **kw))
    with playback_lock(blocking=False, path=path) as acquired:
        assert acquired


def test_speaker_file_playback_holds_same_lock(monkeypatch, tmp_path):
    path = tmp_path / 'audio.lock'
    monkeypatch.setattr(speaker, 'playback_lock', lambda: playback_lock(path=path))
    player = object.__new__(speaker.RoboCupSpeaker)
    player.ensure_bluetooth_sink = lambda: 'bluez_output.speaker'
    wav = tmp_path / 'synthetic.wav'
    wav.write_bytes(b'')
    calls = []

    def run(*args, **kwargs):
        with playback_lock(blocking=False, path=path) as acquired:
            assert not acquired
        calls.append(args)

    monkeypatch.setattr(speaker.subprocess, 'run', run)
    player.play_audio_file(str(wav), blocking=True)
    assert len(calls) == 1


def test_speaker_holds_lock_through_tts_synthesis_and_playback(monkeypatch, tmp_path):
    path = tmp_path / 'audio.lock'
    monkeypatch.setattr(speaker, 'playback_lock', lambda: playback_lock(path=path))
    player = object.__new__(speaker.RoboCupSpeaker)
    player._queue = queue.Queue()
    player._queue.put(('water', lambda: None))
    player._running = True
    player.ensure_bluetooth_sink = lambda: None
    player.piper_bin, player.model_path, player.sample_rate = 'fake_piper', 'fake_model', 22050
    calls = []

    def wait():
        with playback_lock(blocking=False, path=path) as acquired:
            assert not acquired
        player._running = False

    def popen(*args, **kwargs):
        with playback_lock(blocking=False, path=path) as acquired:
            assert not acquired
        calls.append(args)
        return NS(wait=wait)

    monkeypatch.setattr(speaker.subprocess, 'Popen', popen)
    player._worker_loop()
    assert len(calls) == 1
    assert not player.is_speaking


def test_reconnects_a_paired_speaker_that_dropped():
    """El parlante se apaga solo: si esta emparejado y desconectado, se le pide reconectar."""
    from types import SimpleNamespace as NS
    from base_athome_perception import audio_lock
    calls = []

    def run(cmd, **_):
        calls.append(cmd)
        if cmd[:3] == ["bluetoothctl", "devices", "Paired"]:
            return NS(stdout="Device 32:16:82:08:30:02 PPA 205\nDevice AA:BB:CC:DD:EE:FF Phone\n")
        if cmd[:2] == ["bluetoothctl", "info"]:
            if cmd[2].startswith("32:"):
                return NS(stdout="Connected: no\n\tUUID: Audio Sink (0000110b-0000-1000-8000-00805f9b34fb)\n")
            return NS(stdout="Connected: yes\n\tUUID: Handsfree (0000111e-0000-1000-8000-00805f9b34fb)\n")
        if cmd[:2] == ["bluetoothctl", "connect"]:
            return NS(stdout="Attempting to connect\nConnection successful\n")
        raise AssertionError(cmd)
    assert audio_lock.reconnect_bluetooth_speaker(run)
    assert ["bluetoothctl", "connect", "32:16:82:08:30:02"] in calls
    assert ["bluetoothctl", "connect", "AA:BB:CC:DD:EE:FF"] not in calls      # el telefono no es un parlante
