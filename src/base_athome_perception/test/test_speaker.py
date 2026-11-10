"""Complete synthesis precedes playback, with cancellable direct child processes."""
from contextlib import contextmanager
from pathlib import Path
import queue
import threading
import wave

import pytest
from base_athome_perception import speaker


@pytest.mark.parametrize('cancel_phase', [None, 'synthesis', 'playback'])
def test_complete_wav_before_playback_and_stop(monkeypatch, cancel_phase):
    events = []
    locked = [False]
    node = speaker.RoboCupSpeaker.__new__(speaker.RoboCupSpeaker)
    node._queue = queue.Queue()
    node._running = True
    node._stop_generation = 0
    node._process_lock = threading.Lock()
    node._current_process = None
    node._is_speaking = False
    node.piper_bin, node.model_path = 'piper', 'model.onnx'
    node.ensure_bluetooth_sink = lambda: 'test_sink'
    paths = []

    @contextmanager
    def lock():
        events.append('lock')
        locked[0] = True
        yield
        locked[0] = False

    class Process:
        returncode = 0
        def __init__(self, argv, **kwargs):
            self.phase = 'synthesis' if argv[0] == 'piper' else 'playback'
            events.append(self.phase)
            assert not kwargs.get('shell', False)
            if self.phase == 'synthesis':
                assert not locked[0]
                assert argv[:3] == ['piper', '--model', 'model.onnx']
                paths.append(argv[-1])
                assert Path(argv[-1]).exists()
            else:
                assert locked[0] and 'complete' in events
                assert argv == ['paplay', '-d', 'test_sink', paths[0]]
                with wave.open(paths[0], 'rb') as wav:
                    assert wav.getnframes() == 17640 + 100
                    assert wav.readframes(17640) == b'\x00\x00' * 17640
                    assert wav.readframes(100) == b'\x01\x02' * 100
        def communicate(self, input):
            assert input == 'Literal $text `safe`.'
            with wave.open(paths[0], 'wb') as wav:
                wav.setparams((1, 2, 22050, 0, 'NONE', 'not compressed'))
                wav.writeframes(b'\x01\x02' * 100)
            events.append('complete')
            if cancel_phase == self.phase:
                node.stop()
        def wait(self):
            events.append('wait')
            if cancel_phase == self.phase:
                node.stop()
        def poll(self):
            return None
        def terminate(self):
            events.append('terminate')
            self.returncode = -15

    def done():
        node._running = False
        events.append('done')
    monkeypatch.setattr(speaker, 'playback_lock', lock)
    monkeypatch.setattr(speaker.subprocess, 'Popen', Process)
    node._queue.put(('Literal $text `safe`.', done))
    node._worker_loop()
    assert not Path(paths[0]).exists()
    assert node._current_process is None and not node.is_speaking
    assert events[-1] == 'done'
    if cancel_phase == 'synthesis':
        assert 'playback' not in events and 'terminate' in events
    else:
        assert events[:4] == ['synthesis', 'complete', 'lock', 'playback']
        assert ('terminate' in events) == (cancel_phase == 'playback')
