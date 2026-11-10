"""Synthetic PCM and injected sources only: never import/open real audio."""

import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray

from base_athome_perception.doorbell_audio_publisher import (
    DoorbellAudioPublisher, Pcm16Blocks, SoundDeviceSource,
)


@pytest.mark.parametrize("block_ms,size", [(100, 1600), (50, 800), (20, 320)])
def test_resample_stereo_and_frame_arbitrary_chunks(block_ms, size):
    t = np.arange(48000) / 48000
    tone = (12000 * np.sin(2 * np.pi * 1000 * t)).astype(np.int16)
    stereo = np.column_stack([tone, tone // 2])
    converter = Pcm16Blocks(block_ms)
    blocks = []
    for start in range(0, len(stereo), 137):
        blocks.extend(converter.feed(stereo[start:start + 137]))
    assert all(b.dtype == np.int16 and b.shape == (size,) for b in blocks)
    output = np.concatenate(blocks)
    expected = stereo.astype(np.float64).mean(axis=1).reshape(-1, 3).mean(axis=1)
    np.testing.assert_allclose(output, np.rint(expected), atol=0)
    assert np.argmax(abs(np.fft.rfft(output))) == 1000  # 1 s @ 16 kHz


@pytest.mark.parametrize("block_ms", [0, -1, 101, float("nan"), float("inf"), 0.01])
def test_invalid_block_duration(block_ms):
    with pytest.raises(ValueError):
        Pcm16Blocks(block_ms)


class FakeSource:
    def __init__(self, device, frames):
        self.device, self.frames = device, frames
        self.reads = 0
        self.closed = False
        self.pending = []

    def read(self):
        assert not self.closed
        self.reads += 1
        return self.pending.pop(0) if self.pending else None

    def close(self):
        self.closed = True


@pytest.fixture
def audio_node(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    context = rclpy.context.Context()
    context.init(args=[])
    sources = []

    def factory(device, frames):
        source = FakeSource(device, frames)
        sources.append(source)
        return source

    node = DoorbellAudioPublisher(
        context=context, source_factory=factory,
        parameter_overrides=[Parameter("device", value="fake microphone"),
                             Parameter("topic", value="/test/doorbell_audio")])
    yield node, sources, context
    node.destroy_node()
    context.shutdown()


def test_no_capture_without_subscribers_and_discard_on_disconnect(audio_node, monkeypatch):
    node, sources, _ = audio_node
    subscribers = [0]
    monkeypatch.setattr(node.publisher, "get_subscription_count", lambda: subscribers[0])
    for _ in range(5):
        node._poll()
    assert sources == []
    subscribers[0] = 1
    node._poll()
    first = sources[0]
    assert (first.device, first.frames) == ("fake microphone", 4800)
    first.pending = [np.full((2400, 1), 123, np.int16)]  # half a block
    node._poll()
    subscribers[0] = 0
    reads = first.reads
    node._poll()
    node._poll()
    assert first.closed and first.reads == reads
    subscribers[0] = 1
    node._poll()
    assert len(sources) == 2
    published = []
    monkeypatch.setattr(node.publisher, "publish", published.append)
    sources[1].pending = [np.full((2400, 1), 456, np.int16)]
    node._poll()
    assert published == []  # old half block was discarded
    sources[1].pending = [np.full((2400, 1), 456, np.int16)]
    node._poll()
    assert list(published[0].data) == [456] * 1600
    node.destroy_node()
    assert sources[1].closed


def test_ros_delivery_sensor_qos_and_shutdown(audio_node):
    import time

    node, sources, context = audio_node
    received = []
    listener = rclpy.create_node("fake_doorbell_listener", context=context)
    listener.create_subscription(Int16MultiArray, "/test/doorbell_audio",
                                 received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(listener)
    try:
        deadline = time.monotonic() + 4
        while not sources and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert sources
        sources[0].pending = [np.full((4800, 1), -32768, np.int16)]
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(received) == 1
        assert list(received[0].data) == [-32768] * 1600
        qos = node.publisher.qos_profile
        assert qos.reliability == qos_profile_sensor_data.reliability
        assert qos.durability == qos_profile_sensor_data.durability
        listener.destroy_node()
        while not sources[0].closed and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert sources[0].closed
    finally:
        executor.shutdown()
        listener.destroy_node()


def test_sounddevice_backend_is_lazy_and_uses_existing_capture_contract(monkeypatch):
    import sys
    from types import SimpleNamespace

    streams = []

    class Stream:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.started = self.closed = False
            streams.append(self)

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

        def close(self):
            self.closed = True

    monkeypatch.setitem(sys.modules, "sounddevice", SimpleNamespace(InputStream=Stream))
    source = SoundDeviceSource("3", 4800)
    stream = streams[0]
    assert stream.started
    assert {k: stream.kwargs[k] for k in (
        "device", "samplerate", "channels", "dtype", "blocksize")} == {
            "device": 3, "samplerate": 48000, "channels": 1,
            "dtype": "int16", "blocksize": 4800}
    block = np.ones((4800, 1), np.int16)
    stream.kwargs["callback"](block, 4800, None, None)
    block[:] = 2
    assert (source.read() == 1).all()
    source.close()
    assert stream.closed and not stream.started


def test_capture_failure_retries_with_delay_and_stops_retrying_without_subscribers(audio_node, monkeypatch):
    import base_athome_perception.doorbell_audio_publisher as audio

    node, _, _ = audio_node
    subscribers, clock, attempts = [1], [10.0], []
    monkeypatch.setattr(node.publisher, "get_subscription_count", lambda: subscribers[0])
    monkeypatch.setattr(audio.time, "monotonic", lambda: clock[0])

    def unavailable(device, frames):
        attempts.append(clock[0])
        raise OSError("fake missing microphone")

    node.source_factory = unavailable
    node._poll()
    for _ in range(5):
        node._poll()
    assert attempts == [10]
    clock[0] = 12
    node._poll()
    assert attempts == [10, 12]
    subscribers[0], clock[0] = 0, 20
    node._poll()
    assert attempts == [10, 12]


@pytest.mark.parametrize("start_audio", [False, True])
def test_receptionist_companion_option_enables_wait_and_cleans_up(monkeypatch, start_audio):
    """Run only main with fake receptionist/executor; no heavy imports or hardware."""
    import argparse
    import ast
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    from unittest.mock import Mock
    import base_athome_perception.doorbell_audio_publisher as audio

    path = Path(__file__).resolve().parents[1] / "scripts/hri_receptionist_challenge.py"
    tree = ast.parse(path.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    receptionist = Mock(wait_doorbell=True, doorbell_audio_topic="/resolved/doorbell_audio")
    receptionist_factory = Mock(return_value=receptionist)
    companion = Mock()
    companion_factory = Mock(return_value=companion)
    monkeypatch.setattr(audio, "DoorbellAudioPublisher", companion_factory)
    argv = ["hri_receptionist", "--no-gui"]
    if start_audio:
        argv += ["--start-doorbell-audio", "--doorbell-audio-device", "USB fake"]
    monkeypatch.setattr(sys, "argv", argv)
    executor = Mock()
    namespace = {
        "argparse": argparse, "sys": sys, "check_and_fix_cyclonedds": Mock(),
        "remove_ros_args": lambda args: args,
        "HriReceptionistChallengeNode": receptionist_factory,
        "MultiThreadedExecutor": Mock(return_value=executor),
        "threading": SimpleNamespace(Thread=Mock()),
        "rclpy": SimpleNamespace(init=Mock(), ok=lambda: False),
    }
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(path), "exec"), namespace)
    namespace["main"]()
    assert receptionist_factory.call_args.kwargs["wait_doorbell"] is start_audio
    if start_audio:
        params = {p.name: p.value for p in companion_factory.call_args.kwargs["parameter_overrides"]}
        assert params == {"device": "USB fake", "topic": "/resolved/doorbell_audio"}
        executor.add_node.assert_any_call(companion)
        companion.destroy_node.assert_called_once()
    else:
        companion_factory.assert_not_called()
    executor.shutdown.assert_called_once()
    receptionist.destroy_node.assert_called_once()
