"""On-demand microphone PCM for the receptionist; no audio is saved to disk.

Uses the same sounddevice 48 kHz/int16 capture and three-sample averaging as
audio_asr/whisper_node.py. PortAudio is imported only when a subscriber exists.

Standalone: ros2 run base_athome_perception doorbell_audio_publisher --ros-args
  -p device:="USB microphone" -p topic:=/hri/doorbell/audio -p block_ms:=100
Receptionist: hri_receptionist --wait-doorbell --start-doorbell-audio
  --doorbell-audio-device "USB microphone"
device is a string (PortAudio name or numeric index); empty means default input.
block_ms must be <=100 to respect the detector's 1600-sample maximum.
"""

from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int16MultiArray


class Pcm16Blocks:
    """48 kHz int16 (frames, channels) -> mono 16 kHz fixed-size blocks."""

    def __init__(self, block_ms=100.0):
        samples = float(block_ms) * 16
        if not math.isfinite(samples) or not 0 < samples <= 1600 or not samples.is_integer():
            raise ValueError("block_ms must yield whole samples at 16 kHz and be in (0, 100]")
        self.block_samples = int(samples)
        self.pending = np.empty(0, dtype=np.float64)

    def feed(self, pcm):
        pcm = np.asarray(pcm, dtype=np.int16)
        mono = pcm.astype(np.float64)
        if mono.ndim == 2:
            mono = mono.mean(axis=1)
        elif mono.ndim != 1:
            raise ValueError("PCM must be mono or (frames, channels)")
        self.pending = np.concatenate((self.pending, mono))
        frames = self.block_samples * 3
        blocks = []
        while len(self.pending) >= frames:
            block = self.pending[:frames].reshape(-1, 3).mean(axis=1)
            blocks.append(np.rint(block).clip(-32768, 32767).astype(np.int16))
            self.pending = self.pending[frames:]
        return blocks


class SoundDeviceSource:
    """Bounded callback queue; the ROS executor never waits on a microphone read."""

    def __init__(self, device, frames):
        import sounddevice as sd

        self.queue = deque(maxlen=10)
        device = int(device) if device.isdecimal() else device or None
        self.stream = sd.InputStream(
            device=device, samplerate=48000, channels=1, dtype="int16",
            blocksize=frames, callback=self._callback)
        try:
            self.stream.start()
        except Exception:
            self.stream.close()
            raise

    def _callback(self, indata, frames, time_info, status):
        if status:
            self.queue.clear()
        self.queue.append(indata.copy())

    def read(self):
        try:
            return self.queue.popleft()
        except IndexError:
            return None

    def close(self):
        try:
            self.stream.stop()
        finally:
            self.stream.close()
            self.queue.clear()


class DoorbellAudioPublisher(Node):
    def __init__(self, *, source_factory=SoundDeviceSource, **kwargs):
        super().__init__("doorbell_audio_publisher", **kwargs)
        self.device = self.declare_parameter("device", "").value
        topic = self.declare_parameter("topic", "/hri/doorbell/audio").value
        self.block_ms = self.declare_parameter("block_ms", 100).value
        self.converter = Pcm16Blocks(self.block_ms)
        self.publisher = self.create_publisher(Int16MultiArray, topic, qos_profile_sensor_data)
        self.source_factory = source_factory
        self.source = None
        self.next_attempt = 0.0
        self._audio_destroyed = False
        self.timer = self.create_timer(0.02, self._poll)

    def _close_source(self):
        source, self.source = self.source, None
        self.converter = Pcm16Blocks(self.block_ms)
        if source is not None:
            source.close()

    def _poll(self):
        try:
            if self.publisher.get_subscription_count() == 0:
                self._close_source()
                return
            if self.source is None:
                if time.monotonic() < self.next_attempt:
                    return
                self.source = self.source_factory(self.device, self.converter.block_samples * 3)
            # Bounded work even if callbacks arrive faster than this executor.
            for _ in range(10):
                if self.publisher.get_subscription_count() == 0:
                    self._close_source()
                    return
                pcm = self.source.read()
                if pcm is None:
                    break
                for block in self.converter.feed(pcm):
                    self.publisher.publish(Int16MultiArray(data=block.tolist()))
        except Exception as exc:
            self.next_attempt = time.monotonic() + 2.0
            try:
                self._close_source()
            except Exception as close_exc:
                self.get_logger().warning(f"Audio close failed: {close_exc}", throttle_duration_sec=5.0)
            self.get_logger().warning(f"Doorbell audio unavailable: {exc}", throttle_duration_sec=5.0)

    def destroy_node(self):
        if self._audio_destroyed:
            return False
        self._audio_destroyed = True
        self.timer.cancel()
        try:
            self._close_source()
        finally:
            super().destroy_node()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DoorbellAudioPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
