"""Decode measured Gazebo RGB-D depth without inventing background distances."""
import numpy as np


def decode_depth(data, height, width, encoding, step=0, bigendian=False):
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    dtype = np.dtype(('>' if bigendian else '<') + ('f4' if encoding == '32FC1' else 'u2'))
    stride = step or width * dtype.itemsize
    raw = np.ndarray((height, width), dtype=dtype, buffer=data, strides=(stride, dtype.itemsize))
    mm = raw.astype(np.float64) * (1000 if encoding == '32FC1' else 1)
    valid = np.isfinite(mm) & (mm > 0) & (mm <= 65535)
    return np.where(valid, mm, 0).astype(np.uint16)


class FrameMetrics:
    """Frame-level rates per scenario; duplicate RGB stamps count once."""
    def __init__(self):
        self.scenarios = {}
        self.last_stamp = None

    def add(self, stamp, scenario, person, raised, valid_depth):
        if stamp == self.last_stamp:
            return None
        self.last_stamp = stamp
        counts = self.scenarios.setdefault(scenario, dict(frames=0, person=0, raised=0, valid_depth=0))
        counts['frames'] += 1
        for name, value in [('person', person), ('raised', raised), ('valid_depth', valid_depth)]:
            counts[name] += bool(value)
        return dict(scenario=scenario, **counts,
                    percent={k: round(100 * counts[k] / counts['frames'], 2)
                             for k in ('person', 'raised', 'valid_depth')},
                    denominator='unique RGB frames processed by YOLO', sensor='Gazebo RGB-D')
