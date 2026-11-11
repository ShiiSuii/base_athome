"""Bounded Restaurant docking, pure and disabled unless explicitly requested.

Reuses front_dock (752179f/f7797b6), table_dock (b0d5238, via front_dock)
and monotonic freshness guards (bf66bb4). Nav2 must have finished before start.
Only straight, already aligned approaches are accepted; no blind pivot occurs.
Every raw return is checked, including isolated points discarded by line fitting.
Retreat requires a separate rear-facing scan covering the complete rear sector;
the front Hokuyo's blind rear sector is never assumed clear. No torque is enabled.
Distances are provisional software values, not robot reach/collision calibration.
Real activation requires explicit restaurant_dock:=true and allow_motion:=true,
and human Level C authorization; default Restaurant behavior stays manual loading.
"""
from dataclasses import dataclass
import math
from typing import Optional

from .dock_safety import sensor_freshness_error
from .front_dock import FrontDockConfig, front_dock_step


@dataclass(frozen=True)
class DockConfig:
    standoff_m: float = 0.16
    speed_mps: float = 0.04
    max_travel_m: float = 0.55
    timeout_s: float = 25.0
    half_width_m: float = 0.44
    front_m: float = 0.24
    margin_m: float = 0.07
    laser_x_m: float = 0.22

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Dock configuration must be positive and finite')
        if self.speed_mps > 0.04 or self.max_travel_m > 0.55 or self.timeout_s > 30:
            raise ValueError('Dock exceeds conservative limits')
        if (self.front_m <= self.laser_x_m or self.standoff_m < self.margin_m + 0.02
                or self.half_width_m < 0.44):
            raise ValueError('Dock clearance/envelope too small')


@dataclass(frozen=True)
class Scan:
    angles: tuple
    ranges: tuple
    received_at: float
    range_min: float = 0.06
    range_max: float = 8.0


def corridor_error(scan, cfg, *, reverse=False):
    """Sensor faces direction of travel; rear scan origin must be rear bumper.

    Requires finite usable samples throughout +/-90 degrees (<=2 degree gaps).
    +inf is free only when range_max covers the stopping corridor. NaN, -inf,
    missing angular coverage, and below-minimum returns fail closed.
    """
    if scan is None or len(scan.angles) != len(scan.ranges):
        return 'invalid_scan'
    samples = sorted((a, r) for a, r in zip(scan.angles, scan.ranges)
                     if math.isfinite(a) and abs(a) <= math.pi / 2 + 0.02)
    if (not samples or samples[0][0] > -math.pi / 2 + 0.035
            or samples[-1][0] < math.pi / 2 - 0.035
            or any(b[0] - a[0] > 0.035 for a, b in zip(samples, samples[1:]))):
        return 'blind_corridor'
    if not (math.isfinite(scan.range_min) and math.isfinite(scan.range_max)
            and 0 < scan.range_min < scan.range_max):
        return 'invalid_scan'
    origin = 0.0 if reverse else cfg.laser_x_m
    extent = cfg.margin_m if reverse else cfg.front_m + cfg.margin_m
    for angle, distance in samples:
        if math.isnan(distance) or distance < scan.range_min:
            return 'invalid_scan'
        if distance > scan.range_max:
            if distance != math.inf:
                return 'invalid_scan'
            distance = scan.range_max
        x = origin + distance * math.cos(angle)
        y = distance * math.sin(angle)
        if 0 <= x <= extent and abs(y) <= cfg.half_width_m:
            return 'blocked'
    return None


class ManipulationDock:
    """One-shot state machine. Any failure latches zero until a new instance."""
    def __init__(self, now, pose, *, enabled=False, retreat_m=None, cfg=DockConfig()):
        self.cfg = cfg
        self.start = now
        self.origin = pose
        self.enabled = enabled
        self.retreat_m = retreat_m
        self.result: Optional[str] = None
        self.travel_m = 0.0
        self.final_clearance_m = None
        if (not math.isfinite(now) or len(pose) != 3
                or not all(math.isfinite(v) for v in pose)):
            raise ValueError('Invalid initial odometry/time')
        if retreat_m is not None and not 0 < retreat_m <= cfg.max_travel_m:
            raise ValueError('Invalid retreat distance')

    def step(self, now, pose, odom_at, scan, *, active=True):
        if self.result is not None:
            return 0.0
        def stop(reason):
            self.result = reason
            return 0.0
        if not self.enabled or not active:
            return stop('disabled' if not self.enabled else 'cancelled')
        age_error = sensor_freshness_error(now, scan.received_at if scan else None, odom_at)
        if age_error:
            return stop(age_error)
        if not 0 <= now - self.start < self.cfg.timeout_s:
            return stop('timeout')
        if pose is None or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            return stop('invalid_odom')
        dx, dy = pose[0] - self.origin[0], pose[1] - self.origin[1]
        heading = self.origin[2]
        along = dx * math.cos(heading) + dy * math.sin(heading)
        cross = -dx * math.sin(heading) + dy * math.cos(heading)
        yaw_error = math.atan2(math.sin(pose[2]-heading), math.cos(pose[2]-heading))
        self.travel_m = math.hypot(dx, dy)
        if self.travel_m > self.cfg.max_travel_m or abs(cross) > 0.04 or abs(yaw_error) > 0.05:
            return stop('odometry_limit')
        reverse = self.retreat_m is not None
        if (along > 0.025 if reverse else along < -0.025):
            return stop('wrong_direction')
        error = corridor_error(scan, self.cfg, reverse=reverse)
        if error:
            return stop(error)
        if reverse:
            if -along >= self.retreat_m - 0.005:
                return stop('done')
            return -min(self.cfg.speed_mps, max(0.005, self.retreat_m + along))
        decision = front_dock_step(scan.angles, scan.ranges, target='surface', cfg=FrontDockConfig(
            target_standoff_m=self.cfg.standoff_m, corridor_half_width_m=self.cfg.half_width_m,
            chassis_front_m=self.cfg.front_m, laser_x_m=self.cfg.laser_x_m,
            chassis_front_offset_m=self.cfg.front_m-self.cfg.laser_x_m))
        if decision.action == 'done':
            central = sorted(self.cfg.laser_x_m + r*math.cos(a) - self.cfg.front_m
                             for a, r in zip(scan.angles, scan.ranges)
                             if abs(a) < .05 and math.isfinite(r))
            self.final_clearance_m = central[len(central)//2] if central else None
            return stop('done')
        if decision.action != 'go':
            return stop('unaligned' if decision.action == 'turn' else str(decision.value))
        # Also reject isolated/edge returns before the fitted counter surface.
        surface_x = self.cfg.front_m + self.cfg.standoff_m + float(decision.value)
        for a, r in zip(scan.angles, scan.ranges):
            if math.isfinite(r):
                x, y = self.cfg.laser_x_m + r*math.cos(a), r*math.sin(a)
                if self.cfg.front_m <= x < surface_x - 0.06 and abs(y) <= self.cfg.half_width_m:
                    return stop('blocked')
        return min(self.cfg.speed_mps, max(0.005, float(decision.value)))
