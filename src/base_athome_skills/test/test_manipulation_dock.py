"""Synthetic safety regressions; these are UNIT_PASS, never Gazebo grasp evidence."""
import math
import pytest
from base_athome_skills.mission.manipulation_dock import DockConfig, Scan, ManipulationDock


def wall(x=0.8, stamp=1.0):
    angles = tuple(math.radians(i/2) for i in range(-180, 181))
    ranges = tuple((x-0.22)/math.cos(a) if abs(a) < 1.4 else math.inf for a in angles)
    ranges = tuple(r if r <= 8 else math.inf for r in ranges)
    return Scan(angles, ranges, stamp)


def step(dock=None, scan=None, now=1.0, pose=(0., 0., 0.), odom=1.0, **kw):
    dock = dock or ManipulationDock(1., (0., 0., 0.), enabled=True)
    speed = dock.step(now, pose, odom, scan or wall(), **kw)
    return speed, dock.result


def test_default_disabled():
    assert step(ManipulationDock(1., (0., 0., 0.)))[1] == 'disabled'


def test_approach_and_stop_at_counter():
    dock = ManipulationDock(1., (0., 0., 0.), enabled=True)
    assert step(dock)[0] == 0.04
    assert step(dock, wall(.40, 2), now=2, pose=(.40, 0., 0.), odom=2)[0] == 0
    assert dock.result == 'done'


@pytest.mark.parametrize('sensor', ['scan', 'odom'])
def test_stale_sensor_latches_zero(sensor):
    dock = ManipulationDock(1., (0., 0., 0.), enabled=True)
    speed, reason = step(dock, wall(stamp=.5 if sensor == 'scan' else 1), odom=.5 if sensor == 'odom' else 1)
    assert (speed, reason) == (0, 'stale_' + sensor)
    assert step(dock)[0] == 0


@pytest.mark.parametrize('bad', [float('nan'), -float('inf'), .001])
def test_invalid_ray(bad):
    scan = wall()
    ranges = list(scan.ranges)
    ranges[180] = bad
    assert step(scan=Scan(scan.angles, tuple(ranges), 1))[0] == 0


def test_single_ray_obstacle_outside_fitted_clusters():
    scan = wall()
    ranges = list(scan.ranges)
    ranges[250] = .35
    assert step(scan=Scan(scan.angles, tuple(ranges), 1))[1] == 'blocked'


def test_single_point_on_corridor_edge():
    scan = wall()
    ranges = list(scan.ranges)
    ranges[300] = .50  # y=.433 inside 0.44m envelope
    assert step(scan=Scan(scan.angles, tuple(ranges), 1))[1] == 'blocked'


@pytest.mark.parametrize('now,pose,odom,reason', [
    (27., (0.,0.,0.),27.,'timeout'),
    (1., (.56,0.,0.),1.,'odometry_limit'),
    (1., (0.,.05,0.),1.,'odometry_limit'),
    (1., (0.,0.,.06),1.,'odometry_limit'),
    (1., (-.04,0.,0.),1.,'wrong_direction'),
    (1., (math.nan,0.,0.),1.,'invalid_odom'),
])
def test_limits(now,pose,odom,reason):
    assert step(scan=wall(stamp=now),now=now,pose=pose,odom=odom)[1] == reason


def test_cancel():
    assert step(active=False) == (0, 'cancelled')


def test_missing_coverage():
    assert step(scan=Scan((0.,),(.8,),1))[1] == 'blind_corridor'


def test_rear_sensor_required_and_obstacles_stop_retreat():
    dock = ManipulationDock(1., (0.,0.,0.), enabled=True, retreat_m=.2)
    assert dock.step(1., (0.,0.,0.), 1., None) == 0
    assert dock.result == 'no_scan'
    angles = wall().angles
    scan = Scan(angles, (math.inf,)*len(angles), 1)
    dock = ManipulationDock(1., (0.,0.,0.), enabled=True, retreat_m=.2)
    assert step(dock,scan)[0] == -.04
    assert step(dock,Scan(angles,scan.ranges,2), now=2,pose=(-.2,0.,0.),odom=2) == (0,'done')
    ranges = list(scan.ranges)
    ranges[180] = .065
    assert step(ManipulationDock(1.,(0.,0.,0.),enabled=True,retreat_m=.2),Scan(angles,tuple(ranges),1))[1] == 'blocked'


@pytest.mark.parametrize('kw', [{'speed_mps':.1}, {'standoff_m':.02}, {'timeout_s':math.nan}, {'half_width_m':.3}])
def test_config_validation(kw):
    with pytest.raises(ValueError):
        DockConfig(**kw)


def test_body_return_behind_front_is_not_in_forward_swept_area():
    scan = wall()
    # Self return at 120deg: x=.07, y=.26, behind the laser and front bumper.
    scan = Scan(scan.angles + (math.radians(120),), scan.ranges + (.3,), 1.)
    assert step(scan=scan) == (.04, None)
