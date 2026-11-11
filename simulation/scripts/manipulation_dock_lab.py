#!/usr/bin/env python3
"""Gazebo manipulation-dock probe; DOES NOT certify a Restaurant mission.

Run after sourcing Jazzy and a built workspace, with ROS_DOMAIN_ID=55,
GZ_PARTITION=maniplab, DISPLAY=:1. Starts only Gazebo, bridge, robot state
publisher and sim_robot; never a device driver. Adds a simulation-only rear
LiDAR at the rear bumper; the real robot has no equivalent sensor installed.
Fuel Coke is loaded from the public Fuel cache (download it with gz fuel).
Results distinguish measured docking/retreat from UNTESTED physical grasp,
Nav2 delivery and collision contacts. A dock-only result is always PARCIAL in
the shared ledger. All child processes are owned and terminated on exit.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '55' or os.environ.get('GZ_PARTITION') != 'maniplab'
            or os.environ.get('DISPLAY') != ':1'):
        raise SystemExit('Requires domain 55, partition maniplab and DISPLAY=:1')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iteration', required=True)
    parser.add_argument('--trials', type=int, default=1)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(repo/'simulation/base_athome_sim'), str(repo/'src/base_athome_skills')]
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    from base_athome_sim import gz_model
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from base_athome_skills.restaurant_docking import RestaurantDocking

    urdf = Path(get_package_share_directory('hoverboard_base'))/'urdf/robot.urdf'
    rear = '''<link name="restaurant_rear_laser"/>
    <joint name="restaurant_rear_laser_joint" type="fixed"><parent link="base_link"/>
    <child link="restaurant_rear_laser"/><origin xyz="-0.24 0 0.32" rpy="0 0 3.141592653589793"/></joint>
    <gazebo reference="restaurant_rear_laser"><sensor name="manip_rear" type="gpu_lidar">
    <topic>/restaurant/rear_scan</topic><gz_frame_id>restaurant_rear_laser</gz_frame_id><update_rate>15</update_rate>
    <lidar><scan><horizontal><samples>361</samples><min_angle>-1.5707963268</min_angle><max_angle>1.5707963268</max_angle>
    </horizontal></scan><range><min>0.04</min><max>8</max></range></lidar><always_on>1</always_on></sensor></gazebo>'''
    robot = gz_model.gazebo_urdf(urdf.read_text()).replace('</robot>', rear+'</robot>')
    # Own sim profile: reduce image load, preserve geometry and watchdog thresholds.
    robot = robot.replace('<width>640</width><height>480</height>', '<width>320</width><height>240</height>')
    robot = robot.replace('<update_rate>15</update_rate>', '<update_rate>5</update_rate>')
    robot = robot.replace('<gz_frame_id>restaurant_rear_laser</gz_frame_id><update_rate>5</update_rate>',
                          '<gz_frame_id>restaurant_rear_laser</gz_frame_id><update_rate>10</update_rate>')
    model_path = out/'robot.urdf'
    model_path.write_text(robot)
    coke = Path.home()/'.gz/fuel/fuel.gazebosim.org/openrobotics/models/coke/6'
    if not (coke/'model.sdf').is_file():
        raise SystemExit('Download public Fuel OpenRobotics/models/Coke before running')
    world = (repo/'simulation/base_athome_sim/worlds/restaurant.sdf').read_text()
    world = world.replace('</world>', f'''<include><uri>{coke}</uri><name>counter_coke</name><pose>-1.85 -0.1 0.90 0 0 0</pose></include>
    <include><uri>{coke}</uri><name>tray_coke</name><pose>-1.85 0.2 0.90 0 0 0</pose></include></world>''')
    world_path = out/'restaurant.sdf'
    world_path.write_text(world)
    shares = {str(Path(get_package_share_directory(p)).parent) for p in ('piper_description','hoverboard_base')}
    shares.add(str(Path(get_package_prefix('piper_description'))/'share'))
    env = dict(os.environ, OMP_NUM_THREADS='2', ATHOME_SIM='1', ATHOME_SIM_NAV2='1', ATHOME_SIM_WORLD='restaurant.sdf',
               ATHOME_SIM_START='-1,0,180', GZ_SIM_RESOURCE_PATH=':'.join(shares))
    # Preserve full local source for sim_robot instead of another worktree's package.
    env['PYTHONPATH'] = ':'.join([str(repo/'simulation/base_athome_sim'), env.get('PYTHONPATH','')])
    children, logs = [], []
    def start(name, cmd):
        log = (out/(name+'.log')).open('w')
        logs.append(log)
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc
    executor = None
    summaries = []
    try:
        start('gazebo', ['gz','sim','-r',str(world_path)])
        start('state_publisher', ['ros2','run','robot_state_publisher','robot_state_publisher',
                                  '--ros-args','-p','robot_description:='+urdf.read_text()])
        start('bridge', ['ros2','run','ros_gz_bridge','parameter_bridge', *gz_model.bridge_args(),
                        '/restaurant/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '--ros-args','-p','override_timestamps_with_wall_time:=true'])
        time.sleep(4)
        subprocess.run(['ros2','run','ros_gz_sim','create','-name',gz_model.MODEL,'-file',str(model_path),
                        '-x','-1','-y','0','-Y',str(math.pi)], env=env,check=True,timeout=20)
        start('sim_robot', ['ros2','run','base_athome_sim','sim_robot','--ros-args','-p','gazebo:=true'])
        (out/'pids.json').write_text(json.dumps([p.pid for p in children]))
        rclpy.init()
        node = Node('manipulation_dock_probe')
        node.cb_group = ReentrantCallbackGroup()
        dock = RestaurantDocking(node, lambda: rclpy.ok(), lambda: True)
        executor = MultiThreadedExecutor(4)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        end = time.monotonic()+35
        while time.monotonic() < end and (dock.scan is None or dock.rear_scan is None or dock.pose is None):
            time.sleep(.1)
        for trial in range(args.trials):
            begun = time.monotonic()
            before = dock.pose
            if dock.rear_scan:
                (out/f'rear_scan_{trial}.json').write_text(json.dumps(vars(dock.rear_scan)))
            ok = dock.run()
            c = dock.controller
            approach = dict(ok=ok, reason=c.result if c else 'preflight',
                            travel_m=c.travel_m if c else None,
                            clearance_m=c.final_clearance_m if c else None)
            exit_ok = dock.run(reverse=True) if ok else False
            c = dock.controller
            summary = dict(trial=trial+1, elapsed_s=time.monotonic()-begun, start=before, end=dock.pose,
                           approach=approach, retreat_ok=exit_ok, retreat_reason=c.result if c else None,
                           grasp='UNTESTED', navigation='UNTESTED', collision_contacts='UNTESTED')
            summaries.append(summary)
            print(json.dumps(summary),flush=True)
            (out/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
            subprocess.run(['/home/athome/sim_lab/tools/capturar.sh',f'restaurant-manip/{args.iteration}_{trial+1}'],
                           env=env,timeout=20,check=False)
            if not ok or not exit_ok:
                break
            time.sleep(.5)
    finally:
        if executor is not None:
            dock.cancel()
            executor.shutdown(timeout_sec=3)
            node.destroy_node()
            rclpy.try_shutdown()
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(children):
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
        for log in logs:
            log.close()
        base = subprocess.check_output(['git','rev-parse','--short','HEAD'],cwd=repo,text=True).strip()
        with Path('/home/athome/sim_lab/ledger.tsv').open('a') as ledger:
            ledger.write('\t'.join([datetime.datetime.now().astimezone().isoformat(),'codex-manip-dock',
                'restaurant-manip',base,args.iteration,'PARCIAL','unscored',
                'grasp-navigation-contacts-unverified',str(out/'summary.json'),
                f'Gazebo dock-only probe with simulated rear lidar; {len(summaries)} trials; not full Restaurant PASS'])+'\n')


if __name__ == '__main__':
    main()
