#!/usr/bin/env python3
"""Gazebo manipulation-dock probe; DOES NOT certify a Restaurant mission.

Run after sourcing Jazzy and a built workspace, with ROS_DOMAIN_ID=55,
GZ_PARTITION=maniplab, DISPLAY=:1. Starts Gazebo, bridge, robot state
publisher and sim_robot; optional arm probe also starts MoveIt and explicitly
enables only a backend reporting sim=true; never starts a device driver. Adds a simulation-only rear
LiDAR at the rear bumper; the real robot has no equivalent sensor installed.
Fuel Coke is loaded from the public Fuel cache (download it with gz fuel).
Results distinguish measured docking/retreat from UNTESTED physical grasp,
Nav2 delivery and collision contacts. A dock-only result is always PARCIAL in
the shared ledger. Invoke through sim_lab/tools/gz_turno.sh to reserve a turn.
Camera resolution and rates match the production simulator; only the rear
sensor is added. --measure-seconds records receive frequency, maximum/p95 gaps
and header age in sensor_rates.json without publishing motion during measurement.
All child processes are owned and terminated on exit.

--arm-probe attempts the production RestaurantArm callbacks after docking,
using the real YOLO detector on rendered wrist images (not magenta segmentation).
The Fuel props in this probe are static: physical pick is deliberately untested
rather than using sim_robot's unrelated virtual object as grasp feedback.
Failed stages are retained in arm_probe.json; no full PASS is
awarded without independently measured attachment, delivery and contacts.
--contacts is experimental: per-collision sensors and a distant positive
control; telemetry availability and empty messages do not certify zero collisions.
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
import xml.etree.ElementTree as ET


def with_contact_sensors(sdf):
    """Instrument every converted robot collision; preserve geometry/physics.

    Harmonic reads contact/topic (not the sensor topic) and publishes only on
    actual contact: https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc
    A live positive control is therefore required to interpret zero messages.
    """
    root = ET.fromstring(sdf)
    for link in root.findall('./model/link'):
        for i, collision in enumerate(link.findall('collision')):
            sensor = ET.SubElement(link, 'sensor', name=f'manip_contact_{i}', type='contact')
            ET.SubElement(sensor, 'always_on').text = '1'
            ET.SubElement(sensor, 'update_rate').text = '20'
            ET.SubElement(sensor, 'topic').text = '/restaurant/contacts'
            contact = ET.SubElement(sensor, 'contact')
            ET.SubElement(contact, 'collision').text = collision.get('name')
            ET.SubElement(contact, 'topic').text = '/restaurant/contacts'
    return ET.tostring(root, encoding='unicode')


def arm_probe(out, repo):
    """Isolated child: enable only a verified sim backend in the reserved domain."""
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    os.environ['ATHOME_SIM'] = '1'
    os.environ['ATHOME_SIM_WRIST_DETECTOR'] = 'yolo'
    sys.path[:0] = [str(repo/'src/base_athome_perception'), str(repo/'src/base_athome_skills')]
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from base_athome_perception.restaurant_arm import create_restaurant_manipulation, RestaurantArm
    import yaml
    result = {'pregrasp': False, 'detected': False, 'pick': False,
              'attachment': 'UNTESTED', 'delivery': 'UNTESTED', 'contacts': 'UNTESTED'}
    rclpy.init()
    verifier = rclpy.create_node('manipulation_sim_verifier')
    status = {}
    def received(msg):
        try:
            status.update(json.loads(msg.data))
        except ValueError:
            status.clear()
    verifier.create_subscription(String, '/arm/status', received, 10)
    m = executor = None
    backend_threads = set()
    try:
        end = time.monotonic()+5
        while time.monotonic() < end and not status:
            rclpy.spin_once(verifier, timeout_sec=.1)
        if status.get('sim') is not True or status.get('feedback_ok') is not True:
            raise RuntimeError('No verified simulation feedback: refusing enable')
        client = verifier.create_client(SetBool, '/arm/set_enable')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Simulation enable service missing')
        poses = yaml.safe_load((repo/'src/base_athome_perception/config/restaurant_arm_poses.yaml').read_text())['named_poses']
        before_threads = set(threading.enumerate())
        m = create_restaurant_manipulation(poses, lambda: True)
        backend_threads = set(threading.enumerate())-before_threads
        executor = MultiThreadedExecutor(4)
        executor.add_node(m)
        threading.Thread(target=executor.spin, daemon=True).start()
        end = time.monotonic()+45
        while time.monotonic() < end and not (m.feedback_ok and m.moveit.available()):
            time.sleep(.1)
        # Neural model/CLIP initialization may hold the GIL long enough to age
        # feedback. Warm up with generated black frames while DISABLED; never
        # widen the freshness guard to conceal a loading stall during motion.
        import numpy as np
        begun = time.monotonic()
        for _ in range(3):
            m.detector.detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        result['model_warmup_s'] = time.monotonic()-begun
        future = client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(verifier, future, timeout_sec=3)
        if not future.done() or not future.result().success:
            raise RuntimeError('Simulation enable rejected')
        end = time.monotonic()+3
        while time.monotonic() < end and not m.motion_ready():
            time.sleep(.05)
        if not m.motion_ready():
            raise RuntimeError('No fresh enabled simulation feedback after warmup')
        arm = RestaurantArm(lambda: m, poses, lambda: True,
                            unsafe_fn=lambda: result.update(unsafe=True), warn_fn=print)
        # World geometry must be in the planner before a calibrated pose is used:
        # calibration on the robot does not certify a path around this counter.
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        rx, ry, yaw = json.loads((out/'dock_pose.json').read_text())
        counter = CollisionObject(id='restaurant_probe_counter')
        counter.header.frame_id = 'base_footprint'
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.6, 3., .9])
        pose = Pose()
        pose.position.x = math.cos(yaw)*(-2.-rx)+math.sin(yaw)*(-ry)
        pose.position.y = -math.sin(yaw)*(-2.-rx)+math.cos(yaw)*(-ry)
        pose.position.z = .45
        pose.orientation.z, pose.orientation.w = math.sin(-yaw/2), math.cos(-yaw/2)
        counter.primitives, counter.primitive_poses = [box], [pose]
        counter.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [counter]
        applied = m.moveit._call(m.moveit.cli_apply_scene, ApplyPlanningScene.Request(scene=scene))
        if not applied.success:
            raise RuntimeError('Counter planning scene rejected')
        result['counter_in_planning_scene'] = True
        result['pregrasp'], result['pregrasp_message'] = m.go_to_blocking('restaurant_pregrasp')
        if result['pregrasp']:
            from scipy.spatial.transform import Rotation
            tx, ty, tz = json.loads((out/'object_world.json').read_text())
            obj_base = [math.cos(yaw)*(tx-rx)+math.sin(yaw)*(ty-ry),
                        -math.sin(yaw)*(tx-rx)+math.cos(yaw)*(ty-ry), tz]
            tf = m.tf_buf.lookup_transform('arm_camera_optical_frame', 'base_footprint', rclpy.time.Time())
            q, t = tf.transform.rotation, tf.transform.translation
            optical = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(obj_base)+np.array([t.x,t.y,t.z])
            result['object_optical_xyz'] = optical.tolist()
            result['object_pixel'] = [float(320+485*optical[0]/optical[2]), float(240+485*optical[1]/optical[2])]
            # The label must match a real neural detection; no truth-label override.
            begun = time.monotonic()
            result['detected'] = arm.detect_item('coke', 20.)
            result['detection_elapsed_s'] = time.monotonic()-begun
            result['frame_available'] = m.frame is not None
            result['detector'] = m.detector.source
            result['guard'] = dict(enabled=m.enabled, feedback_ok=m.feedback_ok,
                feedback_age_s=time.monotonic()-m.restaurant_feedback_at,
                abort=m.abort, selected_label=m.detector.label)
            tg = m.held_target()
            if tg:
                result['target'] = dict(label=tg['label'], xyz=[float(v) for v in tg['obj']],
                                        distance_m=float(tg['z']), detector=m.detector.source)
            # sim_robot's default virtual object is unrelated to the static Fuel
            # can. Its gripper width must never be counted as a Fuel grasp.
            result['pick_reason'] = 'Static Fuel prop has no physical attachment backend'
        result['gripper_mm'] = m.gripper_mm
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc)
    finally:
        (out/'arm_probe.json').write_text(json.dumps(result, indent=2)+'\n')
        print('ARM_PROBE '+json.dumps(result), flush=True)
        if m is not None:
            m.abort = True
            m.stop_move_group()
            owned = m.move_group_proc
            if owned is not None and owned.poll() is None:
                os.killpg(owned.pid, signal.SIGTERM)
                owned.wait(timeout=10)
        if executor is not None:
            executor.shutdown(timeout_sec=3)
        if m is not None:
            m.destroy_node()
        verifier.destroy_node()
        rclpy.try_shutdown()
        for thread in backend_threads:
            thread.join(timeout=10)


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '55' or os.environ.get('GZ_PARTITION') != 'maniplab'
            or os.environ.get('DISPLAY') != ':1'):
        raise SystemExit('Requires domain 55, partition maniplab and DISPLAY=:1')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iteration', required=True)
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--measure-seconds', type=float, default=0.)
    parser.add_argument('--arm-probe', action='store_true')
    parser.add_argument('--contacts', action='store_true', help='Experimental contact instrumentation with positive control')
    parser.add_argument('--arm-only', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--counter-y', type=float, default=.4,
                        help='Object offset along counter; pregrasp camera points to robot right')
    args = parser.parse_args()
    if args.trials < 1 or not math.isfinite(args.counter_y):
        parser.error('trials must be positive and counter-y must be finite')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[2]
    if args.arm_only:
        arm_probe(out, repo)
        return
    sys.path[:0] = [str(repo/'simulation/base_athome_sim'), str(repo/'src/base_athome_skills')]
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    from base_athome_sim import gz_model
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from base_athome_skills.restaurant_docking import RestaurantDocking
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from ros_gz_interfaces.msg import Contacts
    from rclpy.qos import qos_profile_sensor_data

    urdf = repo/'src/hoverboard_base/urdf/robot.urdf'
    import yaml
    publisher_params = out/'state_publisher.yaml'
    publisher_params.write_text(yaml.safe_dump({'robot_state_publisher': {
        'ros__parameters': {'robot_description': urdf.read_text()}}}))
    rear = '''<link name="restaurant_rear_laser"/>
    <joint name="restaurant_rear_laser_joint" type="fixed"><parent link="base_link"/>
    <child link="restaurant_rear_laser"/><origin xyz="-0.24 0 0.32" rpy="0 0 3.141592653589793"/></joint>
    <gazebo reference="restaurant_rear_laser"><sensor name="manip_rear" type="gpu_lidar">
    <topic>/restaurant/rear_scan</topic><gz_frame_id>restaurant_rear_laser</gz_frame_id><update_rate>15</update_rate>
    <lidar><scan><horizontal><samples>361</samples><min_angle>-1.5707963268</min_angle><max_angle>1.5707963268</max_angle>
    </horizontal></scan><range><min>0.04</min><max>8</max></range></lidar><always_on>1</always_on></sensor></gazebo>'''
    robot = gz_model.gazebo_urdf(urdf.read_text()).replace('</robot>', rear+'</robot>')
    model_path = out/'robot.urdf'
    model_path.write_text(robot)
    if args.contacts:
        converted = subprocess.run(['gz', 'sdf', '-p', str(model_path)], capture_output=True,
                               text=True, check=True, timeout=15).stdout
        model_path = out/'robot.sdf'
        model_path.write_text(with_contact_sensors(converted))
    coke = Path.home()/'.gz/fuel/fuel.gazebosim.org/openrobotics/models/coke/6'
    if not (coke/'model.sdf').is_file():
        raise SystemExit('Download public Fuel OpenRobotics/models/Coke before running')
    world = (repo/'simulation/base_athome_sim/worlds/restaurant.sdf').read_text()
    if args.contacts:
        world = world.replace('</world>', '<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/></world>')
    # Positive control, away from the robot: proves contact messages are flowing.
    # Zero robot messages without this heartbeat is UNVERIFIED, not collision-free.
        world = world.replace('</world>', '''<model name="contact_positive_control"><pose>8 4 .049 0 0 0</pose><link name="l">
      <inertial><mass>1</mass><inertia><ixx>.001</ixx><iyy>.001</iyy><izz>.001</izz></inertia></inertial>
      <collision name="c"><geometry><sphere><radius>.05</radius></sphere></geometry></collision>
      <sensor name="contact_heartbeat" type="contact"><always_on>1</always_on><update_rate>20</update_rate>
        <topic>/restaurant/contact_heartbeat</topic><contact><collision>c</collision><topic>/restaurant/contact_heartbeat</topic></contact></sensor>
      </link></model></world>''')
    world = world.replace('</world>', f'''<include><uri>{coke}</uri><name>counter_coke</name><pose>-1.85 {args.counter_y} 0.90 0 0 0</pose></include>
    <include><uri>{coke}</uri><name>tray_coke</name><pose>-1.85 0.2 0.90 0 0 0</pose></include></world>''')
    world_path = out/'restaurant.sdf'
    world_path.write_text(world)
    shares = {str(Path(get_package_share_directory(p)).parent) for p in ('piper_description','hoverboard_base')}
    shares.add(str(Path(get_package_prefix('piper_description'))/'share'))
    env = dict(os.environ, OMP_NUM_THREADS='2', ATHOME_SIM='1', ATHOME_SIM_NAV2='1', ATHOME_SIM_WORLD='restaurant.sdf',
               ATHOME_SIM_START='-1,0,180', GZ_SIM_RESOURCE_PATH=':'.join(shares))
    # Preserve full local source for sim_robot instead of another worktree's package.
    env['PYTHONPATH'] = ':'.join([str(repo/'simulation/base_athome_sim'),
        str(repo/'src/base_athome_perception'), str(repo/'src/base_athome_skills'), env.get('PYTHONPATH','')])
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
                                  '--ros-args','--params-file',str(publisher_params)])
        start('bridge', ['ros2','run','ros_gz_bridge','parameter_bridge', *gz_model.bridge_args(),
                        '/restaurant/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/restaurant/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/restaurant/contact_heartbeat@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
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
        sensor_times = {'/scan': [], '/restaurant/rear_scan': [], '/odom': []}
        contacts = dict(state='UNVERIFIED', instrumentation_enabled=args.contacts,
                        messages=0, heartbeat_messages=0, pairs={})
        def record_contacts(msg):
            contacts['messages'] += 1
            for contact in msg.contacts:
                pair = '|'.join(sorted((contact.collision1.name, contact.collision2.name)))
                contacts['pairs'][pair] = contacts['pairs'].get(pair, 0)+1
        node.create_subscription(Contacts, '/restaurant/contacts', record_contacts, 10,
                                 callback_group=node.cb_group)
        def heartbeat(msg):
            if msg.contacts:
                contacts['heartbeat_messages'] += 1
        node.create_subscription(Contacts, '/restaurant/contact_heartbeat', heartbeat, 10,
                                 callback_group=node.cb_group)
        def record(topic, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            sensor_times[topic].append((time.monotonic(), time.time()-stamp))
        for topic, typ in (('/scan', LaserScan), ('/restaurant/rear_scan', LaserScan), ('/odom', Odometry)):
            node.create_subscription(typ, topic, lambda msg, t=topic: record(t,msg),
                                     qos_profile_sensor_data, callback_group=node.cb_group)
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
            if dock.scan:
                (out/f'front_scan_{trial}.json').write_text(json.dumps(vars(dock.scan)))
            approach = dict(ok=ok, reason=c.result if c else 'preflight',
                            travel_m=c.travel_m if c else None,
                            clearance_m=c.final_clearance_m if c else None)
            if ok and args.arm_probe:
                (out/'dock_pose.json').write_text(json.dumps(dock.pose))
                (out/'object_world.json').write_text(json.dumps([-1.85,args.counter_y,.97236]))
                probe = start('arm_probe', [sys.executable, str(Path(__file__).resolve()),
                    '--output', str(out), '--iteration', args.iteration, '--arm-only'])
                try:
                    probe.wait(timeout=150)
                except subprocess.TimeoutExpired:
                    os.killpg(probe.pid, signal.SIGTERM)
                    probe.wait(timeout=10)
                    (out/'arm_probe.json').write_text(json.dumps({'error': 'arm_probe_timeout'})+'\n')
                if (out/'arm_probe.json').is_file():
                    probe_result = json.loads((out/'arm_probe.json').read_text())
                    probe_result['process_exit_code'] = probe.returncode
                    (out/'arm_probe.json').write_text(json.dumps(probe_result, indent=2)+'\n')
            exit_ok = dock.run(reverse=True) if ok else False
            c = dock.controller
            summary = dict(trial=trial+1, elapsed_s=time.monotonic()-begun, start=before, end=dock.pose,
                           approach=approach, retreat_ok=exit_ok,
                           retreat_reason=(c.result if c and c.retreat_m is not None else 'preflight_rejected') if ok else 'not_attempted',
                           grasp='UNTESTED', navigation='UNTESTED', collision_contacts='UNTESTED')
            summary['collision_contacts'] = json.loads(json.dumps(contacts))
            summaries.append(summary)
            if args.arm_probe and (out/'arm_probe.json').is_file():
                summary['arm_probe'] = json.loads((out/'arm_probe.json').read_text())
            (out/f'sensor_times_{trial}.json').write_text(json.dumps(sensor_times))
            print(json.dumps(summary),flush=True)
            (out/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
            subprocess.run(['gz','service','-s','/gui/follow','--reqtype','gz.msgs.StringMsg',
                            '--reptype','gz.msgs.Boolean','--timeout','3000','--req',
                            'data: '+json.dumps(gz_model.MODEL)],env=env,timeout=5,check=False)
            subprocess.run(['/home/athome/sim_lab/tools/capturar.sh',f'restaurant-manip/{args.iteration}_{trial+1}'],
                           env=env,timeout=20,check=False)
            if args.measure_seconds > 0:
                # Passive bounded measurement, no motion commands during this interval.
                time.sleep(min(30., args.measure_seconds))
                rates = {}
                for topic, rows in sensor_times.items():
                    rows = list(rows)
                    gaps = sorted(b[0]-a[0] for a,b in zip(rows,rows[1:]))
                    span = rows[-1][0]-rows[0][0] if len(rows)>1 else 0.
                    rates[topic] = dict(messages=len(rows), span_s=span,
                        hz=(len(rows)-1)/span if span else 0.,
                        max_gap_s=max(gaps) if gaps else None,
                        p95_gap_s=gaps[min(len(gaps)-1,int(len(gaps)*.95))] if gaps else None,
                        gaps_over_300ms=sum(g>.3 for g in gaps),
                        max_header_age_s=max((r[1] for r in rows),default=None))
                (out/'sensor_rates.json').write_text(json.dumps(rates,indent=2)+'\n')
                (out/f'sensor_times_{trial}.json').write_text(json.dumps(sensor_times))
                print('SENSOR_RATES '+json.dumps(rates),flush=True)
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
