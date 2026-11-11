"""Restaurant's fixed rendered diners and barman, aligned with restaurant.sh.

Geometry comes from OpenRobotics Male visitor v2 (CC0-1.0), with static poses
built by models/humans/download_restaurant.py. All coordinates are world metres.
"""
CUSTOMERS = ((2.0, 2.8, 'right', 'seated'), (3.0, -2.9, 'left', 'seated'),
             (-2.6, 0.0, 'right', 'standing'))


def rendered_restaurant():
    import os
    return os.environ.get('ATHOME_SIM_WORLD') == 'restaurant.sdf'


def reply_for_state(state):
    """A diner must reject the barman role; never reward a wrong visual identity."""
    import math
    name = state.get('state', '')
    if name == 'CONFIRMING_BARMAN':
        xy = state.get('info', {}).get('xy')
        if xy is None or len(xy) != 2:
            return 'robot no'
        return 'robot yes' if math.hypot(xy[0] + 2.6, xy[1]) < .8 else 'robot no'
    if name == 'TAKING_ORDER':
        orders = ('coke and water', 'juice and tea')
        index = max(0, min(int(state.get('customer_idx', 0)), 1))
        return 'my order is ' + orders[index]
    return 'robot yes'


def setup_measured_head(node):
    """Real-render mode must report Gazebo head joints, not their moving targets."""
    import time
    import math
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data
    node.rendered_head = None
    def measured(msg):
        positions = dict(zip(msg.name, msg.position))
        names = ('pan_joint', 'tilt_joint')
        if all(name in positions and math.isfinite(positions[name]) for name in names):
            node.rendered_head = (time.monotonic(), {name: positions[name] for name in names})
    node.create_subscription(JointState, '/sim/gz/joint_states', measured, qos_profile_sensor_data,
                             callback_group=node.cb)


def apply_measured_head(node, full, head):
    """No fresh sample means absent measured joints, never synthetic zero feedback."""
    import time
    sample = node.rendered_head
    measured = sample[1] if sample is not None and time.monotonic() - sample[0] < 1.0 else {}
    pairs = [(n, measured.get(n, p)) for n, p in zip(full.name, full.position)
             if n not in ('pan_joint', 'tilt_joint') or n in measured]
    full.name = [n for n, _ in pairs]
    full.position = [float(p) for _, p in pairs]
    head.name = list(measured)
    head.position = [float(v) for v in measured.values()]
