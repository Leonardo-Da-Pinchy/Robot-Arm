#!/usr/bin/env python3
"""Check that ROS adopted this startup session's zero holds; sends no commands."""
import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'pen_pitch']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--token')
    parser.add_argument('--assert-idle', action='store_true', help='Refuse an existing robot graph before motor startup')
    parser.add_argument('--timeout', type=float, default=30)
    args = parser.parse_args()
    if not args.assert_idle and not args.token: parser.error('--token is required for handoff checks')
    rclpy.init(args=[]); node = rclpy.create_node('guided_startup_handoff_check')
    if args.assert_idle:
        try:
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                topics = {name for name, _ in node.get_topic_names_and_types()}
                nodes = {name for name, _ in node.get_node_names_and_namespaces()}
                if 'controller_manager' in nodes or topics.intersection({'/robot_description', '/joint_states'}):
                    print('ROS domain already has robot control/model publishers. Stop that launch or select another --domain.', flush=True)
                    return 1
            return 0
        finally:
            node.destroy_node(); rclpy.shutdown()
    state = dict(model=False, positions={}, stamp=0.0, safety='')

    def model(message):
        root = ET.fromstring(message.data)
        param = root.find("./ros2_control/hardware/param[@name='handoff_token']")
        state['model'] = (root.get('name') == 'pinchy_hardware' and param is not None and param.text == args.token)

    def positions(message):
        state['positions'] = dict(zip(message.name, message.position)); state['stamp'] = time.monotonic()

    node.create_subscription(String, '/robot_description', model,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, '/joint_states', positions, 10)
    node.create_subscription(String, '/arm_safety/status', lambda message: state.update(safety=message.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    future = None; deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state['safety'].startswith('HOLD:'):
                print(state['safety']+'; preserving powered holds.', flush=True)
                return 2
            if client.service_is_ready() and future is None:
                future = client.call_async(ListControllers.Request())
            if future is not None and future.done():
                controllers = {c.name: c.state for c in future.result().controller}
                values = state['positions']
                if (state['model'] and time.monotonic()-state['stamp'] < 0.5 and
                    controllers.get('arm_controller') == 'active' and
                    controllers.get('joint_state_broadcaster') == 'active' and
                    all(name in values and math.isfinite(values[name]) and
                        abs(values[name]) <= 10*2*math.pi/4096+1e-12 for name in JOINTS)):
                    print('ROS adopted all six powered zero holds; controllers and feedback ready.', flush=True)
                    return 0
                future = None
        print('ROS did not confirm this session\'s zero-hold handoff before timeout.', flush=True)
        return 1
    finally:
        client.destroy(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
