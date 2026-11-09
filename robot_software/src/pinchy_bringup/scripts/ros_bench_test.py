#!/usr/bin/env python3
"""Move loose physical servos one at a time through ros2_control, then return.

Requires an explicit unloaded-bench launch and captured temporary zero profile.
Each leg is 32 ticks (2.8125 degrees). All other motors hold their start pose.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from mock_trajectory import wait

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'pen_pitch']
TICK = 2 * math.pi / 4096


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ids', nargs='+', type=int, help='Motor IDs to test, default all six captured IDs')
    parser.add_argument('--seconds', type=float, default=3, help='Seconds per outward/return leg, default 3')
    parser.add_argument('--log', type=Path, default=Path('/tmp/pinchy-ros-bench-test.jsonl'))
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 20:
        parser.error('--seconds must be between 1 and 20')
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('physical_ros_bench_test')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    active = result_future = None
    phase = 'preflight'
    code = 1
    log = args.log.open('a')
    actual = {}; last_sample = 0.0

    def record(event, **data):
        log.write(json.dumps(dict(time=datetime.now(timezone.utc).isoformat(),
                                 phase=phase, event=event, **data))+'\n'); log.flush()

    def on_state(message):
        nonlocal actual, last_sample
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            actual = {name: values[name] for name in JOINTS}; last_sample = time.monotonic()
            record('feedback', positions=actual)

    subscription = node.create_subscription(JointState, '/joint_states', on_state, 10)
    model_future = rclpy.task.Future()

    def on_model(message):
        if not model_future.done(): model_future.set_result(message.data)

    model_sub = node.create_subscription(String, '/robot_description', on_model,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        model = ET.fromstring(wait(node, model_future, 10))
        plugin = model.find('./ros2_control/hardware/plugin')
        kind = model.find("./ros2_control/hardware/param[@name='profile_kind']")
        if (model.get('name') != 'pinchy_bench_hardware' or plugin is None or
                plugin.text != 'pinchy_hardware/FeetechSystem' or kind is None or
                kind.text != 'unloaded_ros_bench'):
            raise RuntimeError('Requires the explicit physical unloaded-bench launch; refusing other robot modes')
        mapping = {}
        for name in JOINTS:
            param = model.find(f"./ros2_control/joint[@name='{name}']/param[@name='servo_id']")
            mapping[int(param.text)] = name
        ids = args.ids or list(mapping)
        if not ids or len(ids) != len(set(ids)) or any(servo not in mapping for servo in ids):
            raise ValueError('--ids must select distinct IDs in the captured profile')
        if not client.wait_for_server(timeout_sec=10):
            raise TimeoutError('Trajectory controller not available')
        deadline = time.monotonic() + 10
        while (len(actual) != 6 or time.monotonic()-last_sample > 0.5) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(actual) != 6 or time.monotonic()-last_sample > 0.5:
            raise TimeoutError('Missing fresh six-motor position feedback')
        home = dict(actual)
        stages = []
        for servo in ids:
            selected = mapping[servo]; target = dict(home)
            limit = model.find(f"joint[@name='{selected}']/limit")
            low, high = float(limit.get('lower')), float(limit.get('upper'))
            if not -0.1 <= low <= high <= 0.1:
                raise ValueError('Bench model exceeds the small-motion range')
            delta = 32*TICK if home[selected]+32*TICK <= high else -32*TICK
            target[selected] += delta
            if not low <= target[selected] <= high:
                raise ValueError(f'ID {servo}: insufficient travel; stop launch and capture fresh zeros')
            stages += [(f'ID {servo} {selected}: outward', target),
                       (f'ID {servo} {selected}: return', home)]
        for phase, target in stages:
            print(phase, flush=True); record('command', target=target)
            if time.monotonic()-last_sample > 0.5:
                raise TimeoutError('Feedback is stale; no new goal sent')
            goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
            point = JointTrajectoryPoint(); point.positions = [target[name] for name in JOINTS]
            ns = round(args.seconds*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1_000_000_000)
            goal.trajectory.points = [point]
            active = wait(node, client.send_goal_async(goal), 5)
            if not active.accepted:
                raise RuntimeError('Trajectory rejected; wait until arm_controller is active')
            result_future = active.get_result_async()
            result = wait(node, result_future, args.seconds+6)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'Action failed: status={result.status}, '
                                   f'error={result.result.error_code}, {result.result.error_string}')
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if (time.monotonic()-last_sample < 0.5 and
                        all(abs(actual[name]-target[name]) <= 5*TICK for name in JOINTS)):
                    break
            else:
                raise RuntimeError(f'Feedback did not settle within 5 ticks: {actual}')
            record('reached', positions=actual)
        phase = 'complete'; record('passed', tested_ids=ids)
        print(f'PASS: IDs {ids} moved individually and returned. Driver is still holding; Ctrl+C its launch to release.', flush=True)
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        record('failed', error=str(exc) or 'Interrupted')
        print(f'{phase}: {exc or "Interrupted"}. No further test stages will run.', file=sys.stderr)
    finally:
        if active is not None and active.accepted and result_future is not None and not result_future.done():
            try:
                wait(node, active.cancel_goal_async(), 5); wait(node, result_future, 5)
            except Exception as exc:
                print(f'Could not confirm cancellation: {exc}', file=sys.stderr)
        node.destroy_subscription(subscription); node.destroy_subscription(model_sub)
        client.destroy(); node.destroy_node(); rclpy.shutdown(); log.close()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
