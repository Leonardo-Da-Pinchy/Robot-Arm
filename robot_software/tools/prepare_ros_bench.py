#!/usr/bin/env python3
"""Read six loose servos and create a narrow temporary ROS bench profile.

Read-only: no goal, torque, RAM or EEPROM writes. Defaults to a mock bus.
This is an encoder-relative bench zero, never assembled-arm calibration.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from servo_chain import ChainBus, MockChainBus, health, preflight_settings, validate_ids
from servo_bench import error_text

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'pen_pitch']
TICK = 2 * math.pi / 4096


def make_profile(states, ids, port, hardware):
    validate_ids(ids, minimum=6)
    model = Path(__file__).resolve().parents[1] / 'src/pinchy_description/urdf/pinchy_arm.urdf'
    import xml.etree.ElementTree as ET
    root = ET.parse(model).getroot()
    joints = {}
    for name, servo_id in zip(JOINTS, ids):
        state = states[servo_id]
        health(state, servo_id); preflight_settings(state, servo_id)
        if hardware and state['model_raw'] != 777:
            raise ValueError(f'ID {servo_id}: expected STS3215 model 777')
        zero = state['position']
        if not isinstance(zero, int) or not 0 <= zero <= 4095:
            raise ValueError(f'ID {servo_id}: invalid encoder position')
        limit = root.find(f"joint[@name='{name}']/limit")
        lower = max(0, state['min_position_raw'], zero - 64,
                    zero + math.ceil(float(limit.get('lower')) / TICK))
        upper = min(4095, state['max_position_raw'], zero + 64,
                    zero + math.floor(float(limit.get('upper')) / TICK))
        if not lower <= zero <= upper or max(upper-zero, zero-lower) < 40:
            raise ValueError(f'ID {servo_id}: insufficient non-wrapping bench travel; reposition before capture')
        joints[name] = dict(servo_id=servo_id, zero_tick=zero, direction=1,
                           min_tick=lower, max_tick=upper,
                           min_radians=(lower-zero)*TICK, max_radians=(upper-zero)*TICK,
                           max_velocity=0.1)
    return dict(profile_kind='unloaded_ros_bench', calibration_confirmed=hardware,
                capture_backend='hardware' if hardware else 'mock',
                captured_at=datetime.now(timezone.utc).isoformat(),
                note='Loose unloaded motors only. Current encoder readings are temporary zeros; not arm calibration.',
                port=port, timeout_ms=20, speed=50, acceleration=5, torque_limit=300,
                max_feedback_age=0.25, joints=joints)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--port', default='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00')
    parser.add_argument('--ids', nargs=6, type=int, default=[1, 2, 3, 4, 5, 6],
                        help='Six connected IDs, assigned to the six ROS joints in this order')
    parser.add_argument('--output', type=Path, default=Path('/tmp/pinchy-ros-bench.json'))
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'{args.output}: stop the driver before removing/replacing an old bench profile')
    validate_ids(args.ids, minimum=6)
    bus = ChainBus(args.port, 1_000_000) if args.hardware else MockChainBus(args.ids)
    try:
        states = bus.read(args.ids, full=True)
        profile = make_profile(states, args.ids, args.port, args.hardware)
    finally:
        bus.close()
    # JSON is also valid YAML and can be read by the hardware launch.
    with args.output.open('x') as handle:
        handle.write(json.dumps(profile, indent=2)+'\n')
    print(f'Read-only capture saved: {args.output}')
    for name, joint in profile['joints'].items():
        print(f"ID {joint['servo_id']} ({name}): temporary zero={joint['zero_tick']}, "
              f"allowed encoder range={joint['min_tick']}..{joint['max_tick']}")
    if not args.hardware:
        print('MOCK capture: this file cannot enable physical hardware.')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit(error_text(exc))
