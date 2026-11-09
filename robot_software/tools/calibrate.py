#!/usr/bin/env python3
"""Guide per-joint CAD-zero and direction capture. Reads only; never moves servos.

Default: visit uncaptured zeros in ID order. --id selects one joint; --redo
revisits completed entries. Progress is saved after each capture. JSON output
is valid YAML for the hardware launch, but remains unconfirmed until limits
and physical calibration have been reviewed separately.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import tempfile
import time
import xml.etree.ElementTree as ET

from servo_bench import error_text
from servo_setup import SetupBus, MockSetupBus
from servo_chain import health, preflight_settings

ROOT = Path(__file__).resolve().parents[1]
JOINTS = {1: 'shoulder_pan', 2: 'shoulder_lift', 3: 'elbow_flex',
          4: 'wrist_flex', 5: 'wrist_roll', 6: 'pen_pitch'}
PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00'
TICK = 2 * math.pi / 4096
CONFIG_KEYS = ('stored_id', 'model_raw', 'offset_raw', 'phase_raw', 'resolution_raw',
               'mode', 'baud_code', 'min_position_raw', 'max_position_raw')


def now():
    return datetime.now(timezone.utc).isoformat()


def source_hash():
    source = ROOT / 'src/pinchy_description/cad/cad4/iteration_2_cad_4_arm.urdf'
    return hashlib.sha256(source.read_bytes()).hexdigest()


def new_profile(hardware, port):
    return dict(schema_version=1, profile_kind='assembled_arm', calibration_confirmed=False,
                capture_backend='hardware' if hardware else 'mock',
                cad4_source_sha256=source_hash(), port=port,
                timeout_ms=20, speed=50, acceleration=5, torque_limit=300,
                max_feedback_age=0.25, joints={
                    name: dict(servo_id=servo_id, zero_tick=None, direction=None,
                               min_tick=None, max_tick=None, min_radians=None,
                               max_radians=None, max_velocity=0.1)
                    for servo_id, name in JOINTS.items()}, observations={})


def load_profile(path, hardware, port, status_only=False):
    if not path.exists():
        return new_profile(hardware, port)
    profile = json.loads(path.read_text())
    if profile.get('schema_version') != 1 or profile.get('profile_kind') != 'assembled_arm':
        raise ValueError('Expected an assembled-arm calibration file created by this tool; bench profiles are not accepted')
    if profile.get('cad4_source_sha256') != source_hash():
        raise ValueError('CAD joint definitions changed; review calibration or use a new output file')
    if not status_only and profile.get('capture_backend') != ('hardware' if hardware else 'mock'):
        raise ValueError('Do not mix mock and hardware captures; select the correct --hardware flag or a different --output')
    if not status_only and profile.get('port') != port:
        raise ValueError('Adapter path differs from the saved profile; use its --port or a new output file')
    for servo_id, name in JOINTS.items():
        if profile.get('joints', {}).get(name, {}).get('servo_id') != servo_id:
            raise ValueError('Saved ID-to-joint mapping differs from the confirmed base-to-pen IDs 1–6')
    return profile


def save_profile(path, profile):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=path.parent, prefix='.'+path.name,
                                         suffix='.tmp', delete=False) as handle:
            temporary = Path(handle.name)
            json.dump(profile, handle, indent=2, allow_nan=False); handle.write('\n')
            handle.flush(); os.fsync(handle.fileno())
        if path.exists():
            shutil.copyfile(path, path.with_name(path.name+'.bak'))
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists(): temporary.unlink()


def read_checked(bus, servo_id, hardware):
    bus.servo_id = servo_id
    state = bus.setup_state()
    health(state, servo_id); preflight_settings(state, servo_id)
    if hardware and state.get('model_raw') != 777:
        raise ValueError(f'ID {servo_id}: expected STS3215 model 777')
    if type(state['position']) is not int or not 0 <= state['position'] <= 4095:
        raise ValueError('Encoder is outside the supported single-turn range')
    return state


class UnstablePosition(ValueError):
    pass


def stable_sample(bus, servo_id, hardware, sleep=time.sleep):
    samples = []
    states = []
    for index in range(5):
        state = read_checked(bus, servo_id, hardware)
        if states and any(state[key] != states[0][key] for key in CONFIG_KEYS):
            raise ValueError('Servo configuration changed during capture; nothing saved')
        states.append(state); samples.append(state['position'])
        if index < 4: sleep(0.06)
    if max(samples)-min(samples) > 4:
        raise UnstablePosition('Encoder changed by more than 4 ticks. Support the joint and hold it still, then retry.')
    return dict(position=int(statistics.median(samples)), samples=samples,
                configuration={key: states[-1][key] for key in CONFIG_KEYS}, captured_at=now())


def capture(profile, servo_id, step, sample):
    name = JOINTS[servo_id]; joint = profile['joints'][name]
    observed = profile['observations'].setdefault(name, {})
    if step == 'zero':
        joint.update(zero_tick=sample['position'], direction=None, min_tick=None,
                     max_tick=None, min_radians=None, max_radians=None)
        observed.clear()
        observed['zero'] = dict(sample, pose='Operator aligned this joint to CAD angle 0 rad')
    else:
        if joint['zero_tick'] is None or 'zero' not in observed:
            raise ValueError(f'ID {servo_id}: capture its zero first')
        if sample['configuration'] != observed['zero']['configuration']:
            raise ValueError('Servo configuration differs from zero capture; recapture the zero first')
        delta = sample['position']-joint['zero_tick']
        if not 8 <= abs(delta) <= 1024:
            raise ValueError('Move away from saved zero in the positive CAD direction by any comfortable amount up to 90 degrees (8–1024 encoder ticks, at least about 0.7 degrees); encoder wrap is not accepted')
        joint.update(direction=1 if delta > 0 else -1, min_tick=None,
                     max_tick=None, min_radians=None, max_radians=None)
        observed.pop('limit_lower', None); observed.pop('limit_upper', None)
        observed.pop('safe_position_1', None); observed.pop('safe_position_2', None)
        observed['positive_direction'] = dict(sample, encoder_delta=delta,
            pose='Operator moved this joint a small positive CAD angle from its saved zero')
    profile['calibration_confirmed'] = False
    profile['updated_at'] = now()


def capture_limit(profile, servo_id, side, sample):
    name = JOINTS[servo_id]; joint = profile['joints'][name]
    observed = profile['observations'].get(name, {})
    if joint['zero_tick'] is None or joint['direction'] not in (-1, 1):
        raise ValueError('Capture zero and direction before limits')
    if sample['configuration'] != observed.get('zero', {}).get('configuration'):
        raise ValueError('Servo configuration changed; review/recapture zero first')
    angle = joint['direction'] * (sample['position']-joint['zero_tick']) * TICK
    root = ET.parse(ROOT / 'src/pinchy_description/cad/cad4/iteration_2_cad_4_arm.urdf').getroot()
    limit = root.find(f"joint[@name='{name}']/limit")
    if not float(limit.get('lower')) <= angle <= float(limit.get('upper')):
        raise ValueError(f'{math.degrees(angle):.2f} degrees exceeds the CAD range or crossed encoder wrap')
    config = sample['configuration']
    if not config['min_position_raw'] <= sample['position'] <= config['max_position_raw']:
        raise ValueError('Selected endpoint exceeds the servo EEPROM position range')
    # Legacy side names remain accepted for older callers; capture order is arbitrary.
    side = {'lower': 'first', 'upper': 'second'}.get(side, side)
    if side == 'first':
        joint.update(min_tick=None, max_tick=None, min_radians=None, max_radians=None)
        for key in ['limit_lower', 'limit_upper', 'safe_position_2']:
            observed.pop(key, None)
        observed['safe_position_1'] = dict(sample, angle_radians=angle)
    elif side == 'second':
        first = observed.get('safe_position_1')
        # Resume an older partial session without discarding its saved first endpoint.
        if first is None and 'limit_upper' not in observed:
            first = observed.get('limit_lower')
        if first is None:
            raise ValueError('Capture safe position 1 first')
        if abs(angle-first['angle_radians']) + 1e-12 < 8*TICK:
            raise ValueError('Choose a different safe position, at least 8 ticks (about 0.7 degrees) from position 1')
        second = dict(sample, angle_radians=angle)
        zero = dict(observed['zero'], angle_radians=0.0)
        points = [zero, first, second]
        lower = min(points, key=lambda point: point['angle_radians'])
        upper = max(points, key=lambda point: point['angle_radians'])
        joint.update(min_tick=min(point['position'] for point in points),
                     max_tick=max(point['position'] for point in points),
                     min_radians=lower['angle_radians'], max_radians=upper['angle_radians'])
        observed.update(safe_position_1=first, safe_position_2=second,
                        limit_lower=lower, limit_upper=upper)
    else:
        raise ValueError('Unknown safe position')
    profile['calibration_confirmed'] = False
    profile['updated_at'] = now()


def adjust_zero(profile, servo_id, sample):
    """Small reference correction; retain sign and never expand physical travel."""
    name = JOINTS[servo_id]; joint = profile['joints'][name]
    observed = profile['observations'].get(name, {})
    if (joint['direction'] not in (-1, 1) or joint['zero_tick'] is None or
            joint['min_tick'] is None or joint['max_tick'] is None or
            'zero' not in observed or 'positive_direction' not in observed):
        raise ValueError('Finish zero, direction and limits before making a small zero correction')
    if sample['configuration'] != observed['zero']['configuration']:
        raise ValueError('Servo configuration changed; recapture calibration instead')
    old_zero, new_zero = joint['zero_tick'], sample['position']
    if abs(new_zero-old_zero)*TICK > math.radians(10):
        raise ValueError('Zero correction exceeds 10 degrees; use a full zero/direction/limits capture')
    if not joint['min_tick'] <= new_zero <= joint['max_tick']:
        raise ValueError('New zero must be inside the existing safe physical travel')
    model = ET.parse(ROOT/'src/pinchy_description/urdf/pinchy_arm.urdf').getroot()
    limit = model.find(f"joint[@name='{name}']/limit")
    cad_ticks = [new_zero+joint['direction']*float(limit.get(side))/TICK for side in ('lower', 'upper')]
    low_tick = max(joint['min_tick'], math.ceil(min(cad_ticks)))
    high_tick = min(joint['max_tick'], math.floor(max(cad_ticks)))
    if not low_tick <= new_zero <= high_tick or low_tick >= high_tick:
        raise ValueError('Corrected zero has no valid safe range inside the CAD limits')
    angles = sorted(joint['direction']*(tick-new_zero)*TICK for tick in (low_tick, high_tick))
    observed.setdefault('zero_adjustments', []).append(dict(previous_zero=observed['zero'],
        previous_bounds={key: joint[key] for key in ('min_tick', 'max_tick', 'min_radians', 'max_radians')},
        new_zero_tick=new_zero, captured_at=sample['captured_at']))
    observed['positive_direction'].setdefault('reference_zero_tick', old_zero)
    observed['zero'] = dict(sample, pose='Operator corrected this joint to CAD angle zero')
    for key in ('safe_position_1', 'safe_position_2', 'limit_lower', 'limit_upper'):
        if key in observed:
            observed[key] = dict(observed[key], angle_radians=joint['direction']*(observed[key]['position']-new_zero)*TICK)
    joint.update(zero_tick=new_zero, min_tick=low_tick, max_tick=high_tick,
                 min_radians=angles[0], max_radians=angles[1])
    profile['calibration_confirmed'] = False
    profile['updated_at'] = now()


def run_zero_adjustment(bus, profile, servo_id, hardware, save, ask=input, output=print, sleep=time.sleep):
    name = JOINTS[servo_id]; old_zero = profile['joints'][name]['zero_tick']
    output(f'ID {servo_id} ({name}): support the arm with torque OFF. Saved zero: {old_zero}.')
    output('Manually place ONLY this joint at the corrected CAD zero relative to its parent.')
    output('Corrections up to 10 degrees retain direction and existing physical limits; CAD bounds may narrow travel.')
    while True:
        answer = ask('Enter = save corrected zero, p = read position, q = cancel: ').strip().lower()
        if answer == 'q': return
        if answer == 'p':
            output(f"Current encoder: {read_checked(bus, servo_id, hardware)['position']}"); continue
        if answer: continue
        try:
            sample = stable_sample(bus, servo_id, hardware, sleep)
            adjust_zero(profile, servo_id, sample)
        except ValueError as exc:
            output(f'NOT SAVED: {exc}'); continue
        save(profile)
        output(f"Saved ID {servo_id} zero: {old_zero} -> {sample['position']}. Direction retained; no motor writes.")
        return


def run_limits_session(bus, profile, ids, hardware, save, redo=False, single=False,
                       ask=input, output=print, sleep=time.sleep):
    for servo_id in ids:
        name = JOINTS[servo_id]; joint = profile['joints'][name]
        if joint['zero_tick'] is None or joint['direction'] not in (-1, 1):
            output(f'ID {servo_id}: zero/direction missing; skipping.'); continue
        if joint['min_tick'] is not None and not redo and not single:
            output(f'ID {servo_id}: limits already recorded; keeping them.'); continue
        read_checked(bus, servo_id, hardware)
        output(f'\nID {servo_id}: {name}. Keep the arm supported; select a small CLEAR range, not mechanical stops.')
        output('Record two safe positions in either order. Saved zero is included automatically; the script sorts the bounds.')
        output('Use a small clear range initially. These points do not prove collision safety between poses.')
        observed = profile['observations'].get(name, {})
        first_saved = 'safe_position_1' in observed or 'limit_lower' in observed
        second_missing = 'safe_position_2' not in observed and 'limit_upper' not in observed
        resume_second = first_saved and second_missing and not redo and not single
        for side in (['second'] if resume_second else ['first', 'second']):
            output(f'Choose safe position {1 if side == "first" else 2}; either movement direction is allowed.')
            while True:
                answer = ask('Enter = capture, p = read angle, s = skip motor, q = save progress and quit: ').strip().lower()
                if answer == 'q': return
                if answer == 's': break
                if answer == 'p':
                    state = read_checked(bus, servo_id, hardware)
                    angle = joint['direction']*(state['position']-joint['zero_tick'])*TICK
                    output(f"Current: {math.degrees(angle):+.2f} degrees, encoder {state['position']}")
                    continue
                if answer:
                    output('Use Enter, p, s or q.'); continue
                try:
                    sample = stable_sample(bus, servo_id, hardware, sleep)
                    capture_limit(profile, servo_id, side, sample)
                except ValueError as exc:
                    output(f'NOT SAVED: {exc}'); continue
                save(profile)
                output(f'Saved safe position {1 if side == "first" else 2}. Calibration remains unconfirmed pending review.')
                break
            if answer == 's': break


def show_status(profile, output=print):
    output(f"Capture backend: {profile['capture_backend']}; assembled-arm calibration confirmed: {profile['calibration_confirmed']}")
    output('ID  Joint           Zero tick    Direction    Limits')
    for servo_id, name in JOINTS.items():
        joint = profile['joints'][name]
        zero = '--' if joint['zero_tick'] is None else str(joint['zero_tick'])
        direction = '--' if joint['direction'] is None else f"{joint['direction']:+d}"
        observed = profile['observations'].get(name, {})
        partial = 'safe_position_1' in observed or 'limit_lower' in observed
        limits = ('position 2 pending' if partial else 'pending') if joint['min_tick'] is None else 'recorded; review required'
        output(f'{servo_id:<3} {name:<15} {zero:<12} {direction:<12} {limits}')
    if profile.get('calibration_confirmed'):
        output('This file is marked confirmed. This command does not enable motors; recapture clears confirmation.')
    else:
        output('Motor control remains disabled in this file until physical limits and calibration are reviewed.')


def run_session(bus, profile, ids, step, hardware, save, redo=False, single=False,
                ask=input, output=print, sleep=time.sleep):
    key = 'zero_tick' if step == 'zero' else 'direction'
    for servo_id in ids:
        name = JOINTS[servo_id]; joint = profile['joints'][name]
        if joint[key] is not None and not redo and not single:
            output(f'ID {servo_id} {name}: already captured; keeping it (--redo to replace).')
            continue
        if step == 'direction' and joint['zero_tick'] is None:
            output(f'ID {servo_id} {name}: zero missing; capture zero first.'); continue
        state = read_checked(bus, servo_id, hardware)
        output(f"\nID {servo_id}: {name}; current encoder={state['position']}")
        if step == 'zero':
            output('Align ONLY this joint to its CAD/RViz angle 0, relative to its parent. Other joints need not be zero.')
            reference = ET.parse(ROOT / 'src/pinchy_description/cad/cad4/iteration_2_cad_4_arm.urdf').getroot()
            joint_frame = reference.find(f"joint[@name='{name}']")
            output(f"Compare {joint_frame.find('child').get('link')} relative to {joint_frame.find('parent').get('link')} in the reference model.")
            output('This is a physical reference alignment, not encoder 2048 or the midpoint between stops.')
            if joint['zero_tick'] is not None:
                output(f"Replacing saved zero {joint['zero_tick']} clears this joint's direction and limits.")
        else:
            output(f"Saved zero={joint['zero_tick']}. Start there, then gently move ONLY this joint any comfortable amount in the positive CAD direction, up to 90 degrees. No exact angle is needed.")
            output('Use enough movement to distinguish direction (at least about 0.7 degrees). Stay within the joint\'s clear, comfortable travel.')
            output('For these CAD4 joint frames, positive is clockwise when viewed directly at the driven horn face. Do not force a joint.')
        while True:
            answer = ask('Enter = capture, s = skip, q = save progress and quit: ').strip().lower()
            if answer == 'q': return
            if answer == 's': break
            if answer:
                output('Use Enter, s or q.'); continue
            try:
                sample = stable_sample(bus, servo_id, hardware, sleep)
                capture(profile, servo_id, step, sample)
            except (UnstablePosition, ValueError) as exc:
                output(f'NOT SAVED: {exc}'); continue
            save(profile)
            output(f"Saved ID {servo_id}: zero={joint['zero_tick']}, direction={joint['direction']}.")
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Use real servo reads; default mock')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--id', type=int, choices=JOINTS, help='Visit only this motor, including an existing capture')
    parser.add_argument('--step', choices=['zero', 'direction', 'limits', 'adjust-zero'], default='zero')
    parser.add_argument('--redo', action='store_true', help='Revisit completed entries in an all-motor run')
    parser.add_argument('--status', action='store_true', help='Show saved progress without opening the bus')
    parser.add_argument('--output', type=Path, default=ROOT/'calibration/arm.json')
    args = parser.parse_args()
    if args.step == 'adjust-zero' and args.id is None:
        parser.error('--step adjust-zero requires --id to select one calibrated joint')
    profile = load_profile(args.output, args.hardware, args.port, args.status)
    print(f'Calibration file: {args.output}')
    if args.status:
        show_status(profile); return
    print('READ-ONLY calibration. Support the assembled arm. Torque must already be off.')
    print('No motion, torque, RAM or EEPROM writes are performed. Stop other serial programs first.')
    if not args.hardware: print('MOCK mode: no real motors are read; use --hardware for the arm.')
    # Opening the SDK with no ID does not command or ping an unselected motor.
    bus = SetupBus(args.port, 1_000_000, None) if args.hardware else MockSetupBus()
    try:
        selected = [args.id] if args.id else list(JOINTS)
        options = dict(redo=args.redo, single=args.id is not None)
        save = lambda value: save_profile(args.output, value)
        if args.step == 'adjust-zero':
            run_zero_adjustment(bus, profile, args.id, args.hardware, save)
        elif args.step == 'limits':
            run_limits_session(bus, profile, selected, args.hardware, save, **options)
        else:
            run_session(bus, profile, selected, args.step, args.hardware, save, **options)
    except (KeyboardInterrupt, EOFError):
        print('\nStopped. Previously saved joints are preserved.')
    finally:
        bus.close()
    show_status(profile)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise SystemExit(error_text(exc))
