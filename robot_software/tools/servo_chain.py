#!/usr/bin/env python3
"""Inspect and bench-test 2–6 unloaded STS3215-12V motors on one TTL bus."""
import argparse
import json
import sys
import time
from dataclasses import dataclass

from servo_bench import MAX_BENCH_SPEED, error_text
from servo_setup import MockSetupBus, SetupBus
from servo_diagnostics import Telemetry, temperature_spike_unconfirmed, watch_temperatures


@dataclass(frozen=True)
class Motor:
    servo_id: int
    lower: int
    upper: int


def motor_arg(value):
    try:
        return Motor(*map(int, value.split(':')))
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError('Expected ID:MIN:MAX, e.g. 1:1000:3000') from exc


def validate_ids(ids, minimum=2):
    if not minimum <= len(ids) <= 6 or len(set(ids)) != len(ids):
        raise ValueError(f'Provide {minimum}–6 distinct motor IDs')
    if any(not 1 <= servo_id <= 253 for servo_id in ids):
        raise ValueError('IDs must be in 1..253')


class ChainBus:
    """One serial connection, sequential feedback reads, grouped goal writes."""
    def __init__(self, port, baud):
        self.bus = SetupBus(port, baud, None)

    def read(self, ids, full=False):
        states = {}
        for servo_id in ids:
            self.bus.servo_id = servo_id
            state = (self.bus.setup_state() if full else
                     dict(self.bus.read(), status_raw=self.bus.byte(65)))
            if state['temperature_raw'] >= 60:
                # Retain the first value and confirm it through separate reads.
                state['temperature_recheck_raw'] = self.bus.byte(63)
                state['temperature_block_raw'] = self.bus.word(62) >> 8
                state['status_recheck_raw'] = self.bus.byte(65)
                state['temperature_spike_unconfirmed'] = temperature_spike_unconfirmed(state)
            states[servo_id] = state
        observer = getattr(self, 'observer', None)
        if observer:
            observer(states, getattr(self, 'phase', 'read'))
        return states

    def limit(self, servo_id):
        self.bus.servo_id = servo_id
        self.bus.set_torque_limit(300)

    def enable(self, servo_id):
        self.bus.servo_id = servo_id
        self.bus.enable()

    def goals(self, targets, speeds, acceleration):
        sdk = self.bus.sdk
        sdk.groupSyncWrite.clearParam()
        try:
            for servo_id, target in targets.items():
                if not sdk.SyncWritePosEx(servo_id, target, speeds[servo_id], acceleration):
                    raise RuntimeError(f'Could not queue goal for ID {servo_id}')
            self.bus.check((sdk.groupSyncWrite.txPacket(), 0))
        finally:
            sdk.groupSyncWrite.clearParam()

    def release(self, servo_id):
        self.bus.servo_id = servo_id
        self.bus.release()
        if self.bus.read()['torque'] != 0:
            raise RuntimeError('Torque-disable readback failed')

    def close(self):
        self.bus.close()


class MockChainBus:
    """Instantaneous mock feedback; does not simulate motion timing or loading."""
    def __init__(self, ids):
        self.motors = {servo_id: MockSetupBus(servo_id) for servo_id in ids}

    def read(self, ids, full=False):
        states = {servo_id: (self.motors[servo_id].setup_state() if full else
                          dict(self.motors[servo_id].read(), status_raw=0))
                for servo_id in ids}
        observer = getattr(self, 'observer', None)
        if observer:
            observer(states, getattr(self, 'phase', 'read'))
        return states

    def limit(self, servo_id):
        self.motors[servo_id].set_torque_limit(300)

    def goals(self, targets, speeds, acceleration):
        for servo_id, target in targets.items():
            motor = self.motors[servo_id]
            motor.goal = target
            # A grouped goal write does not necessarily enable torque.
            if motor.torque == 1:
                motor.position = target

    def enable(self, servo_id):
        motor = self.motors[servo_id]
        motor.enable()
        motor.position = motor.goal

    def release(self, servo_id):
        self.motors[servo_id].release()

    def close(self):
        pass


def health(state, servo_id):
    if state['mode'] != 0:
        raise ValueError(f'ID {servo_id}: requires position mode')
    if not 100 <= state['voltage_raw'] <= 130:
        raise ValueError(f"ID {servo_id}: reports {state['voltage_raw']/10:.1f} V; require 10–13 V")
    problems = []
    if state['temperature_raw'] >= 60 and not temperature_spike_unconfirmed(state):
        problems.append('bench temperature limit reached (60 C)')
    status = state['status_raw'] | state.get('status_recheck_raw', 0)
    if status:
        flags = {0: 'voltage', 1: 'encoder', 2: 'temperature',
                 3: 'current', 5: 'load/overload'}
        names = [name for bit, name in flags.items() if status & (1 << bit)]
        unknown = status & ~sum(1 << bit for bit in flags)
        if unknown:
            names.append(f'unknown bits 0x{unknown:02x}')
        problems.append('servo alarm: ' + ', '.join(names))
    if problems:
        rechecks = ''
        if 'temperature_recheck_raw' in state:
            rechecks = (f", temperature_recheck={state['temperature_recheck_raw']} C, "
                        f"temperature_block={state['temperature_block_raw']} C")
        raise ValueError(f"ID {servo_id}: {'; '.join(problems)}; "
                         f"temperature={state['temperature_raw']} C, "
                         f"voltage={state['voltage_raw']/10:.1f} V, "
                         f'status=0x{status:02x}{rechecks}. Inspect before retrying.')


def preflight_settings(state, servo_id):
    if state['stored_id'] != servo_id:
        raise ValueError(f"ID {servo_id}: stored ID reads {state['stored_id']}; inspect bus addressing")
    if state['resolution_raw'] != 1:
        raise ValueError(f"ID {servo_id}: resolution={state['resolution_raw']}, expected 1")
    if state['torque'] != 0:
        raise ValueError(f"ID {servo_id}: torque already enabled (torque={state['torque']}). "
                         'Run chain release --ids <connected IDs> before restarting.')


def make_stages(ids, delta, speed, mixed_speeds, pattern):
    base_speeds = dict.fromkeys(ids, speed)
    stages = []
    if pattern in ('individual', 'all'):
        for selected in ids:
            stages.append((f'individual-{selected}',
                           {servo_id: delta if servo_id == selected else 0 for servo_id in ids},
                           base_speeds))
    if pattern in ('together', 'all'):
        stages.append(('together', dict.fromkeys(ids, delta), base_speeds))
    if pattern in ('opposite', 'all'):
        stages.append(('opposite', {servo_id: delta * (-1 if i % 2 else 1)
                                   for i, servo_id in enumerate(ids)}, base_speeds))
    if pattern in ('mixed', 'all'):
        mixed = mixed_speeds or [speed // 2 if i % 2 == 0 else speed for i in range(len(ids))]
        if len(mixed) != len(ids):
            raise ValueError('--mixed-speeds needs one value per motor, in --motor order')
        stages.append(('mixed', dict.fromkeys(ids, delta), dict(zip(ids, mixed))))
    if not stages:
        raise ValueError('Unknown pattern')
    return stages


def validate_tolerance(tolerance):
    if not isinstance(tolerance, int) or not 1 <= tolerance <= 10:
        raise ValueError('Require --tolerance 1..10 encoder ticks')


def wait_for(bus, motors, targets, timeout, tolerance=2):
    validate_tolerance(tolerance)
    ids = [motor.servo_id for motor in motors]
    started = time.monotonic()
    arrived = {}
    while True:
        states = bus.read(ids)
        for motor in motors:
            servo_id, state = motor.servo_id, states[motor.servo_id]
            health(state, servo_id)
            if state['torque'] != 1:
                raise RuntimeError(f"ID {servo_id}: torque={state['torque']}, expected 1")
            if not motor.lower <= state['position'] <= motor.upper:
                raise RuntimeError(f"ID {servo_id}: position={state['position']} "
                                   f'outside bounds {motor.lower}..{motor.upper}')
            if abs(state['position'] - targets[servo_id]) <= tolerance:
                arrived.setdefault(servo_id, round(time.monotonic() - started, 3))
        if all(abs(states[i]['position'] - targets[i]) <= tolerance for i in ids):
            return dict(feedback=states, reached_after_s=arrived, tolerance_ticks=tolerance,
                        errors_ticks={i: states[i]['position'] - targets[i] for i in ids})
        if time.monotonic() - started >= timeout:
            errors = {i: states[i]['position'] - targets[i] for i in ids}
            raise TimeoutError(f'Targets not reached; position errors in ticks: {errors}; '
                               f'allowed +/-{tolerance}; targets={targets}')
        time.sleep(0.05)


def release_all(bus, ids, original):
    bus.phase = 'release'
    failures = {}
    # Never abandon remaining motors when releasing one fails.
    for servo_id in ids:
        try:
            bus.release(servo_id)
        except Exception as exc:
            failures[servo_id] = str(exc)
    if failures:
        raise RuntimeError(f'Torque release failed for {failures}; cut motor power. '
                           f'Original error: {error_text(original) if original is not None else "none"}')


def release_motors(bus, ids):
    validate_ids(ids, minimum=1)
    release_all(bus, ids, None)
    after = bus.read(ids)
    if any(after[i]['torque'] != 0 for i in ids):
        raise RuntimeError('Torque release not confirmed for every motor; cut motor power')
    return dict(after=after)


def enable_at_current_positions(bus, motors, start, speeds, acceleration, timeout):
    """Verify all current-position goals before explicitly enabling any motor."""
    ids = [motor.servo_id for motor in motors]
    bus.phase = 'startup/hold'
    bus.goals(start, speeds, acceleration)
    staged = bus.read(ids, full=True)
    for servo_id in ids:
        state = staged[servo_id]
        health(state, servo_id)
        if state['goal_position_raw'] != start[servo_id]:
            raise RuntimeError(f'ID {servo_id}: current-position goal readback failed; torque not enabled')
        if abs(state['position'] - start[servo_id]) > 2:
            raise RuntimeError(f'ID {servo_id}: position changed during startup; inspect and retry')
    for servo_id in ids:
        bus.enable(servo_id)
    wait_for(bus, motors, start, timeout)


def position_motors(bus, ids, target, lower, upper, speed=300, acceleration=1, timeout=30,
                    tolerance=2):
    """Move unloaded motors to a common absolute reading, then release torque."""
    validate_ids(ids, minimum=1)
    validate_tolerance(tolerance)
    if not (0 <= lower < upper <= 4095 and lower <= target <= upper):
        raise ValueError('Require 0 <= min <= target <= max <= 4095, with min < max')
    if not (1 <= speed <= MAX_BENCH_SPEED and 1 <= acceleration <= 10 and 0 < timeout <= 30):
        raise ValueError(f'Require speed 1..{MAX_BENCH_SPEED}, acceleration 1..10, timeout >0..30')
    bus.phase = 'preflight'
    before = bus.read(ids, full=True)
    start = {i: before[i]['position'] for i in ids}
    for i, state in before.items():
        health(state, i)
        preflight_settings(state, i)
        if not (state['min_position_raw'] <= lower < upper <= state['max_position_raw']
                and lower <= start[i] <= upper):
            raise ValueError(f'ID {i}: start/bounds outside stored single-turn limits')
        if abs(target - start[i]) > 2048:
            raise ValueError(f'ID {i}: move exceeds 180 degrees; choose an intermediate target')
        if state['phase_raw'] & 4 and speed < 50:
            raise ValueError(f'ID {i}: fine speed units require speed >=50')
    motors = [Motor(i, lower, upper) for i in ids]
    targets = dict.fromkeys(ids, target)
    speeds = dict.fromkeys(ids, speed)
    try:
        for i in ids:
            bus.limit(i)
        enable_at_current_positions(bus, motors, start, speeds, acceleration, timeout)
        bus.phase = 'position/target'
        bus.goals(targets, speeds, acceleration)
        arrived = wait_for(bus, motors, targets, timeout, tolerance)
    finally:
        release_all(bus, ids, sys.exc_info()[1])
    after = bus.read(ids)
    if any(after[i]['torque'] != 0 for i in ids):
        raise RuntimeError('Torque release not confirmed for every motor; cut motor power')
    return dict(before=before, targets=targets, arrived=arrived, after=after,
                errors_ticks={i: arrived['feedback'][i]['position'] - target for i in ids},
                speed_raw=speed, acceleration_raw=acceleration, torque_limit_raw=300,
                tolerance_ticks=tolerance)


def run_test(bus, motors, delta=256, speed=300, acceleration=1, timeout=20,
             pattern='all', mixed_speeds=None, progress=None, tolerance=2):
    ids = [motor.servo_id for motor in motors]
    validate_ids(ids)
    validate_tolerance(tolerance)
    if not 4 <= abs(delta) <= 1024:
        raise ValueError('Require delta +/-4..1024 ticks')
    if abs(delta) <= tolerance:
        raise ValueError('Delta must exceed tolerance so no movement cannot pass')
    if not (1 <= speed <= MAX_BENCH_SPEED and 1 <= acceleration <= 10 and 0 < timeout <= 30):
        raise ValueError(f'Require speed 1..{MAX_BENCH_SPEED}, acceleration 1..10, timeout >0..30')
    stages = make_stages(ids, delta, speed, mixed_speeds, pattern)
    bus.phase = 'preflight'
    before = bus.read(ids, full=True)  # Every requested ID must reply before any write.
    start = {i: before[i]['position'] for i in ids}
    units = set()
    for motor in motors:
        i, state = motor.servo_id, before[motor.servo_id]
        health(state, i)
        preflight_settings(state, i)
        if not (0 <= motor.lower < motor.upper <= 4095 and
                state['min_position_raw'] <= motor.lower < motor.upper <= state['max_position_raw']):
            raise ValueError(f'ID {i}: bounds must fit stored single-turn limits')
        units.add(bool(state['phase_raw'] & 4))
        for name, offsets, speeds in stages:
            if not motor.lower <= start[i] <= motor.upper or not motor.lower <= start[i]+offsets[i] <= motor.upper:
                raise ValueError(f'ID {i}: {name} start/target outside bounds; no motor was moved')
            if not 1 <= speeds[i] <= MAX_BENCH_SPEED or (state['phase_raw'] & 4 and speeds[i] < 50):
                raise ValueError(f'ID {i}: invalid raw speed for its units')
    if len(units) != 1:
        raise ValueError('Motors use different speed units; inspect before comparing raw speeds')
    results = []
    phase = 'startup'
    return_attempted = False
    try:
        for servo_id in ids:
            bus.limit(servo_id)
        enable_at_current_positions(bus, motors, start, dict.fromkeys(ids, speed),
                                    acceleration, timeout)
        for name, offsets, speeds in stages:
            phase = f'{name}/outward'
            bus.phase = phase
            return_attempted = False
            if progress:
                progress(f'{name}: outward')
            targets = {i: start[i] + offsets[i] for i in ids}
            bus.goals(targets, speeds, acceleration)
            outward = wait_for(bus, motors, targets, timeout, tolerance)
            phase = f'{name}/return'
            bus.phase = phase
            return_attempted = True
            if progress:
                progress(f'{name}: return')
            bus.goals(start, speeds, acceleration)
            returned = wait_for(bus, motors, start, timeout, tolerance)
            results.append(dict(stage=name, targets=targets, speeds_raw=speeds,
                                outward=outward, returned=returned))
    except Exception as exc:
        return_status = 'attempted' if return_attempted else 'not attempted'
        exc.add_note(f'Motion phase: {phase}; return-to-start command {return_status}. '
                     'Test stopped; torque release will be attempted for all selected motors.')
        raise
    finally:
        release_all(bus, ids, sys.exc_info()[1])
    after = bus.read(ids)
    if any(after[i]['torque'] != 0 for i in ids):
        raise RuntimeError('Torque release not confirmed for every motor; cut motor power')
    return dict(before=before, stages=results, after=after, torque_limit_raw=300,
                tolerance_ticks=tolerance)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Use real bus; otherwise mock')
    parser.add_argument('--port', help='Serial device; one process must own this port')
    parser.add_argument('--baud', type=int, default=1000000)
    sub = parser.add_subparsers(dest='command', required=True)
    inspect = sub.add_parser('inspect', help='Read-only inspection of requested IDs')
    inspect.add_argument('--ids', nargs='+', type=int, required=True)
    release = sub.add_parser('release', help='Disable torque on selected unloaded/supported motors')
    release.add_argument('--ids', nargs='+', type=int, required=True)
    watch = sub.add_parser('watch', help='Continuously poll temperatures/voltage/status; no motor writes')
    watch.add_argument('--ids', nargs='+', type=int, required=True)
    watch.add_argument('--duration', type=float, default=0, help='Seconds to monitor; default 0 means until Ctrl+C')
    position = sub.add_parser('position', help='Move unloaded motors to one absolute encoder value; ends torque off')
    position.add_argument('--ids', nargs='+', type=int, required=True, help='1–6 unique IDs')
    position.add_argument('--target', type=int, required=True, help='Absolute encoder target 0..4095; 2048 is midrange')
    position.add_argument('--min', dest='lower', type=int, required=True, help='Lowest permitted absolute encoder position')
    position.add_argument('--max', dest='upper', type=int, required=True, help='Highest permitted absolute encoder position')
    position.add_argument('--speed', type=int, default=300, help=f'Raw speed 1..{MAX_BENCH_SPEED}; default 300')
    position.add_argument('--acceleration', type=int, default=1, help='Raw acceleration 1..10; default 1')
    position.add_argument('--timeout', type=float, default=30, help='Seconds per movement leg, >0..30; default 30')
    test = sub.add_parser('test', help='Run unloaded chain movement patterns')
    test.add_argument('--motor', type=motor_arg, action='append', required=True,
                      help='Repeat ID:MIN:MAX for each motor, e.g. 1:1000:3000')
    test.add_argument('--pattern', choices=['individual', 'together', 'opposite', 'mixed', 'all'], default='all')
    test.add_argument('--delta', type=int, default=256, help='Signed outward movement, +/-4..1024 ticks; default 256=22.5 degrees')
    test.add_argument('--speed', type=int, default=300, help=f'Raw speed 1..{MAX_BENCH_SPEED}; default 300')
    test.add_argument('--mixed-speeds', nargs='+', type=int, help='One raw speed per motor for mixed stage; default alternates half/full --speed')
    test.add_argument('--acceleration', type=int, default=1, help='Raw acceleration 1..10; default 1')
    test.add_argument('--timeout', type=float, default=20, help='Seconds per leg, >0..30; default 20')
    for command in (inspect, release, watch, position, test):
        command.add_argument('--log', help='Append timestamped samples and hardware read/reply packets to JSONL')
    for command in (watch, position, test):
        command.add_argument('--interval', type=float, default=0.25,
                             help='Live display interval in seconds, 0.05..10; default 0.25')
    for command in (position, test):
        command.add_argument('--watch-temps', action='store_true', help='Show live temperature/voltage/status during this motion')
        command.add_argument('--tolerance', type=int, default=2,
                             help='Arrival tolerance 1..10 encoder ticks; default 2; actual errors remain reported')
    args = parser.parse_args()
    if args.hardware != bool(args.port) or args.baud <= 0:
        parser.error('Require --hardware together with --port, and positive baud')
    bus = None
    telemetry = None
    try:
        ids = args.ids if args.command in ('inspect', 'release', 'position', 'watch') else [m.servo_id for m in args.motor]
        validate_ids(ids, minimum=1 if args.command in ('release', 'position', 'watch') else 2)
        display = args.command == 'watch' or getattr(args, 'watch_temps', False)
        if display or args.log:
            telemetry = Telemetry(args.log, getattr(args, 'interval', 0.25),
                                  stream=sys.stderr if display else None)
        bus = ChainBus(args.port, args.baud) if args.hardware else MockChainBus(ids)
        bus.phase = args.command
        if telemetry:
            bus.observer = telemetry.sample
            telemetry.event('command', command=args.command,
                            tolerance_ticks=getattr(args, 'tolerance', None))
            if args.hardware:
                bus.bus.sdk.trace = lambda packet: telemetry.packet(packet, bus.phase)
        if args.command == 'inspect':
            result = dict(motors=bus.read(ids, full=True))
        elif args.command == 'release':
            result = release_motors(bus, ids)
        elif args.command == 'watch':
            result = watch_temperatures(bus, ids, args.interval, args.duration)
        elif args.command == 'position':
            result = position_motors(bus, ids, args.target, args.lower, args.upper,
                                     args.speed, args.acceleration, args.timeout, args.tolerance)
        else:
            result = run_test(bus, args.motor, args.delta, args.speed, args.acceleration,
                              args.timeout, args.pattern, args.mixed_speeds,
                              progress=lambda message: print(message, file=sys.stderr, flush=True),
                              tolerance=args.tolerance)
        print(json.dumps(dict(backend='hardware' if args.hardware else 'mock', **result), indent=2))
    except (Exception, KeyboardInterrupt) as exc:
        if telemetry:
            telemetry.event('error', message=error_text(exc))
        if isinstance(exc, KeyboardInterrupt) and args.command == 'watch':
            print('Temperature monitor stopped.', file=sys.stderr)
        else:
            parser.exit(1, error_text(exc) + '\n')
    finally:
        try:
            if bus is not None:
                bus.close()
        finally:
            if telemetry:
                telemetry.close()


if __name__ == '__main__':
    main()
