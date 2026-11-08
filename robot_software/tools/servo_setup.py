#!/usr/bin/env python3
"""Set up one unloaded STS3215-12V servo; mock by default, independent of ROS."""
import argparse
import json

from servo_bench import MAX_BENCH_SPEED, FeetechBus, MockBus, error_text, move_to_position


class SetupBus(FeetechBus):
    def scan(self):
        from scservo_sdk import COMM_RX_TIMEOUT
        found = []
        for servo_id in range(1, 254):
            model, comm, error = self.sdk.ping(servo_id)
            if comm == COMM_RX_TIMEOUT:
                continue
            self.check((comm, error))
            found.append(dict(servo_id=servo_id, model_raw=model))
        return found

    def write_byte(self, address, value):
        self.check(self.sdk.write1ByteTxRx(self.servo_id, address, value))

    def set_id(self, new_id):
        # Some firmware replies under the new ID, making the write ACK time out.
        # Confirm the new address instead of treating a missing ACK as failure.
        comm, error = self.sdk.write1ByteTxRx(self.servo_id, 5, new_id)
        from scservo_sdk import COMM_RX_TIMEOUT
        if comm != COMM_RX_TIMEOUT:
            self.check((comm, error))
        self.servo_id = new_id
        self.check(self.sdk.ping(new_id)[1:])
        if self.byte(5) != new_id:
            raise RuntimeError('ID readback did not match requested ID')

    def set_torque_limit(self, value):
        self.check(self.sdk.write2ByteTxRx(self.servo_id, 48, value))
        if self.word(48) != value:
            raise RuntimeError('Torque-limit readback failed')

    def enable(self):
        self.write_byte(40, 1)

    def setup_state(self):
        return dict(self.inspect(), phase_raw=self.byte(18),
                    resolution_raw=self.byte(30), status_raw=self.byte(65))


class MockSetupBus(MockBus):
    def __init__(self, servo_id=1):
        super().__init__()
        self.servo_id = servo_id
        self.torque = 0
        self.mode, self.lock = 0, 1
        self.goal, self.limit = 0, 1000

    def scan(self):
        return [dict(servo_id=self.servo_id, model_raw=None)]

    def inspect(self):
        return dict(self.read(), mode=self.mode, model_raw=None,
                    stored_id=self.servo_id, baud_code=0, min_position_raw=0,
                    max_position_raw=4095, offset_raw=0,
                    goal_position_raw=self.goal, eeprom_lock=self.lock)

    def setup_state(self):
        return dict(self.inspect(), phase_raw=0, resolution_raw=1, status_raw=0)

    def write_byte(self, address, value):
        if address == 55:
            self.lock = value
        elif address == 33:
            self.mode = value
        else:
            raise ValueError('Unsupported mock configuration address')

    def set_id(self, new_id):
        self.servo_id = new_id

    def set_torque_limit(self, value):
        self.limit = value

    def move(self, position, speed, acceleration):
        self.goal = position
        # STS firmware may auto-enable when a goal is written.
        self.torque = 1
        super().move(position, speed, acceleration)

    def enable(self):
        self.torque = 1


def configure(bus, new_id):
    if not 1 <= new_id <= 253:
        raise ValueError('New ID must be within 1..253')
    before = bus.inspect()
    if before['torque'] != 0:
        raise ValueError('Release torque before configuration')
    found = bus.scan()
    if len(found) != 1 or found[0]['servo_id'] != bus.servo_id:
        raise ValueError('Expected exactly one responding servo at the selected ID')
    changes = []
    if (before['stored_id'] == new_id and before['mode'] == 0
            and before['eeprom_lock'] == 1):
        return dict(before=before, after=before, changes=changes)
    try:
        bus.write_byte(55, 0)
        if before['mode'] != 0:
            bus.write_byte(33, 0)
            if bus.inspect()['mode'] != 0:
                raise RuntimeError('Position-mode readback failed')
            changes.append('mode=0')
        if before['stored_id'] != new_id:
            bus.set_id(new_id)
            changes.append(f'id={new_id}')
    finally:
        # If an ID write failed ambiguously, both addresses may need recovery.
        # Best effort relock; never mask the original failure.
        import sys
        failing = sys.exc_info()[0] is not None
        try:
            bus.write_byte(55, 1)
        except Exception:
            if not failing:
                raise
    after = bus.inspect()
    if after['stored_id'] != new_id or after['mode'] != 0 or after['eeprom_lock'] != 1:
        raise RuntimeError('Configuration verification failed; rescan before retrying')
    return dict(before=before, after=after, changes=changes,
                persistence='Power-cycle and inspect to verify EEPROM persistence')


def motion_test(bus, delta, lower, upper, speed=None, acceleration=1, timeout=8):
    """Unloaded out-and-back test, up to 90 degrees; release torque on exit."""
    if not (0 < abs(delta) <= 1024 and 0 <= lower < upper <= 4095):
        raise ValueError('Require delta +/-1..1024 and 0 <= min < max <= 4095')
    if ((speed is not None and not 1 <= speed <= MAX_BENCH_SPEED)
            or not (1 <= acceleration <= 10 and 0 < timeout <= 30)):
        raise ValueError(f'speed: 1..{MAX_BENCH_SPEED}, acceleration: 1..10, timeout: (0,30]')
    before = bus.setup_state()
    start, target = before['position'], before['position'] + delta
    if before['mode'] != 0 or before['torque'] != 0:
        raise ValueError('Test requires position mode with torque disabled')
    if not 100 <= before['voltage_raw'] <= 130:
        raise ValueError(f"Servo reports {before['voltage_raw']/10:.1f} V; "
                         'this 12 V bench test requires 10.0..13.0 V at the servo')
    if before['temperature_raw'] >= 60 or before['status_raw'] != 0:
        raise ValueError('Servo is hot or reports an alarm; inspect before testing')
    if before['resolution_raw'] != 1:
        raise ValueError('Test requires standard encoder resolution=1')
    if not (lower <= start <= upper and lower <= target <= upper):
        raise ValueError('Start/target exceeds supplied limits; do not cross encoder wrap')
    if not (before['min_position_raw'] <= lower < upper <= before['max_position_raw']):
        raise ValueError('Supplied limits must fit the stored single-turn limits')
    # Fine units need >=50 for a physical speed step. Adapt to the read setting
    # without changing persistent phase bits or silently overriding user values.
    if speed is None:
        speed = 50 if before['phase_raw'] & 4 else 5
    if before['phase_raw'] & 4 and speed < 50:
        raise ValueError(f'Fine speed units selected; choose --speed 50..{MAX_BENCH_SPEED}')
    phase = 'startup'
    try:
        bus.set_torque_limit(300)  # Temporary RAM limit; 30%, not calibrated N m.
        # A goal write may auto-enable output. Its first goal MUST be current pose.
        bus.move(start, speed, acceleration)
        bus.enable()
        ready = bus.read()
        if abs(ready['position'] - start) > 2:
            raise RuntimeError('Unexpected position change during enable')
        phase = 'outward'
        outward = move_to_position(bus, target, lower, upper, speed, acceleration, timeout)
        # Return to the original measured target, independent of arrival error.
        phase = 'return'
        returned = move_to_position(bus, start, lower, upper, speed, acceleration, timeout)
        if abs(returned['position'] - start) > 2:
            raise RuntimeError('Return position outside 2-tick tolerance')
    except Exception as exc:
        return_status = 'attempted' if phase == 'return' else 'not attempted'
        exc.add_note(f'Motion phase: {phase}; return-to-start command {return_status}. '
                     'Test stopped; torque release will be attempted.')
        raise
    finally:
        import sys
        original = sys.exc_info()[1]
        try:
            bus.release()
        except Exception as exc:
            raise RuntimeError(f'Torque release failed; cut motor power. '
                               f'Original failure: {original}; release: {exc}') from exc
    after = bus.read()
    if after['torque'] != 0:
        raise RuntimeError('Torque release readback failed; cut motor power')
    return dict(before=before, outward=outward, returned=returned, after=after,
                start_ticks=start, target_ticks=target, torque_limit_raw=300,
                speed_raw=speed, acceleration_raw=acceleration)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Use physical adapter; default is mock')
    parser.add_argument('--port', help='Serial device, preferably /dev/serial/by-id/...')
    parser.add_argument('--baud', type=int, default=1000000, help='Host baud rate, not an EEPROM change')
    parser.add_argument('--id', type=int, default=1, help='Current servo ID (default 1)')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('scan', help='Read-only search IDs 1..253 at --baud')
    sub.add_parser('inspect', help='Read configuration and diagnostics without writes')
    sub.add_parser('release', help='Disable torque; support the mechanism first')
    config = sub.add_parser('configure', help='Persist unique ID and position mode; torque must be off')
    config.add_argument('--new-id', required=True, type=int, choices=range(1, 254), metavar='1..253')
    config.add_argument('--single-servo', action='store_true', required=True,
                        help='Acknowledge only one physical servo is connected')
    test = sub.add_parser('test-motion', help='Unloaded out-and-back move, up to 90 degrees; ends torque off')
    test.add_argument('--delta', type=int, required=True,
                      help='Signed movement from current position, +/-1..1024 ticks (1024=90 degrees)')
    test.add_argument('--min', dest='lower', type=int, required=True,
                      help='Lowest permitted absolute encoder position (0..4095); does not reprogram EEPROM')
    test.add_argument('--max', dest='upper', type=int, required=True,
                      help='Highest permitted absolute encoder position (0..4095); does not reprogram EEPROM')
    test.add_argument('--speed', type=int,
                      help=f'Raw speed 1..{MAX_BENCH_SPEED}; default 5 standard / 50 fine units')
    test.add_argument('--acceleration', type=int, default=1, help='Raw SDK acceleration, 1..10 (default 1)')
    test.add_argument('--timeout', type=float, default=8, help='Seconds per leg, >0..30 (default 8)')
    args = parser.parse_args()
    if not 1 <= args.id <= 253 or args.baud <= 0:
        parser.error('Require ID 1..253 and positive baud')
    if args.hardware != bool(args.port):
        parser.error('Physical operation requires both --hardware and --port')
    bus = None
    try:
        bus = (SetupBus(args.port, args.baud, None if args.command == 'scan' else args.id)
               if args.hardware else MockSetupBus(args.id))
        if args.command == 'scan':
            result = dict(found=bus.scan(), baud=args.baud)
            if not result['found']:
                raise RuntimeError('No servo found; check power, wiring and --baud')
        elif args.command == 'inspect':
            result = bus.setup_state()
        elif args.command == 'release':
            bus.release()
            result = bus.read()
            if result['torque'] != 0:
                raise RuntimeError('Torque release readback failed')
        elif args.command == 'configure':
            result = configure(bus, args.new_id)
        else:
            result = motion_test(bus, args.delta, args.lower, args.upper,
                                 args.speed, args.acceleration, args.timeout)
        print(json.dumps(dict(backend='hardware' if args.hardware else 'mock',
                              command=args.command, **result), indent=2))
    except (Exception, KeyboardInterrupt) as exc:
        parser.exit(1, error_text(exc) + '\n')
    finally:
        if bus is not None:
            bus.close()


if __name__ == '__main__':
    main()
