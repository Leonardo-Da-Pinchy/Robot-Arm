#!/usr/bin/env python3
"""Single-servo STS bench commands; mock by default, no ROS side effects."""
import argparse
import json
import time
from servo_diagnostics import PortLock, packet_handler

MAX_BENCH_SPEED = 1000


def error_text(exc):
    """Include motion-phase notes even when the CLI suppresses tracebacks."""
    return '\n'.join([f'{type(exc).__name__}: {exc}', *getattr(exc, '__notes__', [])])


class MockBus:
    def __init__(self):
        self.position = 2048
        self.torque = 1

    def read(self):
        return dict(position=self.position, torque=self.torque, mode=0,
                    voltage_raw=120, temperature_raw=25)

    def inspect(self):
        return dict(self.read(), configuration='Mock bus; no physical settings')

    def move(self, position, speed, acceleration):
        self.position = position

    def release(self):
        self.torque = 0

    def close(self):
        pass


class FeetechBus:
    """Adapter for the manufacturer's STS/SMS Python SDK."""
    def __init__(self, port, baud, servo_id):
        from scservo_sdk import PortHandler
        self.port = PortHandler(port)
        self.servo_id = servo_id
        self.sdk = packet_handler(self.port)
        self.port_lock = PortLock(port)
        try:
            if not self.port.openPort() or not self.port.setBaudRate(baud):
                raise RuntimeError('Could not open serial port at requested baud rate')
            if servo_id is not None:
                self.check(self.sdk.ping(servo_id)[1:])
        except Exception:
            try:
                if self.port.is_open:
                    self.port.closePort()
            finally:
                self.port_lock.close()
            raise

    def check(self, result):
        comm, error = result
        if comm != 0:
            detail = getattr(self.sdk, 'last_reply_problem', None)
            raise RuntimeError(self.sdk.getTxRxResult(comm) + (f': {detail}' if detail else ''))
        if error:
            raise RuntimeError(self.sdk.getRxPacketError(error))

    def byte(self, address):
        value, comm, error = self.sdk.read1ByteTxRx(self.servo_id, address)
        self.check((comm, error))
        return value

    def word(self, address):
        value, comm, error = self.sdk.read2ByteTxRx(self.servo_id, address)
        self.check((comm, error))
        return value

    def read(self):
        position, comm, error = self.sdk.ReadPos(self.servo_id)
        self.check((comm, error))
        return dict(position=position, torque=self.byte(40), mode=self.byte(33),
                    voltage_raw=self.byte(62), temperature_raw=self.byte(63))

    def inspect(self):
        """Read raw configuration and feedback without register writes."""
        return dict(self.read(), model_raw=self.word(3), stored_id=self.byte(5),
                    baud_code=self.byte(6), min_position_raw=self.word(9),
                    max_position_raw=self.word(11), offset_raw=self.word(31),
                    goal_position_raw=self.word(42), eeprom_lock=self.byte(55))

    def move(self, position, speed, acceleration):
        self.check(self.sdk.WritePosEx(self.servo_id, position, speed, acceleration))

    def release(self):
        self.check(self.sdk.write1ByteTxRx(self.servo_id, 40, 0))

    def close(self):
        try:
            self.port.closePort()
        finally:
            self.port_lock.close()


def move_relative(bus, delta, lower, upper, speed, acceleration, timeout):
    # These limits deliberately restrict this tool to small bench movements.
    if not (-64 <= delta <= 64) or delta == 0:
        raise ValueError('delta must be nonzero and within -64..64 encoder ticks')
    target = bus.read()['position'] + delta
    return move_to_position(bus, target, lower, upper, speed, acceleration, timeout)


def move_to_position(bus, target, lower, upper, speed, acceleration, timeout):
    """Follow a bounded absolute encoder target and check measured feedback."""
    if not (0 <= lower < upper <= 4095):
        raise ValueError('limits must satisfy 0 <= min < max <= 4095')
    if not (1 <= speed <= MAX_BENCH_SPEED and 1 <= acceleration <= 10 and 0 < timeout <= 30):
        raise ValueError(f'speed: 1..{MAX_BENCH_SPEED}, acceleration: 1..10, timeout: (0, 30]')
    state = bus.read()
    if state['mode'] != 0 or state['torque'] != 1:
        raise ValueError('Requires position mode and torque already enabled')
    start = state['position']
    if not (lower <= start <= upper and lower <= target <= upper):
        raise ValueError('Current or target position is outside supplied bench limits')
    try:
        bus.move(target, speed, acceleration)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = bus.read()
            if state['mode'] != 0 or state['torque'] != 1:
                raise RuntimeError('Servo mode or torque state changed during movement')
            if not lower <= state['position'] <= upper:
                raise RuntimeError('Feedback exceeded supplied bench limits')
            if abs(state['position'] - target) <= 2:
                return state
            time.sleep(0.05)
        raise TimeoutError(f"Target not reached before timeout: target={target}, "
                           f"last_position={state['position']}, "
                           f"error={state['position'] - target} ticks (allowed +/-2)")
    except BaseException:
        # Best effort only: communication loss can prevent a hold command.
        try:
            state = bus.read()
            if state['mode'] == 0 and state['torque'] == 1:
                bus.move(state['position'], speed, acceleration)
        except Exception:
            pass
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', help='Use a real bus')
    parser.add_argument('--port', help='Explicit serial device, preferably /dev/serial/by-id/...')
    parser.add_argument('--baud', type=int, default=1000000)
    parser.add_argument('--id', type=int, default=1)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('read', help='Read only; changes no servo settings')
    commands.add_parser('inspect', help='Read configuration and feedback; no setting changes')
    move = commands.add_parser('move', help='Small relative movement in raw encoder ticks')
    move.add_argument('--delta', type=int, required=True)
    move.add_argument('--min', dest='lower', type=int, required=True)
    move.add_argument('--max', dest='upper', type=int, required=True)
    move.add_argument('--speed', type=int, default=30,
                      help=f'Raw SDK speed 1..{MAX_BENCH_SPEED}, default 30')
    move.add_argument('--acceleration', type=int, default=5, help='Raw SDK acceleration value')
    move.add_argument('--timeout', type=float, default=5)
    commands.add_parser('hold', help='Set goal to measured position; torque must be on')
    commands.add_parser('release', help='Disable torque; support the mechanism first')
    args = parser.parse_args()
    if not 1 <= args.id <= 253 or args.baud <= 0:
        parser.error('Requires one unicast ID in 1..253 and a positive baud rate')
    if args.hardware and not args.port:
        parser.error('--hardware requires --port')
    if args.port and not args.hardware:
        parser.error('--port requires --hardware')
    bus = None
    try:
        bus = FeetechBus(args.port, args.baud, args.id) if args.hardware else MockBus()
        if args.command == 'move':
            state = move_relative(bus, args.delta, args.lower, args.upper,
                                  args.speed, args.acceleration, args.timeout)
        elif args.command == 'inspect':
            state = bus.inspect()
        elif args.command == 'hold':
            state = bus.read()
            if state['mode'] != 0 or state['torque'] != 1:
                raise ValueError('Hold requires position mode with torque enabled')
            bus.move(state['position'], 30, 5)
            state = bus.read()
        elif args.command == 'release':
            bus.release()
            state = bus.read()
        else:
            state = bus.read()
        print(json.dumps(dict(backend='hardware' if args.hardware else 'mock',
                              servo_id=args.id, **state), indent=2))
    except (Exception, KeyboardInterrupt) as exc:
        parser.exit(1, error_text(exc) + '\n')
    finally:
        if bus is not None:
            bus.close()


if __name__ == '__main__':
    main()
