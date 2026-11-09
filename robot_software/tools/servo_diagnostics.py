"""Serial reply checks, single-process port ownership, and bench telemetry."""
import fcntl
import json
import math
import os
import sys
import time
import uuid
from datetime import datetime, timezone


class PortLock:
    """Coordinate ownership among these tools without resetting an active port."""
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(self.fd)
            self.fd = None
            raise RuntimeError('Serial port busy. Stop other servo tools; use --watch-temps in the motion command.')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def read_reply_problem(request, reply):
    """FT-SCS READ response must match ID and requested byte count.

    Wire fields: ID=2, LENGTH=3, INSTRUCTION=4, parameters start at 5.
    READ parameters are address and byte count. Reply LENGTH is data bytes + 2.
    The manufacturer's SDK already checks framing/checksum but its read wrapper
    does not enforce the requested response length.
    """
    if len(request) < 7 or request[4] != 2:
        return None
    expected = request[6] + 2
    if reply is None or len(reply) < 6:
        return 'Missing or short read reply'
    if reply[2] != request[2]:
        return f'Reply ID {reply[2]} does not match requested ID {request[2]}'
    if reply[3] != expected or len(reply) != expected + 4:
        return (f'Register {request[5]}: expected {request[6]} data bytes '
                f'(reply LENGTH={expected}), got LENGTH={reply[3]}, frame bytes={len(reply)}')
    return None


class CheckedReplies:
    """Mixin wrapping the SDK transaction; no external SDK files are modified."""
    trace = None
    last_reply_problem = None
    read_retries = 0 # Opt-in; never replay writes or movement commands.

    def recover_interrupted_transaction(self):
        # The SDK normally clears this in rxPacket, but Ctrl+C can skip that.
        self.portHandler.is_using = False
        try:
            self.portHandler.clearPort()
        except Exception:
            pass

    def txRxPacket(self, packet):
        retries = min(2, max(0, self.read_retries)) if len(packet) >= 7 and packet[4] == 2 else 0
        for attempt in range(retries+1):
            reply, result, error = self._checked_transaction(list(packet), attempt+1)
            if attempt == retries or result == 0 or error:
                return reply, result, error
            from scservo_sdk import COMM_RX_CORRUPT, COMM_RX_TIMEOUT
            if result not in (COMM_RX_CORRUPT, COMM_RX_TIMEOUT):
                return reply, result, error
            time.sleep(0.01)
            self.recover_interrupted_transaction()

    def _checked_transaction(self, packet, attempt):
        request = list(packet)
        try:
            reply, result, error = super().txRxPacket(packet)
        except BaseException:
            self.recover_interrupted_transaction()
            raise
        self.last_reply_problem = None
        if result == 0:
            self.last_reply_problem = read_reply_problem(request, reply)
            if self.last_reply_problem:
                from scservo_sdk import COMM_RX_CORRUPT
                result = COMM_RX_CORRUPT
        if self.trace is not None:
            self.trace(dict(servo_id=request[2], instruction=request[4],
                            address=request[5] if len(request) > 6 and request[4] in (2, 3) else None,
                            attempt=attempt,
                            requested_bytes=request[6] if request[4] == 2 else None,
                            tx_hex=bytes(packet).hex(' '),
                            rx_hex=bytes(reply).hex(' ') if reply is not None else None,
                            communication_result=result, servo_error=error,
                            reply_problem=self.last_reply_problem))
        return reply, result, error

    def syncWriteTxOnly(self, *args, **kwargs):
        try:
            return super().syncWriteTxOnly(*args, **kwargs)
        except BaseException:
            self.recover_interrupted_transaction()
            raise


def packet_handler(port):
    from scservo_sdk.sms_sts import sms_sts

    class CheckedSTS(CheckedReplies, sms_sts):
        pass

    return CheckedSTS(port)


def temperature_spike_unconfirmed(state):
    """A high sample is transient only with two agreeing cool rechecks and no alarms."""
    if state.get('temperature_raw', 0) < 60:
        return False
    if state.get('status_raw') != 0 or state.get('status_recheck_raw') != 0:
        return False
    direct = state.get('temperature_recheck_raw')
    block = state.get('temperature_block_raw')
    return (isinstance(direct, int) and isinstance(block, int)
            and 0 <= direct < 60 and 0 <= block < 60 and abs(direct - block) <= 2)


class Telemetry:
    def __init__(self, path=None, interval=0.25, stream=sys.stderr):
        if not math.isfinite(interval) or not 0.05 <= interval <= 10:
            raise ValueError('--interval must be 0.05..10 seconds')
        self.interval, self.stream = interval, stream
        self.started = time.monotonic()
        self.last_display = float('-inf')
        self.session = uuid.uuid4().hex[:12]
        self.file = open(path, 'a', encoding='utf-8', buffering=1) if path else None
        self.event('session', interval_s=interval)

    def event(self, kind, **fields):
        record = dict(type=kind, utc=datetime.now(timezone.utc).isoformat(),
                      elapsed_s=round(time.monotonic() - self.started, 6),
                      session=self.session, **fields)
        if self.file:
            self.file.write(json.dumps(record) + '\n')
        return record

    def packet(self, packet, phase):
        self.event('packet', phase=phase, **packet)

    def sample(self, states, phase):
        motors = {str(i): dict(state, temperature_C=state['temperature_raw'],
                              voltage_V=state['voltage_raw']/10) for i, state in states.items()}
        record = self.event('sample', phase=phase, motors=motors)
        flagged = any(s['temperature_raw'] >= 60 or s['status_raw'] for s in states.values())
        now = time.monotonic()
        if self.stream and (flagged or now - self.last_display >= self.interval):
            readings = []
            for i, state in states.items():
                text = (f"ID {i}: {state['temperature_raw']} C, {state['voltage_raw']/10:.1f} V, "
                        f"status=0x{state['status_raw']:02x}")
                if 'temperature_recheck_raw' in state:
                    text += (f", recheck={state['temperature_recheck_raw']} C, "
                             f"block={state['temperature_block_raw']} C")
                    if temperature_spike_unconfirmed(state):
                        text += ', isolated spike unconfirmed'
                readings.append(text)
            print(f"{record['utc']} [{phase}] {' | '.join(readings)}" + (' !!!' if flagged else ''),
                  file=self.stream, flush=True)
            self.last_display = now

    def close(self):
        if self.file:
            try:
                self.event('session_end')
            finally:
                self.file.close()
                self.file = None


def watch_temperatures(bus, ids, interval=0.25, duration=0):
    if not math.isfinite(interval) or not 0.05 <= interval <= 10:
        raise ValueError('--interval must be 0.05..10 seconds')
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('--duration must be >=0 seconds (0 means until Ctrl+C)')
    bus.phase = 'watch'
    started, samples = time.monotonic(), 0
    while True:
        bus.read(ids)  # Observer displays/logs the actual values; no health filter.
        samples += 1
        elapsed = time.monotonic() - started
        if duration and elapsed >= duration:
            return dict(samples=samples, duration_s=round(elapsed, 3))
        time.sleep(min(interval, max(0, duration - elapsed)) if duration else interval)
