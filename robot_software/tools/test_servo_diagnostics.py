"""Validate packet checks and monitoring without physical motors."""
import io
import json
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from servo_diagnostics import (CheckedReplies, PortLock, Telemetry, read_reply_problem,
                               temperature_spike_unconfirmed, watch_temperatures)
from servo_chain import health
from servo_chain import MockChainBus, Motor, run_test


def reply(servo_id=2, data=(35,)):
    frame = [255, 255, servo_id, len(data) + 2, 0, *data]
    return frame + [(~sum(frame[2:])) & 255]


READ_TEMP = [255, 255, 2, 4, 2, 63, 1, 0]


class DiagnosticTests(unittest.TestCase):
    def test_opt_in_read_retries_recover_corrupt_frame_and_validate_next_reply(self):
        class SDK:
            calls = 0
            def txRxPacket(self, packet):
                self.calls += 1
                return (reply(data=()), 0, 0) if self.calls == 1 else (reply(), 0, 0)
        class Handler(CheckedReplies, SDK): pass
        handler = Handler(); handler.read_retries = 2
        handler.portHandler = SimpleNamespace(is_using=False, clearPort=Mock())
        with patch.dict('sys.modules', {'scservo_sdk': SimpleNamespace(COMM_RX_CORRUPT=-7, COMM_RX_TIMEOUT=-6)}):
            self.assertEqual(handler.txRxPacket(READ_TEMP)[1], 0)
        self.assertEqual(handler.calls, 2)
        self.assertIsNone(handler.last_reply_problem)
        handler.portHandler.clearPort.assert_called_once()

    def test_retries_are_bounded_and_never_replay_a_write(self):
        class SDK:
            calls = 0
            def txRxPacket(self, packet):
                self.calls += 1
                return None, -7, 0
        class Handler(CheckedReplies, SDK): pass
        handler = Handler(); handler.read_retries = 2
        handler.portHandler = SimpleNamespace(is_using=False, clearPort=Mock())
        with patch.dict('sys.modules', {'scservo_sdk': SimpleNamespace(COMM_RX_CORRUPT=-7, COMM_RX_TIMEOUT=-6)}):
            self.assertEqual(handler.txRxPacket(READ_TEMP)[1], -7)
        self.assertEqual(handler.calls, 3)
        handler.calls = 0
        write = [255, 255, 2, 4, 3, 40, 1, 0]
        self.assertEqual(handler.txRxPacket(write)[1], -7)
        self.assertEqual(handler.calls, 1)

    def test_interrupted_sdk_read_clears_busy_flag_before_cleanup(self):
        class SDK:
            calls = 0
            def txRxPacket(self, packet):
                if self.portHandler.is_using:
                    raise RuntimeError('Port is in use')
                self.calls += 1
                self.portHandler.is_using = True
                if self.calls == 1:
                    raise KeyboardInterrupt()
                self.portHandler.is_using = False
                return reply(), 0, 0

        class Handler(CheckedReplies, SDK):
            pass

        handler = Handler()
        handler.portHandler = SimpleNamespace(is_using=False, clearPort=Mock())
        with self.assertRaises(KeyboardInterrupt):
            handler.txRxPacket(READ_TEMP.copy())
        self.assertFalse(handler.portHandler.is_using)
        handler.portHandler.clearPort.assert_called_once_with()
        self.assertEqual(handler.txRxPacket(READ_TEMP.copy())[1], 0)

    def test_interrupted_group_write_clears_busy_flag(self):
        class SDK:
            def syncWriteTxOnly(self, *args):
                self.portHandler.is_using = True
                raise KeyboardInterrupt()

        class Handler(CheckedReplies, SDK):
            pass

        handler = Handler()
        handler.portHandler = SimpleNamespace(is_using=False, clearPort=Mock())
        with self.assertRaises(KeyboardInterrupt):
            handler.syncWriteTxOnly(41, 7, [], 0)
        self.assertFalse(handler.portHandler.is_using)

    def test_captured_62_28_28_spike_requires_clean_status_recheck(self):
        state = dict(temperature_raw=62, temperature_recheck_raw=28,
                     temperature_block_raw=28, status_raw=0, status_recheck_raw=0,
                     mode=0, voltage_raw=122)
        self.assertTrue(temperature_spike_unconfirmed(state))
        health(state, 4)
        for update in [dict(temperature_recheck_raw=61),
                       dict(temperature_block_raw=62),
                       dict(temperature_block_raw=40),
                       dict(status_raw=4), dict(status_recheck_raw=4)]:
            changed = dict(state, **update)
            self.assertFalse(temperature_spike_unconfirmed(changed))
            with self.assertRaises(ValueError):
                health(changed, 4)
        del state['status_recheck_raw']
        self.assertFalse(temperature_spike_unconfirmed(state))

    def test_sustained_high_temperature_still_releases_all_motors(self):
        bus = MockChainBus([1, 2])
        original = bus.read
        def hot_after_movement(ids, full=False):
            states = original(ids, full)
            if bus.motors[1].position != 2048:
                states[2].update(temperature_raw=65, temperature_recheck_raw=65,
                                 temperature_block_raw=64, status_recheck_raw=0)
            return states
        bus.read = hot_after_movement
        with self.assertRaisesRegex(ValueError, 'temperature limit reached'):
            run_test(bus, [Motor(1, 1000, 3000), Motor(2, 1000, 3000)])
        self.assertTrue(all(m.torque == 0 for m in bus.motors.values()))

    def test_valid_read_reply_matches_id_and_byte_count(self):
        self.assertIsNone(read_reply_problem(READ_TEMP, reply()))

    def test_checksums_alone_do_not_make_wrong_length_a_temperature(self):
        # These are valid-checksum packets, but neither answers a one-byte read.
        for frame in [reply(data=()), reply(data=(35, 124))]:
            self.assertIn('expected 1 data bytes', read_reply_problem(READ_TEMP, frame))

    def test_wrong_id_and_truncated_reply_are_rejected(self):
        self.assertIn('does not match', read_reply_problem(READ_TEMP, reply(servo_id=1)))
        self.assertIn('short', read_reply_problem(READ_TEMP, [255, 255]))

    def test_sdk_wrapper_rejects_bad_read_and_logs_raw_reply(self):
        class SDK:
            def txRxPacket(self, packet):
                return reply(data=()), 0, 0

        class Handler(CheckedReplies, SDK):
            pass

        handler = Handler()
        events = []
        handler.trace = events.append
        with patch.dict('sys.modules', {'scservo_sdk': SimpleNamespace(COMM_RX_CORRUPT=-7)}):
            frame, comm, error = handler.txRxPacket(READ_TEMP.copy())
        self.assertEqual(comm, -7)
        self.assertIn('expected 1 data bytes', handler.last_reply_problem)
        self.assertEqual(events[0]['address'], 63)
        self.assertEqual(events[0]['rx_hex'], bytes(frame).hex(' '))
        self.assertEqual(events[0]['communication_result'], -7)

    def test_port_lock_rejects_second_owner_then_releases(self):
        with tempfile.NamedTemporaryFile() as file:
            first = PortLock(file.name)
            try:
                with self.assertRaisesRegex(RuntimeError, 'port busy'):
                    PortLock(file.name)
            finally:
                first.close()
            second = PortLock(file.name)
            second.close()

    def test_spike_is_displayed_and_logged_despite_display_throttle(self):
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            path = directory + '/temps.jsonl'
            telemetry = Telemetry(path, interval=10, stream=output)
            normal = dict(temperature_raw=35, voltage_raw=124, status_raw=0)
            spike = dict(normal, temperature_raw=65, temperature_recheck_raw=35,
                         temperature_block_raw=35)
            telemetry.sample({1: normal, 2: normal}, 'startup')
            telemetry.sample({1: normal, 2: spike}, 'mixed/outward')
            telemetry.close()
            with open(path) as file:
                rows = [json.loads(line) for line in file]
        samples = [row for row in rows if row['type'] == 'sample']
        self.assertEqual(samples[-1]['motors']['2']['temperature_C'], 65)
        self.assertEqual(samples[-1]['motors']['2']['temperature_recheck_raw'], 35)
        self.assertEqual(samples[-1]['phase'], 'mixed/outward')
        self.assertIn('utc', samples[-1])
        self.assertIn('65 C', output.getvalue())
        self.assertIn('recheck=35 C', output.getvalue())
        self.assertEqual(normal['temperature_raw'], 35)

    def test_motion_observer_sees_stages_and_final_release(self):
        bus = MockChainBus([1, 2])
        phases = []
        bus.observer = lambda states, phase: phases.append(phase)
        result = run_test(bus, [Motor(1, 1000, 3000), Motor(2, 1000, 3000)])
        self.assertIn('preflight', phases)
        self.assertIn('mixed/outward', phases)
        self.assertIn('mixed/return', phases)
        self.assertEqual(phases[-1], 'release')
        self.assertTrue(all(s['torque'] == 0 for s in result['after'].values()))

    def test_monitor_is_read_only(self):
        bus = MockChainBus([1, 2])
        bus.goals, bus.enable, bus.limit, bus.release = Mock(), Mock(), Mock(), Mock()
        samples = []
        bus.observer = lambda states, phase: samples.append(states)
        result = watch_temperatures(bus, [1, 2], interval=0.05, duration=0.01)
        self.assertGreaterEqual(result['samples'], 1)
        for method in [bus.goals, bus.enable, bus.limit, bus.release]:
            method.assert_not_called()
        self.assertEqual(samples[0][2]['temperature_raw'], 25)

    def test_invalid_poll_intervals_fail_before_read(self):
        bus = SimpleNamespace(read=Mock())
        for interval in [0, -1, float('nan'), 11]:
            with self.assertRaises(ValueError):
                watch_temperatures(bus, [1, 2], interval=interval)
        bus.read.assert_not_called()


if __name__ == '__main__':
    unittest.main()
