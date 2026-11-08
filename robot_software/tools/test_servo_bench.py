"""Command guards and failure handling without connected hardware."""
import unittest
from unittest.mock import patch

from servo_bench import FeetechBus, MockBus, move_relative


class RecordingBus(MockBus):
    def __init__(self):
        super().__init__()
        self.commands = []

    def move(self, position, speed, acceleration):
        self.commands.append(position)


class BenchTests(unittest.TestCase):
    def test_inspect_uses_only_reads(self):
        class ReadOnlySDK:
            # No write methods: inspection must succeed without them.
            def read1ByteTxRx(self, servo_id, address):
                return {5: 1, 6: 0, 33: 0, 40: 0, 55: 1,
                        62: 51, 63: 25}[address], 0, 0

            def read2ByteTxRx(self, servo_id, address):
                return {3: 777, 9: 0, 11: 4095, 31: 0, 42: 2048}[address], 0, 0

            def ReadPos(self, servo_id):
                return 4092, 0, 0

        bus = FeetechBus.__new__(FeetechBus)
        bus.servo_id, bus.sdk = 1, ReadOnlySDK()
        state = bus.inspect()
        self.assertEqual(state['stored_id'], 1)
        self.assertEqual(state['baud_code'], 0)
        self.assertEqual(state['position'], 4092)
        self.assertEqual(state['voltage_raw'], 51)
        self.assertEqual(state['torque'], 0)
        self.assertEqual(state['goal_position_raw'], 2048)

    def test_inspect_propagates_register_errors(self):
        from unittest.mock import Mock
        bus = FeetechBus.__new__(FeetechBus)
        bus.servo_id = 1
        bus.sdk = Mock()
        bus.sdk.ReadPos.return_value = (0, -1, 0)
        bus.sdk.getTxRxResult.return_value = 'Communication failed'
        with self.assertRaisesRegex(RuntimeError, 'Communication failed'):
            bus.inspect()
        bus.sdk.read1ByteTxRx.assert_not_called()

    def test_invalid_targets_do_not_write(self):
        for delta, lower, upper in [(65, 2000, 2100), (20, 2000, 2050),
                                     (20, 2050, 2100)]:
            bus = RecordingBus()
            with self.assertRaises(ValueError):
                move_relative(bus, delta, lower, upper, 30, 5, 1)
            self.assertEqual(bus.commands, [])

    def test_disabled_torque_does_not_write(self):
        bus = RecordingBus()
        bus.torque = 0
        with self.assertRaises(ValueError):
            move_relative(bus, 20, 2000, 2100, 30, 5, 1)
        self.assertEqual(bus.commands, [])

    def test_speed_above_1000_does_not_write(self):
        bus = RecordingBus()
        with self.assertRaises(ValueError):
            move_relative(bus, 20, 2000, 2100, 1001, 5, 1)
        self.assertEqual(bus.commands, [])

    def test_timeout_attempts_hold_at_measured_position(self):
        bus = RecordingBus()
        with patch('servo_bench.time.monotonic', side_effect=[0, 2]):
            with self.assertRaisesRegex(TimeoutError, 'target=2068, last_position=2048, error=-20'):
                move_relative(bus, 20, 2000, 2100, 30, 5, 1)
        self.assertEqual(bus.commands, [2068, 2048])


if __name__ == '__main__':
    unittest.main()
