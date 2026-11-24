# Live TCP speed display validation

November 24, 2026. See [readout semantics](../../../cartesian_control.md#live-tcp-speed-readout).
All live motion below used isolated mocks or a generated PTY emulator; physical
motors were never attached or moved.

| Test stack | Moving current peak | Moving reference peak | Final hold |
| --- | --- | --- | --- |
| Mock | 16.117 mm/s | 16.044 mm/s | Both approximately zero |
| Real driver / PTY | 6.042 mm/s | 5.483 mm/s | Both approximately zero |

Both live tests plan and execute an existing safe pose transit and check positive
readings during motion, followed by near-zero readings after completed hold.
These are not the rejected tabletop paths. They use the position-only fallback
(`encoder_position_window`), validating the real driver data shape. Different
reference peaks reflect different selected fixture limits/timing; equality of
filtered encoder and instantaneous reference speeds is not asserted.

The current estimate fits encoder joint positions over 0.2 seconds and evaluates
the shared translational Jacobian at the current pose. The reference uses JTC
reference positions and velocities, independently of encoder history. Updating
the display does not write servo registers, alter controller commands, plan or
execute. No shared safety/hold implementation changed.

Three unit tests check name reordering, translation versus rotation, independent
reference values, position-only slope fitting, warmup, stale/missing/invalid
data, timestamp restart and absent model handling. Seven panel tests pass,
including conversion to mm/s and unavailable labels rather than stale numbers.
Five existing real-driver/PTY regressions pass: execute, scene invalidation,
powered Stop/recovery, action cancellation and thermal-fault recovery.

Reports and compressed launch logs are in `mock/` and `pty/`; PTY includes its
emulator report. Top-level files preserve unit/panel/live/safety test output,
build summary and source hashes. The panel screenshot uses an illustrative
status sample to show formatting; it is not a physical measurement.

Reproduce after sourcing ROS and the workspace:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/test_tcp_telemetry.py
QT_QPA_PLATFORM=offscreen python3 tools/test_pose_panel.py
PINCHY_TEST_DOMAIN=232 python3 tools/test_tcp_telemetry_system.py
PINCHY_TCP_TEST_EMULATED=1 PINCHY_TEST_DOMAIN=231 \
  python3 tools/test_tcp_telemetry_system.py
```

The system helper creates only its own isolated stack and generated PTY; it
accepts no physical serial-port argument. No physical tracking-speed accuracy
or contact drawing claim follows from these software tests.
