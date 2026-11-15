# Saved point sets and fixed-TCP rotation validation

All execution below used an isolated mock or the real C++ adapter attached to a
PTY servo emulator. No physical motor goal was sent, no live hardware stack was
restarted, and calibration, limits, active TCP and powered safety behavior were
preserved. The live session log was read only to recover the owner's points.

## Results

- Six new file/panel tests pass: geometry/settings round trip, removal of plan
  tokens, invalid-write preservation, frame/unit/mode/rate rejection, recovered
  seven-point data, staging-only loading, capture from feedback rather than
  goal fields, published queue content and invalid-load preservation.
- The final [focused suite](unit_tests.log) passes **38 tests**: 17 path/math,
  six point-set, four panel and 11 feasibility/routing/cancellation cases.
- Fixed-TCP example plan/execute passes on both mock and PTY adapter. Seven
  equal-XYZ poses tilt ±3° about local X/Y, with a separately planned initial
  approach. Independent MoveIt FK confirms maximum sampled modeled position
  error about **0.0656 mm**, with a maximum orientation change **3°**. The gateway
  also checks dense actual JTC interpolation against its 1 mm position bound,
  full-pose SLERP geometry, joint rates/bounds and collisions. Differing-XYZ
  fixed-TCP requests are rejected without a plan token.
- Save/Load dialog round trip, read-only queue export, GUI PlanPath preview and
  CLI file replanning pass on both mock and PTY adapter, with no movement during
  loading or preview. Absolute saved joint caps are retained.
- Three [existing workflow cases](existing_workflows.log) pass: full Cartesian
  preview/execution, strict blocked-straight partial rejection, and queued GUI/CLI.
- Build of pinchy_motion_interfaces and pinchy_motion, Python compilation and
  `git diff --check` pass.

[Mock fixed-TCP record](mock_fixed_tcp.json), [PTY fixed-TCP record](pty_fixed_tcp.json),
[file round trip](roundtrip_points.json), [queue export](published_queue.json),
and [PTY CLI preview](pty_cli_preview.json) retain the detailed results.

An initial rotation test exposed progress estimation based on micrometre FK
translation instead of the substantial orientation change. Cartesian progress
now uses both translation and rotation, scaled by the unchanged 1 mm/0.01 rad
validation tolerances. A regression accepts tiny positional noise during a
rotation but still rejects 2 mm drift. This is not a geometry-tolerance increase.

Model drift is not physical accuracy. The current TCP is an unmeasured proxy;
tracking, installed pen geometry, contact and final fixtures need operator
qualification. These examples are pen-up commissioning sequences, not drawing
strokes, approved physical placements or the SVG/HTTPS job executor.

## Reproduction

From robot_software with ROS/workspace sourced:

```bash
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen python3 -m unittest tools.test_paths tools.test_point_sets tools.test_pose_panel tools.test_feasibility
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_CALIBRATED_MOCK=1 PINCHY_TEST_SAVED_LIMITS=1 python3 tools/test_cartesian.py MockCartesian.test_01q_saved_fixed_tcp_rotation_sequence HardwareCartesian.test_01q_saved_fixed_tcp_rotation_sequence
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_CALIBRATED_MOCK=1 PINCHY_TEST_SAVED_LIMITS=1 python3 tools/test_cartesian.py MockCartesian.test_01r_point_set_files_gui_and_queue_cli HardwareCartesian.test_01r_point_set_files_gui_and_queue_cli
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_CALIBRATED_MOCK=1 python3 tools/test_cartesian.py MockCartesian.test_01c_straight_lerp_slerp_path_preview_and_execute MockCartesian.test_01d_blocked_straight_path_never_retains_partial_plan MockCartesian.test_01i_queued_path_gui_and_cli
```

See [point-set usage](../../../point_sets.md) for files, controls and the live
read-only queue topic/CLI. Reopen the updated planning server/panel together to
load fixed_tcp support; do not restart hardware automatically.
