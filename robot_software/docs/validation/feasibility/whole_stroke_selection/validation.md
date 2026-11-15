# Whole-stroke selection validation

All checks are numerical or isolated mock ROS. No physical port was opened,
no real motor command was sent, and no hardware restart occurred.

- `tools/test_paths.py`: 12 passed, including early twist choice before a dead end,
  rejection of every invalid complete candidate, exact-mode/collision restrictions,
  cancellation and a real calibrated-model alternate configuration that passes
  shared controller-polynomial/geometry validation.
- `tools/test_ik.py`: 10 passed, including independent FK/Jacobian checks,
  known-reachable poses, collision filtering, explicit orientation modes and stop.
- `tools/test_feasibility.py`: nine passed. Its runtime diagnostic mock now patches
  the new `select_trace` entry point; the exact failed-target diagnostic is retained.
- Mock workflow: ordered contact timing, free-twist straight transit and blocked
  straight partial rejection passed (nominal CAD bounds).
- Mock known-reachable free-twist workflow: eight target previews passed under
  nominal CAD bounds, with preview causing no movement.
- Operator nearby-XYZ/changed-direction workflow passed using saved calibrated
  mock bounds. The initial replay failed because its rounded copied XYZ was within
  micrometres of the current tip and the trace correctly rejected a duplicate.
  The fixture now asserts that rejection, then checks a genuine 2 mm translation
  using the explicitly copied direction. Runtime duplicate policy is unchanged.
- Python compilation and `git diff --check` passed.

Reproduction from `robot_software/`, after sourcing ROS and install/setup.bash:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/test_paths.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_ik.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_feasibility.py
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen python3 tools/test_cartesian.py MockCartesian.test_01h_ordered_free_twist_path_and_contact_timing MockCartesian.test_01j_free_twist_straight_transit MockCartesian.test_01d_blocked_straight_path_never_retains_partial_plan MockCartesian.test_01e_free_twist_known_reachable_directions_inside_working_region
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_CALIBRATED_MOCK=1 python3 tools/test_cartesian.py MockCartesian.test_01m_operator_nearby_xyz_with_changed_direction
OPENBLAS_NUM_THREADS=1 python3 tools/check_stroke_selection.py --moveit --output-dir /tmp/stroke-selection-checked
```

[Comparison report](report.json) retains successful narrow-path states and
speed checks, as well as both broad failures. The fixed-start continuation case
still stops at 52.9%; the other broad case still exceeds wrist-flex interpolation
bounds. No full drawing coverage, robust alternate starting-posture selection,
collision-planned approach/repositioning or physical contact qualification is
established. Keep the roadmap feasibility gate open.
