# Whole-stroke planner follow-up validation

## Exact historical broad arcs

The [report](report.json) compares local continuation, fixed-start whole-stroke
optimization and explicit starting-configuration selection. All cases use saved
position bounds and shared default 30 deg/s, 180 deg/s² joint caps. The requested
TCP speed is explicitly 3 mm/s and acceleration 20 mm/s². This is not an automatic
retiming of the old 10 mm/s request.

- X=0.40 m, Z=0.20 m, R=40 mm, angles −60° to +60°: old fixed-start continuation
  still fails at 52.9%. Alternative start plus whole-stroke optimization passes;
  modeled steady contact speed is 2.996–3.004 mm/s. Independent MoveIt FK,
  collision checks and an OMPL pen-up route from the original start all pass.
- X=0.35 m, Z=0.15 m, R=40 mm, angles −60° to +60°: old local trace fails
  wrist-flex interpolation bounds. Joint optimization passes from the fixed
  original start at 2.990–3.008 mm/s. The explicit start-search mode also passes
  at 2.989–3.009 mm/s with a checked approach.
- The narrow −45° to −10° control passes all three methods.

The historical tool comparison used model maximum joint rates. The updated
comparison explicitly caps rates at shared defaults and records the limits.
Keep the older reports unchanged as evidence of the original failures.

## Scope

The collision scene is the shared robot and example floor, not the final
objects, fixtures, tabletop or base wings. The numerical ramps lie on the
hypothetical arc; no physical pen-up/contact geometry is qualified. The tool
never creates an execution client or opens the servo port. Pen-up approach
planning requests are previews on an isolated mock. Full front-180° coverage,
physical speed/contact/accuracy and arbitrary artwork remain unqualified.

The [pen length comparison](pen_length_comparison.json) changes only an axial
TCP offset in memory, not the active robot model. All ten sampled tasks solved
for 100/125/149 mm offsets; nine solved for 175 mm. This finite endpoint-only
comparison is not collision or whole-stroke qualification and does not select
a physical pen length.

## Reproduction

From robot_software with ROS/workspace sourced:

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/check_stroke_selection.py --moveit --tcp-speed .003 --tcp-acceleration .02 --budget 120 --output-dir /tmp/stroke-start-checked-3mm
OPENBLAS_NUM_THREADS=1 python3 tools/test_strokes.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_paths.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_feasibility.py
OPENBLAS_NUM_THREADS=1 python3 tools/test_ik.py
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen python3 tools/test_pose_panel.py
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_CALIBRATED_MOCK=1 python3 tools/test_cartesian.py MockCartesian.test_01n_whole_stroke_setup_with_contact_ramps MockCartesian.test_01h_ordered_free_twist_path_and_contact_timing MockCartesian.test_01i_queued_path_gui_and_cli MockCartesian.test_01o_broad_stroke_start_search_previews
OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PINCHY_TEST_SAVED_LIMITS=1 python3 tools/test_cartesian.py HardwareCartesian.test_01n_whole_stroke_setup_with_contact_ramps HardwareCartesian.test_01h_ordered_free_twist_path_and_contact_timing HardwareCartesian.test_01j_free_twist_straight_transit
```

HardwareCartesian is the real adapter attached to a PTY emulator, not physical
motors. Its new-options regression verifies physical-mode contact is still
rejected; existing pen-up path execution regressions use that emulator.

Build `pinchy_motion_interfaces` and `pinchy_motion` together before restarting
planning clients/server. No automatic physical-stack restart was performed.


## Final runtime integration and regressions

The [runtime events](runtime_broad_previews.json) record the final full PlanPath
service previews for both broad cases, using 13 sampled spline waypoints, all
stroke segments contact-tagged, explicit contact endpoint ramps, and a checked
pen-up route from the held zero posture. Both previews leave the mock robot
stationary. The plateau ranges are 2.988–3.009 mm/s (X=0.40 m case) and
2.999–3.003 mm/s (X=0.35 m case). This complements the earlier exact-arc assay
in report.json; spline interpolation is an explicitly selected request shape.

Full runtime checks exposed and corrected issues beyond the first numerical
assay: intermediate continuation cache entries starved alternative-start seeds;
unequal requested-knot spacing needed distance-aware derivative penalties; and
natural joint-spline endpoint curvature distorted contact-endpoint TCP speed.
The final code isolates the start-candidate cache, explores nullspace/global
seeds, optimizes derivatives using actual spacing, densely reprojects the task,
and uses not-a-knot joint interpolation. Original strict task, speed, jump,
polynomial bounds and collision checks remain. Initial failed runs were used
to identify these issues, not counted as passes.

[Final focused suite](unit_tests.log): **40 passed** (16 path, two exact broad
stroke, nine feasibility, ten IK, three panel cases). This includes cancellation,
collision/constraint rejection, requested-speed enforcement, ramp-only reporting
and both exact historical arcs under saved/default caps.

The two broad runtime subcases also pass through PlanPath, full validation and
preview token generation; their trajectory durations include the checked
pen-up approaches. Complex previews can take roughly one to two minutes;
this is offline preflight, not a real-time servo-loop planner.


[Final mock/emulator workflow log](workflow_tests.log): **six passed** after the
final interpolation changes. This covers explicit setup/ramp preview and mock
execution, legacy strict contact timing, queued GUI/CLI planning, real-adapter
PTY contact rejection and pen-up path/transit execution. The separate broad
runtime test also passed both subcases (see runtime_broad_previews.json).
Python compilation, the two-package ROS build and `git diff --check` passed.
No physical motor commands, active TCP edits, joint-limit expansions or hardware
restarts occurred. Calibration and the shared driver/hold/thermal behavior were
unchanged.
