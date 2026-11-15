# MoveIt / Cartesian integration handoff

Updated November 14, 2026. Read the [canonical roadmap](implementation_roadmap.md)
for task status and [Cartesian control guide](cartesian_control.md) for commands.
This document maps the implementation and remaining physical checks. The older
[bringup handoff](checkpoint_handoff.md) still describes the hardware/control
foundation; it is not a second current task list.

## Delivered scope

- Separate `pinchy_moveit_config`, `pinchy_motion_interfaces`, and `pinchy_motion`
  packages on ROS 2 Jazzy, using installed MoveIt 2/KDL/OMPL/FCL.
- `arm` group from `base_link` to the approved `pointer_tip` proxy. Explicit exact-RPY, pen-direction/free-twist and position-only endpoint
  constraints, with native KDL plus a bounded multi-start fallback. Exact RPY
  never falls back to a relaxed mode silently. Every candidate is
  checked against calibrated bounds and MoveIt collision validity. The bounded numerical candidate generator is filtered through the same
  full MoveIt state checker before it can become a plan.
- Plan/preview is separate from Execute. Plans are held by the gateway, expire
  after 60 seconds, are consumed once and are bound to their start/model/scene.
- A changed live model blocks further planning/execution until the planning
  layer is restarted, preventing stale MoveIt geometry from silently mixing
  with a new bringup description. No automatic hardware restart occurs.
- Timed path validation matches JTC interpolation; velocities/accelerations and
  polynomial extrema are checked before dispatch. Uniform slowing preserves
  geometry. The gateway never edits interpolation fields after validation.
- CLI and Qt XYZ/RPY panel, synchronized draggable RViz target, live target axes,
  optional numerical input bounds/FK debug samples, collision overlay and trajectory preview. The
  current robot model continues to show measured state. Shared camera/view/grid,
  TF/model settings come from the existing RViz file rather than a second copy.
- MoveIt direct move/execute capabilities are disabled. Checked execution uses
  the existing `/arm_controller/follow_joint_trajectory` action through the
  gateway, with Stop and recovery retaining powered holds.
- Physical-mode entry reuses the same guided startup and zero-hold transfer.
  The new layer never opens the servo port and never rewrites calibration or
  EEPROM. The old joint-slider workflow is retained.

## Implementation map

| File | Purpose |
| --- | --- |
| [SRDF](../src/pinchy_moveit_config/config/arm.srdf) | Six-axis group, zero reference, six adjacent mechanical collision exclusions only. |
| [Kinematics](../src/pinchy_moveit_config/config/kinematics.yaml) | Full-pose KDL setup; numerical epsilon 1e-4, bounded solver iterations/time. This is model solver convergence, not physical accuracy. |
| [OMPL](../src/pinchy_moveit_config/config/ompl_planning.yaml) | RRTConnect, bounded resolution, start validation and time-optimal/solution adapters. |
| [Controller mapping](../src/pinchy_moveit_config/config/moveit_controllers.yaml) | Existing six-joint action mapping; no new controller. |
| [Example scene](../src/pinchy_moveit_config/config/scene.yaml) | One example floor box; final workspace/fixture/base-wing geometry is pending. |
| [Drawing policy](../src/pinchy_moveit_config/config/drawing_motion_policy.yaml) | Constant contact TCP speed and pen-up acceleration/deceleration requirement; contact implementation/qualified speeds remain absent. |
| [Planner bootstrap](../src/pinchy_moveit_config/scripts/planning_stack.py) | Reads the live calibrated model, assembles parameters, starts MoveIt and derives RViz config from the existing layout. It is not a hardware owner. |
| [Mock launch](../src/pinchy_moveit_config/launch/mock_motion.launch.py), [attach launch](../src/pinchy_moveit_config/launch/planning.launch.py) | Complete mock workflow, or planning/application attached to existing matching bringup. |
| [Motion gateway](../src/pinchy_motion/pinchy_motion/server.py) | Pose validation/frame conversion, IK, plan tokens, scene/path/start checks, preview, execution, progress and failure handling. |
| [Live target editor](../src/pinchy_motion/pinchy_motion/target_editor.py), [workspace](../src/pinchy_motion/pinchy_motion/workspace.py), [bounds](../src/pinchy_motion/config/workspace.json) | Visualization-only pose editing, preview invalidation, numerical input guard and approximate FK samples. No planning or motor command on edits. |
| [Geometry helpers](../src/pinchy_motion/pinchy_motion/geometry.py) | RPY convention, model FK for display, exact JTC polynomial checks and uniform retiming. |
| [CLI](../src/pinchy_motion/pinchy_motion/cli.py), [panel](../src/pinchy_motion/pinchy_motion/panel.py) | Separate Plan/Execute, current pose, Stop and explicit Recover. |
| [Plan service](../src/pinchy_motion_interfaces/srv/PlanPose.srv), [execute action](../src/pinchy_motion_interfaces/action/ExecutePlan.action) | Typed application contracts for future web/drawing callers. |
| [Shared control contract](../src/pinchy_bringup/pinchy_bringup/control_contract.py) | Joint order, endpoints, model/profile speed interpretation, quintic timing and constant hold-goal builder shared with sliders. |
| [Powered hold protocol](../src/pinchy_bringup/pinchy_bringup/powered_hold.py) | Pause acknowledgment → fresh stamped accepted target → replacement constant goal → resume only that hold. Explicit recovery delegates to backend health checks. Late callbacks cannot resume an expired request. |
| [Existing startup](../tools/start_arm.py) | Optional `--motion` selects the planning application after the shared guided startup; `start_motion` is a thin ROS entry wrapper. |

## Preserved contracts

Use one robot description from bringup, one active tool profile, one calibration
mapping, one controller configuration and one hardware writer. A narrower live
profile narrows planner/UI limits; no MoveIt file can silently widen it. Joint
speed and acceleration defaults are 30 deg/s and 180 deg/s² where the profile
allows them; zeroing uses its separate bounded ramp/settings and the shared startup settling
policy described in the bringup handoff.

Keep motor faults in `pinchy_hardware`: finite command-rate excesses are limited
without latching; temperature rechecks and four-suspect confirmation remain;
real alarms, invalid positions, bad power/packets, stale feedback or long control
stalls still hold. Do not move ROS publication back into the motor write thread.
Do not change torque on planning/IK rejection. Runtime recovery requires the
operator's explicit request and the existing backend's four healthy sweeps.

The powered-hold gateway does not use ordinary action cancellation as the motor
stop, because that could jump to lagging measured positions. It installs the
same constant-target hold goal used by the sliders after pausing. Original
application cancellation returns a canceled action result after the hold
handshake, including cancellation before controller dispatch. Recovery/Stop
invalidates old previews; neither silently continues an interrupted job.

Planning errors leave hardware bringup independent. Explicit shutdown of the
physical startup process still stops its children and can release torque, just
as before. SIGKILL, firmware protection, loss of power and an unavailable bus
cannot be made safe by a software hold guarantee. If the motion gateway is
forcibly killed, the controller may finish an already submitted checked
trajectory; normal signal handling requests powered hold. A separate process-
loss watchdog is not yet implemented and belongs in final fault qualification.

## Drawing requirement carried forward

Contact strokes must maintain approximately constant **TCP surface speed**, in
mm/s. Acceleration/deceleration should occur while pen-up/off the object wherever
feasible: reach marking speed before contact and lift before terminal braking.
Do not stop at interior samples. Preserve intended corners or use a lift /
explicitly approved blend or slowdown; never silently trade shape for speed.
Safety stops are immediate exceptions. Actual speed, minimum useful speed and
variation are TBD from physical measurement.

Current commands are free-space pose transits, not straight Cartesian lines or
contact strokes. MoveIt joint timing alone does not implement constant TCP
speed. `contact_motion_supported` is false. The next continuous-path work must
use the shared policy and demonstrate measured speed/path accuracy together.

## IK reliability and line-transit update

Read [the investigation](ik_investigation.md) for benchmark evidence, model
conditioning, saved wrist limits, checked operator examples and geometric reach
bounds. New files: `pinchy_motion/ik.py`, `tools/test_ik.py`, and
`tools/benchmark_ik.py`. `PlanPose` adds `orientation_mode`, `path_mode` and a
`resolved_target` response. Empty orientation mode selects pen direction/free twist;
request `full` for all RPY angles or `position` for explicit XYZ-only operation.
Restart the planning server/clients together after this interface update.

The panel offers three explicit endpoint constraints and a Straight TCP path
(LERP + SLERP) checkbox. CLI equivalents are `--orientation` and `--path`.
MoveIt's Cartesian service must return a complete path; partial and jumping
paths are rejected. The actual timed JTC interpolation is also checked against
the requested line/rotation. This remains single-segment free-space transit;
continuous contact strokes and their TCP speed qualification remain step 11.

`mock_motion.launch.py calibration:=/absolute/path/to/arm.json` now applies the
physical saved bounds to GenericSystem geometry without opening a serial port.
Omitting the argument retains generic nominal CAD bounds. Use calibrated mock
mode when qualifying which poses correspond to the actual physical setup.

Free-twist mode now queries native MoveIt IK across a full turn of allowed twist
before continuous bounded direction search. Targeted coverage is in the updated
investigation: eight collision-checked directed plans inside the working cube,
96 geometry-only direction witnesses, and a nominal-versus-calibrated comparison
of the reported failed request. Five-dimensional conditioning avoids reporting
the unconstrained twist as a task singularity. Optional nominal-range diagnostics
can explain bound restrictions but can never become executable plans.

## Recorded results

The [validation report](validation/moveit/report.json) records the latest
mock/emulated results and logs: 37 Cartesian checks, 31 startup checks,
29 existing control regressions, 29 C++ hardware checks and 9 model checks.
Additional live panel/limits checks passed for mock and emulated hardware. One
concurrent emulator startup exceeded the stale-feedback deadline before its UI
test; the isolated retry passed and that safety guard was retained. See the
report for the initial failure and retry logs.

The six-package workspace built successfully; changed packages were rebuilt
after the shared startup-policy update. The [mock panel](validation/moveit/mock_pose_panel.png)
and [joint-limits window](validation/moveit/mock_joint_limits.png) screenshots
show the new UI. Tests used isolated ROS domains and PTYs, never the real USB
servo port. Physical endpoint accuracy and loaded lifting remain unverified.

## Validation commands

Run from `robot_software/` after sourcing ROS and the built workspace:

```bash
/usr/bin/python3 tools/test_cartesian.py
/usr/bin/python3 tools/test_ros_control.py
.venv/bin/python tools/test_start_arm.py
/usr/bin/python3 tools/test_arm_model.py
colcon test --packages-select pinchy_hardware
```

Cartesian tests run actual MoveIt/KDL/FCL services with mock controllers or the
real C++ hardware plugin connected to a newly created PTY servo emulator. They
never open the real USB device. Artifacts/launch logs and panel screenshots are
under `/tmp/pinchy-cartesian-tests-*`. The guided-entry test uses the existing
six-motor terminal dashboard on a fake bus and checks continuous torque through
MoveIt readiness. Distinguish these results from physical qualification.

RViz has also been loaded on an isolated mock domain with its real trajectory
plugin and a confirmed `/display_planned_path` subscription. Stereo unsupported
is informational. A root-link inertia warning is expected from KDL; kinematic
FK/IK do not require root dynamics. A visual-only `pointer_tip` marker warning
refers to the cyan point marker; actual holder collision geometry is on its
parent link. No depth/octomap sensor is configured: the current collision world
uses explicit scene objects, not camera depth. The AprilTag subsystem is a future
pose input, not an implicit depth collision map.

## RViz goal and working-region clarification

The draggable display uses **Interactive Markers Namespace** =
`/pinchy_motion/goal` on the installed Jazzy RViz; the earlier `Update Topic`
property did not connect it. Set that namespace in an already open RViz and
select Interact to repair the display without restarting hardware. Read-only
inspection confirmed the server contains `pose_goal` and seven controls.

Floating debug labels were removed and the broad input/FK display is disabled
by default. The blue 0.4 m working cube now spans X [0.10, 0.50],
Y [-0.20, 0.20], Z [0.00, 0.40] m in base_link. The owner authorized a round
front offset; modeled base extent and sampled collision checks support 0.10 m
as a provisional placement, not a guarantee of all-pose reachability. Drawing
must allow pen-axis rotation while maintaining the required normal. See the
Cartesian guide for the report references and commissioning/drawing distinction.

## Operator checks when returning

1. Start mock mode and inspect the same model/view plus new pose panel. Use
   Copy current pose, drag/edit a target and confirm the marker/fields stay
   synchronized without movement. Plan, inspect, then Execute. Inspect the
   collision overlay and Joint limits / bounds window.
   Check the CLI example and Stop/hold as well.
2. Inspect the current physical attachment versus the proxy `pointer_tip` and
   the surroundings versus the example scene. Supply the pending new base-wing
   geometry/mount transforms when available, plus measured table plane/edges
   relative to base_link. The example floor was deliberately not guessed into
   a new location. Do not assume the physical table is modeled.
3. Run `ros2 run pinchy_motion start_motion`; the familiar guided startup should
   retain prior zero holds and open the pose workflow instead of joint sliders.
   Do not run both command UIs simultaneously against one physical controller.
4. With the arm supported/base secured, preview and test a nearby free-space
   pose, verify orientation/direction and Stop/hold, then repeat a small pose
   sequence. Measure actual endpoint error separately from encoder/model FK.
5. Record those physical results in the canonical roadmap. Final pen TCP,
   contact/compliance and loaded/duty qualification remain separate tasks.

Steps 9–10 are the software planning/pose layer; step 11 and later drawing,
HTTPS job retrieval and AprilTag integration are not silently marked complete.


Ordered paths: start with [cartesian_paths.md](cartesian_paths.md); latest real-session diagnostics are in [ik_investigation.md](ik_investigation.md). The roadmap remains the status source.


Priority investigation: [motion_feasibility.md](motion_feasibility.md) covers nearby-pose/wrist-twist traps, singularities and current-model drawing-region failures. It links the read-only diagnostic tool, plots, saved-range MoveIt checks and remaining whole-stroke posture-selection work. Do not assume front-180° coverage from individual pose tests.
