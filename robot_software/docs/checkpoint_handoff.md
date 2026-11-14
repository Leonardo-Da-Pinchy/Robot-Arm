# Agent handoff: Cartesian point and waypoint checkpoint

> November 13: the demo checkpoint is retired. Use the
> [final-product roadmap](implementation_roadmap.md) for current scope, full
> status and HTTPS/AprilTag integration. This file remains the technical map
> of the established control stack; its checkpoint-only task list is historical.

Updated November 9, 2026. Read this first, then inspect the linked code and
repository. This file records technical decisions; the final-product roadmap
is the authority for current statuses and ordering. It is a starting map,
not a substitute for checking the implementation and working-tree changes.

## Objective and latest owner decisions

The checkpoint is a physical arm that accepts an end-effector pose or ordered
poses in a known frame, uses IK and collision-aware planning, executes the
motion, and reports actual progress, success, failure and safety holds.

**November 13: use `pointer_tip`, the end of the current RViz pointer, as the
temporary end-effector point.** The owner authorized this proxy while only the
single servo-mounted attachment is installed. Its physical endpoint is not
measured; do not describe the nominal modeled pen tip as a calibrated empty
holder opening. Final pen length, nib offset and compression remain deferred.
See [tool frames](tool_frames.md).
The owner reports that physical robot motion now looks good. This establishes
basic operation, not measured positioning accuracy or completion of every
load/thermal/fault test.

The owner attributes the visible stepping mainly to very slow operation. That
is an observation/working explanation, not a confirmed diagnosis. During normal
continuous paths, maintain approximately constant end-effector linear speed
through intermediate points, above a minimum useful speed to be measured.
Do not stop at every sampled point. Define the operating speed, threshold and
allowed variation in mm/s at the active tool frame. Starts, ends, intentional
pauses, approach/retract, corners and safety stops need explicit exceptions.
Respect path geometry, allowed corner blending, collisions and joint limits;
report sections that cannot meet the requested speed. See the detailed
[continuous-speed requirement](point_motion_plan.md#8-add-ordered-waypoints-with-clear-path-semantics).

Camera/AprilTags, object registration, SVG processing, drawing contact and the
browser workflow follow this checkpoint. The design report is a baseline;
handwritten IK, full dynamics and every equation in it are not prerequisites.
Use the existing control stack and configure MoveIt 2 for runtime IK/planning.

## Ordered checkpoint status

| # | Status | Task |
| --- | --- | --- |
| 1 | Complete | Identify motors, adapter, supply, Jetson and ROS environment. |
| 2 | Complete | Assign/label IDs 1–6 and verify persistence after power cycling. |
| 3 | Complete | Individual servo and two-/six-servo chain bench tests; owner-confirmed passes. |
| 4 | Complete | Assemble the arm and integrate the supplied CAD 4 six-joint model. |
| 5 | Captured | Joint zeros, directions and initial usable limits, including corrected motor-2 zero. Physical accuracy still needs cross-checking. |
| 6 | Working | Mock and real ros2_control, joint feedback, trajectory execution, RViz and sliders. |
| 7 | Implemented | Interactive zero/hold startup, speed/acceleration controls, motion recording and torque-preserving runtime safety holds. |
| 8 | Partly qualified | Owner accepts basic assembled-arm motion. Complete measured loaded holding, temperature, Stop, fault and recovery qualification. Do not treat emulator tests as physical qualification. |
| 9 | Software implemented | Active `pointer_tip` at owner-approved RViz endpoint; physical attachment agreement and final pen calibration remain unmeasured. See current roadmap. |
| 10 | Remaining | Establish measured joint operating limits, useful end-effector speed range, minimum continuous-path speed and positioning tolerance. Current selectable maxima are not certified physical limits. |
| 11 | Remaining | Create MoveIt configuration: six-joint group, base/tool frames, IK solver, limits and reviewed self-collision setup. |
| 12 | Remaining | Add table/fixtures to the planning scene; connect execution to the existing trajectory controller and safety-hold status. |
| 13 | Remaining | Add point/pose interface: frame, position, orientation policy, preview, execution and clear results. |
| 14 | Remaining | Validate mock point planning, including unreachable, colliding and out-of-limit requests. |
| 15 | Remaining | Execute nearby physical holder-tip poses repeatedly and measure error; verify Stop/hold. |
| 16 | Remaining | Add ordered waypoints; distinguish visiting poses from following a Cartesian path. |
| 17 | Remaining | Time continuous paths for approximately constant tool speed through intermediate points, with explicit exceptions/blending. |
| 18 | Remaining | Demonstrate a physical pose sequence and short free-space path; verify accuracy, progress/results, invalid-path rejection and recovery. |

**Practical next work:** start the MoveIt/mock planning configuration using
the explicit `pointer_tip` proxy. Continue targeted physical qualification
alongside that work; do not repeatedly redo completed bench setup or block on
an unknown future pen offset. No project MoveIt configuration or runtime
Cartesian motion package exists yet; verify this again when picking up work.

## Pending hardware update (November 13)

The owner is redesigning the base with wings integrating a marker holder and
camera holder, and will provide updated geometry and exact mounting transforms.
See the [pending-base note](implementation_roadmap.md#pending-base-redesign--owner-follow-up).
Do not assume these transforms are available or overwrite measured calibration.
The moving holder TCP remains a separate step from fixed base/camera mounting.

## Hardware and calibration

- Jetson Orin Nano; Ubuntu 24.04/aarch64; ROS 2 Jazzy.
- Six Feetech STS3215-12V servos, raw model 777, position mode 0,
  resolution 1, single-turn encoder 0..4095; shared TTL bus at 1 Mbaud.
- URT-2 USB adapter. Stable path:
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00`.
- Owner-verified supply: SANSUN 12 V / 5 A. This label does not establish
  continuous assembled-arm power capacity.
- IDs 1–6 map to `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`,
  `wrist_roll`, `pen_pitch`, respectively. Joint six is a positioning axis,
  not a separate gripper; there is no seventh motor.
- [Actual calibration](../calibration/arm.json) is the source for current
  zeros, signs, bounds, captured evidence and operating settings. Do not copy
  example calibration over it or overwrite owner measurements. Its persistent
  `calibration_confirmed` is deliberately false; guided startup creates a
  temporary confirmed handoff profile after establishing all six zero holds.
- Current ROS profile: raw servo speed cap 500, acceleration cap 32, torque
  limit 1000/1000; per-joint velocity ceiling 30 deg/s. GUI selectable maxima:
  30 deg/s and 180 deg/s²; these are also the defaults, subject to profile caps.
  GUI trajectory limits and servo raw register values are different quantities.

## Active tool frame

The runtime model now includes **`pointer_tip`**, an identity attachment to the
preserved CAD `pen_tip` frame at `(0, -0.149, 0.0188)` m relative to
`pen_holder_link`, with roll π/2. Its +z points outward along the modeled tool.
A cyan visual dot marks it. `tool0` stays at the sixth-axis origin.

[Active tool configuration](../src/pinchy_description/config/active_tool.json)
records the explicit RViz endpoint proxy and `physical_tip_verified: false`.
The owner permits this reference while only the single servo-mounted attachment
is installed. Its actual endpoint is not established by the available model.
Use `pointer_tip` for upcoming MoveIt configuration; measure the physical
attachment and update geometry/TCP together before claiming real-tip accuracy.
Model/FK and mock/emulated TF checks passed; no physical motor moved.
See [tool-frame definition and inspection](tool_frames.md).

Read [CAD/model notes](cad4_model.md), the
[source URDF](../src/pinchy_description/cad/cad4/iteration_2_cad_4_arm.urdf),
[source MJCF](../src/pinchy_description/cad/cad4/source/cad4_model.xml),
[holder measurements](../src/pinchy_description/cad/cad4/source/cad4_holder_measurements.json),
[assembly measurements](../src/pinchy_description/cad/cad4/source/cad4_assembly_measurements.json),
[joint transforms](../src/pinchy_description/cad/cad4/joint_transforms.csv) and
[model generator](../tools/build_arm_model.py) before changing generated geometry.

## Current startup and safety contract

For a fresh terminal, [connect.sh](../connect.sh) prepares USB access, the servo
venv, ROS and workspace sources, and helper commands. It does not move motors:

```bash
./connect.sh
arm
```

`source ./connect.sh` prepares the current Bash shell; `./connect.sh -- arm`
does setup and launches the interactive startup in one invocation. Missing
port permissions can prompt for sudo. `--no-port --domain 0` supports mock/ROS
work without USB. Venv packages are selected by wrappers, not global activation,
to keep ROS Python imports working. [Setup tests](../tools/test_connect.py)
use disposable PTYs and mocked privilege escalation.

After setup, normal ROS commands are available. The interactive physical
workflow has a separate ROS entry point:

```bash
ros2 run pinchy_bringup start_arm
```

Its [thin wrapper](../src/pinchy_bringup/scripts/start_arm) keeps terminal input
attached and invokes the same guided startup with the servo venv. Unlike the
low-level `hardware_control.launch.py`, it performs zero/hold and creates the
temporary reviewed session automatically; do not manually toggle persistent
calibration confirmation simply to bypass that workflow.

The underlying startup can also be invoked directly once dependencies/access
are ready; it sources ROS for its own child processes:

Run from `robot_software/`:

```bash
.venv/bin/python tools/start_arm.py --hardware
```

The terminal dashboard needs one **s** keypress to arm automatic capture.
It visits IDs 1–6 sequentially. The selected joint must be within 25 degrees
of its saved zero, inside its CAD bounds, and stationary for one second.
It moves to zero and holds; earlier holds stay powered. Yellow means locking,
green means locked. **p** pauses capture; **q** aborts/releases. `--plain`
provides the older Enter-per-joint prompts.

Zeroing uses a 0.075 rad/s ramp (about 4.3 deg/s, recently increased 1.5x),
subject to the joint velocity cap and catch-up pauses. Startup raw servo caps
remain at most 100/5, independently of the higher ROS operating caps.
After all six holds, the script transfers serial ownership to ROS without
releasing torque and opens RViz/sliders. Default physical ROS domain is 42.

Live `/arm_safety/held_positions` telemetry now uses a nonblocking realtime
publisher handoff; DDS publication runs on its background thread. It must never
be moved back to synchronous per-cycle publication in the motor write thread.
A 326 ms idle-startup write overrun triggered the stalled-period guard after the
original synchronous publisher was added; transport blocking is the suspected
cause. Keep actual feedback-age/stalled-loop guards enabled.

November 13 velocity behavior: finite, in-range command steps are now
**rate-limited rather than latched off**. Shared six-joint progress respects the
configured speeds; invalid numeric/position targets and hardware health/timing
faults still hold. Joint goal tolerance is now 0.03 rad (~1.72 deg), preventing
a modest settling residual from repeatedly aborting commissioning moves. This
is not a Cartesian accuracy claim. See [hardware control](hardware_control.md).

Critical owner requirement: **ordinary runtime faults must block movement
while retaining existing powered holds, rather than deliberately dropping a
gravity-loaded arm.** The driver latches new goal writes off, publishes
`HOLD: ...` on `/arm_safety/status`, and requests trajectory pause. The GUI
disables Send. Cooling does not resume movement. After correcting the cause,
press **Stop / hold** to discard the old move and recover without restarting.
The reset service requires four healthy feedback sweeps and a replacement hold
at the last accepted hardware command. Startup failures still require rerunning
startup. A fault during Python-owned startup likewise retains
established holds pending explicit release. The handoff checker preserves
holds if ROS reports a safety latch during transfer.

At >=60 C, temperature is rechecked via direct and block reads plus status.
Two agreeing cool rechecks can clear an isolated high sample. Otherwise four
consecutive suspect checks latch the hold; a valid cool check resets the
counter before it latches. Four checks are not four seconds. Real alarms and
other runtime faults can trigger a hold immediately. The underlying transient
temperature anomaly is unresolved; observed examples include 70/70/36 C in
successive requests. Do not claim it is proven harmless sensor noise.

GUI **Stop / hold** pauses trajectory progression, installs a constant hold
goal, then resumes only that constant goal. It avoids the controller's ordinary
cancel path, which caused a commanded-position jump to lagging feedback.
A lagging motor may still settle toward the frozen command.

**Explicit shutdown/deactivation, q or Ctrl+C can release torque.** Support the
arm first. Firmware protection, communication/power failure and mechanical
problems cannot be solved by a software hold guarantee. A hot motor can keep
heating while held. Keep the motor power disconnect available. Bench tools
often release torque on completion and are not substitutes for assembled-arm
startup. Only one process may own the servo serial port.

## Runtime interfaces and files to inspect

| File/interface | Responsibility |
| --- | --- |
| [Startup CLI](../tools/start_arm.py), [dashboard](../tools/startup_dashboard.py) | Interactive sequential capture, ramp/hold, safety handling and ROS handoff. |
| [Calibration CLI](../tools/calibrate.py), [guide](arm_calibration.md) | Read-only zero/direction/limit capture and `--id 2 --step adjust-zero`; preserve measured data and backups. |
| [Hardware launch](../src/pinchy_bringup/launch/hardware_control.launch.py) | Calibration injection, 50 Hz control, session adoption and motion recorder. |
| [C++ plugin](../src/pinchy_hardware/src/feetech_system.cpp), [header](../src/pinchy_hardware/include/pinchy_hardware/feetech_system.hpp) | Radians/ticks, checked serial packets, six feedback transactions per sweep, confirmed goal readback, duplicate-goal suppression, guards and fault latch. |
| [Controller configuration](../src/pinchy_bringup/config/arm_controllers.yaml) | Joint-state broadcaster and six-joint trajectory controller. |
| [Sliders](../src/pinchy_bringup/scripts/joint_sliders.py) | Joint targets in radians; speed/acceleration in degrees; duration estimate; quintic endpoints; Stop/hold and safety gating. |
| [Handoff checker](../src/pinchy_bringup/scripts/startup_ready.py) | Checks this session's model/controllers/feedback; distinguishes safety hold from normal readiness. |
| [Motion recorder](../src/pinchy_bringup/scripts/motion_monitor.py) | Records output, measured position, error, reference velocity and scaling; separate ROS process, no serial connection. Prints its log path, normally `/tmp/pinchy-motion-<PID>.jsonl`; bounded rotation. |
| [Mock launch](../src/pinchy_bringup/launch/mock_control.launch.py) | Explicit mock control and optional RViz/sliders. |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory`, six joint names, radians and timed points. |
| `/arm_controller/controller_state` | Controller reference, actual feedback and output command; use output to maintain command continuity. |
| `/joint_states` | Measured joint positions for TF/RViz/planning. |
| `/arm_safety/reset` | `std_srvs/Trigger`: explicit recovery after four healthy sweeps and a synchronized replacement hold; the GUI orchestrates this through Stop / hold. |
| `/arm_safety/held_positions` | Transient-local stamped `sensor_msgs/JointState`: last adapter-accepted command in the six-joint order, published during normal writes and latched on fault; Stop/recovery uses this rather than an upstream target ahead of the limiter. |
| `/arm_safety/status` | Transient-local `std_msgs/String`: `READY` or latched `HOLD: ...`. Future motion clients must honor it; action acceptance alone does not prove motion is permitted. |
| `/arm_controller/speed_scaling_input` | `control_msgs/msg/SpeedScalingFactor`; 0 pauses, 1 resumes. The backend latch blocks writes even if another client publishes 1. |

Sliders currently send a single joint move with explicit start/end position,
zero velocity and zero acceleration at both endpoints. They derive duration
from selected speed/acceleration and per-joint limits. The controller samples
the quintic trajectory every 20 ms; the in-process hardware plugin sends binary
servo packets when encoder targets change. This single-move interface does
**not** implement continuous Cartesian waypoint speed control.

## Verification and safe development

The real user has verified loose-motor tests, assembled slider motion and basic
physical operation. Emulator tests cover startup/ownership, conversion, guards,
lost replies, thermal confirmation, powered fault latches, Stop under lag,
GUI limits, motion recording and subsequent movement. Raised operating limits
are checked with targeted emulated tests; the faster zeroing ramp passed focused mock tests.
These do not establish loaded accuracy, useful minimum speed, thermal duty or
long-run physical reliability. Do not label those complete without measurements.

Relevant tests: [ROS integration](../tools/test_ros_control.py),
[startup](../tools/test_start_arm.py),
[C++ driver](../src/pinchy_hardware/test/test_feetech_system.cpp),
[calibration](../tools/test_calibrate.py),
[protocol diagnostics](../tools/test_servo_diagnostics.py),
[Python bus emulator](../tools/servo_bus_emulator.py),
[model tests](../tools/test_arm_model.py), and
[CAD validator](../tools/validate_cad4_model.py).

Build/test from `robot_software/`; these tests use mocks/pseudo-terminals:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select pinchy_hardware
colcon test-result --test-result-base build/pinchy_hardware
/usr/bin/python3 -m unittest discover -s tools -p test_ros_control.py
.venv/bin/python -m unittest discover -s tools -p test_start_arm.py
.venv/bin/python -m unittest discover -s tools -p test_calibrate.py
```

Coordinate physical motion tests with the operator. Check repository status
before edits and preserve existing work; do not assume all modifications are
committed. Follow [AGENTS.md](../AGENTS.md): README is general overview/setup,
not progress, results or checklists. Startup must remain an interactive terminal
workflow, not instructions that only exist in a Markdown checklist.

## Supporting references and historical caveats

- [Point-motion plan](point_motion_plan.md): architecture, IK/path semantics,
  continuous tool speed and milestone acceptance. Its older status table is
  superseded by this file.
- [Broader roadmap](implementation_roadmap.md): later drawing/perception/UI work.
- [Hardware inventory](hardware_inventory.md), [older control handoff](control_handoff.md),
  [hardware-control guide](hardware_control.md): hardware provenance and earlier
  commands. Some older assembly/calibration status, 20 Hz timing and automatic
  fault-release descriptions are outdated; current code and this handoff apply.
- [Single-servo guide](servo_bench.md), [chain guide](servo_chain.md),
  [temperature guide](servo_temperature.md), [ROS bench guide](ros_motor_bench.md),
  [mock guide](mock_control.md): setup and unloaded-test background.
- [Archived validation](validation/): model and earlier control artifacts; they
  do not qualify newer physical settings or the future holder-tip frame.
- [Offline calculations](../../calculations/README.md),
  [URDF importer](../../calculations/urdf_import.py),
  [IK backends](../../calculations/ik_backend.py): engineering exploration, not
  the configured runtime IK/control path. Some assumptions are illustrative or
  refer to earlier arm models. Do not require rewriting this math before MoveIt.
- [Original design report](../../18_500_Capstone_Design_Report.pdf),
  [design slides](<../../Design Presentation - Leonardo Da Pinchy.pptx>),
  [proposal slides](<../../Proposal Presentation - Leonardo Da Pinchy.pptx>):
  background, not proof of implemented behavior or measured performance.

Explore the repository as needed and update this handoff when decisions change.

The sliders now show live estimated encoder speed/acceleration alongside position.
**Motion graphs / recordings** compares planned and measured position/speed, can
overlay the last ten moves and export CSV. See [hardware control](hardware_control.md)
for the estimation window, recording limits, and fault-recovery behavior.
