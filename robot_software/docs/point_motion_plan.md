# Plan to reach Cartesian point and waypoint control

> For current status, settings, safety behavior and agent entry point, read the
> [checkpoint handoff](checkpoint_handoff.md). The owner now accepts basic
> physical motion and wants the empty holder's tip used for this checkpoint;
> final pen-tip calibration is deferred. Earlier status tables below are historical.

Original assessment: November 4, 2026. Status reviewed November 8, 2026.

This plan compares the October 9 design report with the code currently in this
repository. The report is a design baseline; its equations and proposed software
are not all prerequisites for first motion. All six motors are labelled,
power-cycled and individually working per the user, including motor 6 passing
on retry. Both two-motor and six-motor shared-bus sequences passed; the six-motor
run used an explicit five-tick bench arrival tolerance.
The immediate milestone is **a real arm that accepts a pen-tip pose or ordered
poses in a known frame, plans the motion, executes it, and reports success or
failure from feedback**. Drawing on an object comes afterward.

## Current checkpoint status — November 8

| Milestone | Status |
| --- | --- |
| Identify electronics and Jetson/ROS environment | ✅ Complete: equipment and Jazzy confirmed |
| Motor setup scripts and concise guides | ✅ Complete: 67 software tests pass, including chained-bus and telemetry tests |
| Label IDs 1–6 and verify after power cycling | ✅ Complete: confirmed by user for every motor |
| Basic individual movement of all six motors | ✅ Complete: confirmed by user |
| Physical two-motor and six-motor chain tests | ✅ Complete for unloaded bench operation; six-motor sequence passed with tolerance 5 |
| Six-axis motor model | 🟨 CAD 4 axes, nominal pen tip, visuals/inertias and collision proxies integrated; physical calibration pending |
| Assembly, joint calibration, load and stop/recovery tests | 🟨 Assembly and installed ID order confirmed; calibration and qualification pending |
| Mock ros2_control trajectory execution | ✅ Complete: action, feedback, cancellation, rejection and slider tests |
| Feetech hardware plugin and real ROS joint trajectories | ✅ First physical proof complete: user confirmed all six unloaded motors passed the ROS out-and-back sequence; assembled-arm qualification remains pending |
| MoveIt configuration and one real Cartesian pose | ⬜ Pending |
| Ordered pose lists and Cartesian path execution | ⬜ Pending |

The scripts allow up to 1024 ticks (90°) per movement leg and raw speed 1000.
Those software limits do not establish successful physical tests at those values.
The directly recorded first-motor pass used 20 ticks and raw speed 100; the user
subsequently confirmed operation and ID persistence for all six motors. See the
[hardware record](hardware_inventory.md) for measurements and
[motor setup guide](servo_bench.md) for the per-motor procedure.

## What exists

| Area | Implemented | Remaining before real Cartesian motion |
| --- | --- | --- |
| ROS description | Stock reference plus CAD 4 runtime model, nominal tip and source geometry | Installed-pen compression/offset, measured limits and physical calibration |
| Visualization | Reference display plus separate mock/hardware control launches, target sliders, TF and RViz | Review assembled EE alignment and physical joint correspondence |
| Motor bench tools | Single-motor setup and chain-test tools; all six labelled IDs power-cycle verified, individual movement and full chain sequence verified | Assembled-joint calibration, load and stop/recovery qualification |
| Bench guards | 67 software tests; individual movement and full six-motor chain sequence verified | Temperature telemetry root cause, emergency stop, power-loss and assembled-arm behavior |
| Offline kinematics | `calculations/`: FK, Jacobians, numerical IK, geometry-specific IK-Geo, sampled workspace/stroke checks | Match the actual arm; integrate runtime planning separately |
| Load analysis | Gravity torque, model import, optional comparison with external SO-101/MuJoCo files | Custom-arm load assessment and measured operating limits; no implemented full inverse dynamics/contact-force analysis |
| ROS hardware control | Feetech hardware plugin, controller/action pipeline and target sliders | Measured calibration, real-bus timing, supervised joint/stop qualification |
| MoveIt | No project configuration or motion client | SRDF/planning group, IK, planning scene, limits, execution controller mapping |
| Browser | Flask health route; canvas records strokes in browser memory | No robot commands, drawing submission, job execution or robot status integration |
| Full project | Design and a broad [implementation roadmap](implementation_roadmap.md) | Registration, SVG mapping, contact handling, drawing validation and optional gripping |

Three distinctions matter:

- The stock URDF has **five joints that position/orient the tool plus a gripper**.
  Per the owner's November 8 clarification, the custom robot reuses all six
  motor axes and repurposes that gripper output as `pen_pitch`. No seventh
  gripping motor is assumed; the pen is held mechanically.
- The six-joint DH calculation model is separate from the displayed stock URDF.
  Its configuration explicitly contains demonstration limits, masses, tool
  offsets and collision shapes. Supplied geometry still needs comparison with
  the assembled custom arm. Existing results do not establish physical accuracy.
- The IK-Geo adapter requires a particular axis arrangement: three parallel axes
  and intersecting final axes. Six joints alone do not establish compatibility.
  The URDF importer uses SciPy IK and disables the illustrative collision model;
  importing a URDF does not provide runtime collision checking. Some importer
  provenance text and tests still refer specifically to the stock five-axis arm.

## Recommended control architecture

```text
pose / ordered pose list in base_link
    -> pinchy_motion: validate request, plan, preview, execute, cancel/status
    -> MoveIt 2: IK, collision checking, path planning and trajectory timing
    -> FollowJointTrajectory action
    -> ros2_control joint_trajectory_controller
    -> pinchy_hardware: calibrated radians <-> Feetech commands and feedback
    -> six arm servos

measured servo positions -> joint_state_broadcaster -> /joint_states -> TF/RViz/MoveIt
```

Use a position-command hardware interface initially; the smart servos already
have internal position control. Start with MoveIt's KDL IK plugin for the actual
serial arm. Keep the offline calculations for engineering checks rather than
making their solver a dependency of runtime control. MoveIt supports this
[IK configuration](https://moveit.picknik.ai/main/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html)
and [trajectory-controller connection](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html).

## Ordered implementation work

### 1. Record hardware, deployment and motion conventions

Record the actual servo voltage variants/models, adapter, supply, motor IDs,
joint order/axes, CAD revision and mechanical ranges. Define stable joint names,
metres/radians, `base_link`, `tool0`, and `pen_tip`, including which tip axis points
along the pen. Record how the six arm motors are powered and addressed. Choose the deployed ROS distribution before writing version-
specific configuration: this machine has Jazzy; the report proposes Humble.

**Done when:** a versioned inventory and calibration template identify every arm
joint and motor. Unknown values remain explicitly unknown.

### 2. Test each physical servo individually

Start with one unloaded servo, the confirmed supply and adapter, and read-only
communication. Follow [servo_bench.md](servo_bench.md) to discover/configure unique
IDs, common baud and position mode. Save existing settings. The setup tool's
`test-motion` command establishes a current-position goal before explicit torque
enable, performs an out-and-back test and releases torque. Goal writes may
themselves enable output, so the initial current-position goal matters.

Use the bench tool for small slow moves only after independently checking the
allowed range. Record actual position change, direction, voltage/temperature,
hold/release behavior and reconnect behavior. Verify emergency power cut with
the mechanism supported. Repeat for all six arm motors. The tool's hold is best effort, not a guaranteed stop on bus loss.

**Done when:** every arm motor communicates and moves as expected, has a unique
labelled ID, and has recorded bench results. Mock tests do not satisfy this step.

### 3. Build and check the actual six-joint URDF

Retain the stock reference and add a separate custom description from current
CAD/measurements. Include all six arm axes, measured link transforms, realistic
joint ranges, simple collision geometry, pen holder and fixed tip transform.
Use the confirmed former gripper axis as `pen_pitch`; mount the manually
loaded pen assembly with verified fixed/rotating part assignments.

Check zero pose, axis directions, tip position and clearances in RViz against the
assembly. Add the table/base fixture to the planning scene. Accurate inertials
support load analysis and physics; a complete dynamics simulator is not needed
for initial kinematic IK and planning.

**Done when:** moving each virtual joint matches the corresponding physical axis,
and the tip transform and ranges match measurements. Steps 2 and 3 can proceed
independently once hardware information is available.

### 4. Calibrate the assembled joints and establish startup/stop behavior

Support the arm and move one joint at a time at low speed. Measure encoder zero,
direction, ticks per radian, usable range and any wrap behavior. Store the
calibration under stable joint names and servo IDs. Choose conservative initial
speed/acceleration limits and check holding/heating under actual loads.

Check physical poses against model FK. On startup read current positions before
accepting goals; never move automatically to URDF zero. Check disconnect,
controller restart and power restoration behavior. Define software cancel/hold
separately from emergency motor-power cut, including support if the arm sags.

**Done when:** tick/radian conversions agree with measured joint motion, model and
hardware agree at several poses, and startup and stop behavior are documented.

### 5. Add ros2_control with explicit mock hardware

Add six position command/state interfaces to the custom description, a
joint-state broadcaster, and a joint trajectory controller. Use explicit mock
hardware initially. Create a small joint-target client using the
`FollowJointTrajectory` action and check completion, timeout and cancellation.
Use the same joint names and controller action later for real hardware.

The six motor transforms now reuse the SO-101 reference as confirmed by the
owner. Mock control and sliders are described in [mock control](mock_control.md);
the integrated model now follows the supplied [CAD 4 source](cad4_model.md),
including nominal pen tip and simplified EE geometry. Physical calibration and
collision-aware planning remain pending.

**Done when:** mock trajectories move the custom model through the controller,
with feedback and action results. RViz sliders alone do not satisfy this step.
Mock hardware verifies integration, not motor dynamics or tracking accuracy.

### 6. Implement the Feetech hardware interface and prove joint control

Add `pinchy_hardware` as a ros2_control system for the shared bus. Own the port in
one process; read actual positions, convert using calibration, and send bounded
joint targets. Validate packet errors and feedback age; define error/hold behavior
without substituting sent commands for measurements. Measure bus throughput to
choose a sustainable update rate for six motors. Validate how servo speed and
acceleration settings interact with streamed trajectory setpoints.

Add a separate hardware bringup launch. Initialize commands from current
feedback and require deliberate motion requests. Exercise slow single-joint
trajectories, then short coordinated moves between known clear poses. Check
tracking tolerances and cancellation on the real arm. Do not run slider publishers
or the bench tool alongside hardware control.

**Done when:** measured states match RViz and bounded six-joint trajectories finish
or fail explicitly through the controller. A motor fault or stale feedback cannot
be mistaken for successful execution. Software work on steps 5–7 can proceed in
mock mode while physical calibration is completed.

### 7. Configure MoveIt and demonstrate one Cartesian pose

Generate `pinchy_moveit_config` using the custom URDF. Define the six-joint arm
group/SRDF, base/tip, reviewed self-collision exclusions, KDL solver, measured joint
limits and conservative velocity/acceleration limits. Map trajectory execution to
the controller from step 5. Include the pen holder, tabletop and fixtures in
collision checking.

Plan a nearby `pen_tip` pose in explicit mock mode first. Start with an explicit
orientation; an XYZ point does not specify how the pen should point. Check FK of
the result and preview the complete plan. After step 6 passes, execute one short
clear move on hardware, then several poses and an unreachable-goal rejection.

**Done when:** a pose request moves the physical tip repeatably and the reported
result distinguishes planning failure, execution failure and success.

### 8. Add ordered waypoints with clear path semantics

Create `pinchy_motion` with a request containing the reference frame, ordered
poses, orientation policy, motion mode, speed limits and geometric tolerance.
Provide plan/preview, execute, progress/result and cancel interfaces. A terminal
client is enough for the first milestone.

Support two different requests deliberately:

- **Visit poses:** reach them in order using planned transit motions; the tip may
  curve between endpoints. A commissioning mode may stop at each pose, but
  continuous operation must not decelerate to rest at every intermediate waypoint.
- **Follow a Cartesian path:** preserve the intended tool path between poses,
  needed for drawing. Use Cartesian interpolation and IK continuity checks;
  reject incomplete paths, collisions and excessive joint jumps before execution.

**End-effector speed requirement:** During normal continuous operation, maintain
approximately constant pen-tip linear speed through intermediate points, above
a measured minimum useful speed. Express this in mm/s at the active tool frame
(the empty holder's tip initially, the calibrated pen tip later), separately
from joint speed in degrees/s. The requested operating speed, minimum threshold
and allowed speed variation are **TBD from physical measurements**.

The owner observes stepping at very low speeds; this is a working explanation,
not a confirmed diagnosis. Qualification should characterize useful operating
speeds rather than require arbitrarily slow slider motion to look continuous.
Plan and time the whole continuous segment together, with compatible velocities
through intermediate waypoints, instead of chaining independent moves that each
have zero endpoint velocity. This requirement is not implemented by the current
single-target sliders.

Acceleration/deceleration at segment starts and ends, explicit pauses, approach/
retract moves, sharp corners and safety stops are exceptions to the minimum
speed. Preserve sharp corners or use an explicitly allowed blending tolerance;
do not distort the path or exceed joint, collision or acceleration constraints
to maintain speed. If the requested speed cannot be maintained over a continuous
section, report that limitation before execution. Verify measured tip speed and
path accuracy together when choosing the operating profile.

For the first Cartesian implementation, use explicit orientations. Later support
pen-axis alignment with free roll deliberately: full-pose IK does not automatically
search every roll about a surface normal. Do not use position-only IK when the
pen must remain perpendicular to the surface.

Verify every segment and start from fresh measured state. Require the entire
requested path, apply time parameterization, and check the final trajectory's
geometry, limits and continuity before sending it. Preserve corners or make any
blending tolerance explicit. MoveIt's
[Cartesian-path example](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
and [time-parameterization documentation](https://moveit.picknik.ai/humble/doc/examples/time_parameterization/time_parameterization_tutorial.html)
explain these facilities; match the APIs to the selected ROS/MoveIt release.

**Done when:** the real arm visits a small pose list and follows a short line or
triangle in free space, rejects a path containing an unreachable point without
executing a prefix, and can cancel/hold with a clear result. Measure actual tip
error independently; matching encoder targets alone does not prove tip accuracy.

## Work after the point-motion milestone

Add a fixed, measured object frame before depending on camera registration.
Generate surface poses and outward pen-up moves, preserve separate strokes,
and validate holder/object clearance. Only intended pen-tip contact may be
allowed; do not disable collision checking for the whole object or arm. Establish
repeatable contact with the actual holder, then add SVG sampling, camera/tag
transforms and browser job handling. Automatic gripping can follow manual-pen
operation. These extend the same pose/path interface rather than creating a
second motor-control path.

## Math and simulation needed now versus later

Use MoveIt for FK/IK, planning, collision checks and trajectory timing; use the
trajectory controller for timed joint interpolation and action feedback. Custom
closed-form IK, another DH derivation, full inverse dynamics and a Gazebo/MuJoCo
simulation are not prerequisites for the first slow point moves. The
[position controller documentation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
describes how position commands are passed to the hardware interface.

Still needed are correct joint transforms, encoder conversion, tool calibration,
measured operating limits and basic load/holding checks. MoveIt does not determine
safe motor torque or continuous duty, compensate printed-link deflection, measure
contact force, or guarantee the report's 2 mm physical accuracy. Revisit detailed
loads and dynamics as speed, reach and contact requirements increase.

## Verification performed for the original November 4 assessment

- Both existing ROS packages built successfully against installed Jazzy, with
  build/install/log outputs under `/tmp/pinchy-review-*`.
- `python3 -m unittest discover -s robot_software/tools -p 'test_*.py' -v`:
  **3 passed**, using mock buses only.
- `python3 -m pytest calculations/test_analysis.py -q`:
  **10 passed, 2 failed, 1 skipped**. Both failures are missing `ik_geo` in the
  current interpreter; the skip requires an external SO-ARM100 source model.
  The README's previously described local venv is absent here. MuJoCo and the
  Feetech SDK are also absent from the inspected Python environment.
- ROS package discovery finds MoveIt and controller_manager installed. Their
  installation does not mean the project has configured or exercised them.
- No physical servo commands, motor tests, RViz GUI verification or full motion
  execution were performed during this assessment.
