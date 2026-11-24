# Cartesian pose control with MoveIt 2

This is the next layer above the existing joint-testing bringup. It accepts an
XYZ and optional orientation constraints for `pointer_tip`, solves IK with
MoveIt KDL plus bounded multi-start fallback, checks a complete transit path,
previews it, and executes only after an
explicit Execute request. It reuses the live calibrated model, controller,
Feetech adapter, zero/hold startup, thermal checks and powered fault recovery.

`pointer_tip` is the [approved RViz pointer proxy](tool_frames.md), not a measured
endpoint of the currently installed single part. The supplied scene contains
an example floor, not the final tabletop/fixture or pending base wings/camera.
No physical pose test or physical TCP accuracy is implied by software tests.

## Current free-twist workflow

The current task preserves XYZ **and pen pointing direction**, with twist free.
The panel, CLI and an empty service `orientation_mode` default to **Pen direction
/ free twist**. Enter XYZ and a direction through RPY/the blue target +z arrow,
leave Straight TCP path unchecked for OMPL routing, Plan/preview, inspect and
Execute. The solver may rotate the pen around that direction. **Position only**
remains an explicit diagnostic/alternate task; it is never used to execute a
failed direction request.

Measured feedback may settle slightly beyond a command boundary: the adapter
already allows 10 encoder ticks. The planning gateway now applies that same
margin to measured-state validation while collision-checking the **raw measured
pose**. Held commands, IK candidates and all trajectory positions retain strict
bounds. It does not clamp telemetry, recalibrate zero or extend commanded travel.
A focused emulator case reproduces one tick below wrist-roll zero with an
accepted zero hold and verifies XYZ-only planning/execution. See the investigation.

## Nearby XYZ can still request a large direction change

Goal fields remain fixed when the physical arm changes orientation; they are
separate from the live Current TCP display. The panel now shows goal distance
and the difference between current and requested pen pointing directions.
**Copy current direction** keeps goal XYZ and deliberately replaces its RPY with
the current measured/model orientation; it only edits the visual goal and does
not plan or move. **Copy current pose** replaces both XYZ and RPY. Neither updates
existing queue rows: edit/re-add those rows explicitly.

The latest screenshot asks for XYZ (0.471461,-0.003679,0.239280) m and RPY
(-130.083,-86.586,4.025)°. It is about 16.5 mm from the displayed tip, but changes
pointing direction by 53.9°. Saved-range multi-start and an independent global
numerical search both retain about 8.06 mm / 6.42° residual for that combination.
This is evidence of a difficult/infeasible combination, not a mathematical
impossibility proof. The same XYZ with the displayed current pointing direction
has valid bounded IK and passed collision-checked mock transit planning. Raising
speed limits cannot solve a position/direction mismatch.

Current selectable caps: joints 92.14 deg/s and 398.5 deg/s²; queued TCP 400 mm/s
and 2000 mm/s². Starting values remain 30 deg/s, 180 deg/s², 20 mm/s and
50 mm/s². Zeroing speed remains unchanged. Joint caps live in the shared control contract and are also
bounded by calibration, URDF, raw motor settings and the driver (which currently
rejects configured max_velocity above 92.14 deg/s). TCP request caps are checked by
both panel and gateway. Change the matching layers together when selecting new
caps; GUI maxima are not measured physical performance. The TCP timing fields
apply to queued trace/spline paths; OMPL pose visits use joint timing caps.

## Choosing what the solver must preserve

| Orientation selection | Required at the endpoint |
| --- | --- |
| **Exact RPY** | XYZ and all three orientation angles. Never silently relaxed. |
| **Pen direction / free twist** (default) | XYZ and the requested tool +z direction; rotation around that direction is free, as allowed by the drawing report. |
| **Position only / free orientation** | XYZ only. RPY inputs are disabled/ignored explicitly. |

The solved orientation is reported in the plan response (`resolved_target`) and
panel notice, and is visible in the robot trajectory preview. Target axes remain
the input reference; in relaxed modes their unconstrained twist/orientation need
not match the solved endpoint. Choose Position only when testing whether an XYZ
point can be reached, and Pen direction when the pen must point along a normal.

Check **Straight TCP path (LERP + SLERP)** for a straight position segment with
shortest quaternion orientation interpolation in Exact RPY mode. Unchecked is OMPL free-space
transit and can curve around obstacles. Pen-direction mode interpolates the pointing direction with twist free at every
step; it does not impose an arbitrary endpoint twist along the route. A reachable endpoint can still have an
impossible straight route: incomplete or jumping paths are rejected, never
executed partially and never silently replaced with a curved route.

This is a free-space single-pose transit option, not constant-speed contact
drawing. The [IK investigation](ik_investigation.md) records benchmark results,
singularities, joint-limit constraints and the [ordered-path guide](cartesian_paths.md) describes step 11.
After upgrading these interfaces, restart the planning server and its panel/CLI
together; an already running old server will not gain the new controls. The
usual guided physical entry starts the matching versions. Support the arm before
explicitly shutting down hardware; no running physical stack was auto-restarted.

## Build / dependencies

From `robot_software/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

MoveIt/KDL/OMPL were already installed on this Jetson. No replacement servo SDK,
firmware, calibration or EEPROM setup is required. Package manifests declare
runtime dependencies; on a fresh ROS Jazzy installation use the normal rosdep
workspace setup. ROS tools use system Python; servo startup retains its venv.

## Mock: start here

```bash
cd ~/Robot-Arm/robot_software
source ./connect.sh --no-port --domain 0
ros2 launch pinchy_moveit_config mock_motion.launch.py \
  calibration:="$PWD/calibration/arm.json"
```

This starts the existing mock controllers, MoveIt, the pose gateway, RViz and the
XYZ/RPY panel with the saved physical bounds applied to mock geometry only.
It never opens the physical servo port. Omit `calibration:=...` for nominal CAD
bounds; use it when rehearsing poses for the actual assembled arm. Optional launch flags:
`rviz:=false` and `panel:=false` for CLI/headless operation.

1. Wait for the panel to become ready. **Copy current pose** fills all six
   target fields from encoder/model FK. XYZ is in **metres** in `base_link`;
   RPY is in **degrees**, with R = Rz(yaw) Ry(pitch) Rx(roll), matching URDF.
2. Edit XYZ/RPY in the panel or drag the goal arrows/rings with RViz’s
   **Interact** tool. The marker and fields update immediately in both directions;
   editing does not run IK or move motors. RPY describes the **tool frame**,
   not three joint angles; its +z points along the pointer toward the tip.
3. Press **Plan / preview**. This does not move the motors. RViz shows target
   axes and a ghost trajectory while the ordinary robot model remains driven
   by actual joint states. The existing view, colors, grid, TF and model are
   reused from `pinchy_bringup/config/arm.rviz`.
4. Inspect the whole path, then press **Execute previewed plan**. Changing any
   target/speed invalidates the panel's preview. Plans expire after 60 seconds;
   a changed scene/start requires a new plan. A changed live model blocks motion
   until the planning layer is restarted with that model.
5. **Stop / hold** discards interrupted motion and establishes powered hold.
   After correcting a fault, **Stop / hold** also requests the same explicit
   backend recovery as the old sliders; **Recover / hold** is an equivalent
   dedicated control. Neither button requests torque off. A corrected temperature reading
   alone never automatically resumes the old movement.

To repair an already open RViz from the earlier layout: expand **Draggable
goal**, set **Interactive Markers Namespace** to `/pinchy_motion/goal`, and
select **Interact** (I). This RViz version takes the namespace, not an Update
Topic setting. The saved layout has been corrected for future launches.

## Reach, orientation and collisions

Six joints do not guarantee every orientation throughout the position workspace.
Full-pose IK holds **all three RPY values** while solving XYZ; a small translation
can require a large wrist motion. The extended zero configuration is poorly
conditioned: an offline local Jacobian check predicts that moving Y by -5 mm
while holding its orientation would initially require roughly -34° wrist bend
and -21° pen pitch. This is a **local estimate**, not a validated finite IK path.
The initial saved pen-pitch interval was only about -1.67° to +90.35°;
it is now -0.70° to +99.14° after recapture, and wrist roll was
initially 0° to +131.48°. After operating-range recapture, wrist roll is now
-143.96° to +141.77°; the three recent logged directed goals plan successfully. This constrains near-zero motion in one direction. The Jacobian
was full rank, so it would be incorrect to call the arm generally five-axis.

The report allows the round pen to rotate freely about its own axis while
maintaining the surface-normal direction (sections IV.B and VI.F). That is a
requirement now available as the explicit Pen direction/free twist endpoint
mode. Exact RPY still honors all explicitly supplied orientation values.

Use **Copy current pose**, start with small edits in mock mode, and allow an
appropriate orientation change or choose a bent starting configuration when
needed. The solver tries several seeds, checks actual bounds and collisions,
and never silently discards RPY or widens calibrated limits. Its diagnostic
reports candidate collision pairs/nearby limits when available. Numerical IK
failure is not proof that no mathematical solution exists. The earlier warning
log did not record its requested XYZ/RPY, so that exact rejection cannot be
reconstructed; new plan logs include them.

### Checked examples from the operator panel

For XYZ (0.395763, -0.058652, 0.129172) m and RPY
(-120.927, -86.750, -85.681)°, a bounded offline full-pose search did not converge;
its best result approached wrist-bend/roll bounds and still missed by about
7.1 mm / 14.9°. A position-only search found the exact point and MoveIt confirmed
that candidate collision-free, with a different orientation. This establishes
position reachability, not a proof that no other full-pose branch exists.

For XYZ (0.333508, 0.000154, 0.125493) m and RPY
(-5.159, -62.390, -173.985)°, a search preserving position and pen +z direction
found a collision-free solution with only 2.88° of axial pen rotation. One full
orientation for that solution is approximately RPY (0.356, -62.511, 179.796)°.
The bounded full-RPY search instead hit the wrist-bend lower bound and retained
about 0.58 mm / 0.98° residual. These are diagnostic searches, not dispatched
plans; no physical motors moved. Evidence is in
[example one](validation/moveit/ik_example_one.json) and
[example two](validation/moveit/ik_example_two.json).

The design report (sections II, IV.A and Appendix A) specifies a **0.4 × 0.4 ×
0.4 m working region in front of the arm**, containing the object/platform and
all drawing and pen-up motions. The owner authorized a base-aligned cube and a
round forward offset. The chosen near face is **X=0.10 m**, giving bounds
**X [0.10, 0.50], Y [-0.20, 0.20], Z [0.00, 0.40] m** in `base_link`.
The blue wireframe **Working region (400 mm cube)** replaces the earlier debug
box as the default workspace display, with no floating text.

The modeled base reaches X≈0.0646 m, leaving about 35 mm to the near face.
A read-only 512-state sample found 67 collision-free modeled configurations in
the 0.10 m-offset cube, versus 65 at 0.15 and 57 at 0.20; some configurations in
all three boxes still collided. These counts are not Cartesian coverage or a
clearance guarantee. The box designates a working region; it cannot guarantee
all positions/orientations inside it, and is not a collision obstacle.

The numerical input bounds remain separate for pose commissioning/transit;
this cube does not clip whole-arm travel or silently reject all transit outside
it. Registered drawing-region enforcement belongs to the object/job workflow.
The source is `pinchy_motion/config/workspace.json`.

The broad bounds/FK display is now **disabled by default** under **Input bounds
and kinematic samples (debug)**, with no floating explanatory labels. Explanations
stay in the panel/docs. Enable the display only when inspecting those diagnostics.

RViz displays the following; **Joint limits / bounds** opens the actual loaded
angular intervals and numerical XYZ bounds:

| Display | Meaning |
| --- | --- |
| Blue working-region wire box | X [0.10, 0.50], Y [-0.20, 0.20], Z [0.00, 0.40] m; designated region, not certified full-pose reachability. |
| Yellow wire box (optional debug) | Accepted numerical input bounds: X/Y [-1, 1] m, Z [-0.2, 1.2] m. **Not** a guaranteed reachable or collision-free volume. |
| Cyan point cloud | Deterministic FK samples inside the loaded joint bounds, with varied orientations and **without collision filtering**. Not fixed-RPY reachability. |
| Green floor | Example scene box: center (0, 0, -0.04) m, size (1.2, 1.2, 0.04) m; top at Z=-0.02 m in `base_link`. **Not a measured tabletop.** |
| Robot collision geometry (toggle) | Optional translucent overlay of the URDF collision geometry actually used by MoveIt. Visual meshes and collision proxies can differ. |

Planning checks robot/world and nonadjacent robot/tool self collisions. Only six
adjacent mechanical pairs are excluded in the SRDF. The camera does not provide
an automatic depth collision map. Direct joint sliders are commissioning tools
and do **not** invoke MoveIt collision checks.

The floor has deliberately **not** been repositioned to explain physical table
contact. We need the measured tabletop plane/edges relative to `base_link` and
fixture/base-wing geometry before correcting the scene. The currently installed
single attachment also differs from the long RViz pointer proxy; actual geometry
and TCP remain to be measured. Inspect the whole path before physical execution.

Defaults/maxima are 30 deg/s and 180 deg/s², narrowed by the live profile. These
are joint limits, not a guaranteed Cartesian speed or physical qualification.

## CLI: the same plan/execute workflow

Use another prepared terminal on the **same ROS domain**:

```bash
ros2 run pinchy_motion pose status
ros2 run pinchy_motion pose plan \
  --xyz 0.454592161375 -0.020932924774 0.267158992366 \
  --rpy 0.583579555345 -82.551163122968 176.555799377997
```

That example comes from a nearby valid configuration of the current model and
is intended for mock testing. It is not approval of a physical path around an
unmodeled fixture. The command prints the checked plan ID and duration, then
the exact next command. After reviewing RViz:

```bash
ros2 run pinchy_motion pose execute PLAN_ID_FROM_OUTPUT
ros2 run pinchy_motion pose stop
```

| Command/flag | Meaning |
| --- | --- |
| `status` | Mode, readiness, tool frame and current encoder/model XYZ/RPY. |
| `plan --xyz X Y Z --rpy R P Y` | Plan/preview only; metres and degrees. |
| `--orientation full\|tool_axis\|position` | Exact RPY, pen +z with free twist (default), or XYZ only. RPY is optional only for explicit `position` mode. |
| `--path joint\|cartesian` | OMPL transit (default), or straight TCP position LERP plus orientation SLERP. |
| `--frame FRAME` | Reference frame; defaults to `base_link`. Known TF frames are transformed to base at planning time, not tracked while moving. |
| `--radians` | Interpret input RPY in radians. |
| `--speed VALUE` | Joint speed ceiling in deg/s; default is loaded cap, at most 30. |
| `--acceleration VALUE` | Joint acceleration ceiling in deg/s²; default is loaded cap, at most 180. |
| `--out FILE.json` | Save plan ID/target/summary; the executable trajectory remains bound to the current server session. |
| `execute PLAN_ID` | Consume a checked, fresh plan exactly once. |
| `execute PLAN_ID --hardware` | Explicit physical-mode authorization; rejected against a mock server. |
| `stop` | Pause, replace the goal with a constant accepted-target hold, and discard the old move; explicitly request backend recovery if already fault-held. |
| `recover` | Run the same hold handshake and ask the hardware reset service to check recovery. |

Ctrl+C during CLI execution requests powered Stop/hold while the gateway is
available. It does not shut down hardware bringup.

## Physical startup when ready to check it

The existing joint-testing command remains available unchanged. For the new
pose workflow:

```bash
cd ~/Robot-Arm/robot_software
source ./connect.sh --domain 0
ros2 run pinchy_motion start_motion
```

This enters the **same** one-second, sequential guided zero capture and powered
ROS handoff, then opens the planning/panel workflow instead of joint sliders.
`--no-gui` starts the planning services without RViz/panel. No pose is executed
merely by startup. Existing measured zeros/signs/bounds are preserved.

Before the first physical Execute, inspect the model against the actual
attachment and surroundings, including any installed base changes. Use a
nearby collision-free pose with the supported arm and power disconnect ready.
The base-wing geometry/transforms and final pen TCP are still pending; do not
assume the example scene models them. The next operator checks are listed in
[the integration handoff](moveit_handoff.md).

To attach planning to an **already running matching hardware bringup**, close
other command UIs and use:

```bash
ros2 launch pinchy_moveit_config planning.launch.py hardware:=true
```

It does not open the serial port or redo startup. Run one planning stack and
one active command UI on a robot domain. Do not run the mock launch alongside
physical bringup on that same domain.

Closing only the idle pose panel does not shut down hardware. As before,
**Ctrl+C in the physical startup terminal or hardware deactivation can release
torque**; support the arm before explicitly shutting it down. A planning failure
or an ordinary runtime safety hold does not deliberately release torque.

## What is shared and what is new

| Component | Responsibility |
| --- | --- |
| Existing `pinchy_description` and active tool profile | One model/tool definition; no separate MoveIt geometry or alternate joint names. |
| Existing `pinchy_bringup` / `pinchy_hardware` | Same controllers, calibration, bus ownership, 50 Hz adapter, rate limiting, four-suspect thermal confirmation and explicit powered recovery. |
| `pinchy_bringup.control_contract` | Shared joint order, controller endpoints, commissioning timing and constant hold goal; sliders and Cartesian execution use these definitions. |
| `pinchy_bringup.powered_hold` | Cartesian gateway's asynchronous pause/accepted-target/hold handshake, matching the commissioned slider protocol. Uses stamped targets captured after Stop. |
| `pinchy_moveit_config` | SRDF/KDL/OMPL configuration, live-model parameter assembly, example scene, shared-view RViz additions and drawing motion policy. |
| `pinchy_motion_interfaces` | Typed planning service and execution action. |
| `pinchy_motion` | Pose panel/CLI, full-pose planning gateway, preview tokens, validation, execution/progress and powered Stop/recovery. No direct servo access. |

For example, an XYZ-only preview is:

```bash
ros2 run pinchy_motion pose plan --xyz 0.333508 0.000154 0.125493 --orientation position
```

The planner bootstrap reads `/robot_description` from existing bringup and
passes that exact calibrated model to MoveIt. Kinematics and SRDF are separate
planning configuration; the robot's origins/axes/position limits stay shared.
Neighboring mechanical link pairs are the only SRDF collision exclusions.
Nonadjacent arm/tool collisions and world objects remain checked. No automatic
"never collides" sampling disables distant link pairs.

MoveIt supplies collision-aware KDL IK, OMPL transit and optional Cartesian
interpolation. A bounded numerical search supplements IK using the same live
model and limits; every candidate still passes MoveIt collision checks. Time-optimal
parameterization respects configured joint limits. The gateway then checks the
actual linear/cubic/quintic interpolation used by JTC, including polynomial
position/velocity/acceleration extrema, uniformly slows if needed without
changing geometry, and collision-checks dense interpolated states. These are
discrete checks, not a mathematical continuous swept-volume proof. No fields
are added at dispatch that would change interpolation after validation.

The gateway rechecks current state, model, scene and the exact validated trajectory
fingerprint before execution, reusing preview collision validation only for an
identical trajectory/model/scene (see [Execute latency](execute_latency.md));
scene changes or lost feedback during execution request powered hold. Endpoint
results report encoder/model FK residual, not independent physical accuracy.
The low-level hardware protections remain authoritative even if another client
tries to resume speed scaling.

## ROS interfaces

| Interface | Type / data |
| --- | --- |
| `/pinchy_motion/plan_pose` | `pinchy_motion_interfaces/srv/PlanPose`: stamped pose, orientation/path mode and speed/acceleration scaling; returns success/reason, opaque plan ID, checked trajectory, resolved target and duration. |
| `/pinchy_motion/plan_path` | `pinchy_motion_interfaces/srv/PlanPath`: ordered stamped waypoints, constraints/style, TCP timing and contact labels; returns a checked whole-path preview and existing execution token. See [ordered paths](cartesian_paths.md). |
| `/pinchy_motion/execute_plan` | `pinchy_motion_interfaces/action/ExecutePlan`: plan ID plus physical-mode confirmation; stage/progress and final result/encoder-model pose residual. Cancellation establishes hold. |
| `/pinchy_motion/stop` | `std_srvs/Trigger`: powered Stop/hold; invalidates previews. |
| `/pinchy_motion/recover` | `std_srvs/Trigger`: explicit hold/recovery through the existing backend checks. |
| `/pinchy_motion/status` | Transient-local JSON `std_msgs/String`: ready/reason, mode, stage, current XYZ/RPY, safety and limits. |
| `/display_planned_path` | MoveIt `DisplayTrajectory` for RViz preview; never execution authorization. |
| `/pinchy_motion/target_input` | `PoseStamped` live editor input in `base_link`; visualization only. |
| `/pinchy_motion/target_pose` | Latched `PoseStamped` edited target; panel/marker synchronization. |
| `/pinchy_motion/goal/update` | Interactive marker updates; RViz sends drag feedback under this namespace. |
| `/pinchy_motion/markers` | Live target XYZ axes as a `MarkerArray`. |
| `/pinchy_motion/working_region` | Blue designated working-region box as a latched `MarkerArray`; visual guidance only. |
| `/pinchy_motion/workspace_markers` | Latched bounds wireframe and approximate FK samples. |

`move_group` is configured for planning only and its direct execute/move action
capabilities are disabled. The motion gateway executes through the existing
`/arm_controller/follow_joint_trajectory`, retaining the commissioned powered
Stop/recovery contract. Direct ROS controller tools still exist for commissioning;
they are not collision-checked application requests.

## Drawing-speed contract (later continuous-path work)

**During normal pen contact, maintain approximately constant TCP linear speed
along the surface. Reach that speed before marking and put acceleration /
deceleration in pen-up, off-surface motion wherever feasible.** Plan a whole
contact stroke with nonzero speed through intermediate samples; do not send a
series of independent pose moves that stop at every sample.

The versioned [drawing policy](../src/pinchy_moveit_config/config/drawing_motion_policy.yaml)
records this requirement. Actual contact speed, minimum useful speed and
allowed variation remain unqualified. Keep geometric accuracy, collisions and
joint constraints authoritative. If a sharp corner or short stroke cannot meet
the speed requirement, use a lift/reposition, explicitly approved blending or
an explicitly reported alternative; do not silently slow down across the ink
or distort its shape. Safety stops remain allowed immediately, including during
contact. Unavoidable touchdown/lift transitions need explicit handling.

This release implements **free-space pose transit (OMPL or checked LERP/SLERP)**, not contact drawing or
constant-speed surface paths. Status explicitly reports
`contact_motion_supported: false`. Step 11 and the later surface/contact work
must implement and measure this policy before advertising drawing support.


The doubled operating envelope uses raw RAM speed 1000 and acceleration 64 in
the saved profile, allowing the requested 60 deg/s / 360 deg/s² joint caps. Raw
settings are ceilings for following trajectories, not a request to run every
move at that speed. Calibration zeros, directions, position bounds and torque
are unchanged. Guided startup still uses its separate 150/8 raw caps and the
same zeroing ramp. Pose panel, joint sliders, CLI and service defaults retain
30 deg/s and 180 deg/s²; explicit higher selections/scales opt into the new caps.
Load the updated driver/model/profile on the next normal hardware startup;
running hardware processes were not restarted or actuated by this change.


For twist-dependent nearby failures and singularity/range investigations, see
[motion feasibility](motion_feasibility.md). The live display now shows low joint
travel reserve, and the limits window includes a Reserve column. Preview notices
flag low reserve over the actual controller interpolation. These are diagnostic
warnings, not additional motor-stop conditions or automated unwinding.

Planning now exposes live elapsed time, stage counts/percentages and approximate
stage time remaining in the Cartesian window. See [planning progress](planning_progress.md).


November 22: an explicitly enabled [TCP range](bounded_speed_proposal.md) adds
minimum/maximum interior timing without changing old fixed-speed point sets.
The [joint ceilings](estimated_operating_limits.md) now follow the owner's
coupled estimate-minus-10 rule; they are not measured physical safe limits.
