# Robot software implementation roadmap

This is the working path from the current ROS 2 display to a robot that draws
SVG strokes on a known curved object. The [October 9 design report](../../18_500_Capstone_Design_Report.pdf)
sets the initial goals and architecture; measured hardware, newer CAD, and team
decisions should update this plan. A checked box means the named deliverable
exists and has been exercised at the level stated, not just discussed.

## Starting point

- [x] `pinchy_description` contains the stock SO-101 URDF and meshes: five arm
  joints plus a gripper. This is a reference, not the six-joint custom arm.
- [x] `pinchy_bringup/display.launch.py` shows that URDF in RViz with virtual
  joint sliders and synthetic `/joint_states`.
- [ ] A URDF for the actual six positioning joints, pen holder, and pen tip.
- [x] Communication and a small out-and-back test with one physical servo.
- [x] IDs 1–6 labelled and power-cycle verified; individual motor operation confirmed by user.
- [x] Physical unloaded six-motor chain sequence, including individual, together,
  opposite-direction and mixed-speed out-and-back stages (five-tick tolerance).
- [ ] ROS controllers, planned arm motion, and drawing.

Keep the display launch useful for viewing a model without motor power. A
separate hardware launch must own the real `/joint_states` source; never run
virtual sliders and hardware state publishers together on the same robot.

## 1. Record the actual robot and choose the first drawing milestone

- [ ] Collect the latest CAD/assembly, joint order and axes, link dimensions,
  masses/inertias, travel limits, servo models and IDs, bus voltage, firmware or
  protocol details, and driver board connection. Record measured values and
  assumptions separately.
- [ ] Decide which pen mount is used for the first drawing: manual loading is
  enough for the first milestone in the design report. The planned seventh
  gripping servo can be integrated later without making the first drawing
  depend on it.
- [ ] Define frames and joint names once: `base_link`, each arm joint/link,
  `tool0`, `pen_tip`, and eventually camera, platform, and object frames. Record
  the pen axis and the physical tip offset after the pen is tightened.
- [ ] Agree on the deployed ROS 2 distribution and computer image. This
  workspace currently runs on Jazzy locally; the report proposes Humble on the
  Jetson. Keep package code compatible with the chosen deployment target.

**Exit:** A versioned hardware inventory and frame/joint convention that the
URDF, motor mapping, planning group, and calibration can all use.

## 2. Replace the reference display with the current arm model

- [ ] Add a **new custom arm description** in `pinchy_description`; retain the
  stock SO-101 as a clearly named reference. Model six revolute arm joints from
  the newest CAD, the actual pen holder, and fixed `tool0` and `pen_tip` frames.
- [ ] Add measured joint limits, link mass and inertia, visual meshes, and
  simpler collision geometry. Include the tabletop, fixture, and object in the
  planning scene later, rather than baking them into the moving arm URDF.
- [ ] Make the display launch select the custom description and verify each
  slider moves the expected physical axis in the expected direction. Check
  zero pose, link lengths, tip location, joint limits, and self-collision
  geometry against CAD and the assembled arm.
- [ ] Use the updated description for workspace, reach, and load calculations;
  re-evaluate the report's 0.50 m pen-tip reach and motor capacity targets.

**Exit:** RViz shows the actual six-joint mechanism and calibrated pen-tip frame.
The model's joint names and limits are ready for controllers and planning.

## 3. Bench-test the Feetech smart servos and stop behavior

See [servo setup and bench control](servo_bench.md) for ID/baud/mode setup,
calibration records, and the initial single-servo control tool. Servos run their
existing firmware; persistent setup and normal position commands are separate
operations. All six motors are labelled, power-cycle verified and individually
working per the user, including motor 6 passing on retry. The full unloaded
six-motor chain sequence passed; assembled-joint qualification remains pending.
See the [current point-motion status](point_motion_plan.md) and
[hardware record](hardware_inventory.md) for verified results.

- [x] Test each unloaded motor, assign/label unique IDs and verify persistence
  after power cycling. Verify the six-motor bus with individual and grouped
  movements and feedback. See the hardware record for parameters and results.
- [ ] Measure assembled-joint zero, direction, encoder conversion and usable
  ranges; record the final motor-to-joint mapping and calibration.
- [ ] On the assembled arm, start with small, slow movements. Check reported
  position against measured motion, unexpected reversals, limit behavior,
  heating, holding behavior, and response to lost communication.
- [ ] Verify software Stop (cancel motion and hold position) and the physical
  emergency stop (cut servo power) separately. Establish what happens on
  controller restart, disconnect, or loss of motor power, including how the
  arm is supported if it sags.
- [ ] Document the exact bench procedure and observed results as hardware
  becomes available; no automated motor test is required to begin this plan.

**Exit:** Every arm motor has a known ID, zero, direction, range, and feedback
behavior. Stop and power-loss behavior are understood before ROS motion control.

## 4. Bring real joint state and control into ROS 2

- [ ] Implement a `pinchy_hardware` interface (the report proposes
  `ros2_control`) for the Feetech serial bus. Convert radians to servo units
  using the calibration from step 3; read actual joint positions and relevant
  diagnostics. Never treat a sent goal as measured feedback.
- [ ] Add controller configuration in `pinchy_bringup`: a joint-state
  broadcaster and a trajectory controller for the six arm joints. Configure
  position/velocity limits, command timeout, and a defined stop/hold response.
- [ ] Add a **separate hardware bringup launch** for the driver, controllers,
  robot state publisher, and diagnostics. Keep the existing virtual display
  launch for model inspection. If a mock/fake hardware mode is useful, keep its
  selection explicit so it cannot be confused with live motors.
- [ ] Compare `/joint_states`, TF, and RViz with measured arm poses. Confirm
  the robot follows slow single-joint goals and rejects out-of-range goals.

**Exit:** ROS reports actual joint positions and accepts bounded, low-speed
joint trajectories through one documented command interface.

## 5. Implement deliberate arm movement

- [ ] Add a small joint-space motion tool or ROS node that can command a named
  safe pose and a specified six-joint target through the trajectory controller.
  It should validate names, units, limits, current state, and timing before
  sending a goal, then report completion or failure.
- [ ] Define homing or startup behavior from the measured current pose; do not
  assume the arm starts at URDF zero. Add a way to stop an active movement and
  hold position.
- [ ] Exercise a sequence of known, collision-free poses at low speed, first
  with fake hardware if available, then on the real arm. Compare commanded and
  measured joint paths and check for discontinuities.

**Exit:** An operator can move the real arm repeatably between known poses and
see whether each move succeeded. This is the foundation for automated paths.

## 6. Move through sets of points

- [ ] Define the input contract: joint waypoints versus Cartesian pen-tip poses,
  coordinate frame, units, orientation, ordering, speed, and whether the pen is
  up or down. Keep stroke boundaries; do not connect unrelated strokes.
- [ ] Add `pinchy_moveit_config` or an equivalent planner for the finalized
  six-joint URDF. Configure the arm planning group, `pen_tip` end effector,
  joint limits, collision geometry, IK, and the trajectory controller.
- [ ] For Cartesian points, solve IK using the previous valid joint solution as
  a seed, check every segment for joint limits and collisions, and reject jumps
  or unreachable points. Require the entire requested path, then use forward
  kinematics to check the planned pen-tip position and orientation between
  sampled points against a chosen geometric tolerance. Apply time
  parameterization within measured speed and acceleration limits before
  sending a trajectory; check the timed trajectory again if processing changes
  its geometry.
- [ ] Implement path execution that reports progress and failure, supports
  cancellation/hold, and checks measured joint feedback against the commanded
  trajectory. Set practical per-joint path tolerances and abort/hold when they
  are exceeded. Start with pen-up paths away from the object.

**Exit:** A point sequence can be previewed, checked, and executed continuously
without asking the low-level motor code to interpret drawing geometry.

## 7. Turn artwork into checked drawing paths

- [ ] Parse supported SVG paths, preserve each stroke and corners, and sample
  curves at a documented spacing or geometric tolerance. Validate file size,
  supported elements, scale, and path length before planning.
- [ ] Map 2D artwork to the known cylinder using surface arc length and height,
  then generate `pen_tip` poses with the pen normal to the surface. Keep the
  allowed roll about the pen axis available to the planner.
- [ ] Insert approach, contact, lift, travel, and withdrawal segments. The
  report suggests a provisional 10 mm outward lift; measure and adjust it for
  the actual holder and fixture. Check the entire path, including pen-up moves.
- [ ] Use the planned spring-loaded pen holder as the first contact method.
  Measure spring stiffness and usable travel, then choose a small compression
  that gives a repeatable normal force without bottoming out or damaging the
  pen/object. Calibrate the tip and contact offset with the pen loaded, and
  use slow initial drawing speeds. Servo position or reported load alone does
  not measure pen-tip force.
- [ ] Preview all paths and reject any job with unreachable points, joint jumps,
  or collisions before enabling Start. Dry-run with no object, then mark simple
  lines and separate strokes on a fixed cylinder. Compare measured ink lines
  against the planned curves; adjust calibration, sampling, speed, and spring
  preload based on the results. Add contact sensing or force control only if
  those measurements show passive compliance is insufficient.

**Exit:** A known cylinder at a known pose can receive a complete, single-color
SVG drawing with no unintended connecting marks.

## 8. Integrate object pose and close the user workflow

- [ ] Agree on the pose interface with the teammate implementing AprilTag
  detection: frame names, units, timestamp, pose uncertainty/quality, and
  what counts as a valid detection. They own tag detection and camera pose
  estimation; this software consumes and checks their output.
- [ ] Add the camera/platform/object frame chain and transform drawing poses
  into `base_link`. Reject missing, stale, or unreliable pose estimates before
  motion. Check fixture seating and pen-tip offset separately; tags alone
  cannot detect object slip. Revise the report's D455/platform geometry if
  the physical setup changes.
- [ ] Add a job interface for upload, scale/placement, preview, Start, status,
  Stop, and clear rejection reasons. Keep all preflight checks between preview
  and motion. Connect this to the team's browser UI when its interface is set.
- [ ] Integrate the optional gripper servo and pen pickup only after manual-pen
  drawing works; give it its own hardware mapping, model, controller, and tests.

**Exit:** An artist can submit and preview a drawing, place the known object,
and run or stop the checked job without manually entering its pose.

## 9. Verify the complete system and iterate

- [ ] Measure pen-tip positioning, camera registration, radial reach, and
  drawing accuracy on physical hardware. The report proposes separate 2 mm
  pen-tip and registration targets and a 2 mm maximum stroke-centerline
  deviation after translation/rotation alignment; retain or revise acceptance
  criteria explicitly with the team.
- [ ] Repeat drawings at several platform placements; test corners, curves,
  separate strokes, pen-up clearance, unreachable jobs, covered tags, Stop,
  emergency stop, and long/large jobs. Record failures and the configuration
  used for each run.
- [ ] Revisit URDF geometry, tool calibration, fixture, path sampling, motor
  limits, and contact mechanics from measured results. Consider a compliant
  pen holder if missing strokes show insufficient contact.

**Exit:** A reproducible end-to-end demonstration meets the team's current
measured requirements, with remaining limitations documented.

## Suggested repository layout as work begins

| Location | Responsibility |
| --- | --- |
| `pinchy_description` | Reference and custom URDFs, meshes, tool frames |
| `pinchy_bringup` | Display, fake-hardware, and real-hardware launches; controller configuration |
| `pinchy_hardware` | Servo bus interface, calibration mapping, feedback, diagnostics |
| `pinchy_moveit_config` | Planning groups, IK, collision setup, trajectory execution |
| Drawing package(s), names TBD | Joint motion tool, waypoint planner, SVG mapping, job execution |
| Perception/UI package(s), names TBD | Camera registration and browser job interface |

The report is a design baseline, not proof that any target is already met.
Update this roadmap when the actual arm, deployment OS, and software interfaces
are settled.
