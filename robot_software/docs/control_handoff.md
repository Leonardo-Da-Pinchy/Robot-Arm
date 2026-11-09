# Robot control handoff — November 8, 2026

> Current agent starting point: [checkpoint handoff](checkpoint_handoff.md),
> updated November 9. Historical status and fault-release descriptions below
> are superseded where they conflict with that handoff and the current code.

**Step 10:** mock ros2_control pipeline and target sliders implemented.
**Step 11:** hardware adapter implemented and now physically exercised: the user
confirmed all six loose motors passed the ROS out-and-back bench sequence.
Assembled-arm calibration, load tests and stop/recovery qualification remain
pending. Earlier validation records below describe the software implementation
stage; the physical pass is recorded in the hardware inventory.

## First thing to try after reboot

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ros2 launch pinchy_bringup mock_control.launch.py rviz:=true sliders:=true
```

The normal workspace is built. The missing joint-state broadcaster and joint
trajectory controller packages were installed by the user and verified.
No servo power or USB connection is required for this mock workflow.

Move a slider and press **Send**. Its target is sent through the trajectory
controller; the **Actual** value and RViz use returned `/joint_states`.
Try **Cancel / hold** during a move. The GUI stages targets, rather than
overwriting measured joint states or moving automatically when it opens.
The seventh-servo assumption has been removed: the former gripper motor is the
sixth arm axis, `pen_pitch`.

## What changed

| Files | Purpose |
| --- | --- |
| `src/pinchy_description/urdf/pinchy_arm.urdf` | Derives all six motor transforms from the SO-101; replaces gripper joint naming, removes old claw and old tip frame |
| `urdf/pinchy_mock.urdf.xacro` in that package | Adds the six explicit mock position interfaces |
| `cad/EE_STEP_V1/` and `meshes/ee_v1/` in that package | Original six STEP files moved unchanged, plus converted STL meshes |
| `urdf/ee_parts_preview.urdf.xacro` in that package | Separate gallery of supplied parts; no invented assembly poses |
| `src/pinchy_bringup/config/arm_controllers.yaml` | Joint-state broadcaster and six-joint trajectory controller |
| `src/pinchy_bringup/launch/mock_control.launch.py` | Mock controller bringup, optional RViz and sliders |
| `src/pinchy_bringup/scripts/mock_trajectory.py` | Joint waypoint CLI with action result, timeout and cancellation |
| `src/pinchy_bringup/scripts/joint_sliders.py` | Target sliders, actual feedback, Send and Cancel; explicit hardware mode available |
| `src/pinchy_bringup/launch/hardware_control.launch.py` | Separate hardware launch requiring reviewed calibration and explicit enable |
| `src/pinchy_bringup/config/servo_calibration.example.yaml` | Unconfirmed template; IDs, zeros, directions and limits must be measured |
| `src/pinchy_hardware/` | C++ ros2_control plugin using the official MIT-licensed Feetech SDK, with calibrated conversion, port locking and fault handling |
| `tools/build_arm_model.py`, `tools/convert_ee_step.py` | Reproduce the derived arm model and converted meshes |
| `tools/test_arm_model.py`, `tools/test_ros_control.py`, `tools/servo_bus_emulator.py` | Model, controller, GUI and emulated-bus validation |

Package build/dependency files and overview links were updated. Progress and
results remain under `docs/`; the README stays a general overview. The original
`so101_reference.urdf` is preserved. No commit was created.

## Current model — CAD 4 update

The newer [CAD 4 model notes](cad4_model.md) supersede the earlier model state.
All six transforms and CAD ranges now come from the supplied URDF; joint six
is `pen_pitch`. Nominal `pen_tip` is included at 5 mm sleeve compression.
Visuals, collision proxies and inertias use the matching source MJCF, with a
reconstructed NoClaw mesh. The exact individual STEP assembly is not rendered;
the new source supplies a simplified pen-holder representation.

Review installed pen length/compression, motor-six zero, encoder signs and
physical stops. Joint-range enforcement works in the sliders, but there is no
self-collision/path checker. Restart the control launch to load this new model.
See [CAD 4 validation](validation/cad4/) for the independent 107-pose FK check,
geometry loading and current ROS integration results. Earlier validation
records below describe the original control implementation stage.

## Before real ROS motor movement

Read [hardware control](hardware_control.md). The adapter requires all six
mapped IDs and reviewed calibration. The example file remains deliberately
unconfirmed; changing the boolean alone does not calibrate the arm.

- Verify ID-to-joint mapping, encoder zero, positive direction, encoder bounds,
  mechanical radian limits and conservative speed for each axis.
- Support the mechanism and verify the motor-power cut. Activation holds the
  current measured pose; confirm no jump before requesting any movement.
- Start with one small joint move while the other targets hold their measured
  positions. Check feedback, direction, cancel/hold and torque release.
- Then qualify slow coordinated moves, loaded holding, temperatures, supply
  capacity, physical bus timing, disconnect/restart and power restoration.
- On driver fault/shutdown, torque release is attempted. The arm may sag, and
  loss of communication prevents confirmation. This is not an emergency stop.

The hardware launch has now been exercised on the physical USB adapter: the
user confirmed the full six-motor unloaded ROS bench sequence passed.
No bench tools should share the serial port with ros2_control.

## Ordered checkpoint list

| # | Task | Status |
| --- | --- | --- |
| 1 | Review design and software | ✅ Done |
| 2 | Identify electronics, Jetson and ROS | ✅ Done |
| 3 | Assign/label IDs, power-cycle, test individual motors | ✅ Done, confirmed previously |
| 4 | Two- and six-motor chain tests | ✅ Done for unloaded bench operation |
| 5 | Establish actual joint arrangement | 🟨 CAD 4 six-axis transforms and nominal pen-tip supplied; physical checks pending |
| 6 | Actual URDF, limits, pen tip and collision model | 🟨 CAD 4 model, nominal tip, inertias and geometry proxies integrated; physical calibration pending |
| 7 | Assemble, wire, support, emergency power cut | 🟨 Assembly/reconnection and ID order confirmed; support/base/power-cut checks remain |
| 8 | Calibrate zero/direction/limits and verify physical FK | 🟨 Zeros, directions and usable bounds recorded; starting-pose review and physical FK verification pending |
| 9 | Qualify load, supply, temperatures, stop/recovery | ⬜ Pending |
| 10 | Mock ros2_control, feedback, trajectory action and sliders | ✅ Implemented and tested |
| 11 | Feetech ros2_control adapter | ✅ Implemented; full six-motor physical unloaded bench pass confirmed by user |
| 12 | Supervised real ROS joint trajectories | 🟨 Loose-motor ROS out-and-back sequence passed for all six; assembled/calibrated-arm execution pending |
| 13 | MoveIt group, IK, collision scene and controller mapping | ⬜ Pending |
| 14 | One real Cartesian pen-tip pose | ⬜ Pending |
| 15 | Ordered poses and Cartesian paths with completion/cancel | ⬜ Pending — point-control checkpoint |

After that checkpoint: pen contact and object frames, SVG paths and pen-up
travel, camera/AprilTag registration, browser workflow, then drawing accuracy
and repeatability tests. Automatic pen gripping would be a separate future
mechanism; it is not required by the current sleeve/thumbscrew design.

## Validation and shutdown

Validation passed in the normal `robot_software/build` and `install` workspace:

- 67 existing servo bench tests.
- 4 model tests: six preserved axes, correct sixth-axis tool chain, mesh paths,
  and removal of the old claw/inertias.
- 17 C++ hardware/calibration tests on pseudo-terminals: startup from nonzero
  positions, six-motor synchronization, port ownership, conversion/bounds,
  checksum/ID/length errors, timeouts, alarms, health faults and stale feedback.
- 11 ROS integration tests: waypoint completion, measured joint feedback,
  cancellation and rejection on both mock and Feetech-emulated hardware,
  offscreen slider Send/Cancel including cancellation before goal acceptance,
  CLI success/timeout/cancel, and a hardware fault during an active trajectory.
- The fault check also confirmed controllers became inactive and rejected a
  subsequent goal. Emulation reports show all six torque values zero and no
  EEPROM writes after shutdown. These are simulated register values, not
  measurements from the physical motors.
- Both custom-arm and CAD-gallery URDFs parsed successfully; the installed
  gallery launch ran headlessly. Python compilation and `git diff --check` passed.

That is **99 test cases**. Colcon's result summary reports 18 for the hardware
package because it also counts its CTest wrapper around the 17 GoogleTest cases.
The GUI was tested offscreen; please review the desktop RViz appearance and TF
axes yourself. No claim of physical accuracy, load qualification or real motor
operation follows from these tests.

[Saved validation records](validation/2026-11-08-control/) include build output,
ROS integration results, hardware test XML and the final emulated register
report. The build emits a Jazzy API deprecation warning from the compatible
hardware-interface initialization method. Controller manager also emits error
messages during the deliberately injected fault and may emit shutdown
statistics warnings; the tested action/controller/torque results are recorded
above. No control or emulator processes were left running.

Shutdown: files and validation saved. The system accepted a clean power-off
scheduled for **November 8, 2026 at 15:52:28 EST**, as requested by the owner.
The scheduling command returned successfully; this records the accepted
schedule rather than claiming the future power-off was observed.
