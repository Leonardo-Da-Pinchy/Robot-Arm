# Arm development notes

> Current status and latest tool-frame decisions are in the
> [checkpoint handoff](checkpoint_handoff.md); the development history below
> includes earlier assumptions and outstanding-work descriptions.

## Project context

Based on `Proposal Presentation - Leonardo Da Pinchy.pptx` and
`Design Presentation - Leonardo Da Pinchy.pptx` in the parent capstone directory.
The design deck marks slides 12 onward as old material; slides 2–10 describe the
current approach.

The project transfers SVG line artwork onto a known curved object, drawing only
on its reachable front region. The design targets a 0.4 m cube working area,
at least 0.50 m reach, and at most 2 mm end-effector and camera pose errors.
Those are project targets, not properties verified for the SO-101 reference.
The proposed stack uses a Jetson Orin Nano, smart serial bus servos,
ros2_control, MoveIt 2, and RViz. Camera/AprilTag alignment, SVG surface mapping,
and browser UI remain outside this starter workspace's scope.

## Confirmed custom arm hardware

The owner confirmed on November 8 that all six motor positions and axes match
the SO-101. The former gripper output becomes joint six (`pen_pitch`);
there is no separate seventh gripping motor in the current design. A sleeve
and thumbscrew hold the pen mechanically.

The original `so101_reference.urdf` is preserved. The current `pinchy_arm.urdf`
is generated from the supplied CAD 4 URDF and matching MJCF source. It includes
all six joint transforms, nominal `pen_tip`, source geometry proxies and inertias.
Joint six is named `pen_pitch`. The new source is archived under
`src/pinchy_description/cad/cad4/`; see [CAD 4 model notes](cad4_model.md).

The nominal pen tip assumes 5 mm spring compression. Installed pen geometry,
encoder zero/sign conventions and physical travel limits remain to be checked.
`tool0` stays at the sixth-axis origin. Joint limits constrain individual angles;
they do not provide collision avoidance. The current slider/mock workflow has
no collision-aware planner. `pinchy_mock.urdf.xacro` provides explicit mock
position control; separate hardware bringup requires reviewed calibration.

## Suggested next packages

1. Extend `pinchy_description` with the actual derivative model, pen mount,
   measured joint limits, collision geometry, and tip transform.
2. Add `pinchy_hardware` for the serial servo ros2_control hardware interface
   once motor IDs, serial protocol, calibration, and stop behavior are defined.
3. Add controller configuration and a separate hardware launch to
   `pinchy_bringup`; verify feedback before accepting motion commands.
4. Generate `pinchy_moveit_config` from the finalized arm model for planning,
   collision checking, and trajectory execution.
5. Add drawing path execution once surface coordinates and pen frames agree.

The display launch is a kinematic preview, with no servo communication or force
control. Keep it independent of future hardware bringup.
