# Replacement end effector, V1

These six STEP files were supplied by the project owner and moved here from
`robot_software/EE_STEP_V1` without modifying their contents. They are individual
SolidWorks STEP AP203 part exports, with length units in millimetres. No assembly
mates or inter-part joint relationships are included.

Converted binary STL files are in `../../meshes/ee_v1/`, tessellated at 0.08 mm
linear and 0.3 rad angular deflection. STL coordinates remain in millimetres;
URDF consumers must use `scale="0.001 0.001 0.001"`. To regenerate, use
`robot_software/tools/convert_ee_step.py` in an environment with
`cadquery-ocp==7.9.3.1.1` and its pinned VTK dependency.

`ros2 launch pinchy_bringup ee_parts.launch.py` shows a separated part gallery.
Gallery offsets are for inspection and do not describe an assembly.

Before mounting these meshes into the arm URDF, confirm:

1. Which components are rigidly attached to the servo housing, and which rotate
   with the former gripper output shaft.
2. Each component's assembled pose relative to that shaft or housing, including
   shim choice/count. An assembled STEP export with a known mounting frame is useful.
3. The pen axis and physical tip pose for the actual installed pen.
4. EE mass/centre of mass if load analysis or dynamics is needed.

The current runtime arm now follows the newer `../cad4/` source bundle, with
joint six named `pen_pitch`, a nominal pen tip, and source geometry proxies.
These earlier loose STEP parts remain available for detailed CAD inspection;
they are not a source of guessed assembly transforms. See
`robot_software/docs/cad4_model.md` for the current integration.
