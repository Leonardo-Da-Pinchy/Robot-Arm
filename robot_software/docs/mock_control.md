# Mock joint control and sliders

This uses the SO-101 motor axes, including its former gripper axis as
`pen_pitch`. Six position interfaces feed the trajectory controller and
`/joint_states`. Mock hardware mirrors commanded positions; it does not simulate
motor inertia, loads, contact, supply limits or tracking error.

## Build once

The two missing controller packages were installed on the Jetson. On a fresh
Jazzy installation:

```bash
sudo apt install ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller
```

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## RViz and joint sliders

Stop the old `display.launch.py` first; run one robot state source at a time.

```bash
ros2 launch pinchy_bringup mock_control.launch.py rviz:=true sliders:=true
```

The sliders open in a **separate window**, titled **Pinchy MOCK motor targets**,
alongside RViz. Set slider targets in radians and press **Send**. The **Actual** column comes
from `/joint_states`; RViz follows those returned positions. **Copy actual to
targets** stages the measured pose. **Cancel / hold** cancels the active action.
Closing the slider window during a move requests cancellation and waits for
its result before allowing closure. No slider command is sent automatically.

The controller interpolates a timed trajectory. The GUI extends the requested
duration to respect both its 0.2 rad/s cap and any lower calibrated model
velocity. Real hardware independently enforces its calibrated velocity bound.

The model now uses the supplied CAD 4 transforms and `pen_pitch` joint, with
pen-holder/pen visual proxies and a nominal `pen_tip` at 5 mm spring compression.
The old claw is removed. `tool0` remains at the sixth-axis origin. See the
[CAD 4 model notes](cad4_model.md) for joint ranges and geometry provenance.
Sliders enforce joint ranges; they do not check self-collision or environment
collision. Intersections can occur at otherwise valid joint angles.

## Headless commands

```bash
ros2 launch pinchy_bringup mock_control.launch.py
```

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run pinchy_bringup mock_trajectory.py \
  --point 0.2 -0.2 0.1 0 0.1 0.2 \
  --point 0 0 0 0 0 0 --seconds 3
```

Point order: `shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll pen_pitch`.

| Flag | Meaning |
| --- | --- |
| `--point` | Six joint angles in radians; repeat for ordered joint waypoints |
| `--seconds` | Time between consecutive waypoints, default 3 s |
| `--timeout` | Execution deadline, default 30 s; requests cancellation on expiry |
| `--cancel-after` | Request cancellation this many seconds after acceptance |

The small CLI permits demo ranges [-1, 1] rad for the first five joints and
[0, 1] rad for joint six, and checks the model uses mock hardware. It prints
actual feedback and the final action result. Ctrl+C requests cancellation.
These are joint waypoints; Cartesian coordinates and IK come with MoveIt later.

## Tests

```bash
colcon test --packages-select pinchy_hardware
colcon test-result --verbose
python3 tools/test_arm_model.py
python3 tools/test_ros_control.py
```

The ROS integration script creates isolated domains and pseudo-terminals. It
tests waypoint execution, feedback, cancellation, rejection and the sliders
against both mock hardware and the actual Feetech plugin on an emulated bus.
It also checks CLI timeout cancellation. It never opens the physical servo port.
Its logs and emulation reports are printed as `/tmp/pinchy-ros-tests-*` paths.

To review supplied CAD parts separately:

```bash
ros2 launch pinchy_bringup ee_parts.launch.py
```

That is a gallery of the earlier individual STEP parts. The control model uses
the newer CAD 4 source geometry proxies. See [CAD 4 model notes](cad4_model.md)
and [hardware control](hardware_control.md).
