# Robot software

ROS 2 workspace and servo bench tools for the Leonardo Da Pinchy robot arm.
Run colcon from this directory, not from the repository root.

The custom arm reuses the SO-101's six motor locations and axes. The original
five arm joints remain, and the former gripper motor becomes `pen_pitch`,
the sixth arm joint driving the replacement pen end effector. The pen is held
mechanically; no separate seventh gripping servo is assumed.

## Packages

- `src/pinchy_description`: stock reference, derived six-axis arm model, meshes and source EE CAD.
- `src/pinchy_bringup`: visualization, mock/hardware control launches and joint-target UI.
- `src/pinchy_hardware`: calibrated Feetech STS3215 ros2_control hardware adapter.

The display launch provides a kinematic view of the reference arm with virtual
joint sliders. It publishes synthetic joint states for visualization and does
not command motors.

Standalone tools in `tools/` provide single-servo and chained-bus bench commands.
They default to mock operation; physical connections require an explicit hardware option.

## Prepare a robot terminal

Run [connect.sh](connect.sh) from this directory:

```bash
./connect.sh
```

It finds the USB adapter, requests sudo if port permissions need fixing,
prepares the servo virtual environment, builds an unbuilt workspace, sources
ROS and `install/setup.bash`, and opens a shell with `servo`, `chain`,
`calibrate` and `arm` commands. The helpers select the servo venv automatically;
ROS retains its system Python. Setup itself sends no motor requests.

Use `source ./connect.sh` to prepare the current Bash terminal instead.
All normal `ros2` commands remain available. To start physical control through
the ROS CLI after terminal setup, run:

```bash
ros2 run pinchy_bringup start_arm
```

This runs the interactive zero/hold workflow and then starts ROS control,
RViz and sliders; it does not require manually confirming calibration.
`./connect.sh -- arm` prepares everything and starts the interactive physical
zero/hold and RViz workflow. USB and external motor power must be connected
for physical motor commands. Setup checks USB access; motor health is checked
by the control tools.

For ROS-only/mock work without an adapter, use `./connect.sh --no-port --domain 0`.
Use `--build` to rebuild an existing workspace or `--help` for all options.

## Build and display

Use a ROS 2 environment with colcon and rosdep installed. Example below uses
Jazzy on Ubuntu 24.04; choose the ROS distribution matching the deployed OS,
including the Jetson image. No distribution is hard-coded in the packages.

```bash
source /opt/ros/jazzy/setup.bash
cd "<path-to-Robot-Arm>/robot_software"
# If rosdep has not been initialized on this machine: sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch pinchy_bringup display.launch.py
```

On a terminal without a graphical display:

```bash
ros2 launch pinchy_bringup display.launch.py gui:=false rviz:=false
# In another terminal, source ROS and this workspace, then inspect:
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link gripper_frame_link
```

Joint sliders publish synthetic `/joint_states`; `robot_state_publisher` publishes
`/robot_description`, `/tf`, and `/tf_static`. GUI display requires a desktop or
working WSLg/X forwarding. Stop the launch before connecting a hardware joint
state source to avoid publishing conflicting states.

## Project documentation

- [Assembled-arm calibration](docs/arm_calibration.md): guided per-joint zero capture, direction checks and saved progress.

- [Loose-motor ROS bench](docs/ros_motor_bench.md): physical controller tests before assembly.

- [CAD 4 model and joint limits](docs/cad4_model.md): source layout, nominal pen tip, geometry and collision limitations.
- [Mock control and sliders](docs/mock_control.md): build, RViz, joint trajectories and tests.
- [Hardware control](docs/hardware_control.md): calibration, physical bringup and stop behavior.
- [Control handoff](docs/control_handoff.md): implementation results and physical checks.

- [Servo setup and bench control](docs/servo_bench.md): connection and command usage.
- [Hardware inventory](docs/hardware_inventory.md): equipment, calibration records, and bench results.
- [Point-motion plan](docs/point_motion_plan.md): implementation assessment and ordered tasks.
- [Implementation roadmap](docs/implementation_roadmap.md): milestones through drawing integration.
- [Arm development notes](docs/arm_development.md): modeling context and conventions.

## References

- [ROS 2 package layout](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS target platforms](https://www.ros.org/reps/rep-2000.html)
- [Reference model provenance](src/pinchy_description/THIRD_PARTY.md)
