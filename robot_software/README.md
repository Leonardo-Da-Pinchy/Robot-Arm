# Robot software

ROS 2 workspace and servo bench tools for the Leonardo Da Pinchy robot arm.
Run colcon from this directory, not from the repository root.

The custom arm has **six arm joints driven by Feetech smart bus servos**, plus
a **seventh servo for the pen gripper**. This gives six arm degrees of freedom
and a separately actuated gripper. The bundled stock SO-101 reference has
five arm joints plus its gripper and serves as a separate reference model.

## Packages

- `src/pinchy_description`: self-contained stock SO-101 follower URDF and meshes.
- `src/pinchy_bringup`: launch files and RViz configuration.

The display launch provides a kinematic view of the reference arm with virtual
joint sliders. It publishes synthetic joint states for visualization and does
not command motors.

Standalone tools in `tools/` provide single-servo bench commands. They default
to mock operation; physical connections require an explicit hardware option.

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

- [Servo setup and bench control](docs/servo_bench.md): connection and command usage.
- [Hardware inventory](docs/hardware_inventory.md): equipment, calibration records, and bench results.
- [Point-motion plan](docs/point_motion_plan.md): implementation assessment and ordered tasks.
- [Implementation roadmap](docs/implementation_roadmap.md): milestones through drawing integration.
- [Arm development notes](docs/arm_development.md): modeling context and conventions.

## References

- [ROS 2 package layout](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS target platforms](https://www.ros.org/reps/rep-2000.html)
- [Reference model provenance](src/pinchy_description/THIRD_PARTY.md)
