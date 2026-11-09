# Loose-motor ROS bench

For six **separate, unloaded motors** on the TTL chain, IDs 1–6. This tests
ros2_control before assembly. Keep shafts clear, connect USB and 12 V, and close
other serial bench/watch programs. ROS domain 42 separates this from mock control.

## 1. Capture temporary zeros — no movement

From `robot_software/`, with the hardware driver stopped:

```bash
.venv/bin/python tools/prepare_ros_bench.py --hardware \
  --output /tmp/pinchy-unloaded-ros-bench.json
```

This requires torque off and checks ID/model, mode, position, limits and health.
It sends no goal, torque or settings writes. Each measured encoder position
becomes temporary ROS zero, with at most 64 ticks (5.625°) travel each way,
narrowed by encoder and model bounds. Pen pitch's negative limit stays within
−3°. Direction +1 means increasing encoder ticks, not a calibrated arm direction.

| Flag | Meaning |
| --- | --- |
| `--hardware` | Physical reads; omit for mock, which cannot produce an enabled physical profile |
| `--port PATH` | Adapter path; defaults to our established USB by-ID path |
| `--ids A B C D E F` | Six distinct IDs in ROS joint order, default 1–6 |
| `--output PATH` | New profile file; existing files are never overwritten |

The tagged JSON profile is valid YAML. Its confirmation applies only to measured
bench values, **not assembled-arm calibration**. It requires the explicit bench
launch flag below. If shafts leave the captured window, or after reboot, stop
the driver and capture again. To reuse a filename, remove its old file after
stopping the driver.

## 2. Start the physical driver

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch pinchy_bringup hardware_control.launch.py \
  calibration:=/tmp/pinchy-unloaded-ros-bench.json \
  unloaded_bench:=true motion_enabled:=true
```

Wait for `arm_controller` to be activated. Startup enables holding at measured
positions; it does not home. All six motors hold, even when testing only one.
The profile uses raw speed 50, acceleration 5, RAM torque limit 300/1000 and
0.1 rad/s software velocity. It changes no EEPROM settings or IDs.

## 3. Test motor 1, then all six

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 run pinchy_bringup ros_bench_test.py --ids 1
```

Watch ID 1 move about **2.8° out and back**, while the others hold. Then run:

```bash
ROS_DOMAIN_ID=42 ros2 run pinchy_bringup ros_bench_test.py
```

This tests each motor in order: 32 ticks (2.8125°) out, verify, return, verify.
Each leg defaults to 3 seconds. The direction stays inside the captured range.
Every motor's feedback, including the ones holding, must settle within 5 ticks.
It stops at the first error and prints which outward/return phase failed.
A PASS verifies encoder feedback; also check the labelled physical motor matches
its printed ID. Only the explicit unloaded-bench robot description is accepted.

| Flag | Meaning |
| --- | --- |
| `--ids 1 3` | Test those captured IDs only; default all six |
| `--seconds N` | Time per outward/return leg, 1–20 seconds, default 3 |
| `--log PATH` | Append results/positions; default `/tmp/pinchy-ros-bench-test.jsonl` |

Ctrl+C in the test client requests cancel/hold. **Ctrl+C in Terminal 1 stops the
driver and attempts torque release for all six.** A completed test leaves the
driver holding until you stop it. It does not blindly send a return after faults.

## Optional sliders

In a sourced terminal with that driver running:

```bash
ROS_DOMAIN_ID=42 ros2 run pinchy_bringup joint_sliders.py --hardware
```

The window is labelled as the real unloaded bench, with narrow ranges. Joint
names are temporary labels for loose motors; RViz is not their physical placement.

The physical read-only capture passed during implementation: zero ticks were
2052, 2052, 2051, 2050, 2052, 2052. These are session-specific values. The user subsequently confirmed the full
six-motor physical ROS sequence passed, including individual outward/return
movements. This completes the loose-motor ROS control proof before assembly.

Validation: five profile-generation tests and four full emulated-bench tests
passed, including real Python-SDK capture over a pseudo-terminal, each of six
motors moving/returning through the C++ ROS driver, cancellation, rejected joint
sets and narrow-range sliders. Fourteen existing mock/hardware ROS regressions
also passed, including refusing the bench client on normal robot modes. The subsequent physical run was performed by the user, who confirmed all six
motors passed. Detailed physical timing/error measurements were not independently
reviewed; assembled-arm qualification remains separate.
