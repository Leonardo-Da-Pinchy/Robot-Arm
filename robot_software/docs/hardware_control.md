# Feetech ros2_control adapter

> Read the [current checkpoint handoff](checkpoint_handoff.md) first. The guided
> startup, 50 Hz timing and torque-preserving runtime safety latch supersede
> the older launch workflow, 20 Hz timing and fault-release descriptions below.

For separate motors, start with the [loose-motor ROS bench](ros_motor_bench.md).
It uses temporary encoder zeros and small ranges before assembled-arm calibration.

`pinchy_hardware/FeetechSystem` owns one shared 1 Mbaud TTL serial port and
reuses the official C++ Feetech SDK. The Python SDK remains the bench-tool
library; it does not itself implement ROS hardware lifecycle, joint units,
calibration or controller feedback interfaces.

The plugin has been developed for STS3215 model 777, position mode 0, resolution
1, with single-turn 0..4095 encoder positions. It never rewrites EEPROM or
changes IDs, mode, offset, resolution or firmware. Configure those with the
existing bench tools first. No encoder wrap is allowed.

## Required calibration

Use the [guided calibration tool](arm_calibration.md) to record CAD-aligned
zeros and manually verified directions one joint at a time. Its draft stays
unconfirmed until conservative limits and physical checks are reviewed.

Copy `src/pinchy_bringup/config/servo_calibration.example.yaml` to your own file.
Its IDs, zeros, directions, limits and velocities are **examples**, not measured
arm calibration. Do not mark them confirmed merely to bypass the check.

For each joint, establish the physical servo ID, encoder reading at URDF zero,
positive direction, usable encoder bounds, mechanical radian limits and tested
maximum joint speed. Keep the configured ranges inside the URDF ranges.

```text
joint_radians = direction × (ticks − zero_tick) × 2π / 4096
```

`direction` is +1 or -1. No hidden wrap, gearing or EEPROM-offset adjustment is
applied: the calibration uses the encoder values returned by the configured
servo. If the mechanism has gearing, the current direct-drive conversion must
be extended first. The launch supports all six named joints on the shared bus.

## Supervised first launch

Support the mechanism, provide a reachable motor-power cut, stop all bench
scripts, and start with conservative measured limits. Inspect the reviewed
calibration file; only then set `calibration_confirmed: true`.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Robot-Arm/robot_software
source install/setup.bash
ros2 launch pinchy_bringup hardware_control.launch.py \
  calibration:=/absolute/path/to/reviewed_servo_calibration.yaml \
  motion_enabled:=true
```

Configuration reads ID/model/mode/resolution, position, voltage, temperature,
alarm and torque state. It requires torque initially off. Activation stages
each **current measured position** as its goal and verifies it, applies the RAM
torque limit, then enables torque. It does not move to encoder 2048 or URDF zero.
Check first that activation holds the existing pose without a jump.

Use the standard controller action for trajectories. For the six-axis slider UI:

```bash
ros2 run pinchy_bringup joint_sliders.py --hardware
rviz2 -d src/pinchy_bringup/config/arm.rviz
```

Verify one small joint movement at a time before coordinated motion. The mock
CLI deliberately refuses hardware; use the explicit hardware sliders or
`/arm_controller/follow_joint_trajectory` with all six joint names.

## Feedback and stop behavior

- `read()` requests and validates actual servo replies. It does not substitute
  sent goals for measured positions. The cycle samples six encoders and health
  registers sequentially; readings are not simultaneous.
- `write()` checks all targets before sending one SDK sync-write packet with
  position, raw speed and acceleration. Broadcast writes have no per-motor ACK;
  subsequent reads and trajectory tolerances establish tracking.
- The launch requests 20 Hz. Its actual sustainability on the URT-2 must be
  measured under physical load; the emulated bus cannot qualify that rate.
  Each serial transaction has a bounded timeout, default 20 ms. Feedback age
  and long control periods are also checked; this is not a hard real-time driver.
- Faults include packet checksum/ID/length errors, missing replies, encoder or
  joint bounds, unexpected torque state, servo alarms, voltage outside 10..13 V,
  temperature >=60 C, stale feedback and excessive command steps/velocity.
- A fault stops command streaming, returns ERROR and attempts torque release
  on every configured servo. A disconnected bus cannot acknowledge release;
  logs identify unconfirmed releases. The arm may sag when torque releases.
  This software behavior is separate from an independent motor-power cut.
- Cancelling a trajectory holds position through the controller. Deactivating,
  shutting down or faulting the hardware attempts **torque off**, not powered
  hold. A killed process or power/USB failure cannot guarantee a stop.
- Runtime faults retain the existing powered hold and pause the controller.
  Correct the cause, then press **Stop / hold** in the sliders. It replaces the
  interrupted trajectory with the last accepted hardware target, then requests
  `/arm_safety/reset` (`std_srvs/Trigger`). Recovery requires four consecutive
  healthy feedback sweeps, torque on, fresh measurements, and matching hold
  commands. It discards the old move; use Send for the next move. An active fault
  stays blocked. No ROS restart is needed after a recoverable physical condition.
  Startup/configuration failures still need startup to be rerun.

The actual column shows position in radians and estimated speed/acceleration in
deg/s and deg/s². Estimates use a quadratic fit over about 0.25 s of encoder
samples, so they lag slightly and encoder quantization can cause noise.
**Motion graphs / recordings** opens planned position and speed against measured
motion, plus the controller command. Select a joint and optionally overlay the
last ten moves. **Save recordings CSV** exports these comparisons; recordings
remain in memory until the slider window closes (up to 10,000 samples per move).
The existing motion-monitor JSONL log also continues recording controller data.
Graph time begins when Send is pressed, so communication adds a small offset.

The driver currently stops on any temperature reading >=60 C. Unlike the bench
tool's logged multi-read confirmation policy, it does not dismiss an isolated
high value. Existing temperature-spike diagnostics remain relevant; do not run
a second serial reader alongside ros2_control. The threshold is a conservative
bench limit, not a claim about physical motor temperature.

## What remains to qualify on the actual robot

1. Joint-ID mapping, encoder zeros/directions and physical limits.
2. One supported/unloaded joint: startup hold, small motion, feedback, cancel,
   torque release and measured bus cycle timing; then repeat for each joint.
3. Supported six-joint arm: coordinated slow moves, tracking tolerances, power
   capacity, loaded hold, temperatures and servo speed/acceleration interactions.
4. Motor-power cut, cable disconnect, process stop/restart and power restoration.
5. Verify CAD 4 geometry/collision proxies, installed pen-tip offset/compression,
   measured joint ranges and MoveIt configuration before Cartesian execution.

The automated pseudo-terminal tests validate the software protocol and fault
handling; physical qualification remains pending. See [the handoff](control_handoff.md).

The current model is described in [CAD 4 model notes](cad4_model.md). Motor six
now uses `pen_pitch`, with a CAD soft range of -3..100 degrees. Review its
pen-forward zero and limits; old `tool_rotation` calibration is not automatically
reused. The nominal CAD tip does not replace an installed-pen measurement.
