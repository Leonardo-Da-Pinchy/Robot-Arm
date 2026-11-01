# Feetech smart-servo setup and bench control

The report lists **STS3215-12V servos and a URT-2 adapter**. Confirm the labels
on the actual equipment before using this tool. The code targets the Feetech
STS/SMS position-control SDK, not every Feetech servo family.

## What programming the servos means

The servos already contain firmware and an internal position controller. Initial
setup normally means configuring that firmware, then sending goal positions
from the computer. No custom firmware flashing is planned.

Using the manufacturer's configuration software and model-specific manual:

1. Connect **one unloaded servo at a time** to the adapter, with the supply
   appropriate to its exact voltage variant. USB alone does not supply motor
   power. Check adapter wiring and TTL port against its manual.
2. Discover the servo at its current baud rate and read its model, ID, operating
   mode, and existing limits. STS commonly uses 1,000,000 baud; verify rather
   than assuming it. Save the original settings.
3. Assign a unique ID and label the servo. The report proposes IDs 1–6 from
   base to wrist; confirm this mapping with the actual assembly. Reconnect and
   verify each ID before daisy-chaining. Keep the same bus baud rate.
4. Configure position mode and appropriate travel limits. Measure each joint's
   zero and direction; record calibration in software. Do not overwrite the
   servo's stored encoder offset until the team's calibration convention is set.
5. With the servo unloaded, establish a current-position goal before enabling
   torque, using the manufacturer's supported procedure. An old goal can cause
   motion when torque is enabled. Then perform the small motion check below.
6. Record model, ID, baud, mode, limits, zero, direction, and observed behavior.
   Repeat for every servo. Change persistent settings only during deliberate
   setup, not in a normal movement loop.

## Run the bench tool

From `robot_software`, mock operation needs only Python:

```bash
python3 tools/servo_bench.py read
python3 tools/servo_bench.py move --delta 20 --min 2000 --max 2100
```

Every mock invocation starts at tick 2048; it does not model dynamics. Real
operation requires the manufacturer's SDK. Use a local virtual environment:

```bash
python3 -m venv .venv-servo
.venv-servo/bin/pip install ftservo-python-sdk pyserial
.venv-servo/bin/python tools/servo_bench.py --hardware --port /dev/serial/by-id/YOUR_ADAPTER --id 1 read
```

Use the read position and independently checked clearance to choose a small
allowed interval. The numbers below are an **example only**, appropriate only
if the measured position is near 2048 and that range is mechanically clear:

```bash
.venv-servo/bin/python tools/servo_bench.py --hardware --port /dev/serial/by-id/YOUR_ADAPTER --id 1 move --delta 20 --min 2000 --max 2100
```

The tool requires position mode and torque already enabled. It allows at most
64 ticks per command, checks feedback, and attempts a hold after timeout,
interruption, or motion failure. Speed and acceleration are raw SDK values;
the bounds are initial bench restrictions, not measured arm operating limits.
It makes no persistent ID, baud, offset, limit, or firmware changes.

`hold` sets the goal to measured position; `release` disables torque. Support
the arm before releasing torque. Neither software command substitutes for the
physical motor-power emergency stop. A failed serial connection can prevent a
hold command; closing the program does not itself disable torque or stop a
servo's internally running command. Do not run multiple bus programs together.

## Connection to ROS and RViz

This is the first bench-control layer, not yet the assembled-arm controller.
RViz joint sliders publish synthetic `/joint_states`; they are not motor
commands. The next layer is a calibrated Feetech hardware interface behind
`ros2_control`, with measured joint positions published back to RViz. A motion
node or MoveIt will command trajectories through that controller. A future
manual control panel should use that same command interface.

Real motor operation is unverified until the correct hardware is connected.
Bench checks should establish feedback, small motion, hold/release behavior,
disconnect behavior, and emergency power cut before moving the assembled arm.

## Manufacturer references

- [Feetech Python SDK and installation](https://github.com/ftservo/FTServo_Python)
- [STS/SMS SDK implementation](https://github.com/ftservo/FTServo_Python/blob/main/scservo_sdk/sms_sts.py)
- [Feetech setup tutorial](https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf)
