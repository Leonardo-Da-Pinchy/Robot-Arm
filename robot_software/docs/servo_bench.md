# Set up and test each arm motor

For **STS3215-12V + URT-2**, one unassembled, mechanically unloaded motor at a
time. Programming means assigning settings, not flashing firmware. Connect its
three-wire plug to **TTL**; external supply goes to **V1 (+12 V) / G**. Turn power
off before changing wires. USB can power parameter setup without supplying 12 V.

## 1. Prepare the Jetson

From `robot_software/` (dependencies are already installed on this Jetson):

```bash
python3 -m venv --without-pip .venv
python3 -m pip --python .venv/bin/python install -r tools/requirements-servo.txt
```

Set the adapter path shown by `ls /dev/serial/by-id/` and define a shortcut:

```bash
PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79078770-if00
servo() { .venv/bin/python tools/servo_setup.py --hardware --port "$PORT" "$@"; }
```

If opening the port gives “Permission denied,” run
`sudo setfacl -m u:"$USER":rw "$(readlink -f "$PORT")"` in your own terminal.
This permission lasts until the adapter is unplugged.

## 2. Find and inspect the connected motor

```bash
servo scan
servo --id 1 inspect
```

`scan` pings IDs 1–253 at 1,000,000 baud without writes. Use the returned ID for
`--id`. `inspect` reads settings, position, voltage and alarms without changes.
`voltage_raw` is tenths of a volt: **120 = 12.0 V; 51 = 5.1 V**. Save the original
inspection output before configuration, e.g. `servo --id 1 inspect > /tmp/motor-before.json`.

## 3. Assign an ID and position mode

```bash
servo --id 1 configure --new-id 1 --single-servo
```

Replace `--new-id 1` with the intended label **1–6**; `--id` is its *current* ID.
This persists the new ID and position mode 0, verifies readback and relocks EEPROM.
It skips unchanged settings and leaves baud, encoder offset, limits and gains
unchanged. Requires torque off; run `servo --id 1 release` if necessary.
Only one physical motor may be connected: duplicate IDs cannot reliably be
counted by scanning.

Power off USB **and** external power, reconnect, then verify using the new ID:

```bash
servo --id 1 inspect
```

Use the new ID in that command. Label the motor; power-cycle verification
establishes persistence.

## 4. Run a movement test

Place the motor securely with the shaft/horn clear. Choose a mechanically clear
interval from the **fresh position reading**. Example **only if near 4092**:

```bash
servo --id 1 test-motion --delta -20 --min 4000 --max 4095 --speed 100
```

- `--delta -20`: move **20 encoder ticks below the current position**, then return.
  Positive values move toward higher readings; negative values toward lower ones.
- `--min 4000`: lowest **absolute encoder reading** allowed during this test.
- `--max 4095`: highest **absolute encoder reading** allowed during this test.
  Both the start and target must fit this interval. These bounds do not change
  the servo's EEPROM limits, and widening them alone does not increase movement.

**4096 ticks = 360°**: 20 ticks ≈ 1.76°, 512 ticks = 45°, 1024 ticks = 90°.
`target = current position + delta`; these readings are not calibrated arm angles.

For a clearly visible **45° out-and-back** on the unloaded motor, first inspect
again. If the reading is near 4000 and the shaft/horn has clear travel, run:

```bash
servo --id 1 test-motion --delta -512 --min 3000 --max 4095 --speed 1000 --timeout 20
```

For example, start **4000** → target **3488** → return **4000**. The wider
3000–4095 interval accommodates that path. The negative direction avoids crossing
4095. The setup test accepts up to ±1024 ticks (90°) per leg; give larger moves
more time with `--timeout` (seconds per leg).

This sets a temporary 30% output limit, commands the current position before
explicit torque enable, moves by the requested delta, returns to the starting
position, and disables torque. Goal writes may themselves enable torque.
It checks feedback and attempts release on failure/Ctrl+C. A failed bus can
prevent release; cut motor power if needed. Use this only on an unloaded motor.

The test refuses non-position mode, torque already on, alarms, hot motors,
encoder wrap and servo-reported voltage outside **10.0–13.0 V**. A measured 12 V
at the board plus 5.1 V feedback remains a discrepancy to resolve before motion.
The RAM output limit lasts until changed or power-cycled; it is not calibrated N·m.

Observe the shaft moving both ways. Record ID, readings, direction and whether
motion/release worked in the [hardware inventory](hardware_inventory.md). Repeat
steps 2–4 for each motor individually; connect the full bus after assigning IDs.
Arm-joint zero/direction/range calibration happens after assembly, on the Jetson.

After assigning unique IDs and testing two motors individually, use the
[chained-motor test](servo_chain.md) for individual, together, opposite-direction
and different-speed movement on the shared bus.

## Flags and other tools

| Flag | Meaning |
| --- | --- |
| `--hardware --port PATH` | Use the real adapter; omit both for mock operation |
| `--id N` | Current ID, default 1; put before the command |
| `--baud N` | Host connection baud, default 1000000; does not reprogram baud |
| `--new-id N --single-servo` | `configure`: target ID and acknowledgement of one connected motor |
| `--delta N` | `test-motion`: signed move, ±1–1024 ticks (up to 90°) |
| `--min N --max N` | `test-motion`: allowed encoder interval, within 0–4095 |
| `--speed N` | Raw speed 1–1000; default 5 standard / 50 fine units; fine units require ≥50 |
| `--acceleration N` | Raw acceleration 1–10, default 1 |
| `--timeout N` | Seconds per movement leg, default 8, maximum 30 |

Speed units depend on the servo phase setting; these are raw SDK values, not
radians/second. With fine units (phase bit 2 set), **1000 ≈ 14.6 RPM ≈ 87.8°/s**.
The script cap is separate from the motor's documented **45 RPM no-load speed
at 12 V** (0.222 s per 60°, ±10%) for the 12 V / 30 kg·cm variant. Loaded speed
and short moves depend on load, acceleration and output limits.
Run `python3 tools/servo_setup.py COMMAND --help` for command help.
`servo_bench.py` supplies individual read/inspect/move/hold/release commands;
`test_servo_*.py` test software with fake motors and never connect to hardware.

[Manufacturer register reference](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)
· [SDK](https://github.com/ftservo/FTServo_Python)
· [URT-2 wiring diagram](https://akizukidenshi.com/goodsaffix/URT-2.pdf)
· [12 V motor datasheet](https://www.feetechrc.com/Data/feetechrc/upload/file/20240507/6385067068652648096680943.pdf)
