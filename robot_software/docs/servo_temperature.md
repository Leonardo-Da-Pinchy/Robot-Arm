# Watch servo temperature and capture spikes

Commands below use the `chain` shortcut from [servo_chain.md](servo_chain.md),
run from `robot_software/`.

## Watch without moving motors

```bash
chain watch --ids 1 2 --interval 0.25 --log /tmp/servo-temps.jsonl
```

Shows timestamped temperature (°C), voltage and alarm status for both motors
about four times per second. **Ctrl+C stops the monitor.** It sends only read
requests; it does not enable torque or command movement. Add `--duration 30`
to stop after 30 seconds. High readings are displayed and logged without being
filtered out. A communication failure stops the monitor with an error.

## Watch during the chain movement test

**Stop the standalone monitor first.** Use the same motor bounds and movement
settings as your normal test, adding `--watch-temps` and `--log`:

```bash
chain test --motor 1:1000:3000 --motor 2:1000:3000 \
  --delta 256 --speed 300 --mixed-speeds 150 300 \
  --watch-temps --interval 0.25 --log /tmp/chain-motion.jsonl
```

This example requires starts/targets within 1000–3000 as in the chain guide.
The motion test prints temperatures while it reads feedback and still stops at
its temperature/alarm limits. `position` accepts the same monitoring flags.

**One program must own the serial port.** Running a watch script and a movement
script simultaneously can mix replies. Our tools now reject a second concurrent
owner; other serial applications must also be stopped. Integrated monitoring
uses the motion program's existing serial connection.

## Flags and log contents

- `--ids`: motor IDs for the read-only watcher; one to six IDs.
- `--interval`: watch sampling / live display interval, 0.05–10 s; default 0.25.
  During motion, feedback/logging retain the control loop's polling rate; this
  flag only throttles terminal display. High readings bypass display throttling.
- `--duration`: watch duration in seconds; default 0 runs until Ctrl+C.
- `--watch-temps`: show live values during `test` or `position`.
- `--log PATH`: append JSONL records with UTC time, session ID, movement phase,
  samples, errors, and raw SDK request/reply bytes. Reusing a path appends a new
  session. Grouped goal writes have no reply and are not part of that packet trace.

Communication is **request/response polling**, not an unsolicited temperature
stream. The program reads register 63 (temperature, °C); voltage is register 62
(tenths of a volt), and alarm flags are register 65. The motor returns binary
packets; the terminal/log formatting is generated on the Jetson.

The SDK checks reply IDs/checksums. Our wrapper additionally rejects incorrect
read-reply lengths instead of interpreting the wrong payload as a value. A high
temperature triggers an immediate direct recheck, a voltage/temperature block
read and a second alarm check. Motion may continue past that isolated spike only
if both temperature rechecks are below 60 °C, agree within 2 °C, and both alarm
checks are clear. Sustained/uncertain high readings and actual alarms still stop
motion. All original/repeat values are retained. This confirmation rule is based
on captured single-read spikes; their underlying cause remains unresolved.
