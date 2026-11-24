# Bounded TCP speed validation

November 22, 2026. This is software/controller-reference qualification on
isolated mocks and a generated PTY servo emulator, never physical motors.
See [usage and timing contract](../../../bounded_speed_proposal.md).

## Complete larger-cylinder runs

Example `08_cylinder_tcp_speed_range.json` retains the 76.2 mm surface oval,
normal-to-cylinder direction, 5 mm air gap and explicit pen-up approach. Selected
joint limits are 30 deg/s / 180 deg/s², below the new global ceilings. Requested
TCP range is 4–18 mm/s with 80 mm/s² tangential acceleration.

| Check | Mock | Real driver / PTY emulator |
| --- | --- | --- |
| Full plan and execution | Pass | Pass |
| Interior input waypoints checked | 59 | 59 |
| Live interior reference speed | 4.831–17.925 mm/s | 4.785–17.929 mm/s |
| Live peak tangential acceleration | 38.40 mm/s² | 38.40 mm/s² |
| Maximum normal error | 0.030° | 0.030° |
| Execute to moving reference | 0.218 s | 0.239 s |
| Reuse of consumed preview | Rejected | Rejected |

Each run has 956 live cruise samples; endpoints alone use the declared ramps.
Offline trajectory sampling also stays in band (4.138–17.927 mm/s).
Planning took about 89–91 seconds including whole-stroke IK/configuration,
collision services and approach; complete trajectories take about 24 seconds.
The additional range solver took 0.097–0.099 seconds. These timings are measured
examples, not latency guarantees.

Reports and compressed controller traces/launch logs are in `mock/` and `pty/`.
PTY also includes the emulator report. Top-level execution logs retain planning
and completion messages. `source_hashes.json` identifies the validated code,
model, profile and example; `panel_range.png` shows the panel controls.

Reproduce in the sourced ROS workspace (these tools create their own isolated
mock or generated-PTY stack):

```bash
PINCHY_TEST_DOMAIN=227 OPENBLAS_NUM_THREADS=1 python3 tools/qualify_continuous_execution.py \
  --points src/pinchy_motion/examples/point_sets/08_cylinder_tcp_speed_range.json \
  --scene src/pinchy_moveit_config/config/fast_cylinder_pedestal_dry_run.yaml
PINCHY_TEST_DOMAIN=228 OPENBLAS_NUM_THREADS=1 python3 tools/qualify_continuous_execution.py \
  --emulated \
  --points src/pinchy_motion/examples/point_sets/08_cylinder_tcp_speed_range.json \
  --scene src/pinchy_moveit_config/config/fast_cylinder_pedestal_dry_run.yaml
```

## Regression and timing evidence

- 77 focused unit tests pass: range timing, fixed-path compatibility, file
  validation, UI, progress, previews, transport, model and tracking helpers.
- 12 additional panel/file tests pass after the final UI label change.
- 29 startup tests and 30 C++ hardware cases pass (31 CTest results including
  the wrapper), retaining startup ramp and driver bounds.
- Six gateway checks pass: range preview/execute/report, invalid requests with
  no motion, fresh-scene rejection, cancel/hold and two legacy regressions.
- Five real-driver/PTY safety checks pass, covering execution, scene changes,
  recovery, cancellation and thermal fault behavior.
- Numerical tests include rescue of a joint-rate-limited constant-speed
  candidate, rejection of an impossible minimum, endpoint/contact permissions,
  interpolation bounds, cancellation, smooth joins and resource limits.
- The smaller cylinder also passes complete mock planning/execution at
  3–12 mm/s: all 39 interior waypoints, live reference 3.282–11.944 mm/s,
  0.119° maximum normal error and consumed-preview rejection. Full evidence
  and its explicit point set are in `small_mock/`; the independent selected
  branch numerical check is in `small_numerical_report.json`.

The initial broad inherited gateway run exposed two state-sensitive legacy test
assumptions about a rounded current pose and zero start. Tests now check exact
FK duplicates and explicitly establish the isolated mock start; affected cases
and the four new gateway cases pass together. This was a test-fixture correction,
not a relaxation of production acceptance.

`timing_benchmark.json` compares full timing/validation on one cached geometric
branch: fixed 7 mm/s took 9.32 seconds versus range 4–18 mm/s taking 8.13 seconds;
the range LP itself took 0.053 seconds. Different durations/sample counts make
this a measured example, not a general performance comparison. It confirms that
range selection does not repeat IK at every possible speed. The reproducible
probe is `timing_benchmark.py`, updated to use the archived engineering geometric
branch in place of its original temporary path. Run it from the sourced
`robot_software` workspace with `OPENBLAS_NUM_THREADS=1 python3` (output goes to
`/tmp/range_timing_benchmark_reproduced.json`).

## Scope

The selected path keeps geometry and normals; joint polynomial extrema and
dense FK/speed/acceleration checks remain authoritative, followed by collision
validation. Stop, thermal confirmation, powered hold/fault recovery and the
single hardware writer are retained. Positive minimum speed is never silently
lowered. Existing files remain fixed-speed by default.

These tests do not establish full drawing coverage, physical TCP tracking or
contact line quality. The previously recorded physical halfway feedback pause
remains a separate unresolved investigation. No powered physical stack was
restarted. Use the next normal startup for model/driver limits and reopen
planner/panel together for the changed PlanPath interface.
