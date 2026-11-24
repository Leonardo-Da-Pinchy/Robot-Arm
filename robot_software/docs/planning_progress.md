# Planning progress in the Cartesian window

After **Plan / preview** or **Plan queued path**, the status area now shows:

- Elapsed planning time and the current stage.
- Completed/total path samples, timing intervals or collision checks when known.
- A progress bar and percentage **for that stage**.
- Approximate time left **in that stage**, after enough samples establish a rate.
- The configuration attempt, out of the maximum eight, when whole-stroke start
  selection is active. Optimizer evaluations are counted against their maximum;
  the optimizer can finish earlier.

Counts describe internal samples/checks and can exceed the number of loaded
points: a 41-point stroke can require thousands of collision checks.

Example:

```text
Planning · 1m 32s elapsed · overall ETA unknown
Checking complete path collisions — 1,204/4,200 collision checks (29% of this stage)
Configuration attempt 1/8 maximum · about 18s left in this stage
```

IK can retry alternative configurations, optimization can finish early, and a
candidate can fail timing/collisions before another is tried. Consequently the
total work is not known in advance. Search stages use an indeterminate bar;
there is no invented whole-job percentage or countdown. A stage reaching 100%
can be followed by another required check. The final planned trajectory's
reported duration is **execution time**, separate from time spent planning.

Full plane and cylinder point files were tested through the actual panel and
mock planning service. Live updates continued through about 160 and 213 seconds
respectively before completion in that concurrent Jetson run; both produced a
checked preview. Earlier runs varied substantially. These examples can take
minutes, especially when selecting another starting configuration. This change
adds visibility; it does not remove or thin safety/geometry/speed checks.

Stop/hold and safety messages take priority over planning progress. Old request
IDs and stale status cannot supply a new request's percentage. The bar hides on
completion/failure. Execution retains its separate execution-progress messages.
No physical motors were moved to develop or test this display.

## Load the update

Build `pinchy_motion`, then reopen the planning server and Cartesian panel
together using the normal planning workflow. Running processes retain their old
Python code; rebuilding alone does not update an already open window/server.
No ROS interface rebuild is required for this change. Do not automatically
restart the physical hardware driver to update a planning display.

An older server can still be used with the new panel, but only local elapsed
time and “waiting for progress” are available until the server is also updated.

## Status topic and tests

`/pinchy_motion/status` gains a read-only `planning_progress` JSON object:
`active`, `request_id`, `operation`, `phase`, `elapsed_s`, `stage_elapsed_s`,
`completed`, `total`, `unit`, `stage_remaining_s`, and optional search/pose/section
context. Unknown totals/remaining times are null; inactive reports contain
`active: false`. Existing fields and planning/execution services are unchanged.
Telemetry uses a small thread-safe snapshot; it publishes on the existing status
timer, not once per IK or controller sample. Motor/control writes are unchanged.

With ROS/workspace sourced, reproduce the plan-only UI checks:

```bash
PINCHY_TEST_DOMAIN=213 OPENBLAS_NUM_THREADS=1 \
  python3 tools/check_planning_progress.py --surface plane
PINCHY_TEST_DOMAIN=214 OPENBLAS_NUM_THREADS=1 \
  python3 tools/check_planning_progress.py --surface cylinder
```

These start isolated mocks, load the saved point sets into the panel, observe
live progress and require a successful preview. They send no execution request.
See [validation](validation/moveit/planning_progress/validation.md).
