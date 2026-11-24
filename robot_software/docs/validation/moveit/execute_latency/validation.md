# Execute latency and exact-preview validation reuse

November 17, 2026. Previously Execute repeated every dense trajectory collision
request after the same complete path had passed preview validation. On a long
loop this adds thousands of DDS/MoveIt requests before controller dispatch.
The full preview collision checks remain; Execute now reuses their result only
when the complete trajectory message, immutable model and scene fingerprints
match exactly. Missing/changed validation rejects and requires another preview.

Both pose and ordered-path previews are covered. Fresh actual-state collision,
scene/model/target/expiry, held/measured start, feedback/controller/safety checks
remain. Scene/readiness checks repeat before dispatch; Stop/cancel/model changes
block dispatch. Runtime scene changes, feedback loss and hardware faults still
use the existing powered hold protocol. Plans remain single-use and in-memory.
The panel immediately identifies pre-execution checks instead of leaving the
preview message visible while waiting.

Ten focused gateway/transport tests pass, including trajectory position,
velocity, timestamp and joint-order mutation; changed/missing model/scene
validation; expired/consumed previews; changed held/measured starts; scene or
safety changes during preflight; Stop and cancel before dispatch. The complete
focused path, point-file and panel suites plus nominal geometry checks total
48 passing tests. The final regression run also passes eight live integration
checks: mock execution/Stop/cancel and PTY execution/scene change/Stop/cancel/
temperature-hold recovery. Tests only create isolated mock or PTY bringup.

The long example-07 path was fully planned and executed on both complete
stacks: roughly 1,936 timed trajectory points, 59 interior input waypoints, 7 mm/s
cruise. Preflight-to-dispatch took **0.348 s mock / 0.278 s PTY**; request-to-first
moving controller reference took **0.382 s / 0.314 s**. The final short-path runs
also exercise an additional readiness check immediately after the last scene
request, so a slow scene response cannot conceal newly stale feedback or safety.
Full-loop timing predates only that extra readiness check; its source snapshot
is recorded separately from the final source hashes.

The original full-system evidence is in the
[fast cylinder validation](../../feasibility/fast_cylinder/validation.md), with
complete controller traces, scene, point-file hashes and driver-emulator report.
`long_path_dispatch.json` records the measured long-path timings. Unit and live
regression logs are archived here. No physical motor moved or live hardware
process was restarted, and no collision density, joint bound, speed cap,
thermal confirmation or powered fault behavior was relaxed.

See [the implementation and usage explanation](../../../execute_latency.md).
