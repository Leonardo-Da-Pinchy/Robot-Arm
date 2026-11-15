# Reported pen-up queue failure replay

The owner reported the [panel screenshot](validation/feasibility/reported_pen_up_queue/reported_panel.png)
with a 6.0% Cartesian trace failure while held at zero. The matching live
[request events](validation/feasibility/reported_pen_up_queue/original_events.json)
show an explicitly queued `polyline`, `tool_axis` request, one waypoint at
(0.428247, 0.142794, 0.239279) m and `contact_segments=[false]`. A later queued
request at (0.359299, -0.073879, 0.164140) m failed continuation at 39.2%.
These were strict path-following failures, not free-space route failures.

## Fixes

- The panel queue now defaults to **Move between poses / pen-up routing**.
  Unspecified service/JSON path style defaults to `visits` for all-pen-up
  requests and `polyline` for contact requests. Explicit `polyline`/`spline`
  requests retain their geometry; there is no hidden detour fallback for drawing.
- Point and queued destination plans share bounded alternate IK-configuration
  route retries. A first valid endpoint with a blocked route is not the only
  configuration tried. Cancellation/model changes abort retries.
- Editing fields/queue/options clears stale planning results and execution
  tokens. **Update selected** replaces a selected queued row from the top fields
  while retaining its contact label. Queue editing is blocked during planning
  and execution so a returned preview cannot silently refer to a changed queue. The screenshot's upper fields and queued row contain different XYZs;
  the earlier queue failure must not continue to label newly edited fields.
- Requested direction constraints remain enforced. No calibration, tool offset,
  position limit, collision policy or powered/thermal safety behavior changed.

## Exact endpoint results

The [mock replay](validation/feasibility/reported_pen_up_queue/mock_replay.json)
and [real adapter on PTY replay](validation/feasibility/reported_pen_up_queue/pty_replay.json)
start from held zero and send previews only:

| Target (m) | Result |
| --- | --- |
| Upper fields: (0.370924, 0.076653, 0.239279), RPY (−129.153, −86.519, −86.050)° | Point and default queued route pass, preserving XYZ and the requested pen axis. |
| Later logged goal: (0.359299, −0.073879, 0.164140), exact logged quaternion | Point and default queued route pass, preserving XYZ and the requested pen axis. |
| Older queued row: (0.428247, 0.142794, 0.239279), exact logged quaternion | Explicit XYZ-only route passes. Requested-axis endpoint IK remains unresolved; it is rejected without a partial plan rather than misreported as a route failure. |

An [independent endpoint search](validation/feasibility/reported_pen_up_queue/independent_endpoint_search.json)
used bounded multi-start IK and differential evolution (seed 37, population
multiplier 12, 400 generations), followed by analytic-Jacobian least squares,
with saved bounds. For the older queued row both methods retain approximately
3.297 mm / 2.715° combined residual. This is a numerical result, not a physical
error measurement or proof of impossibility. The other two targets solve to
numerical tolerance. Reaching XYZ alone does not validate its requested axis.
No automatic orientation relaxation was introduced; the owner's pen-up
orientation preference was requested separately.

Both replay classes confirm preview causes no joint movement. HardwareCartesian
means the real driver on a PTY bus emulator, not the physical USB arm. The live
hardware application was not restarted and no physical goal was sent.

Reopen the pose panel to load its new default. Restart only the planning
application when ready to load server route-retry/default changes; no hardware
startup, zeroing or motor movement is required by this source change.


Validation passed: 11 feasibility/routing/cancellation tests, four panel tests,
exact target preview replays on saved-limit mock and the PTY adapter, and three
mock GUI/CLI, visits/spline and strict straight-path rejection regressions.
Python compilation and `git diff --check` passed. The independent search and
preview records are preserved above; no solver miss was relabeled as success.
