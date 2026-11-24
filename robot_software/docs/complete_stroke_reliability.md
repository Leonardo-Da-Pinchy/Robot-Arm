# Reliable complete-stroke planning

This is the detailed work plan for **step 12** of the canonical
[roadmap](implementation_roadmap.md), with dependencies on tool calibration
(step 13), updated geometry (13a), and verification (23–24). It is not a second
numbered product roadmap. Complete-stroke reliability remains open even though
the step-11 timing/execution infrastructure passes its documented dry-run tests.

## Current evidence — November 24, 2026

- The owner recaptured wrist flex at approximately **−93.96° to +91.23°**,
  retaining zero/direction. This confirms the saved update, not independent
  physical travel or accuracy qualification.
- With updated bounds, strict-normal full mock previews still reject the
  requested tabletop circle and first square side. Alternative circle branches
  reach approximately 47%; square-side attempts reach approximately 96%.
  Reachable endpoints do not establish a connected complete path.
- A fixed 5° pen tilt at the original height passes the full mock circle
  preview, including collision checks, approach and 4–18 mm/s timing. It retains
  only about **0.16° wrist-flex reserve**; no physical execution occurred.
- All four square sides have separate numerical geometry/timing witnesses
  within 5° tilt. Square collisions, lifts and inter-stroke approaches for those
  chosen orientations are not qualified.
- Raising only the paper by 1.5–2 inches worsens the nine sampled pose results
  at the same XY placement/tool geometry. If robot and paper are raised together,
  their relative height is unchanged. Real platform/base transforms still need
  measurement. Finite searches are not global impossibility proofs.

The [analysis evidence](validation/feasibility/stroke_reliability_2026_11_24/validation.md)
preserves these results. Original tabletop geometry/use is documented
[here](tabletop_paper_examples.md). Do not treat the old strict-normal files as
requalified by the tilted-circle analysis.

## Work sequence and completion criteria

1. **Confirm real geometry — steps 13 and 13a.** Measure active pen tip/axis,
   holder, base/feet, platform surface and object frames. Replace assumed tool
   offsets and fixture transforms with recorded measurements, and retain
   uncertainty for choosing operating reserve. Do not change calibrated joint
   bounds without physically supported capture.
2. **Define stroke requirements — step 12.** Record supported regions,
   position/normal tolerances, TCP speed band, endpoints, sharp-corner/lift rules
   and the required joint reserve. Preserve exact-normal defaults. The proposed
   optional normal cone remains **analysis only; implementation deferred**.
   Any future tolerance must be explicit and apply to the angle from the normal,
   with axial twist free, not independent Euler-angle boxes.
3. **Check complete geometry early — step 12.** Sample interiors and bottlenecks,
   then refine adaptively. Distinguish joint bounds, orientation restrictions,
   collisions, conditioning and finite search/budget exhaustion. A failed search
   must not claim unreachable geometry without stronger supporting evidence.
4. **Select connected configurations globally — step 12.** Retain multiple IK
   candidates throughout the stroke and validate their connections, rather than
   relying mainly on alternative starts followed by local continuation. Compare
   a waypoint-solution graph and whole-stroke constrained optimization; prefer
   reserve, conditioning and manageable joint derivatives. No joint jumps or
   unrequested lifts may be inserted into a stroke.
5. **Optimize and time the whole stroke — steps 11–12.** Improve recovery from
   an incomplete/approximate seed; the current smoother mainly relies on a
   complete seed history. Preserve TCP geometry and requested normals, smooth
   joint motion and keep the explicit speed floor/ceiling. A future normal cone
   would require smooth orientation selection across the stroke, not unrelated
   per-point tilts. Profile computation and budget usage before claiming faster
   planning or a fixed overhead for added orientation freedom.
6. **Validate the entire operation — steps 12 and 16.** Accept only a complete
   checked approach, stroke and retract with valid connections and configured
   reserve. Retain shared interpolation, joint-rate/acceleration, FK, normal,
   collision, fresh-scene/start, Stop/fault and replay checks. A barely passing
   trajectory is a witness, not a robust operating-region qualification.
7. **Establish repeatable acceptance — steps 12, 17 and 23–24.** Test lines,
   circles, exact square sides with lifts, tight curves and representative SVG
   strokes across placements, starts, directions and speeds. Record complete
   passes, failures, reserve, latency and physical accuracy. Progress through
   numerical tests, full mock/PTY planning/execution, then separately authorized
   physical air and contact tests. No physical motion is authorized by this plan.

## SVG readiness

SVG import (step 15) does not create robot reach or connected configurations.
Support must mean supported artwork scaled and placed within a qualified surface
region, with explicit stroke/corner/lift and timing policies. General SVG drawing
at the current tabletop placement is not established. One tilted circle and
isolated square-side witnesses do not close this work.

Keep the shared model, active tool, calibrated limits, control contract, hardware
writer and powered safety behavior. Preserve existing uncommitted work. No
planner, hardware, normal-tolerance feature or operating limit is changed by
recording this work plan.
