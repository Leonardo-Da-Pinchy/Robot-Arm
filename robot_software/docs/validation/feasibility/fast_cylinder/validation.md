# Larger, faster cylinder air loop

November 17, 2026. All motion below ran on isolated mocks or a generated PTY
servo bus. No physical motor moved, live hardware restarted, joint position
limit changed, or active TCP changed. Existing uncommitted work is preserved.

Example 07 is a 61-point, single continuous oval, **76.2 mm in both surface
coordinates**, versus 50.8 mm before. It requests strict **7 mm/s**, versus
3 mm/s before: 1.5× span and 2.33× cruise speed. It retains Ø80 × H100 mm nominal
cylinder dimensions, 5 mm stand-off, inward pen normal, free axial twist and
30 deg/s / 180 deg/s² selected joint caps. The explicit alternate cylinder
center is `[0.40, 0.10, 0.15]` m, with the oval centered at −15°; the corresponding
proposed pedestal scene is packaged separately. Old examples/scenes are retained.

The new explicit `spline_axis` mode keeps the original natural cubic XYZ path
and smoothly interpolates the unit pen directions. This removes segmentwise
angular-rate jumps; it does not silently alter existing spline files. Actual
controller-polynomial geometry, joint bounds/rates, scene collisions and
constant-speed requirements all still pass before a preview exists.

| Full-system result | Mock | Real driver + PTY emulator |
| --- | --- | --- |
| Initial planning time | 159 s | 158 s |
| Approach + trace duration | 41.21 s | 41.07 s |
| Trace duration | 36.92 s | 36.92 s |
| Execute request → first moving reference | 0.382 s | 0.314 s |
| Execute checks → controller dispatch | 0.348 s | 0.278 s |
| Controller cruise min/max | 6.9697–7.0356 mm/s | 6.9701–7.0354 mm/s |
| Interior input waypoints checked at cruise | 59/59 | 59/59 |
| Live cruise samples | 1,781 | 1,781 |
| Largest live modeled normal error | 0.0205° | 0.0202° |
| Controller completion | Pass | Pass |
| Consumed-plan replay rejection and subsequent Stop/hold | Pass | Pass |

Independent dense polynomial assays also pass: normal error below 0.030°,
5 mm air gap within 0.0005 mm, and cruise speed within 1%. These are modeled
reference results; tiny computed residuals are not physical metrology. PTY
encoder feedback traversed the complete loop; the endpoint residual was about
0.13 mm / 0.03° in that emulator. No physical accuracy or speed is established.

Both complete runs used actual MoveIt scene collisions, an explicitly planned
pen-up approach, the existing trajectory controller and shared driver/safety
stack. No partial candidate was executed; no automatic speed reduction or
interior waypoint stops were accepted. These runs do not include a retract;
the earlier slower nominal paths retain separate retract qualification.

Initial 10 mm/s candidates failed joint timing. The saved numerical screen is
historical finite-search evidence, not a global speed limit or impossibility
proof. Normal-direction interpolation, posture, placement and selected joint
caps all affect the achievable continuous speed. See the
[usage and speed explanation](../../../continuous_cylinder_test.md#larger-faster-example)
and [Execute validation](../../moveit/execute_latency/validation.md).

Seven geometry/file tests and the panel loading/round-trip checks pass. Full
reports, controller traces, launch logs, emulator report and source hashes are
stored alongside this file. Reproduction commands are in the usage guide.
Full front-180° coverage, physical fixture/tool agreement and contact remain
unqualified; roadmap step 12 stays open.
