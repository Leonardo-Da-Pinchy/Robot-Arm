# Owner-selected nominal object geometries

The owner selected these three nominal geometries on November 14, 2026.
Dimensions are in millimetres. These supersede the earlier statement that no
nominal object sizes had been selected; object placement remains undecided.

| Geometry | Nominal dimensions | Intended orientation |
| --- | --- | --- |
| Flat panel | 100 × 100 mm drawing face; panel thickness TBD | Upright, face toward the robot. |
| Cylinder or straight-sided cylindrical mug | 80 mm outside diameter (40 mm outside radius) × 100 mm tall | Upright; any mug handle faces away from the robot. |
| Quarter pipe | 100 mm inner radius (200 mm inner diameter) × 100 mm axial length; 90° cross-section; 3–5 mm wall thickness if printed | Concave inside faces the robot; exact axis orientation TBD. This is a longitudinal quarter of a straight tube, not an elbow. |

The cylinder and cylindrical mug share one drawing-surface geometry. A mug's
handle still needs measured collision geometry; facing it away does not exempt
it from clearance checks. A tapered mug does not match the nominal cylinder.
The quarter pipe's exact wall thickness within the selected range must be fixed
before generating final solid collision geometry.

The design report's initial scope was a cylinder/cylindrical mug and explicitly
excluded concave surfaces. The owner now selects the plane and concave quarter
pipe as additional nominal geometries. This is an explicit scope addition, not
a claim that either is already implemented or qualified.

These dimensions do not establish supported drawing areas or reachable poses.
The previously suggested small drawing patches are test proposals, not approved
coverage limits. Position/orientation in base_link, seating/fixtures, the actual
tabletop and base-wing geometry, final pen/TCP, contact clearance and full
stroke/approach/retract feasibility remain to be established. The existing
80 mm-cylinder numerical studies do not validate a 100 mm tall drawing band,
front-180° coverage, concave drawing, or physical contact operation.

This records design choices only. It does not install collision objects into a
live scene, change calibration or safety behavior, authorize physical movement,
or close roadmap steps 11–12. Use the canonical [roadmap](implementation_roadmap.md)
and [motion-feasibility investigation](motion_feasibility.md) for next work.
