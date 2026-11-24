# Documentation conventions

Keep `README.md` a general overview of the robot software, package/tool
responsibilities, setup and usage, and links to detailed documentation.
Do not add project updates, dated results, completed/pending task lists, or
bench-session status to the README. Record those in the appropriate files under
`docs/`, such as the hardware inventory or implementation plans.

# Roadmap and status lists

`docs/implementation_roadmap.md` is the canonical final-product task list.
When the user asks for the full list or current project status:

- Read that file first and update it with newly confirmed decisions/statuses
  before answering; do not invent progress or edit just to refresh a timestamp.
- Preserve its 1–24 numbering and existing table/status formatting. Reproduce
  the numbered table directly when giving the full list; do not independently
  rewrite a second version in chat.
- Keep rows concise: a short step name and brief status note. Put expanded
  implementation detail in linked notes rather than repeating it in chat.
- Retain implementation caveats, external ownership and pending owner inputs
  in the roadmap or linked technical documents when shortening descriptions.
- Historical checkpoint/status tables are references, not competing task lists.

# Shared motion contracts

Keep planning and commissioning layered on the same live robot description,
active-tool profile, calibrated limits, controllers and hardware adapter.
Use `pinchy_bringup.control_contract` for joint order, limit interpretation,
trajectory endpoints and constant hold goals. Preserve the shared RViz layout;
planning adds displays rather than keeping an independent copy of the view.
Never implement a second servo writer or weaken thermal confirmation/powered
fault handling in the Cartesian application.

Continuous drawing must follow
`src/pinchy_moveit_config/config/drawing_motion_policy.yaml`: approximately
constant TCP surface speed through stroke interiors; accelerate/decelerate in
pen-up motion wherever feasible. The owner also permits smooth contact endpoint
ramps when explicitly requested via `allow_contact_ramps`; report ramp-only
short strokes. See `docs/stroke_planning.md`. Do not stop at sampled contact waypoints or silently
relax geometry/speed to get a trajectory. Pose transit is not contact drawing.
