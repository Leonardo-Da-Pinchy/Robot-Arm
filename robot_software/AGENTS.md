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
- Retain implementation caveats, external ownership and pending owner inputs
  in the roadmap or linked technical documents when shortening descriptions.
- Historical checkpoint/status tables are references, not competing task lists.
