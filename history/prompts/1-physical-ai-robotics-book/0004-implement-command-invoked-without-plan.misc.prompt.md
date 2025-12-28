---
id: 0004
title: Implement command invoked without plan
stage: misc
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-physical-ai-robotics-book
branch: 1-physical-ai-robotics-book
user: null
command: /sp.implement
labels: ["implementation", "error-handling", "prerequisite"]
links:
  spec: specs/1-physical-ai-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/1-physical-ai-robotics-book/checklists/requirements.md
- specs/1-physical-ai-robotics-book/tasks.md
tests:
- null
---

## Prompt

$ARGUMENTS

## Response snapshot

"The `plan.md` file, which is a required design document for implementation, is missing. Please run `/sp.plan` to create the implementation plan before proceeding with `/sp.implement`."

## Outcome

- ✅ Impact: Identified missing prerequisite (`plan.md`) for /sp.implement and informed the user.
- 🧪 Tests: N/A
- 📁 Files: Read specs/1-physical-ai-robotics-book/checklists/requirements.md, specs/1-physical-ai-robotics-book/tasks.md
- 🔁 Next prompts: User needs to run /sp.plan.
- 🧠 Reflection: Successfully detected missing prerequisite and provided clear guidance to the user. Avoided proceeding with implementation without essential planning documents.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
