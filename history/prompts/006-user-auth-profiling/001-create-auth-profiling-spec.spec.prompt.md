---
id: 001
title: Create Auth Profiling Spec
stage: spec
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 006-user-auth-profiling
branch: 006-user-auth-profiling
user: ismail
command: /sp.specify
labels: ["authentication", "user-profiling", "better-auth", "chatbot-gate"]
links:
  spec: specs/006-user-auth-profiling/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-user-auth-profiling/spec.md
  - specs/006-user-auth-profiling/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

# Spec 5: Authentication + Personalized User Profiling (Bonus)

## Goal
Add authentication and user profiling so that **only logged-in users can access the chatbot**, and content can be **personalized based on the user's background**.

---

## Scope
- Implement **Signup & Signin** using **Better Auth**
- Collect **software & hardware background** during signup
- Persist user profile data
- Enforce **authentication gate** before chatbot usage
- Pass user profile context to the backend agent for personalization

---

## Tech Stack
- Auth: https://www.better-auth.com/
- Frontend: Docusaurus (React)
- Backend: FastAPI (existing Agent)
- Storage: Database already used (or lightweight user table)
- Package manager: `uv`

---

## Functional Requirements

### FR1: Authentication
- Users must **Sign Up or Sign In** using Better Auth
- Authentication state must persist across sessions
- Logout supported

### FR2: Signup Profiling Form
During **Signup**, ask the user:

**Software Background**
- Programming level: Beginner / Intermediate / Advanced
- Familiar technologies (checkboxes): Python, JS, ROS2, AI/ML, Web, Other
- Experience with AI/Robotics (Yes/No)

**Hardware Background**
- Hardware access: None / Simulator only / Real robots
- Devices owned (optional): Jetson, Raspberry Pi, Arduino, GPU, Other

Form must be **required before account creation completes**.

---

### FR3: Auth Gate for Chatbot
- Chatbot **must not open** unless user is authenticated
- If user clicks chatbot while logged out:
  - Show modal: "Please log in to use the AI assistant"
  - Provide Sign In / Sign Up buttons
- If authenticated but profile incomplete:
  - Redirect to profile completion form

---

### FR4: User Context Injection (Critical)
When calling backend `/ask` endpoint:
- Include authenticated `user_id`
- Include `user_profile`:
```json
{
  "software_level": "intermediate",
  "technologies": ["Python", "ROS2"],
  "hardware_access": "simulator_only"
}
```

## Response snapshot

Created comprehensive specification for Authentication + Personalized User Profiling feature:

**Branch**: `006-user-auth-profiling`
**Spec File**: `specs/006-user-auth-profiling/spec.md`

**Key Deliverables**:
- 5 prioritized user stories (P1: Signup, Sign In, Auth Gate; P2: Personalization; P3: Logout)
- 21 functional requirements across 4 categories (Auth Core, Profiling, Gate, Context Injection)
- 3 key entities defined (User, UserProfile, Session)
- 7 measurable success criteria
- Edge cases and assumptions documented

**Validation**: All checklist items passed - spec is ready for `/sp.clarify` or `/sp.plan`

## Outcome

- Impact: Feature specification created with complete requirements coverage
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, requirements checklist)
- Next prompts: `/sp.clarify` to refine or `/sp.plan` to begin architecture
- Reflection: User input was comprehensive - minimal clarification needed

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided detailed requirements
- Graders run and results (PASS/FAIL): Spec quality checklist - PASS (all items)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
