# Tasks: Authentication + User Profiling

**Input**: Design documents from `/specs/006-user-auth-profiling/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/, research.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in this spec. Manual E2E testing checklist is provided in Phase 8.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

---

## Implementation Guidelines (MUST READ FIRST)

### Prime Directive

```
Use these tasks as the single source of truth.
Do not invent flows, tables, or endpoints.
Follow Better Auth defaults unless explicitly overridden.
Preserve RAG grounding at all times.
```

### Environment Variables (Exact Names)

**Auth Server (`auth-server/.env`)**:
```bash
DATABASE_URL=postgresql://user:pass@host:5432/dbname
BETTER_AUTH_SECRET=your-32-char-random-secret-here
BETTER_AUTH_URL=http://localhost:3001        # prod: https://<render-auth>.onrender.com
PORT=3001
```

**Frontend (config or `.env`)**:
```bash
BETTER_AUTH_URL=https://<render-auth-service>.onrender.com
CHATBOT_API_URL=https://ai-book-h6kj.onrender.com
```

**Backend (`database/.env`)**:
```bash
DATABASE_URL=postgresql://user:pass@host:5432/dbname  # Same DB as auth server
OPENAI_API_KEY=...
QDRANT_URL=...
```

**Do NOT hardcode secrets. Always read from `process.env` (Node) or `os.getenv()` (Python).**

---

### Database Ownership (Critical)

| Owner | Tables | Access |
|-------|--------|--------|
| Better Auth | `user`, `session`, `verification` | READ/WRITE |
| FastAPI Backend | (none) | READ-ONLY for user/profile |

**Rules**:
- Better Auth OWNS the `user` table
- User profile fields live in Better Auth `additionalFields` (NOT a separate table)
- FastAPI backend is **READ-ONLY** for user/profile data
- Backend must **NEVER** create or mutate user records
- Do NOT create duplicate user models in Python

---

### Better Auth Field Encoding (Non-Obvious)

**Limitation**: Better Auth does not support native array fields.

**Solution**: Store arrays as JSON-encoded strings.

| Field | Type in DB | Encode Location | Decode Location |
|-------|------------|-----------------|-----------------|
| `technologies` | TEXT (JSON string) | `signUp.email()` in AuthModal | `useApiClient.js` before /ask |
| `devices_owned` | TEXT (JSON string) | `signUp.email()` in AuthModal | `useApiClient.js` before /ask |

```typescript
// Encode (signup)
technologies: JSON.stringify(["Python", "ROS2"])

// Decode (before API call)
technologies: JSON.parse(session.user.technologies)
```

---

### Auth Session Trust Model

**Trust Better Auth session cookie. No custom JWT logic.**

| Component | Responsibility |
|-----------|---------------|
| Better Auth Server | Issues/validates session cookies |
| Frontend | Uses `useSession()` hook, sends user_id + profile to backend |
| Backend | Trusts frontend-provided user_id + profile (MVP) |

**Do NOT**:
- Create custom JWT tokens
- Implement custom session validation
- Add middleware that duplicates Better Auth logic

**Optional future enhancement**: Backend may verify user_id against DB, but NOT required for MVP.

---

### RAG Safety Guardrail (Non-Negotiable)

**User profile affects ONLY**:
- Explanation depth (beginner: detailed, advanced: concise)
- Language simplicity
- Examples (simulator vs hardware)

**User profile must NEVER**:
- Add facts not in retrieved context
- Change retrieval query or results
- Override or modify citations
- Hallucinate based on profile assumptions

**If unsure, default to retrieved context only.**

```python
# CORRECT: Adjust tone
"For beginners: Use simpler language, explain concepts"

# WRONG: Add facts
"Since you have a Jetson, you can also try X"  # X not in context = WRONG
```

---

### UI/UX Constraints (Do Not Guess)

| Constraint | Required Behavior |
|------------|-------------------|
| Auth UI | Modal-based (NOT full page redirect) |
| Chatbot icon | Behavior unchanged (click opens modal OR chatbot) |
| Profile form | Inline inside AuthModal (NOT separate /signup page) |
| Signup flow | Email/password → Profile form → Submit atomically |

---

### Out of Scope (Do NOT Implement)

- OAuth / social login
- Password reset flow
- Email verification flow
- Admin roles or permissions
- Profile editing UI (future feature)
- Custom JWT tokens
- Separate user_profile table

---

### Deployment Facts

| Service | Platform | Notes |
|---------|----------|-------|
| Frontend | GitHub Pages | Static, no SSR |
| Auth Server | Render Web Service | Node.js |
| Backend | Render Web Service | FastAPI (existing) |

**Cross-Origin Cookie Requirements**:
```typescript
// Auth server CORS
app.use(cors({
  origin: ["https://ismailabdulkareem.github.io", "http://localhost:3000"],
  credentials: true,  // REQUIRED for cookies
}));

// Frontend fetch
fetch(url, { credentials: "include" })  // REQUIRED for cookies
```

---

### Execution Instructions

1. **Work task-by-task in order** (T001 → T002 → ...)
2. **Stop at each Phase checkpoint** to validate
3. **Do NOT skip Phase 2** (Foundational) - it blocks everything
4. **If a dependency is missing**:
   - Node.js: `npm install <package>`
   - Python: `uv add <package>`
5. **Ask before making architectural changes**
6. **Commit after each task or logical group**

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions

## Path Conventions

This is a multi-service web application:
- **Auth Server**: `auth-server/src/`
- **Frontend**: `src/plugins/docusaurus-plugin-chatbot/`
- **Backend**: `database/`

---

## Phase 1: Setup (Auth Server Infrastructure)

**Purpose**: Create the Better Auth server project structure and dependencies

- [x] T001 Create auth-server directory structure with `auth-server/src/`, `auth-server/package.json`
- [x] T002 Initialize npm project with dependencies: `better-auth`, `express`, `pg`, `dotenv`, `cors`
- [x] T003 [P] Install dev dependencies: `typescript`, `@types/node`, `@types/express`, `tsx`
- [x] T004 [P] Create TypeScript config in `auth-server/tsconfig.json`
- [x] T005 [P] Create environment template in `auth-server/.env.example`
- [x] T006 Add npm scripts to `auth-server/package.json`: dev, build, start, db:generate, db:migrate

---

## Phase 2: Foundational (Auth Server + Frontend Setup)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

### Auth Server Core

- [x] T007 Implement Better Auth configuration with PostgreSQL adapter and additionalFields in `auth-server/src/auth.ts`
- [x] T008 Implement Express server with CORS and Better Auth handler in `auth-server/src/index.ts`
- [x] T009 Configure trustedOrigins for `https://ismailabdulkareem.github.io` and `http://localhost:3000`
- [x] T010 Generate database schema with `npm run db:generate`
- [x] T011 Run database migrations with `npm run db:migrate`
- [x] T012 Test auth server health endpoint at `http://localhost:3001/health`

### Frontend Auth Client Setup

- [x] T013 Install `better-auth` in frontend: `npm install better-auth`
- [x] T014 Create Better Auth client instance in `src/plugins/docusaurus-plugin-chatbot/lib/auth-client.ts`
- [x] T015 [P] Create TypeScript types for auth in `src/plugins/docusaurus-plugin-chatbot/types.d.ts`

### Backend Model Setup

- [x] T016 Create UserProfile Pydantic model in `database/models.py` with programming_level, technologies, hardware_access

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - New User Signup with Profile (Priority: P1)

**Goal**: Allow new users to create accounts with email/password and complete their profile atomically during signup

**Independent Test**: Complete signup form with profile → verify user account created with profile data stored

### Implementation for User Story 1

- [x] T017 [P] [US1] Create AuthProvider React context in `src/plugins/docusaurus-plugin-chatbot/components/Auth/AuthProvider.tsx`
- [x] T018 [P] [US1] Create isProfileComplete helper function in AuthProvider
- [x] T019 [US1] Create ProfileForm component with all required fields in `src/plugins/docusaurus-plugin-chatbot/components/Auth/ProfileForm.tsx`
- [x] T020 [US1] Implement programming level select (Beginner/Intermediate/Advanced) in ProfileForm
- [x] T021 [US1] Implement technologies multi-select (Python, JS, ROS2, AI/ML, Web, Other) in ProfileForm
- [x] T022 [US1] Implement AI/Robotics experience toggle (Yes/No) in ProfileForm
- [x] T023 [US1] Implement hardware access select (None/Simulator only/Real robots) in ProfileForm
- [x] T024 [US1] Implement optional devices owned multi-select in ProfileForm
- [x] T025 [US1] Create AuthModal component with Sign In/Sign Up tabs in `src/plugins/docusaurus-plugin-chatbot/components/Auth/AuthModal.tsx`
- [x] T026 [US1] Implement signup form with email, password, name fields in AuthModal
- [x] T027 [US1] Integrate ProfileForm into signup flow (shown BEFORE account creation completes)
- [x] T028 [US1] Implement atomic signup: validate all fields before calling signUp.email()
- [x] T029 [US1] Handle signup errors with clear user-friendly messages
- [x] T030 [US1] Auto-login user after successful signup (session cookie set)
- [x] T031 [US1] Add CSS styles for AuthModal and ProfileForm in `src/plugins/docusaurus-plugin-chatbot/components/Auth/styles.module.css`

**Checkpoint**: New users can complete signup with profile. Test: signup → verify profile in database

---

## Phase 4: User Story 2 - Returning User Sign In (Priority: P1)

**Goal**: Allow existing users to sign in with credentials and immediately access the chatbot

**Independent Test**: Sign in with valid credentials → verify chatbot becomes accessible with stored profile

### Implementation for User Story 2

- [x] T032 [US2] Implement sign-in form in AuthModal with email/password fields
- [x] T033 [US2] Call signIn.email() with credentials
- [x] T034 [US2] Handle sign-in success: close modal, refresh session state
- [x] T035 [US2] Handle sign-in errors without revealing which field was incorrect
- [x] T036 [US2] Add "Remember me" option (default: true for 7-day session)
- [x] T037 [US2] Implement mode toggle between Sign In and Sign Up in AuthModal

**Checkpoint**: Returning users can sign in. Test: signin → verify session contains profile data

---

## Phase 5: User Story 3 - Authentication Gate for Chatbot (Priority: P1)

**Goal**: Block chatbot access for unauthenticated users; force profile completion if missing

**Independent Test**: Click chatbot while logged out → verify auth modal appears instead of chatbot

### Implementation for User Story 3

- [x] T038 [US3] Update Root.js to wrap app with AuthProvider in `src/plugins/docusaurus-plugin-chatbot/theme/Root.js`
- [x] T039 [US3] Import and use useAuth hook in Chatbot component at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/index.jsx`
- [x] T040 [US3] Add auth gate logic: if !session → show AuthModal instead of opening chatbot
- [x] T041 [US3] Add profile check: if authenticated but !isProfileComplete → redirect to ProfileForm
- [x] T042 [US3] Only allow chatbot to open when authenticated AND profile complete
- [x] T043 [US3] Add loading state handling while session is being checked
- [x] T044 [US3] Add AuthModal state management in Chatbot component

**Checkpoint**: Chatbot is fully gated. Test: unauthenticated click → modal; authenticated click → chatbot

---

## Phase 6: User Story 4 - Personalized Chatbot Responses (Priority: P2)

**Goal**: Use profile data to personalize response tone and complexity while preserving RAG grounding

**Independent Test**: Ask same question with different profiles → verify tone differs but facts unchanged

### Backend Extension

- [x] T045 [US4] Import UserProfile model in `database/app.py`
- [x] T046 [US4] Extend AskRequest model with user_id (required str) in `database/app.py`
- [x] T047 [US4] Extend AskRequest model with user_profile (required UserProfile) in `database/app.py`
- [x] T048 [US4] Add validation: reject request if user_id missing (401 error)
- [x] T049 [US4] Add validation: reject request if user_profile missing (401 error)
- [x] T050 [US4] Implement build_personalized_system_prompt() function in `database/app.py`
- [x] T051 [US4] Inject personalization into generate_answer() while preserving RAG grounding
- [x] T052 [US4] Add RAG safety constraint in system prompt: "ALL facts must come from context"

### Frontend User Context Injection

- [x] T053 [US4] Update useApiClient hook to accept user context in `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/hooks/useApiClient.js`
- [x] T054 [US4] Get user_id from session.user.id in useApiClient
- [x] T055 [US4] Build user_profile object from session.user fields
- [x] T056 [US4] Parse technologies JSON string to array before sending
- [x] T057 [US4] Include user_id and user_profile in all /ask requests

**Checkpoint**: Personalization works. Test: beginner vs advanced user → different tone, same facts

---

## Phase 7: User Story 5 - User Logout (Priority: P3)

**Goal**: Allow authenticated users to log out, ending their session

**Independent Test**: Click logout → verify session ends and chatbot becomes inaccessible

### Implementation for User Story 5

- [x] T058 [US5] Add logout button to chatbot UI or auth state indicator
- [x] T059 [US5] Call signOut() from Better Auth client on logout click
- [x] T060 [US5] Clear session state in AuthProvider after logout
- [x] T061 [US5] Verify chatbot gate re-activates after logout (modal on next click)

**Checkpoint**: Logout works. Test: logout → chatbot click shows auth modal

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, deployment, and cleanup

### CORS & Security

- [x] T062 Verify CORS allows credentials from GitHub Pages origin
- [x] T063 Test session cookie settings (SameSite, Secure flags)
- [x] T064 Verify passwords hashed with Argon2 (Better Auth default)

### Deployment

- [x] T065 Create Render deployment config for auth-server
- [ ] T066 Add DATABASE_URL and BETTER_AUTH_SECRET to Render environment
- [x] T067 Update BETTER_AUTH_URL in frontend to production auth server URL
- [ ] T068 Deploy auth server to Render
- [ ] T069 Test production auth flow (signup, signin, session persistence)

### E2E Validation (Manual Testing Checklist)

- [x] T070 Test: Auth server health check responds at /health
- [x] T071 Test: Signup creates user with profile (verify in database)
- [x] T072 Test: Signin returns session with profile fields
- [ ] T073 Test: Session persists across page reload (7 days)
- [ ] T074 Test: Chatbot blocked when not authenticated
- [ ] T075 Test: Auth modal appears on chatbot click (unauthenticated)
- [ ] T076 Test: Profile data sent in /ask requests
- [ ] T077 Test: Responses personalized based on profile (beginner vs advanced)
- [ ] T078 Test: Logout clears session and re-gates chatbot

### Documentation

- [x] T079 Update quickstart.md with actual deployment URLs
- [x] T080 Document environment variables in auth-server/.env.example

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ─────────────────────┐
                                     ↓
Phase 2 (Foundational) ──────────────┤ BLOCKS ALL USER STORIES
                                     ↓
         ┌───────────────────────────┼───────────────────────────┐
         ↓                           ↓                           ↓
Phase 3 (US1: Signup)    Phase 4 (US2: Signin)    Phase 5 (US3: Gate)
         │                           │                           │
         └───────────────────────────┼───────────────────────────┘
                                     ↓
                          Phase 6 (US4: Personalization)
                                     ↓
                          Phase 7 (US5: Logout)
                                     ↓
                          Phase 8 (Polish)
```

### User Story Dependencies

| User Story | Depends On | Can Start After |
|------------|------------|-----------------|
| US1 (Signup) | Foundational | Phase 2 complete |
| US2 (Signin) | US1 (AuthModal) | T025 complete |
| US3 (Gate) | US1 (AuthProvider) | T017 complete |
| US4 (Personalization) | US3 (Gate working) | Phase 5 complete |
| US5 (Logout) | US1 (Auth infrastructure) | Phase 3 complete |

### Within Each User Story

1. Models/Types before components
2. Components before integration
3. Core functionality before error handling
4. Frontend before backend (for US4)

### Parallel Opportunities

**Phase 1 (Setup)**: T003, T004, T005 can run in parallel

**Phase 2 (Foundational)**: T015 can run parallel to T007-T014

**Phase 3 (US1)**:
- T017, T018 can run in parallel
- T019-T024 (ProfileForm fields) can run in parallel after T019 scaffold
- T020, T021, T022, T023, T024 (form fields) can run in parallel

**Phase 6 (US4)**:
- Backend tasks (T045-T052) can run parallel to frontend tasks (T053-T057)

---

## Parallel Example: User Story 1

```bash
# Launch AuthProvider and helpers together:
Task T017: "Create AuthProvider React context"
Task T018: "Create isProfileComplete helper function"

# Launch ProfileForm fields in parallel (after T019 scaffold):
Task T020: "Implement programming level select"
Task T021: "Implement technologies multi-select"
Task T022: "Implement AI/Robotics experience toggle"
Task T023: "Implement hardware access select"
Task T024: "Implement optional devices owned multi-select"
```

---

## Parallel Example: User Story 4 (Backend + Frontend)

```bash
# Backend extension (in database/):
Task T045: "Import UserProfile model"
Task T046: "Extend AskRequest with user_id"
Task T047: "Extend AskRequest with user_profile"

# Frontend injection (in src/plugins/):
Task T053: "Update useApiClient hook"
Task T054: "Get user_id from session"
Task T055: "Build user_profile object"

# These can run in parallel as they're in different directories
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 3 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: User Story 1 (Signup)
4. Complete Phase 5: User Story 3 (Gate)
5. **STOP and VALIDATE**: Chatbot is now gated, users can sign up
6. Deploy MVP

### Incremental Delivery

| Increment | Stories | What's Working |
|-----------|---------|----------------|
| MVP | US1 + US3 | Signup, chatbot gate |
| +Signin | US2 | Returning users can sign in |
| +Personalization | US4 | Responses adapt to user |
| +Logout | US5 | Full session lifecycle |
| +Polish | - | Production-ready |

### Single Developer Recommended Order

1. Phase 1: Setup (T001-T006)
2. Phase 2: Foundational (T007-T016)
3. Phase 3: US1 Signup (T017-T031)
4. Phase 4: US2 Signin (T032-T037)
5. Phase 5: US3 Gate (T038-T044)
6. **Deploy and test MVP**
7. Phase 6: US4 Personalization (T045-T057)
8. Phase 7: US5 Logout (T058-T061)
9. Phase 8: Polish (T062-T080)

---

## Summary

| Metric | Count |
|--------|-------|
| Total Tasks | 80 |
| Phase 1 (Setup) | 6 |
| Phase 2 (Foundational) | 10 |
| Phase 3 (US1: Signup) | 15 |
| Phase 4 (US2: Signin) | 6 |
| Phase 5 (US3: Gate) | 7 |
| Phase 6 (US4: Personalization) | 13 |
| Phase 7 (US5: Logout) | 4 |
| Phase 8 (Polish) | 19 |
| Parallel Opportunities | 12 tasks marked [P] |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- RAG grounding MUST be preserved - personalization affects tone only
- Arrays (technologies, devices_owned) stored as JSON strings
