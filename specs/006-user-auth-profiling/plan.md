# Implementation Plan: Authentication + User Profiling

**Branch**: `006-user-auth-profiling` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-user-auth-profiling/spec.md`

## Summary

Implement authentication and user profiling using Better Auth to gate chatbot access and enable personalized responses. The system adds email/password authentication, collects user background (programming level, technologies, hardware access) during signup, and injects this context into RAG chatbot requests for tone/complexity personalization while maintaining grounded responses.

## Technical Context

**Language/Version**: TypeScript 5.x (auth server), Python 3.11 (backend), React 18 (frontend)
**Primary Dependencies**: Better Auth 1.2+, Express 4.x, FastAPI 0.115+, PostgreSQL
**Storage**: PostgreSQL (shared between auth server and FastAPI backend)
**Testing**: Vitest (auth server), pytest (backend), manual E2E
**Target Platform**: Web (GitHub Pages frontend, Render backend services)
**Project Type**: Web application (frontend + backend + auth server)
**Performance Goals**: Auth operations < 500ms, session retrieval < 100ms
**Constraints**: Static frontend (no SSR), shared database, RAG grounding preserved
**Scale/Scope**: Single-tenant, ~100-1000 users expected

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Spec-first authoring | PASS | Feature has complete spec.md with FRs, user stories |
| Technical accuracy | PASS | Better Auth patterns verified against official docs |
| Reproducibility | PASS | quickstart.md provides step-by-step setup |
| Consistency | PASS | Uses existing PostgreSQL, follows plugin pattern |
| RAG Grounding | PASS | Personalization only affects tone, not content |
| Secrets handling | PASS | All secrets via environment variables |
| Smallest viable change | PASS | Extends existing /ask endpoint minimally |

## Project Structure

### Documentation (this feature)

```text
specs/006-user-auth-profiling/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup guide
├── contracts/
│   ├── auth-api.yaml    # Better Auth OpenAPI spec
│   └── chatbot-api.yaml # Extended /ask endpoint spec
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
# Auth Server (NEW)
auth-server/
├── src/
│   ├── auth.ts          # Better Auth configuration
│   └── index.ts         # Express server
├── package.json
├── tsconfig.json
└── .env.example

# Frontend (MODIFIED)
src/plugins/docusaurus-plugin-chatbot/
├── components/
│   ├── Auth/
│   │   ├── AuthProvider.tsx    # NEW: Auth context provider
│   │   ├── AuthModal.tsx       # NEW: Sign in/up modal
│   │   └── ProfileForm.tsx     # NEW: Profile collection form
│   └── Chatbot/
│       ├── index.jsx           # MODIFIED: Auth gate
│       └── hooks/
│           └── useApiClient.js # MODIFIED: User context injection
├── lib/
│   └── auth-client.ts          # NEW: Better Auth client
└── theme/
    └── Root.js                 # MODIFIED: Wrap with AuthProvider

# Backend (MODIFIED)
database/
├── app.py                      # MODIFIED: Extended /ask endpoint
└── models.py                   # NEW: UserProfile Pydantic model
```

**Structure Decision**: Web application pattern - separate auth server (Node.js/Better Auth), static frontend (Docusaurus), existing Python backend (FastAPI). Auth server is new; frontend and backend are modified.

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Auth Library | Better Auth | Spec requirement, TypeScript-first, extensible |
| Auth Server | Standalone Express | Docusaurus is static, can't run server-side |
| Database | PostgreSQL (shared) | Already in use, enables FastAPI to read profiles |
| Profile Storage | additionalFields in user table | Atomic signup, type-safe, no joins |
| Session Type | Cookie-based (7-day) | Better Auth default, meets SC-005 |
| Profile Arrays | JSON strings | Better Auth doesn't support array fields |
| Deployment | Render (auth server) | Existing infrastructure |

## Complexity Tracking

No complexity violations. Design uses:
- 1 new service (auth server) - justified by language mismatch
- Standard patterns (React Context, Express middleware)
- Minimal backend changes (2 new fields in request model)

## Implementation Phases

### Phase 1: Auth Server Setup
1. Create auth-server directory with Express + Better Auth
2. Configure PostgreSQL adapter with additionalFields
3. Generate and run database migrations
4. Deploy to Render

### Phase 2: Frontend Auth Integration
1. Add Better Auth React client
2. Create AuthProvider context
3. Build AuthModal (sign in/sign up with profile)
4. Integrate auth gate into Chatbot component

### Phase 3: Backend User Context
1. Add UserProfile Pydantic model
2. Extend /ask endpoint request schema
3. Build personalization prompt injection
4. Validate user context on requests

### Phase 4: Testing & Polish
1. E2E auth flow testing
2. Session persistence verification
3. Personalization quality review
4. Error handling and edge cases

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| CORS issues between static site and auth server | Configure trustedOrigins, test cross-origin |
| Session not persisting | Verify cookie settings, credentials: true |
| Profile data race condition | Atomic signup enforced by FR-022 |
| Personalization leaking into facts | RAG safety constraint in system prompt |

## Next Steps

Run `/sp.tasks` to generate implementation tasks from this plan.
