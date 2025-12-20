# Implementation Plan: Authentication + User Profiling

**Branch**: `006-user-auth-profiling` | **Date**: 2025-12-20 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-user-auth-profiling/spec.md`

---

## High-Level Scope & Boundaries

### What This Implements
- Better Auth for authentication (email/password, cookie-based sessions)
- Chatbot gated behind authentication
- Signup requires profile completion (atomic - no partial accounts)
- Profile data personalizes tone/depth only, NOT facts

### What This Does NOT Change
- RAG grounding logic remains unchanged
- All answers must still come from Qdrant vector store
- Citation and source logic unchanged

### Architecture Overview
| Service | Technology | Role |
|---------|------------|------|
| Auth Server | Node.js + Express + Better Auth | Handles signup, signin, sessions; stores user + profile in PostgreSQL |
| Frontend | Docusaurus (static React) | Uses Better Auth client; blocks chatbot if unauthenticated; forces profile completion; sends user context to backend |
| Backend | FastAPI (Python) | Validates authenticated requests; accepts user context in /ask; uses profile ONLY to adjust explanation depth |

---

## Technical Context

**Language/Version**: TypeScript 5.x (auth server), Python 3.11 (backend), React 18 (frontend)
**Primary Dependencies**: Better Auth 1.2+, Express 4.x, FastAPI 0.115+, PostgreSQL
**Storage**: PostgreSQL (shared between auth server and FastAPI backend)
**Testing**: Vitest (auth server), pytest (backend), manual E2E
**Target Platform**: Web (GitHub Pages frontend, Render backend services)
**Performance Goals**: Auth operations < 500ms, session retrieval < 100ms
**Constraints**: Static frontend (no SSR), shared database, RAG grounding preserved
**Scale/Scope**: Single-tenant, ~100-1000 users expected

---

## Constitution Check

| Gate | Status | Notes |
|------|--------|-------|
| Spec-first authoring | PASS | Feature has complete spec.md with FRs, user stories |
| Technical accuracy | PASS | Better Auth patterns verified against official docs |
| Reproducibility | PASS | quickstart.md provides step-by-step setup |
| Consistency | PASS | Uses existing PostgreSQL, follows plugin pattern |
| RAG Grounding | PASS | Personalization only affects tone, not content |
| Secrets handling | PASS | All secrets via environment variables |
| Smallest viable change | PASS | Extends existing /ask endpoint minimally |

---

## File Targets

### Files to CREATE

```text
auth-server/
├── src/
│   ├── index.ts          # Express server entry point
│   └── auth.ts           # Better Auth configuration
├── package.json          # Dependencies
├── tsconfig.json         # TypeScript config
└── .env.example          # Environment template

src/plugins/docusaurus-plugin-chatbot/
├── components/Auth/
│   ├── AuthProvider.tsx  # Auth context provider
│   ├── AuthModal.tsx     # Sign in/up modal with profile form
│   └── ProfileForm.tsx   # Profile collection form
└── lib/
    └── auth-client.ts    # Better Auth client instance

database/
└── models.py             # UserProfile Pydantic model
```

### Files to MODIFY

```text
src/plugins/docusaurus-plugin-chatbot/
├── components/Chatbot/
│   ├── index.jsx         # Add auth gate logic
│   └── hooks/
│       └── useApiClient.js  # Inject user_id + user_profile into requests
└── theme/
    └── Root.js           # Wrap with AuthProvider

database/
└── app.py                # Extend /ask endpoint with user context
```

---

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Auth Library | Better Auth | Spec requirement, TypeScript-first, extensible |
| Auth Server | Standalone Express | Docusaurus is static, can't run server-side |
| Database | PostgreSQL (shared) | Already in use, enables FastAPI to read profiles |
| Profile Storage | additionalFields in user table | Atomic signup, type-safe, no joins |
| Session Type | Cookie-based (7-day) | Better Auth default, meets SC-005 |
| Profile Arrays | JSON strings | Better Auth doesn't support native array fields |
| Deployment | Auth Server on Render | Existing infrastructure |

---

## Better Auth Configuration Requirements

### User Schema with additionalFields

```typescript
// auth-server/src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24,     // Update session age daily
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes
    },
  },
  user: {
    additionalFields: {
      programming_level: {
        type: "string",
        required: true,
        input: true, // Allows input during signup
      },
      technologies: {
        type: "string", // JSON array as string
        required: true,
        input: true,
      },
      ai_robotics_experience: {
        type: "boolean",
        required: true,
        input: true,
      },
      hardware_access: {
        type: "string",
        required: true,
        input: true,
      },
      devices_owned: {
        type: "string", // JSON array as string, optional
        required: false,
        input: true,
      },
    },
  },
  trustedOrigins: [
    process.env.FRONTEND_URL || "http://localhost:3000",
    "https://ismailabdulkareem.github.io",
  ],
});
```

**Important**: Store arrays (technologies, devices_owned) as JSON strings due to Better Auth limitation.

---

## Signup Flow Logic (Non-Negotiable)

```
┌─────────────────────────────────────────────────────────────────┐
│                      SIGNUP FLOW                                │
├─────────────────────────────────────────────────────────────────┤
│ 1. User clicks chatbot icon                                     │
│ 2. AuthModal opens (user not authenticated)                     │
│ 3. User selects "Sign Up"                                       │
│ 4. User enters email + password                                 │
│ 5. BEFORE account creation completes:                           │
│    └── ProfileForm is shown                                     │
│ 6. Validate ALL required profile fields                         │
│ 7. Create account + profile ATOMICALLY                          │
│ 8. Auto-login user (session cookie set)                         │
│ 9. Chatbot access granted                                       │
└─────────────────────────────────────────────────────────────────┘

❌ No partial accounts allowed
❌ No skipping profile step
❌ No account creation without complete profile
```

---

## Chatbot Auth Gate Logic

```typescript
// Pseudocode for Chatbot/index.jsx
function ChatbotGate() {
  const { session, user, isLoading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showProfileForm, setShowProfileForm] = useState(false);

  const handleChatbotClick = () => {
    if (!session) {
      // Rule 1: NOT authenticated → open AuthModal
      setShowAuthModal(true);
      return;
    }

    if (!isProfileComplete(user)) {
      // Rule 2: Authenticated but profile incomplete → show ProfileForm
      setShowProfileForm(true);
      return;
    }

    // Rule 3: Authenticated + complete profile → open Chatbot normally
    openChatbot();
  };
}

function isProfileComplete(user) {
  return (
    user?.programming_level &&
    user?.technologies &&
    JSON.parse(user.technologies).length > 0 &&
    typeof user?.ai_robotics_experience === "boolean" &&
    user?.hardware_access
  );
}
```

---

## Backend /ask Contract Extension

### Extended Request Schema

```python
# database/models.py
from pydantic import BaseModel, Field
from typing import List, Literal, Optional

class UserProfile(BaseModel):
    """User profile for personalization (tone only, not facts)."""
    programming_level: Literal["beginner", "intermediate", "advanced"]
    technologies: List[str] = Field(default_factory=list)
    hardware_access: Literal["none", "simulator_only", "real_robots"]

class AskRequest(BaseModel):
    """Extended request model for /ask endpoint."""
    question: str = Field(..., min_length=1, description="The question to ask")
    context: Optional[str] = Field(None, description="Optional selected text context")
    top_k: int = Field(5, ge=1, le=10, description="Number of sources to retrieve")
    user_id: str = Field(..., description="Authenticated user ID")
    user_profile: UserProfile = Field(..., description="User profile for personalization")
```

### Backend Validation Rules

```python
# database/app.py - Extended /ask endpoint

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    # RULE 1: Reject request if user_id missing
    if not request.user_id:
        raise HTTPException(status_code=401, detail="user_id is required")

    # RULE 2: Reject request if user_profile missing
    if not request.user_profile:
        raise HTTPException(status_code=401, detail="user_profile is required")

    # Continue with existing RAG logic...
    # Profile affects ONLY explanation depth in generate_answer()
```

---

## Personalization Guardrail (RAG Safety)

### What Profile MAY Do
- Adjust response tone (formal vs. conversational)
- Adjust explanation depth (beginner: more detail, advanced: concise)
- Adjust examples (simulator vs. hardware-focused)

### What Profile MUST NOT Do
- Add facts not in retrieved content
- Override or modify citations
- Hallucinate based on profile assumptions

### System Prompt Extension

```python
def build_personalized_system_prompt(user_profile: UserProfile) -> str:
    base_prompt = SYSTEM_PROMPT  # Existing grounding prompt

    # Add personalization instructions
    personalization = f"""
PERSONALIZATION CONTEXT (affects tone only, NOT facts):
- User programming level: {user_profile.programming_level}
- Familiar technologies: {', '.join(user_profile.technologies) or 'None specified'}
- Hardware access: {user_profile.hardware_access}

Adjust your explanations:
- For beginners: Use simpler language, more step-by-step detail
- For intermediate: Balance detail with conciseness
- For advanced: Be concise, assume familiarity with concepts
- For simulator_only: Focus on simulation-based workflows
- For real_robots: Include practical hardware considerations

CRITICAL: Do NOT invent information. ALL facts must come from the provided context."""

    return base_prompt + "\n\n" + personalization
```

---

## Implementation Phases

### Phase 1: Auth Server Setup (Better Auth + Express)

**Goal**: Standalone auth server handling signup, signin, sessions

**Tasks**:
1. Create `auth-server/` directory structure
2. Initialize npm project with dependencies:
   - `better-auth`, `express`, `pg`, `dotenv`, `cors`
3. Configure Better Auth with PostgreSQL adapter + additionalFields
4. Set up Express routes for Better Auth handler
5. Configure CORS for frontend origins
6. Generate and run database migrations
7. Create `.env.example` with required variables
8. Test locally: signup, signin, session retrieval

**Deliverables**:
- `auth-server/src/auth.ts` - Better Auth config
- `auth-server/src/index.ts` - Express server
- `auth-server/package.json`, `tsconfig.json`
- `auth-server/.env.example`

---

### Phase 2: Frontend Auth Integration

**Goal**: Better Auth client, auth modal, profile form, chatbot gate

**Tasks**:
1. Install `@better-auth/react` in frontend
2. Create `lib/auth-client.ts` with Better Auth client instance
3. Create `AuthProvider.tsx` React context wrapper
4. Create `AuthModal.tsx` with sign in/sign up tabs
5. Create `ProfileForm.tsx` for profile collection during signup
6. Update `theme/Root.js` to wrap app with AuthProvider
7. Update `Chatbot/index.jsx` with auth gate logic
8. Update `hooks/useApiClient.js` to inject user context

**Deliverables**:
- `components/Auth/AuthProvider.tsx`
- `components/Auth/AuthModal.tsx`
- `components/Auth/ProfileForm.tsx`
- `lib/auth-client.ts`
- Modified `theme/Root.js`
- Modified `Chatbot/index.jsx`
- Modified `hooks/useApiClient.js`

---

### Phase 3: Backend /ask Extension

**Goal**: Extend /ask endpoint to accept and use user context

**Tasks**:
1. Create `database/models.py` with UserProfile Pydantic model
2. Extend AskRequest in `app.py` with user_id and user_profile
3. Add validation: reject requests without user context
4. Build personalization prompt injection
5. Ensure RAG grounding is preserved (facts from Qdrant only)
6. Test with different profile types

**Deliverables**:
- `database/models.py`
- Modified `database/app.py`
- Personalization system prompt extension

---

### Phase 4: E2E Flow Validation & Deployment

**Goal**: Complete end-to-end testing and production deployment

**Tasks**:
1. Test complete signup flow (email → profile → chatbot access)
2. Test signin flow (returning user)
3. Test session persistence across page reloads
4. Test chatbot gate (unauthenticated, incomplete profile, complete)
5. Test personalization with 3 different profile types
6. Deploy auth server to Render
7. Update frontend environment for production auth URL
8. Verify CORS configuration in production

**CORS Configuration**:
```typescript
// Allow credentials for cookie-based auth
// auth-server
app.use(cors({
  origin: [
    "http://localhost:3000",
    "https://ismailabdulkareem.github.io"
  ],
  credentials: true,
}));
```

---

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| CORS issues between static site and auth server | Configure trustedOrigins in Better Auth, test cross-origin in dev |
| Session cookie not persisting | Verify SameSite, Secure flags; use credentials: true on fetch |
| Profile data race condition | Atomic signup enforced (FR-022) - single request creates user + profile |
| Personalization leaking into facts | RAG safety constraint in system prompt; code review for prompt injection |
| Better Auth version compatibility | Pin version in package.json; test migrations |

---

## Deployment Configuration

| Service | Platform | URL |
|---------|----------|-----|
| Auth Server | Render (Web Service) | `https://auth.your-domain.com` |
| Backend | Render (existing) | `https://ai-book-h6kj.onrender.com` |
| Frontend | GitHub Pages | `https://ismailabdulkareem.github.io` |

### Environment Variables (Auth Server)

```bash
# auth-server/.env.example
DATABASE_URL=postgresql://user:pass@host:5432/dbname
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
FRONTEND_URL=https://ismailabdulkareem.github.io
```

---

## Complexity Tracking

No complexity violations. Design uses:
- 1 new service (auth server) - justified by language mismatch (Node vs Python)
- Standard patterns (React Context, Express middleware, Pydantic models)
- Minimal backend changes (2 new fields in request model, prompt extension)

---

## Next Steps

Run `/sp.tasks` to generate implementation tasks from this plan.
