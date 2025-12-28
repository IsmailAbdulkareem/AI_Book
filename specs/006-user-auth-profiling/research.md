# Research: Authentication + User Profiling

**Feature**: 006-user-auth-profiling
**Date**: 2025-12-17
**Status**: Complete

## Research Summary

This document captures all research findings for implementing authentication and user profiling using Better Auth in the existing Docusaurus + FastAPI RAG chatbot system.

---

## 1. Better Auth Integration Pattern

### Decision: Client-Side React Integration with Separate Auth Server

**Rationale**: Better Auth is a TypeScript-first authentication framework. Since Docusaurus is a static site generator with client-side React, we need:
1. A separate Better Auth server (Node.js/Express or standalone)
2. Client-side `authClient` in the Docusaurus React components

**Alternatives Considered**:
- NextAuth.js - Rejected: Tightly coupled to Next.js, not Docusaurus-compatible
- Custom JWT auth - Rejected: Spec explicitly prohibits custom auth logic
- Firebase Auth - Rejected: Not specified in requirements

### Implementation Pattern

```typescript
// Frontend: lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL // Points to auth server
});

export const { useSession, signIn, signUp, signOut } = authClient;
```

```typescript
// Auth Server: lib/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({ connectionString: process.env.DATABASE_URL }),
  emailAndPassword: { enabled: true },
  user: {
    additionalFields: {
      // Profile fields defined here
    }
  }
});
```

---

## 2. Database Adapter Choice

### Decision: PostgreSQL with `pg` Pool

**Rationale**:
- Backend already uses PostgreSQL driver (`psycopg[binary]` in requirements.txt)
- SQLAlchemy 2.0 is already a dependency
- Better Auth supports direct `pg` Pool connection
- Shared database enables FastAPI to read user profiles

**Alternatives Considered**:
- SQLite - Rejected: Not suitable for production deployment on Render
- Drizzle ORM - Rejected: Additional complexity, no existing usage
- Prisma - Rejected: Additional build step, overkill for auth tables

### Database Tables Required

Better Auth auto-creates:
- `user` - Core user table (email, name, emailVerified, image, createdAt, updatedAt)
- `session` - Session management
- `account` - OAuth accounts (not needed for email/password)
- `verification` - Email verification tokens

Custom profile fields added to `user` table via `additionalFields`.

---

## 3. User Profile Schema Design

### Decision: Store Profile in User Table via additionalFields

**Rationale**:
- Better Auth supports `additionalFields` for extending user schema
- Single table simplifies queries and atomic signup
- Profile data is type-safe in TypeScript

**Alternatives Considered**:
- Separate `user_profile` table - Rejected: Breaks atomic signup, requires joins
- JSON blob field - Rejected: Loses type safety and query capability

### Profile Fields Definition

```typescript
user: {
  additionalFields: {
    programmingLevel: {
      type: "string",
      required: true,
      input: true, // Allow during signup
    },
    technologies: {
      type: "string", // JSON array stored as string
      required: true,
      input: true,
    },
    aiRoboticsExperience: {
      type: "boolean",
      required: true,
      input: true,
    },
    hardwareAccess: {
      type: "string",
      required: true,
      input: true,
    },
    devicesOwned: {
      type: "string", // JSON array stored as string, optional
      required: false,
      input: true,
    },
  },
}
```

**Note**: Arrays stored as JSON strings since Better Auth doesn't support array types directly.

---

## 4. Session Management

### Decision: Cookie-Based Sessions with 7-Day Persistence

**Rationale**:
- Better Auth uses secure HTTP-only cookies by default
- `rememberMe: true` enables persistent sessions
- Meets SC-005 requirement (7-day persistence)

**Session Flow**:
1. User signs in → Better Auth creates session record
2. Session token stored in HTTP-only cookie
3. `useSession()` hook retrieves session on each page load
4. Session auto-refreshes on activity

### Session Access Pattern

```typescript
// React Component
const { data: session, isPending, error } = authClient.useSession();

if (isPending) return <Loading />;
if (!session) return <AuthModal />;
// User is authenticated, session.user contains profile
```

---

## 5. Auth Server Architecture

### Decision: Standalone Express Auth Server (Deployed Alongside FastAPI)

**Rationale**:
- Docusaurus is static (GitHub Pages) - cannot run server code
- FastAPI is Python, Better Auth is TypeScript - cannot co-locate
- Render supports multiple services - deploy auth server separately

**Alternatives Considered**:
- Cloudflare Workers - Rejected: Additional platform complexity
- Edge functions - Rejected: Not needed for auth workload
- Embed in FastAPI - Rejected: Language mismatch (TypeScript vs Python)

### Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  GitHub Pages (Static)                                      │
│  Docusaurus + React Chatbot + Better Auth Client           │
└──────────────┬─────────────────────────────────────────────┘
               │
    ┌──────────┴──────────┐
    ↓                     ↓
┌────────────────┐  ┌─────────────────────────────────────────┐
│ Better Auth    │  │ FastAPI Backend                         │
│ Server (Node)  │  │ POST /ask (with user_id, user_profile) │
│ Render Service │  │ Render Service                          │
└───────┬────────┘  └──────────────┬──────────────────────────┘
        │                          │
        └───────────┬──────────────┘
                    ↓
           ┌────────────────┐
           │ PostgreSQL DB  │
           │ (Shared)       │
           │ - user table   │
           │ - session table│
           │ - qdrant refs  │
           └────────────────┘
```

---

## 6. Frontend Auth Gate Implementation

### Decision: Context Provider with Modal-Based Auth UI

**Rationale**:
- React Context provides global auth state
- Modal prevents full page navigation (better UX)
- Lazy loading prevents bundle bloat

### Implementation Pattern

```typescript
// AuthProvider wraps entire app
<AuthProvider>
  <App />
</AuthProvider>

// Chatbot checks auth before opening
const Chatbot = () => {
  const { session, isPending } = useSession();
  const [showAuthModal, setShowAuthModal] = useState(false);

  const handleOpen = () => {
    if (!session) {
      setShowAuthModal(true);
      return;
    }
    // Check profile completeness
    if (!isProfileComplete(session.user)) {
      redirectToProfileCompletion();
      return;
    }
    openChatbot();
  };
};
```

---

## 7. Backend User Context Injection

### Decision: Extend /ask Endpoint to Accept User Context

**Rationale**:
- Minimal change to existing FastAPI endpoint
- Profile validation at backend
- RAG safety maintained (profile only influences tone)

### FastAPI Changes

```python
# Extended request model
class AskRequest(BaseModel):
    question: str
    context: Optional[str] = None
    top_k: int = Field(default=5, ge=1, le=10)
    user_id: str  # NEW: Required
    user_profile: UserProfile  # NEW: Required

class UserProfile(BaseModel):
    programming_level: Literal["beginner", "intermediate", "advanced"]
    technologies: List[str]
    hardware_access: Literal["none", "simulator_only", "real_robots"]
```

### Prompt Personalization (RAG-Safe)

```python
PERSONALIZATION_PROMPT = """
Adapt your response style based on user context:
- Programming Level: {programming_level}
- Familiar Technologies: {technologies}
- Hardware Access: {hardware_access}

Guidelines:
- For beginners: Use simpler language, explain concepts
- For advanced: Be concise, assume familiarity
- Reference technologies the user knows when possible
- Tailor examples to their hardware access level

IMPORTANT: Do NOT change factual content. Only adjust tone and complexity.
All information must come from the provided context.
"""
```

---

## 8. Package Dependencies

### Frontend (package.json additions)

```json
{
  "dependencies": {
    "better-auth": "^1.2.0"
  }
}
```

### Auth Server (new package.json)

```json
{
  "dependencies": {
    "better-auth": "^1.2.0",
    "express": "^4.18.0",
    "pg": "^8.11.0",
    "dotenv": "^16.0.0"
  }
}
```

### Backend (requirements.txt additions)

None required - PostgreSQL drivers already present.

---

## 9. Environment Variables

### Frontend (.env)
```
BETTER_AUTH_URL=https://your-auth-server.onrender.com
```

### Auth Server (.env)
```
DATABASE_URL=postgresql://user:pass@host:5432/dbname
BETTER_AUTH_SECRET=<random-32-char-string>
BETTER_AUTH_URL=https://your-auth-server.onrender.com
```

### Backend (.env)
```
DATABASE_URL=postgresql://user:pass@host:5432/dbname
# Same DB as auth server - can read user table
```

---

## 10. Security Considerations

### Implemented by Better Auth
- Password hashing (Argon2 by default)
- CSRF protection
- Secure HTTP-only cookies
- Session token rotation
- Rate limiting (configurable)

### Additional Measures Required
- CORS configuration (auth server)
- HTTPS enforcement (Render handles)
- Input validation (Pydantic for FastAPI)

---

## Research Gaps Resolved

| Unknown | Resolution |
|---------|------------|
| Better Auth + Docusaurus compatibility | Client SDK works with any React app |
| Profile storage mechanism | additionalFields in user table |
| Atomic signup implementation | Single signUp.email call with all fields |
| Session persistence duration | Cookie-based, configurable expiry |
| Auth server deployment | Standalone Express on Render |
| FastAPI user context access | Shared PostgreSQL database |

---

## Next Steps

1. Generate `data-model.md` with entity definitions
2. Generate API contracts in `/contracts/`
3. Create `quickstart.md` with setup instructions
4. Update plan.md with technical decisions
