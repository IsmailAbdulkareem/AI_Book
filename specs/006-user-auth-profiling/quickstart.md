# Quickstart: Authentication + User Profiling

**Feature**: 006-user-auth-profiling
**Date**: 2025-12-17

This guide provides step-by-step instructions for implementing authentication and user profiling.

---

## Prerequisites

- Node.js 18+ (for Better Auth server)
- Python 3.11+ (existing FastAPI backend)
- PostgreSQL database (existing)
- npm/pnpm (frontend package manager)
- uv (Python package manager)

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────┐
│  GitHub Pages (Static)                                           │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Docusaurus + Chatbot Plugin                               │  │
│  │  + AuthProvider (React Context)                            │  │
│  │  + AuthModal (Sign In / Sign Up)                           │  │
│  │  + ProfileForm (during signup)                             │  │
│  └──────────────┬─────────────────────────────────────────────┘  │
└─────────────────┼────────────────────────────────────────────────┘
                  │
       ┌──────────┴──────────┐
       ↓                     ↓
┌──────────────────┐  ┌─────────────────────────────────────┐
│ Better Auth      │  │ FastAPI Backend                     │
│ Server (Node.js) │  │ POST /ask + user context            │
│                  │  │                                     │
│ /api/auth/*      │  │ Reads user profile from shared DB   │
└────────┬─────────┘  └──────────────┬──────────────────────┘
         │                           │
         └─────────────┬─────────────┘
                       ↓
              ┌────────────────┐
              │  PostgreSQL    │
              │  (Shared DB)   │
              └────────────────┘
```

---

## Step 1: Set Up Better Auth Server

### 1.1 Create Auth Server Directory

```bash
mkdir auth-server
cd auth-server
npm init -y
```

### 1.2 Install Dependencies

```bash
npm install better-auth express pg dotenv cors
npm install -D typescript @types/node @types/express tsx
```

### 1.3 Create TypeScript Config

```bash
# tsconfig.json
cat > tsconfig.json << 'EOF'
{
  "compilerOptions": {
    "target": "ES2022",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "outDir": "dist"
  },
  "include": ["src/**/*"]
}
EOF
```

### 1.4 Create Auth Configuration

```typescript
// src/auth.ts
import { betterAuth } from "better-auth";
import { Pool } from "pg";

export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },
  user: {
    additionalFields: {
      programmingLevel: {
        type: "string",
        required: true,
        input: true,
      },
      technologies: {
        type: "string",
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
        type: "string",
        required: false,
        input: true,
      },
    },
  },
  trustedOrigins: [
    "https://your-book-site.github.io",
    "http://localhost:3000",
  ],
});
```

### 1.5 Create Express Server

```typescript
// src/index.ts
import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth";
import "dotenv/config";

const app = express();
const port = process.env.PORT || 3001;

// CORS for frontend
app.use(cors({
  origin: [
    "https://your-book-site.github.io",
    "http://localhost:3000",
  ],
  credentials: true,
}));

// Better Auth handler
app.all("/api/auth/*", toNodeHandler(auth));

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "healthy" });
});

app.listen(port, () => {
  console.log(`Auth server running on port ${port}`);
});
```

### 1.6 Add Scripts to package.json

```json
{
  "scripts": {
    "dev": "tsx watch src/index.ts",
    "build": "tsc",
    "start": "node dist/index.js",
    "db:generate": "npx @better-auth/cli generate",
    "db:migrate": "npx @better-auth/cli migrate"
  }
}
```

### 1.7 Environment Variables

```bash
# .env
DATABASE_URL=postgresql://user:password@host:5432/dbname
BETTER_AUTH_SECRET=your-32-character-random-secret
BETTER_AUTH_URL=http://localhost:3001
```

### 1.8 Generate and Run Migrations

```bash
npm run db:generate
npm run db:migrate
npm run dev
```

---

## Step 2: Frontend Integration

### 2.1 Install Better Auth Client

```bash
cd /path/to/docusaurus-site
npm install better-auth
```

### 2.2 Create Auth Client

```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3001",
});

export const {
  useSession,
  signIn,
  signUp,
  signOut,
} = authClient;
```

### 2.3 Create Auth Provider

```tsx
// src/components/AuthProvider.tsx
import React, { createContext, useContext, ReactNode } from "react";
import { useSession } from "../lib/auth-client";

interface AuthContextType {
  session: any;
  isPending: boolean;
  error: Error | null;
  isAuthenticated: boolean;
  isProfileComplete: boolean;
}

const AuthContext = createContext<AuthContextType | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: session, isPending, error } = useSession();

  const isAuthenticated = !!session?.user;
  const isProfileComplete = isAuthenticated &&
    session.user.programmingLevel &&
    session.user.technologies &&
    session.user.hardwareAccess;

  return (
    <AuthContext.Provider value={{
      session,
      isPending,
      error,
      isAuthenticated,
      isProfileComplete,
    }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error("useAuth must be used within AuthProvider");
  }
  return context;
}
```

### 2.4 Create Auth Modal Component

```tsx
// src/components/AuthModal.tsx
import React, { useState } from "react";
import { signIn, signUp } from "../lib/auth-client";

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export function AuthModal({ isOpen, onClose }: AuthModalProps) {
  const [mode, setMode] = useState<"signin" | "signup">("signin");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [error, setError] = useState("");
  const [loading, setLoading] = useState(false);

  // Profile fields (for signup)
  const [programmingLevel, setProgrammingLevel] = useState("");
  const [technologies, setTechnologies] = useState<string[]>([]);
  const [aiRoboticsExperience, setAiRoboticsExperience] = useState(false);
  const [hardwareAccess, setHardwareAccess] = useState("");

  if (!isOpen) return null;

  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    const result = await signIn.email({
      email,
      password,
    });

    if (result.error) {
      setError(result.error.message);
    } else {
      onClose();
    }
    setLoading(false);
  };

  const handleSignUp = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError("");

    const result = await signUp.email({
      email,
      password,
      name,
      programmingLevel,
      technologies: JSON.stringify(technologies),
      aiRoboticsExperience,
      hardwareAccess,
    });

    if (result.error) {
      setError(result.error.message);
    } else {
      onClose();
    }
    setLoading(false);
  };

  return (
    <div className="auth-modal-overlay">
      <div className="auth-modal">
        {/* Modal content - see full implementation in plan */}
      </div>
    </div>
  );
}
```

### 2.5 Update Chatbot Component

```tsx
// In chatbot component
import { useAuth } from "../AuthProvider";
import { AuthModal } from "../AuthModal";

export function Chatbot() {
  const { isAuthenticated, isProfileComplete, session } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [isOpen, setIsOpen] = useState(false);

  const handleChatbotClick = () => {
    if (!isAuthenticated) {
      setShowAuthModal(true);
      return;
    }
    if (!isProfileComplete) {
      // Redirect to profile completion
      return;
    }
    setIsOpen(true);
  };

  // Pass user context to /ask endpoint
  const askQuestion = async (question: string, context?: string) => {
    const response = await fetch(`${API_URL}/ask`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        question,
        context,
        top_k: 5,
        user_id: session.user.id,
        user_profile: {
          programming_level: session.user.programmingLevel,
          technologies: JSON.parse(session.user.technologies),
          hardware_access: session.user.hardwareAccess,
        },
      }),
    });
    return response.json();
  };

  return (
    <>
      <AuthModal isOpen={showAuthModal} onClose={() => setShowAuthModal(false)} />
      {/* Chatbot UI */}
    </>
  );
}
```

---

## Step 3: Backend Extension

### 3.1 Update Pydantic Models

```python
# database/models.py
from typing import List, Literal, Optional
from pydantic import BaseModel, Field

class UserProfile(BaseModel):
    programming_level: Literal["beginner", "intermediate", "advanced"]
    technologies: List[str]
    hardware_access: Literal["none", "simulator_only", "real_robots"]

class AskRequest(BaseModel):
    question: str
    context: Optional[str] = None
    top_k: int = Field(default=5, ge=1, le=10)
    user_id: str
    user_profile: UserProfile
```

### 3.2 Update /ask Endpoint

```python
# database/app.py
from models import AskRequest, UserProfile

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    # Validate user_id (could verify against DB)
    if not request.user_id:
        raise HTTPException(status_code=401, detail="user_id is required")

    # Build personalized system prompt
    personalization = build_personalization_prompt(request.user_profile)

    # Existing RAG flow
    retrieval_start = time.time()
    results = pipeline.search(request.question, top_k=request.top_k)
    retrieval_time = (time.time() - retrieval_start) * 1000

    # Generate with personalization
    generation_start = time.time()
    answer = generate_answer(
        question=request.question,
        context=results,
        personalization=personalization,
    )
    generation_time = (time.time() - generation_start) * 1000

    return AskResponse(...)


def build_personalization_prompt(profile: UserProfile) -> str:
    return f"""
Adapt your response style based on user context:
- Programming Level: {profile.programming_level}
- Familiar Technologies: {', '.join(profile.technologies)}
- Hardware Access: {profile.hardware_access}

Guidelines:
- For beginners: Use simpler language, explain concepts thoroughly
- For intermediate: Balance detail with conciseness
- For advanced: Be concise, assume familiarity with concepts
- Reference technologies the user knows when giving examples
- Tailor hardware examples to their access level

IMPORTANT: Do NOT change factual content. Only adjust tone and complexity.
All information must come from the provided context sources.
"""
```

---

## Step 4: Environment Setup

### Frontend (.env)

```bash
BETTER_AUTH_URL=https://your-auth-server.onrender.com
```

### Auth Server (.env)

```bash
DATABASE_URL=postgresql://user:pass@host:5432/dbname
BETTER_AUTH_SECRET=<32-char-random-string>
BETTER_AUTH_URL=https://your-auth-server.onrender.com
PORT=3001
```

### Backend (.env)

```bash
# Existing vars
OPENAI_API_KEY=...
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...
# No changes needed - reads from same DATABASE_URL
```

---

## Step 5: Deployment

### Auth Server on Render

1. Create new Web Service
2. Connect to repository (auth-server directory)
3. Build command: `npm install && npm run build`
4. Start command: `npm start`
5. Add environment variables
6. Deploy

### Update Frontend Build

1. Add `BETTER_AUTH_URL` to GitHub Actions secrets
2. Update build to inject env var
3. Rebuild and deploy

---

## Testing Checklist

- [ ] Auth server health check responds
- [ ] Signup creates user with profile
- [ ] Signin returns session with profile
- [ ] Session persists across page reload
- [ ] Chatbot blocked when not authenticated
- [ ] Auth modal appears on chatbot click (unauthenticated)
- [ ] Profile data sent in /ask requests
- [ ] Responses personalized based on profile
- [ ] Logout clears session

---

## Troubleshooting

### CORS Errors
- Ensure frontend origin in `trustedOrigins`
- Check CORS middleware in auth server

### Session Not Persisting
- Verify `credentials: true` in fetch calls
- Check cookie settings for cross-origin

### Profile Fields Missing
- Run `db:migrate` to update schema
- Check `additionalFields` in auth config
