---
title: AI Book Auth Server
emoji: 🔐
colorFrom: green
colorTo: blue
sdk: docker
pinned: false
license: mit
---

# AI Book Authentication Server

Better Auth server for the Physical AI & Humanoid Robotics book chatbot.

## Features

- Email/password authentication
- Session management with cookies
- User profile storage
- PostgreSQL database integration
- CORS configured for frontend access

## API Endpoints

- `GET /health` - Health check
- `GET /` - Service info
- `POST /api/auth/sign-in/email` - Email sign in
- `POST /api/auth/sign-up/email` - Email sign up
- `POST /api/auth/sign-out` - Sign out
- `GET /api/auth/session` - Get current session

## Environment Variables

Required secrets (set in Hugging Face Space settings):

```
DATABASE_URL=your-postgresql-connection-string
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
BETTER_AUTH_URL=https://your-space.hf.space
```

## Usage

Frontend integration:
```javascript
import { createAuthClient } from "better-auth/react";

const authClient = createAuthClient({
  baseURL: "https://your-auth-space.hf.space"
});
```

## Local Development

```bash
npm install
npm run dev
```

Visit http://localhost:3001/health for health check.

## Database Setup

The auth server uses PostgreSQL (Neon) for user data storage. Make sure to:
1. Create a Neon database
2. Set DATABASE_URL in environment variables
3. Better Auth will auto-create tables on first run
