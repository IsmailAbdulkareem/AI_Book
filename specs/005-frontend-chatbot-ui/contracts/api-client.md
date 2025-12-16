# API Client Contract: Frontend RAG Chatbot

**Feature**: 005-frontend-chatbot-ui
**Date**: 2025-12-17
**Backend Spec**: 003-agent-rag-integration

---

## Overview

This document defines the API contract between the frontend chatbot UI and the RAG backend. The frontend is a pure consumer of the backend API and performs no retrieval or generation logic.

---

## Base URL Configuration

```javascript
// Priority order:
// 1. Plugin config: options.apiUrl
// 2. Environment variable: process.env.RAG_CHATBOT_API_URL
// 3. Default: 'http://localhost:8000'

const API_BASE_URL = options.apiUrl
  || process.env.RAG_CHATBOT_API_URL
  || 'http://localhost:8000';
```

---

## Endpoints

### POST /ask

Ask a question and receive a grounded answer with citations.

#### Request

```http
POST /ask HTTP/1.1
Host: {API_BASE_URL}
Content-Type: application/json
```

**Body**:
```json
{
  "question": "What is ROS 2?",
  "top_k": 5
}
```

| Field | Type | Required | Default | Constraints |
|-------|------|----------|---------|-------------|
| question | string | Yes | - | min length: 1 |
| top_k | integer | No | 5 | min: 1, max: 20 |

**Note**: The spec defines a `context` field for selected text, but the current backend doesn't implement it. Frontend should send it for forward compatibility:

```json
{
  "question": "Explain this concept",
  "context": "ROS 2 uses DDS for communication...",
  "top_k": 5
}
```

#### Response

**Success (200 OK)**:
```json
{
  "question": "What is ROS 2?",
  "answer": "ROS 2 (Robot Operating System 2) is an open-source robotics middleware framework [1]. It provides tools and libraries for building robot applications [2].",
  "sources": [
    {
      "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-2",
      "content": "ROS 2 is a middleware framework that provides a collection of tools, libraries, and conventions...",
      "score": 0.89,
      "position": 1
    },
    {
      "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-3",
      "content": "Building robot applications with ROS 2 involves creating nodes that communicate...",
      "score": 0.82,
      "position": 2
    }
  ],
  "retrieval_time_ms": 120,
  "generation_time_ms": 640
}
```

| Field | Type | Always Present | Description |
|-------|------|----------------|-------------|
| question | string | Yes | Echo of original question |
| answer | string | Yes | Generated answer with `[N]` citation markers |
| sources | array | Yes | Array of source objects (may be empty) |
| sources[].url | string | Yes | URL to book section |
| sources[].content | string | Yes | Content snippet |
| sources[].score | number | Yes | Relevance score (0-1) |
| sources[].position | number | Yes | Citation number (1-indexed) |
| retrieval_time_ms | integer | Yes | Retrieval latency in ms |
| generation_time_ms | integer | Yes | Generation latency in ms |

**Error (400 Bad Request)**:
```json
{
  "detail": "Validation error message"
}
```

**Error (500 Internal Server Error)**:
```json
{
  "detail": "Error retrieving context"
}
```

**Error (503 Service Unavailable)**:
```json
{
  "detail": "Service temporarily unavailable"
}
```

---

### GET /health

Check backend service health.

#### Request

```http
GET /health HTTP/1.1
Host: {API_BASE_URL}
```

#### Response

**Healthy (200 OK)**:
```json
{
  "status": "healthy",
  "dependencies": {
    "qdrant": {
      "status": "connected",
      "details": { "collections": 1 }
    },
    "openai": {
      "status": "configured",
      "details": null
    },
    "cohere": {
      "status": "configured",
      "details": null
    }
  }
}
```

**Degraded (200 OK)**:
```json
{
  "status": "degraded",
  "dependencies": {
    "qdrant": {
      "status": "error",
      "details": { "error": "Connection refused" }
    }
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| status | string | "healthy", "degraded", or "unhealthy" |
| dependencies | object | Status of each dependency |

---

## Frontend API Client Implementation

```typescript
// api-client.ts

interface AskRequest {
  question: string;
  context?: string;
  top_k?: number;
}

interface Source {
  url: string;
  content: string;
  score: number;
  position: number;
}

interface AskResponse {
  question: string;
  answer: string;
  sources: Source[];
  retrieval_time_ms: number;
  generation_time_ms: number;
}

interface ApiError {
  detail: string;
}

const DEFAULT_TIMEOUT = 30000; // 30 seconds

export async function askQuestion(
  apiUrl: string,
  request: AskRequest
): Promise<AskResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), DEFAULT_TIMEOUT);

  try {
    const response = await fetch(`${apiUrl}/ask`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        question: request.question,
        context: request.context,
        top_k: request.top_k ?? 5,
      }),
      signal: controller.signal,
    });

    if (!response.ok) {
      const error: ApiError = await response.json().catch(() => ({
        detail: `HTTP ${response.status}: ${response.statusText}`,
      }));
      throw new Error(error.detail);
    }

    return await response.json();
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new Error('Request timed out. Please try again.');
    }
    throw error;
  } finally {
    clearTimeout(timeoutId);
  }
}

export async function checkHealth(apiUrl: string): Promise<boolean> {
  try {
    const response = await fetch(`${apiUrl}/health`, {
      method: 'GET',
      signal: AbortSignal.timeout(5000),
    });
    if (!response.ok) return false;
    const data = await response.json();
    return data.status === 'healthy';
  } catch {
    return false;
  }
}
```

---

## Error Handling Strategy

### Frontend Error Messages

| Backend Status | Frontend Message |
|----------------|------------------|
| Network error | "Unable to reach the assistant. Please check your connection." |
| 400 | "Invalid question. Please try rephrasing." |
| 500 | "Something went wrong. Please try again." |
| 503 | "The assistant is temporarily unavailable. Please try again later." |
| Timeout | "Request timed out. Please try again." |
| Empty sources | Display answer as-is (backend handles refusal) |

### Retry Logic

```typescript
const MAX_RETRIES = 2;
const RETRY_DELAY = 1000;

async function askWithRetry(
  apiUrl: string,
  request: AskRequest
): Promise<AskResponse> {
  let lastError: Error;

  for (let attempt = 0; attempt <= MAX_RETRIES; attempt++) {
    try {
      return await askQuestion(apiUrl, request);
    } catch (error) {
      lastError = error;
      if (attempt < MAX_RETRIES) {
        await new Promise(r => setTimeout(r, RETRY_DELAY * (attempt + 1)));
      }
    }
  }

  throw lastError;
}
```

---

## CORS Requirements

The backend must allow:
- Origin: `https://ismailabdulkareem.github.io` (production)
- Origin: `http://localhost:3000` (development)
- Methods: `GET`, `POST`, `OPTIONS`
- Headers: `Content-Type`

**Backend CORS configuration** (already in `database/agent.py`):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Or specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Response Mapping

### Backend → Frontend Entity Mapping

```typescript
function mapApiResponseToMessage(
  response: AskResponse,
  messageId: string
): ChatMessage {
  return {
    id: messageId,
    role: 'assistant',
    content: response.answer,
    timestamp: Date.now(),
    sources: response.sources.map(s => ({
      id: s.position,
      url: s.url,
      contentPreview: s.content.slice(0, 200),
      score: s.score,
    })),
    timing: {
      retrieval_ms: response.retrieval_time_ms,
      generation_ms: response.generation_time_ms,
    },
  };
}
```
