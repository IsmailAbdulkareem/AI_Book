# API Contract: Frontend → RAG Agent API

**Feature Branch**: `004-frontend-chatbot`
**Date**: 2025-12-16
**Backend Spec**: Spec 3 (003-agent-rag-integration)

---

## Overview

This document describes the HTTP API contract between the frontend chatbot and the RAG Agent backend. The backend API is defined in Spec 3 and MUST NOT be modified by this feature.

---

## Base URL

```
Development: http://localhost:8000
Production: [Configured via environment variable]
```

---

## Endpoints

### POST /ask

Submit a question and receive a grounded answer with source citations.

**Request**:
```http
POST /ask HTTP/1.1
Host: localhost:8000
Content-Type: application/json

{
  "question": "What is ROS 2?",
  "top_k": 5
}
```

**Request Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| question | string | Yes | The question to ask (min 1 char) |
| top_k | integer | No | Number of sources to retrieve (1-20, default: 5) |

**Response (200 OK)**:
```json
{
  "question": "What is ROS 2?",
  "answer": "ROS 2 (Robot Operating System 2) is middleware specifically designed for robotics. It provides a communication framework, tools, and libraries for building robot software [1][2].",
  "sources": [
    {
      "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module1/module1-chapter2",
      "content": "ROS 2 is middleware specifically designed for robotics...",
      "score": 0.66,
      "position": 1
    }
  ],
  "retrieval_time_ms": 1804,
  "generation_time_ms": 3558
}
```

**Response Fields**:
| Field | Type | Description |
|-------|------|-------------|
| question | string | Echo of the submitted question |
| answer | string | Generated answer with [N] citations |
| sources | Source[] | Array of source citations |
| retrieval_time_ms | integer | Retrieval time in milliseconds |
| generation_time_ms | integer | Generation time in milliseconds |

**Error Responses**:

| Status | Error Type | When |
|--------|------------|------|
| 400 | validation_error | Empty question or invalid top_k |
| 500 | generation_error | OpenAI API failure |
| 503 | service_unavailable | Qdrant unavailable |

**Error Response Format**:
```json
{
  "detail": {
    "error": "validation_error",
    "message": "Question cannot be empty",
    "details": {}
  }
}
```

---

### GET /health

Check service and dependency health.

**Request**:
```http
GET /health HTTP/1.1
Host: localhost:8000
```

**Response (200 OK)**:
```json
{
  "status": "healthy",
  "dependencies": {
    "qdrant": {
      "status": "connected",
      "details": { "vector_count": 135 }
    },
    "openai": {
      "status": "configured",
      "details": { "key_prefix": "sk-proj-..." }
    }
  }
}
```

**Status Values**:
| Overall Status | Meaning |
|----------------|---------|
| healthy | All dependencies working |
| degraded | Some dependencies have issues |
| unhealthy | Critical dependencies (Qdrant) unavailable |

---

## Frontend API Client Implementation

```typescript
// src/components/Chatbot/api.ts

const API_BASE_URL = process.env.RAG_API_URL || 'http://localhost:8000';

export interface AskRequest {
  question: string;
  top_k?: number;
}

export interface Source {
  url: string;
  content: string;
  score: number;
  position: number;
}

export interface AskResponse {
  question: string;
  answer: string;
  sources: Source[];
  retrieval_time_ms: number;
  generation_time_ms: number;
}

export interface HealthResponse {
  status: 'healthy' | 'degraded' | 'unhealthy';
  dependencies: Record<string, { status: string; details?: unknown }>;
}

export class APIError extends Error {
  constructor(
    public status: number,
    public errorType: string,
    message: string
  ) {
    super(message);
    this.name = 'APIError';
  }
}

export async function askQuestion(request: AskRequest): Promise<AskResponse> {
  const response = await fetch(`${API_BASE_URL}/ask`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({}));
    throw new APIError(
      response.status,
      error.detail?.error || 'unknown_error',
      error.detail?.message || 'An error occurred'
    );
  }

  return response.json();
}

export async function checkHealth(): Promise<HealthResponse> {
  const response = await fetch(`${API_BASE_URL}/health`);

  if (!response.ok) {
    throw new APIError(response.status, 'health_check_failed', 'Health check failed');
  }

  return response.json();
}
```

---

## CORS Configuration

The backend (Spec 3) is configured with permissive CORS for development:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Note**: For production, CORS should be restricted to the deployed frontend domain.

---

## Error Handling Strategy

Frontend should handle these scenarios:

| Scenario | User Message | Action |
|----------|--------------|--------|
| Network error | "Unable to connect. Please check your connection." | Show retry button |
| 400 validation | "Please enter a valid question." | Highlight input field |
| 500 server error | "Something went wrong. Please try again." | Show retry button |
| 503 service unavailable | "The service is temporarily unavailable." | Show retry button |
| Timeout (>30s) | "Request timed out. Please try again." | Show retry button |

---

## Request Timeout

Frontend should implement a 30-second timeout for API requests:

```typescript
const controller = new AbortController();
const timeoutId = setTimeout(() => controller.abort(), 30000);

try {
  const response = await fetch(url, {
    ...options,
    signal: controller.signal,
  });
  clearTimeout(timeoutId);
  return response;
} catch (error) {
  clearTimeout(timeoutId);
  if (error.name === 'AbortError') {
    throw new APIError(0, 'timeout', 'Request timed out');
  }
  throw error;
}
```
