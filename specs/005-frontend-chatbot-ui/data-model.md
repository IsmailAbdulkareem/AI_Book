# Data Model: Frontend RAG Chatbot UI

**Feature**: 005-frontend-chatbot-ui
**Date**: 2025-12-17
**Status**: Complete

---

## Overview

This document defines the frontend data structures for the chatbot UI. All data is client-side only with session-scoped persistence.

---

## Entities

### ChatMessage

Represents a single message in the conversation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique identifier (UUID or timestamp-based) |
| role | 'user' \| 'assistant' | Yes | Message author |
| content | string | Yes | Message text (may contain `[N]` citations for assistant) |
| timestamp | number | Yes | Unix timestamp in milliseconds |
| sources | Source[] | No | Array of sources (assistant messages only) |
| context | string | No | User-selected text context (user messages only) |
| timing | Timing | No | Performance metrics (assistant messages only) |
| error | string | No | Error message if request failed |

**Validation Rules**:
- `content` must be non-empty for user messages
- `sources` is only present when `role === 'assistant'`
- `context` is only present when `role === 'user'` and text was selected

**Example**:
```json
{
  "id": "msg_1702828800000",
  "role": "user",
  "content": "What is ROS 2?",
  "timestamp": 1702828800000,
  "context": "ROS 2 is a middleware framework..."
}
```

---

### Source

A citation reference from the backend response.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | number | Yes | Citation number (1-indexed, matches `[N]` in answer) |
| url | string | Yes | Full URL to book section |
| contentPreview | string | Yes | Snippet of source content |
| score | number | No | Relevance score from retrieval |

**Validation Rules**:
- `id` must be >= 1
- `url` must be a valid URL (book domain)
- `contentPreview` should be truncated to ~200 chars for display

**Example**:
```json
{
  "id": 1,
  "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-2",
  "contentPreview": "ROS 2 (Robot Operating System 2) is an open-source robotics middleware...",
  "score": 0.89
}
```

---

### Timing

Performance metrics from the backend.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| retrieval_ms | number | Yes | Time spent retrieving context |
| generation_ms | number | Yes | Time spent generating answer |

**Example**:
```json
{
  "retrieval_ms": 120,
  "generation_ms": 640
}
```

---

### ChatSession

The current conversation state stored in sessionStorage.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| messages | ChatMessage[] | Yes | Ordered array of messages |
| createdAt | number | Yes | Session creation timestamp |
| lastUpdated | number | Yes | Last activity timestamp |

**Storage Key**: `chatbot_session`

**Validation Rules**:
- `messages` array preserves insertion order
- Session clears when browser tab closes (sessionStorage behavior)

**Example**:
```json
{
  "messages": [
    { "id": "msg_1", "role": "user", "content": "What is ROS 2?", "timestamp": 1702828800000 },
    { "id": "msg_2", "role": "assistant", "content": "ROS 2 is...", "timestamp": 1702828801000, "sources": [...] }
  ],
  "createdAt": 1702828800000,
  "lastUpdated": 1702828801000
}
```

---

### TextSelection

User-selected text captured for "Ask AI" feature.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | string | Yes | Full selected text |
| truncatedPreview | string | No | Display version if > 1000 chars |
| sourceUrl | string | Yes | URL of page where selection occurred |
| position | SelectionPosition | No | Visual position for tooltip placement |

**Validation Rules**:
- `text` minimum length: 1 character
- `truncatedPreview` created when `text.length > 1000`
- `position` used for "Ask AI" button placement

---

### SelectionPosition

Position data for "Ask AI" button placement.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| top | number | Yes | Top offset from viewport |
| left | number | Yes | Left offset from viewport |

---

### ChatUIState

React component state for the chatbot UI.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| isOpen | boolean | Yes | false | Whether chat panel is visible |
| isLoading | boolean | Yes | false | Whether a request is in progress |
| error | string \| null | Yes | null | Current error message |
| inputValue | string | Yes | '' | Current input field value |
| pendingContext | TextSelection \| null | Yes | null | Text selection waiting to be sent |

---

### PluginConfig

Configuration passed to the Docusaurus plugin.

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| apiUrl | string | Yes | 'http://localhost:8000' | Backend API base URL |
| enabled | boolean | No | true | Enable/disable chatbot |
| position | 'bottom-right' \| 'bottom-left' | No | 'bottom-right' | Icon position |
| welcomeMessage | string | No | (see below) | Initial welcome text |
| exampleQuestions | string[] | No | (see below) | Clickable example questions |

**Default Welcome Message**:
```
"Hi! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the content!"
```

**Default Example Questions**:
```json
[
  "What is ROS 2?",
  "How does Isaac Sim work?",
  "Explain VLA models"
]
```

---

## State Transitions

### Chat Panel State Machine

```
┌─────────────┐     click icon      ┌─────────────┐
│   CLOSED    │ ─────────────────► │    OPEN     │
│             │ ◄───────────────── │             │
└─────────────┘  click outside/esc  └─────────────┘
                                          │
                                    submit question
                                          │
                                          ▼
                                   ┌─────────────┐
                                   │   LOADING   │
                                   └─────────────┘
                                     │         │
                              success│         │error
                                     ▼         ▼
                              ┌─────────┐ ┌─────────┐
                              │ READY   │ │  ERROR  │
                              └─────────┘ └─────────┘
                                     │         │
                                     └────┬────┘
                                          │ retry/new question
                                          ▼
                                   ┌─────────────┐
                                   │   LOADING   │
                                   └─────────────┘
```

### Text Selection Flow

```
┌───────────────┐
│ No Selection  │
└───────────────┘
        │
        │ mouseup in content area
        ▼
┌───────────────┐  click outside    ┌───────────────┐
│ Selection     │ ─────────────────►│ No Selection  │
│ Detected      │                   └───────────────┘
└───────────────┘
        │
        │ click "Ask AI"
        ▼
┌───────────────┐
│ Chat Opens    │
│ with Context  │
└───────────────┘
```

---

## API Request/Response Mapping

### Frontend → Backend

| Frontend Field | Backend Field | Notes |
|----------------|---------------|-------|
| inputValue | question | Required |
| pendingContext.text | context | Optional, for future use |
| (hardcoded) | top_k | Default: 5 |

### Backend → Frontend

| Backend Field | Frontend Field | Transform |
|---------------|----------------|-----------|
| answer | ChatMessage.content | None |
| sources[].url | Source.url | None |
| sources[].content | Source.contentPreview | Truncate if needed |
| sources[].score | Source.score | None |
| sources[].position | Source.id | None |
| retrieval_time_ms | Timing.retrieval_ms | None |
| generation_time_ms | Timing.generation_ms | None |

---

## Storage Schema

### sessionStorage

**Key**: `chatbot_session`
**Value**: JSON string of `ChatSession`
**Lifecycle**: Cleared when tab/browser closes

---

## Type Definitions (TypeScript)

```typescript
interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: number;
  sources?: Source[];
  context?: string;
  timing?: Timing;
  error?: string;
}

interface Source {
  id: number;
  url: string;
  contentPreview: string;
  score?: number;
}

interface Timing {
  retrieval_ms: number;
  generation_ms: number;
}

interface ChatSession {
  messages: ChatMessage[];
  createdAt: number;
  lastUpdated: number;
}

interface TextSelection {
  text: string;
  truncatedPreview?: string;
  sourceUrl: string;
  position?: SelectionPosition;
}

interface SelectionPosition {
  top: number;
  left: number;
}

interface ChatUIState {
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  inputValue: string;
  pendingContext: TextSelection | null;
}

interface PluginConfig {
  apiUrl: string;
  enabled?: boolean;
  position?: 'bottom-right' | 'bottom-left';
  welcomeMessage?: string;
  exampleQuestions?: string[];
}
```
