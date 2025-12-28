# Data Model: Frontend Integration for Agentic RAG Chatbot

**Feature Branch**: `004-frontend-chatbot`
**Date**: 2025-12-16
**Source**: Feature specification entities + research.md

---

## Overview

This document defines the TypeScript interfaces and data structures for the frontend chatbot integration. All types are client-side only; the backend API contract is documented separately in `contracts/`.

---

## Core Entities

### ChatMessage

Represents a single message in the conversation.

```typescript
interface ChatMessage {
  id: string;                    // Unique identifier (UUID)
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text
  timestamp: Date;               // When message was created
  sources?: Source[];            // Citations (assistant messages only)
  context?: string;              // Selected text context (user messages only)
  isError?: boolean;             // True if this is an error message
}
```

**Validation Rules**:
- `id`: Required, unique per session
- `content`: Required, non-empty string
- `sources`: Present only when `role === 'assistant'` and response was successful
- `context`: Present only when `role === 'user'` and message originated from text selection

---

### Source

A citation from the backend response.

```typescript
interface Source {
  url: string;           // Full URL to book section
  content: string;       // Snippet of relevant content (max 500 chars)
  score: number;         // Relevance score (0-1)
  position: number;      // Position in results (1-indexed)
}
```

**Validation Rules**:
- `url`: Must be a valid URL starting with the book's base URL
- `content`: Truncated to 500 characters if longer
- `score`: Float between 0 and 1
- `position`: Positive integer

---

### ChatSession

The current conversation state.

```typescript
interface ChatSession {
  messages: ChatMessage[];       // Conversation history
  isLoading: boolean;            // True while waiting for API response
  error: string | null;          // Current error message or null
  pendingContext: string | null; // Text selection waiting to be sent
}
```

**State Transitions**:

```
Initial State:
  messages: []
  isLoading: false
  error: null
  pendingContext: null

User sends message:
  → isLoading: true
  → messages: [...messages, newUserMessage]

API responds successfully:
  → isLoading: false
  → messages: [...messages, assistantMessage]
  → error: null

API responds with error:
  → isLoading: false
  → error: errorMessage

User selects text and clicks "Ask AI":
  → pendingContext: selectedText
  → (panel opens if closed)

User clears selection:
  → pendingContext: null
```

---

### TextSelection

User-selected text with position information.

```typescript
interface TextSelection {
  text: string;              // Selected text content
  pageUrl: string;           // Current page URL
  rect: DOMRect;             // Bounding rectangle for popup positioning
  truncated: boolean;        // True if text was truncated for display
}
```

**Validation Rules**:
- `text`: Non-empty string
- `rect`: Valid DOMRect from `getBoundingClientRect()`
- Text over 1000 characters should set `truncated: true`

---

### ChatbotConfig

Configuration for the chatbot component.

```typescript
interface ChatbotConfig {
  apiBaseUrl: string;            // Backend API URL (e.g., 'http://localhost:8000')
  position: 'bottom-right';      // Icon position (only bottom-right supported)
  defaultTopK: number;           // Default number of sources (5)
  maxMessageLength: number;      // Max question length (1000)
  welcomeMessage: string;        // Initial message shown in empty chat
  exampleQuestions: string[];    // Suggested questions for empty state
}
```

**Default Values**:
```typescript
const defaultConfig: ChatbotConfig = {
  apiBaseUrl: 'http://localhost:8000',
  position: 'bottom-right',
  defaultTopK: 5,
  maxMessageLength: 1000,
  welcomeMessage: 'Hi! Ask me anything about Physical AI & Humanoid Robotics.',
  exampleQuestions: [
    'What is ROS 2?',
    'How do I set up a robot simulation?',
    'What is URDF?',
  ],
};
```

---

## API Request/Response Types

### AskRequest

Request body for POST /ask endpoint.

```typescript
interface AskRequest {
  question: string;          // The question to ask
  top_k?: number;            // Number of sources (default: 5, max: 20)
}
```

---

### AskResponse

Response body from POST /ask endpoint.

```typescript
interface AskResponse {
  question: string;              // Echo of the question
  answer: string;                // Generated answer with citations
  sources: Source[];             // Source citations
  retrieval_time_ms: number;     // Time spent on retrieval
  generation_time_ms: number;    // Time spent on generation
}
```

---

### HealthResponse

Response body from GET /health endpoint.

```typescript
interface HealthResponse {
  status: 'healthy' | 'degraded' | 'unhealthy';
  dependencies: {
    qdrant: DependencyHealth;
    openai: DependencyHealth;
  };
}

interface DependencyHealth {
  status: 'connected' | 'configured' | 'error';
  details?: Record<string, unknown>;
}
```

---

### ErrorResponse

Error response from API.

```typescript
interface ErrorResponse {
  error: string;                 // Error type
  message: string;               // Human-readable message
  details?: Record<string, unknown>;
}
```

---

## Component Props

### ChatbotLauncherProps

```typescript
interface ChatbotLauncherProps {
  isOpen: boolean;
  onClick: () => void;
}
```

### ChatbotPanelProps

```typescript
interface ChatbotPanelProps {
  isOpen: boolean;
  onClose: () => void;
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  onSendMessage: (message: string, context?: string) => void;
  pendingContext: string | null;
  onClearContext: () => void;
}
```

### ChatMessageProps

```typescript
interface ChatMessageProps {
  message: ChatMessage;
}
```

### TextSelectionPopupProps

```typescript
interface TextSelectionPopupProps {
  selection: TextSelection | null;
  onAskAI: (text: string) => void;
}
```

---

## State Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         ChatProvider                             │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                     ChatSession State                    │    │
│  │  messages[], isLoading, error, pendingContext           │    │
│  └─────────────────────────────────────────────────────────┘    │
│                              │                                   │
│         ┌────────────────────┼────────────────────┐              │
│         ▼                    ▼                    ▼              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐   │
│  │ ChatbotIcon  │    │ ChatbotPanel │    │ TextSelection    │   │
│  │              │    │              │    │ Popup            │   │
│  │ onClick →    │    │ messages →   │    │                  │   │
│  │ toggleOpen   │    │ ChatMessage  │    │ onAskAI →        │   │
│  └──────────────┘    │              │    │ setPendingContext│   │
│                      │ onSubmit →   │    └──────────────────┘   │
│                      │ sendMessage  │                            │
│                      └──────────────┘                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## File Organization

```
src/components/Chatbot/
├── index.ts                 # Barrel export
├── ChatProvider.tsx         # Context provider with state management
├── ChatbotIcon.tsx          # Floating launcher icon
├── ChatbotIcon.module.css
├── ChatbotPanel.tsx         # Main chat panel
├── ChatbotPanel.module.css
├── ChatMessage.tsx          # Individual message component
├── ChatMessage.module.css
├── TextSelectionPopup.tsx   # "Ask AI" popup
├── TextSelectionPopup.module.css
├── api.ts                   # API client functions
└── types.ts                 # TypeScript interfaces (this document)
```
