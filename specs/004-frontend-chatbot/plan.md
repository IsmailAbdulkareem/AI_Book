# Implementation Plan: Frontend Integration for Agentic RAG Chatbot

**Branch**: `004-frontend-chatbot` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Depends On**: 001-embedding-pipeline, 002-rag-retrieval, 003-agent-rag-integration
**Input**: Feature specification from `/specs/004-frontend-chatbot/spec.md`

## Summary

Integrate the Agentic RAG backend into the Docusaurus-based book website by adding a floating chatbot and contextual "Ask AI" text selection feature. The frontend communicates exclusively with the existing FastAPI RAG Agent via HTTP and never directly accesses embeddings or the vector database.

This implementation is fully client-side, non-invasive, and does not modify backend code.

## Technical Context

**Language/Version**: TypeScript 5.x (React components)
**Primary Dependencies**: React (bundled with Docusaurus), CSS Modules
**Storage**: Client-side session state only (React Context)
**Testing**: Manual verification + browser DevTools
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge)
**Project Type**: Docusaurus theme extension (React components)
**Performance Goals**: <200ms panel open/close, <15s total response time
**Constraints**: No new npm dependencies, no backend modifications, mobile support (320px+)
**Scale/Scope**: Single chatbot component, ~8 React components, ~500 LOC

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-first authoring | PASS | Full spec and plan created before implementation |
| Technical accuracy | PASS | Uses documented Docusaurus/React APIs |
| Reproducibility | PASS | Quickstart.md provides step-by-step guide |
| Consistency | PASS | Follows existing Docusaurus patterns (CSS Modules, theme swizzling) |
| Transparent AI usage | PASS | AI-assisted development documented |
| RAG Retrieval Guarantees | PASS | Frontend displays sources from backend; no direct retrieval |
| Separation of Concerns | PASS | Frontend layer separate from retrieval and generation |
| Spec 4 lifecycle | PASS | This is Spec 4 - Frontend integration as defined in constitution |

## Project Structure

### Documentation (this feature)

```text
specs/004-frontend-chatbot/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology research findings
├── data-model.md        # TypeScript interfaces documentation
├── quickstart.md        # Usage guide
├── contracts/
│   └── api-client.md    # API contract documentation
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── HomepageFeatures.js        # Existing
│   └── Chatbot/                   # NEW - All chatbot components
│       ├── index.ts               # Barrel export
│       ├── types.ts               # TypeScript interfaces
│       ├── api.ts                 # API client
│       ├── ChatProvider.tsx       # Context provider
│       ├── ChatbotIcon.tsx        # Floating launcher
│       ├── ChatbotIcon.module.css
│       ├── ChatbotPanel.tsx       # Chat panel container
│       ├── ChatbotPanel.module.css
│       ├── ChatMessage.tsx        # Message renderer
│       ├── ChatMessage.module.css
│       ├── TextSelectionPopup.tsx # "Ask AI" popup
│       └── TextSelectionPopup.module.css
├── css/
│   └── custom.css                 # Existing (may add chatbot variables)
├── pages/
│   └── index.js                   # Existing
└── theme/
    └── Root.tsx                   # NEW - Global wrapper for chatbot

docusaurus.config.js               # May need minor update for env var
```

**Structure Decision**: Use `src/theme/Root.tsx` swizzling pattern to mount chatbot globally without modifying existing pages or content. All chatbot code isolated in `src/components/Chatbot/`.

---

## Frontend Architecture

The chatbot is implemented as a **global client-side React extension**.

### Integration Strategy

- Mount chatbot globally using `src/theme/Root.tsx` (Docusaurus theme swizzling)
- No changes to MDX content or existing components
- No backend coupling beyond HTTP API
- Environment variable for API URL configuration

### Rationale

- Works across all pages automatically
- Preserves static site generation
- Easy rollback (delete Chatbot folder + Root.tsx)
- Zero impact on build pipeline

---

## Component Breakdown

### Core Components

**`<ChatProvider />`** - Context provider
- Manages chat session state (messages, loading, error)
- Provides dispatch functions to children
- Wraps entire application via Root.tsx

**`<ChatbotIcon />`** - Floating launcher
- Fixed position bottom-right
- Click to toggle panel open/close
- Badge for unread indicator (optional)

**`<ChatbotPanel />`** - Main chat interface
- Message list with auto-scroll
- Input form with submit button
- Loading indicator
- Error display with retry
- Welcome state with example questions

**`<ChatMessage />`** - Message renderer
- Distinct styling for user vs assistant
- Source citations as clickable links
- Context indicator for text selection queries
- Timestamp display

**`<TextSelectionPopup />`** - Selection action
- Appears near text selection
- "Ask AI" button
- Disappears on selection clear
- Positioned using getBoundingClientRect

### Supporting Modules

**`api.ts`** - HTTP client
- `askQuestion(request: AskRequest): Promise<AskResponse>`
- `checkHealth(): Promise<HealthResponse>`
- Error handling with typed exceptions
- 30-second timeout

**`types.ts`** - TypeScript interfaces
- All interfaces from data-model.md
- Exported for use across components

---

## Function Design

### 1. ChatProvider State Management

**Purpose**: Centralized state management for chat session

```typescript
// State shape
interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  pendingContext: string | null;
}

// Actions
type ChatAction =
  | { type: 'TOGGLE_OPEN' }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'SET_PENDING_CONTEXT'; payload: string | null }
  | { type: 'CLEAR_MESSAGES' };
```

---

### 2. sendMessage Function

**Purpose**: Submit question to API and handle response

```typescript
async function sendMessage(question: string, context?: string): Promise<void> {
  // 1. Add user message to state
  // 2. Set loading state
  // 3. Call API
  // 4. On success: add assistant message with sources
  // 5. On error: set error state
  // 6. Clear loading state
}
```

---

### 3. Text Selection Detection

**Purpose**: Detect text selection and show "Ask AI" popup

```typescript
function useTextSelection(): TextSelection | null {
  // 1. Listen to 'mouseup' and 'selectionchange' events
  // 2. Get selection from window.getSelection()
  // 3. Filter to only content area (article element)
  // 4. Return selection with bounding rect for positioning
}
```

---

### 4. Source Citation Rendering

**Purpose**: Render clickable source links in messages

```typescript
function renderSourceLinks(sources: Source[]): JSX.Element {
  // 1. Map sources to numbered list
  // 2. Each source is a link to URL
  // 3. Show truncated content on hover/expand
}
```

---

## CSS Design Tokens

Define CSS custom properties for theming consistency:

```css
/* In custom.css or Chatbot.module.css */
:root {
  --chatbot-bg: #ffffff;
  --chatbot-bg-dark: #1e1e1e;
  --chatbot-text: #333333;
  --chatbot-text-dark: #e0e0e0;
  --chatbot-primary: #4a90d9;
  --chatbot-border: #e0e0e0;
  --chatbot-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  --chatbot-icon-size: 56px;
  --chatbot-panel-width: 380px;
  --chatbot-panel-height: 500px;
  --chatbot-z-index: 1000;
}
```

---

## Error Handling

| Scenario | Detection | User Feedback | Recovery |
|----------|-----------|---------------|----------|
| Network error | fetch throws | "Unable to connect" | Retry button |
| API 400 | response.status | Show validation message | Edit input |
| API 500 | response.status | "Something went wrong" | Retry button |
| API 503 | response.status | "Service unavailable" | Retry button |
| Timeout | AbortController | "Request timed out" | Retry button |
| Empty response | !answer | "No answer received" | Retry button |

---

## Mobile Considerations

### Responsive Breakpoints

```css
/* Mobile (default): < 768px */
.chatbotPanel {
  position: fixed;
  inset: 0;  /* Full screen */
  width: 100%;
  height: 100%;
  border-radius: 0;
}

/* Tablet and Desktop: >= 768px */
@media (min-width: 768px) {
  .chatbotPanel {
    width: var(--chatbot-panel-width);
    height: var(--chatbot-panel-height);
    bottom: 80px;
    right: 20px;
    border-radius: 12px;
  }
}
```

### Touch Interactions

- Tap icon to open/close
- Scroll messages with momentum
- Keyboard appears for input without covering panel (use visualViewport API)

---

## Success Verification

After implementation:

1. **Chatbot Icon Visible**: Icon appears on all pages in bottom-right
2. **Panel Opens/Closes**: Click icon → panel opens; click X → closes
3. **Question Submission**: Type question → Submit → Loading → Answer appears
4. **Source Citations**: Answer contains [1], [2] links → Click navigates to source
5. **Text Selection**: Select text → "Ask AI" appears → Click → Panel opens with context
6. **Error Handling**: Stop backend → Submit question → Error message with retry
7. **Mobile**: Test on 320px viewport → All features work
8. **Keyboard**: Enter submits, Escape closes

---

## Risks & Mitigations

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| Docusaurus version incompatibility | Low | Pin to tested version, document requirements |
| SSR hydration issues | Medium | Use useEffect for browser-only code, BrowserOnly wrapper |
| Style conflicts with theme | Low | Use CSS Modules, high-specificity selectors |
| Text selection conflicts | Low | Scope to article element only |
| Mobile keyboard issues | Medium | Use visualViewport API for position adjustment |

---

## Complexity Tracking

No complexity violations - simple component structure maintained.

---

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Create `src/theme/Root.tsx` wrapper
3. Implement components in dependency order
4. Test on local development server
5. Validate against acceptance scenarios
