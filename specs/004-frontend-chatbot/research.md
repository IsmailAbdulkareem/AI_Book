# Research: Frontend Integration for Agentic RAG Chatbot

**Feature Branch**: `004-frontend-chatbot`
**Date**: 2025-12-16
**Purpose**: Resolve technical unknowns and document technology decisions

---

## Research Questions

### RQ1: How to integrate React components globally in Docusaurus?

**Decision**: Use `src/theme/Root.tsx` wrapper component

**Rationale**:
- Docusaurus supports theme component swizzling via `src/theme/`
- `Root.tsx` wraps the entire application and persists across page navigations
- No plugin infrastructure required - simpler implementation
- Works with static site generation (SSG) without SSR dependencies

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Custom Docusaurus plugin | More complex, requires plugin lifecycle management |
| Direct DOM injection | Not React-compatible, breaks hydration |
| Per-page component import | Requires MDX changes on every page |

**Reference**: [Docusaurus Swizzling Guide](https://docusaurus.io/docs/swizzling)

---

### RQ2: How to detect text selection and position a popup?

**Decision**: Use `window.getSelection()` API with `mouseup`/`selectionchange` events

**Rationale**:
- Native browser API with excellent cross-browser support
- `getBoundingClientRect()` provides exact selection position for popup placement
- No external library dependencies
- Works within Docusaurus content area (`article` element)

**Implementation Pattern**:
```typescript
const handleSelectionChange = () => {
  const selection = window.getSelection();
  if (selection && selection.toString().trim().length > 0) {
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();
    // Position popup relative to rect
  }
};
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Third-party selection library | Unnecessary dependency for simple use case |
| CSS `::selection` pseudo-element | Cannot attach interactive elements |

---

### RQ3: State management approach for chat session?

**Decision**: React `useState` + `useContext` for session state

**Rationale**:
- Lightweight, no external dependencies
- Session state is simple (messages array, loading, error)
- Context enables state sharing between ChatbotLauncher and ChatPanel
- Persists within browser session via React state (cleared on page refresh)

**State Shape**:
```typescript
interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
}
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Redux/Zustand | Overkill for single-component state |
| localStorage persistence | Out of scope for MVP (spec says session-only) |
| URL state | Not appropriate for chat UI state |

---

### RQ4: API communication pattern?

**Decision**: Native `fetch` API with async/await

**Rationale**:
- Built into all modern browsers
- No additional dependencies
- Simple request/response pattern matches backend API
- Error handling via try/catch

**API Client Pattern**:
```typescript
const askQuestion = async (question: string, topK: number = 5) => {
  const response = await fetch(`${API_BASE_URL}/ask`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question, top_k: topK }),
  });
  if (!response.ok) throw new Error('API request failed');
  return response.json();
};
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Axios | Unnecessary dependency for simple HTTP calls |
| React Query | Adds complexity; caching not needed for chat |
| GraphQL | Backend uses REST, not GraphQL |

---

### RQ5: Styling approach for chatbot components?

**Decision**: CSS Modules with dedicated stylesheet

**Rationale**:
- Already used in Docusaurus project (`HomepageFeatures.module.css`)
- Scoped styles prevent conflicts with Docusaurus theme
- No build configuration changes required
- Simple to maintain alongside React components

**File Structure**:
```
src/components/Chatbot/
├── Chatbot.tsx
├── Chatbot.module.css
├── ChatMessage.tsx
├── ChatMessage.module.css
└── TextSelectionPopup.tsx
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Tailwind CSS | Not configured in project, adds build complexity |
| styled-components | Runtime CSS-in-JS, potential SSG issues |
| Global CSS | Risk of style conflicts with Docusaurus |

---

### RQ6: Mobile responsiveness approach?

**Decision**: CSS media queries with mobile-first breakpoints

**Rationale**:
- Standard web approach, no JavaScript required
- Chat panel can be full-width on mobile, fixed-width on desktop
- Floating icon remains accessible at all screen sizes
- Success criteria specifies 320px minimum width

**Breakpoints**:
```css
/* Mobile: 320px - 767px (default styles) */
/* Tablet: 768px - 1023px */
/* Desktop: 1024px+ */
```

---

### RQ7: Keyboard navigation implementation?

**Decision**: Native keyboard event handlers on form elements

**Rationale**:
- `onKeyDown` handlers for Enter (submit) and Escape (close)
- Follows standard web form patterns
- Accessibility compliance (FR-013)

**Implementation**:
```typescript
const handleKeyDown = (e: KeyboardEvent) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    handleSubmit();
  }
  if (e.key === 'Escape') {
    handleClose();
  }
};
```

---

## Technology Stack Summary

| Category | Choice | Version |
|----------|--------|---------|
| Framework | Docusaurus (React) | 3.x |
| Language | TypeScript | 5.x |
| State Management | React Context + useState | Built-in |
| HTTP Client | Fetch API | Built-in |
| Styling | CSS Modules | Built-in |
| Text Selection | window.getSelection() | Built-in |

---

## Dependencies Required

**No new npm dependencies required.**

All functionality implemented using:
- React (bundled with Docusaurus)
- TypeScript (already configured)
- CSS Modules (already configured)
- Native browser APIs

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Backend API unavailable | Medium | High | Error handling with retry UI |
| Text selection conflicts with existing JS | Low | Medium | Scope selection listener to content area only |
| Mobile keyboard covers chat input | Medium | Medium | Use `visualViewport` API to adjust position |
| Long API response times | Medium | Low | Loading indicator, timeout handling |

---

## Resolved Clarifications

All technical unknowns have been resolved. No NEEDS CLARIFICATION items remain.
