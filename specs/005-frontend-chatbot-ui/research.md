# Research: Frontend RAG Chatbot UI Integration

**Feature**: 005-frontend-chatbot-ui
**Date**: 2025-12-17
**Status**: Complete

---

## Research Questions

### RQ-1: Docusaurus Plugin Architecture for Global UI Components

**Question**: How to integrate a floating chatbot icon globally across all Docusaurus pages without modifying core theme?

**Decision**: Use a custom Docusaurus plugin with `@theme/Root` wrapper component.

**Rationale**:
- Docusaurus v3 supports custom plugins that can inject global components via lifecycle hooks
- The `@theme/Root` component wraps the entire site and renders on every page
- This approach doesn't require swizzling core theme components, reducing upgrade friction
- Plugin architecture allows configuration via `docusaurus.config.js`

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Swizzle entire theme | High maintenance burden, breaks on upgrades |
| Inject via custom CSS/JS | No React state management, poor UX |
| Modify Layout component | Requires swizzling, conflicts with theme updates |
| iframe embedding | Poor accessibility, complex state sharing |

**Implementation Pattern**:
```javascript
// docusaurus-plugin-chatbot/index.js
module.exports = function chatbotPlugin(context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',
    getClientModules() {
      return [require.resolve('./chatbot-client')];
    },
  };
};
```

---

### RQ-2: Text Selection Detection in Docusaurus Content

**Question**: How to detect text selection only within the main content area (`.theme-doc-markdown` or `<article>`)?

**Decision**: Use browser `Selection` API with DOM containment check against content selectors.

**Rationale**:
- Native `Selection` API provides `getRangeAt()` for selection bounds
- Can check if selection `anchorNode` is within target container
- Event delegation on `mouseup`/`touchend` minimizes listeners
- Docusaurus uses predictable CSS classes: `.theme-doc-markdown`, `article[class*="docMainContainer"]`

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Global selection listener | Would trigger on nav/sidebar selections |
| contenteditable regions | Changes DOM behavior, accessibility issues |
| Custom selection library | Unnecessary dependency for simple check |

**Implementation Pattern**:
```javascript
const CONTENT_SELECTORS = [
  '.theme-doc-markdown',
  'article[class*="docMainContainer"]',
  '[class*="docItemContainer"]'
];

function isSelectionInContent(selection) {
  if (!selection.rangeCount) return false;
  const node = selection.anchorNode;
  return CONTENT_SELECTORS.some(selector =>
    node?.parentElement?.closest(selector)
  );
}
```

---

### RQ-3: Client-Side Session State Management

**Question**: How to persist chat history within a browser session without React context complexity?

**Decision**: Use `sessionStorage` with React state synchronization via custom hook.

**Rationale**:
- `sessionStorage` survives page navigation within session, clears on tab close
- No server-side persistence required (spec constraint)
- Simple `useEffect` sync pattern avoids complex state libraries
- Supports SSR safety (only access `sessionStorage` on client)

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Redux/Zustand | Overkill for single-component state |
| React Context | Requires provider at root, complicates plugin |
| localStorage | Persists across sessions (not desired) |
| In-memory only | Lost on page navigation |

**Implementation Pattern**:
```javascript
const STORAGE_KEY = 'chatbot_session';

function useChatSession() {
  const [messages, setMessages] = useState(() => {
    if (typeof window === 'undefined') return [];
    const stored = sessionStorage.getItem(STORAGE_KEY);
    return stored ? JSON.parse(stored) : [];
  });

  useEffect(() => {
    sessionStorage.setItem(STORAGE_KEY, JSON.stringify(messages));
  }, [messages]);

  return [messages, setMessages];
}
```

---

### RQ-4: Backend API Integration Pattern

**Question**: How to communicate with the RAG backend from the Docusaurus frontend?

**Decision**: Direct fetch to configured API URL with environment variable fallback.

**Rationale**:
- Backend already exposes `POST /ask` with CORS enabled (Spec 3)
- Plugin config passes `apiUrl` at build time
- Environment variable `RAG_CHATBOT_API_URL` for runtime override
- Simple `fetch` wrapper handles errors and timeout

**API Contract** (from `database/agent.py`):
```typescript
// Request
POST /ask
{
  question: string,       // Required
  top_k?: number          // Optional, default 5
}

// Response
{
  question: string,
  answer: string,         // Contains [1], [2] citation markers
  sources: [{
    url: string,
    content: string,
    score: number,
    position: number      // 1-indexed
  }],
  retrieval_time_ms: number,
  generation_time_ms: number
}
```

**Note**: The spec defines `context` field but current backend doesn't use it. Frontend should send it for forward compatibility.

**Implementation Pattern**:
```javascript
async function askQuestion(question, context = null, topK = 5) {
  const response = await fetch(`${API_URL}/ask`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question, context, top_k: topK }),
  });
  if (!response.ok) throw new Error(`API error: ${response.status}`);
  return response.json();
}
```

---

### RQ-5: Citation Rendering and Navigation

**Question**: How to convert `[1]`, `[2]` markers in answer text to clickable links?

**Decision**: Regex replacement with React components during render.

**Rationale**:
- Answer text contains `[N]` markers that map to `sources[N-1].url`
- Simple regex `/\[(\d+)\]/g` captures citation numbers
- Replace with `<a>` or React Link component
- Sources section renders full list with previews

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Markdown parsing | Overkill, citations aren't markdown |
| Pre-processed HTML from backend | Couples backend to frontend rendering |
| Footnote-style (separate section only) | Users can't click inline citations |

**Implementation Pattern**:
```javascript
function renderAnswerWithCitations(answer, sources) {
  const parts = answer.split(/(\[\d+\])/g);
  return parts.map((part, i) => {
    const match = part.match(/\[(\d+)\]/);
    if (match) {
      const idx = parseInt(match[1], 10) - 1;
      const source = sources[idx];
      return source ? (
        <a key={i} href={source.url} target="_blank" rel="noopener">
          [{match[1]}]
        </a>
      ) : part;
    }
    return part;
  });
}
```

---

### RQ-6: SSR/Hydration Safety

**Question**: How to ensure chatbot doesn't break Docusaurus static build or SEO?

**Decision**: Use `BrowserOnly` wrapper and lazy loading.

**Rationale**:
- Docusaurus builds static HTML at build time (SSR)
- Chatbot has no SEO value, should not be in static HTML
- `@docusaurus/BrowserOnly` prevents SSR rendering
- Dynamic import reduces initial bundle size

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| `typeof window` checks everywhere | Error-prone, verbose |
| Disable SSR entirely | Breaks SEO for main content |
| Load via external script | No React integration |

**Implementation Pattern**:
```jsx
import BrowserOnly from '@docusaurus/BrowserOnly';

function ChatbotWrapper() {
  return (
    <BrowserOnly fallback={null}>
      {() => {
        const Chatbot = require('./Chatbot').default;
        return <Chatbot />;
      }}
    </BrowserOnly>
  );
}
```

---

### RQ-7: Mobile Responsiveness Strategy

**Question**: How to make the chat panel usable on mobile devices down to 320px?

**Decision**: Full-screen modal on viewport < 768px, slide-in panel on larger screens.

**Rationale**:
- Fixed-position slide-in panels are awkward on mobile
- Full-screen provides adequate space for keyboard and input
- CSS media queries handle transition seamlessly
- Touch events work naturally in full-screen mode

**Breakpoints**:
| Viewport Width | Chat Panel Behavior |
|----------------|---------------------|
| < 768px | Full-screen modal with close button |
| >= 768px | Fixed slide-in panel (400px width) |
| >= 1200px | Optional: wider panel (480px) |

**Implementation Pattern**:
```css
.chatbot-panel {
  position: fixed;
  right: 0;
  bottom: 0;
  width: 400px;
  height: 500px;
}

@media (max-width: 767px) {
  .chatbot-panel {
    width: 100%;
    height: 100%;
    top: 0;
    left: 0;
  }
}
```

---

## Technology Decisions Summary

| Aspect | Decision | Confidence |
|--------|----------|------------|
| Plugin Architecture | Docusaurus custom plugin with client module | High |
| State Management | sessionStorage + React useState | High |
| Text Selection | Native Selection API with containment check | High |
| API Communication | fetch with error handling | High |
| Citation Rendering | Regex replacement with React elements | High |
| SSR Safety | BrowserOnly + dynamic import | High |
| Mobile UX | Full-screen modal below 768px | High |

---

## Unresolved Items

None. All clarifications have been resolved through research.

---

## References

- [Docusaurus Plugin Development](https://docusaurus.io/docs/creating-plugins)
- [Docusaurus BrowserOnly](https://docusaurus.io/docs/docusaurus-core#browseronly)
- [MDN Selection API](https://developer.mozilla.org/en-US/docs/Web/API/Selection)
- [Spec 3: Agent API](../003-agent-rag-integration/spec.md)
- [Backend Implementation](../../database/agent.py)
