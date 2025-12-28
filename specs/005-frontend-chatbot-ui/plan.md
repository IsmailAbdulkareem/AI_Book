# Implementation Plan: Frontend RAG Chatbot UI Integration

**Branch**: `005-frontend-chatbot-ui` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-frontend-chatbot-ui/spec.md`

---

## Summary

Implement a frontend chatbot UI as a Docusaurus plugin that provides:
1. A floating chat icon on all pages (bottom-right)
2. A chat panel for asking questions about book content
3. Text selection "Ask AI" feature for contextual queries
4. Citation rendering with clickable source links

The frontend is a pure UI layer that consumes the existing RAG backend API (POST /ask) without performing any retrieval or generation logic.

---

## Technical Context

**Language/Version**: JavaScript/TypeScript (React 18.x, ES2020+)
**Primary Dependencies**:
- Docusaurus 3.9.2 (existing)
- React 18.3.1 (existing)
- No additional UI libraries required

**Storage**: sessionStorage (client-side only, browser tab scope)
**Testing**: Manual testing initially; optional Jest + React Testing Library
**Target Platform**: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
**Project Type**: Docusaurus plugin (frontend-only integration)

**Performance Goals**:
- Chat panel open/close: < 300ms
- Text selection to Ask AI button: < 150ms
- Total response time: < 15s (includes backend latency)

**Constraints**:
- No SSR rendering (client-only component)
- No interference with Docusaurus SEO/hydration
- Mobile responsive down to 320px viewport
- No direct access to Qdrant or vector databases

**Scale/Scope**:
- Single Docusaurus site
- Unlimited concurrent users (stateless frontend)
- No authentication required

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Evidence |
|------|--------|----------|
| Spec-first authoring | PASS | Spec 005 created via /sp.specify |
| Technical accuracy | PASS | Research validates Docusaurus plugin patterns |
| Reproducibility | PASS | quickstart.md provides setup instructions |
| Consistency | PASS | Follows existing project structure |
| RAG Separation of Concerns | PASS | Frontend is UI-only, no retrieval logic |
| RAG Retrieval Guarantees | PASS | Citations pass through from backend unchanged |

**Post-Design Re-check** (Phase 1 complete):
- Data model aligns with backend API contract
- No new external dependencies added
- Plugin architecture follows Docusaurus best practices

---

## Project Structure

### Documentation (this feature)

```text
specs/005-frontend-chatbot-ui/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Frontend data structures
├── quickstart.md        # Setup guide
├── contracts/
│   └── api-client.md    # Backend API contract
├── checklists/
│   └── requirements.md  # Quality checklist
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
src/
├── components/
│   └── Chatbot/              # Existing empty directory (to be populated)
├── css/
│   └── custom.css            # Existing site styles
├── pages/                    # Existing pages
└── plugins/
    └── docusaurus-plugin-chatbot/
        ├── index.js          # Plugin entry point
        ├── chatbot-client.js # Client module registration
        ├── types.d.ts        # TypeScript definitions
        └── components/
            ├── Chatbot/
            │   ├── index.jsx
            │   ├── ChatPanel.jsx
            │   ├── ChatIcon.jsx
            │   ├── Message.jsx
            │   ├── SourceList.jsx
            │   ├── styles.module.css
            │   └── hooks/
            │       ├── useChatSession.js
            │       └── useApiClient.js
            └── TextSelection/
                ├── index.jsx
                ├── AskAIButton.jsx
                └── useTextSelection.js
```

**Structure Decision**: Docusaurus plugin architecture with client-side components. Uses the existing src/ directory structure with a new plugins/ folder. Components are organized by feature (Chatbot, TextSelection) with co-located hooks and styles.

---

## Technical Decisions

### Architecture

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Plugin Architecture | Docusaurus custom plugin | Non-invasive, configurable, survives upgrades |
| State Management | sessionStorage + React hooks | Simple, no external deps, session-scoped |
| Styling | CSS Modules | Scoped styles, no conflicts with theme |
| API Client | Native fetch | No axios needed, browser-native |
| SSR Safety | BrowserOnly wrapper | Prevents hydration mismatches |

### Component Hierarchy

```
<Root> (Docusaurus)
  └── <ChatbotWrapper> (BrowserOnly)
        ├── <ChatIcon />
        ├── <ChatPanel>
        │     ├── <Message /> x N
        │     └── <SourceList />
        └── <TextSelectionListener>
              └── <AskAIButton />
```

### Key Implementation Patterns

1. **Plugin Entry Point** (index.js):
   - Exports getClientModules() to inject chatbot client
   - Passes configuration via context

2. **Session Persistence** (useChatSession.js):
   - Reads from sessionStorage on mount
   - Writes on message changes
   - Clears on tab close (automatic)

3. **Citation Rendering** (Message.jsx):
   - Regex splits answer text by [N] markers
   - Maps markers to source URLs
   - Renders as <a> elements

4. **Text Selection** (useTextSelection.js):
   - Listens to mouseup/touchend
   - Validates selection is within content area
   - Returns selection text and position

---

## Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus upgrade breaks plugin | Low | Medium | Use documented APIs, avoid swizzling |
| Backend unavailable | Medium | High | Graceful error messages, retry button |
| Mobile usability issues | Medium | Medium | Test on real devices, use full-screen modal |
| Text selection conflicts | Low | Low | Scope to content area only |

---

## Dependencies

### Existing (No Changes)

- @docusaurus/core: ^3.9.2
- react: ^18.3.1
- react-dom: ^18.3.1

### New Dependencies

None required. All functionality implemented with existing dependencies.

---

## Generated Artifacts

| Artifact | Path | Status |
|----------|------|--------|
| Research | specs/005-frontend-chatbot-ui/research.md | Complete |
| Data Model | specs/005-frontend-chatbot-ui/data-model.md | Complete |
| API Contract | specs/005-frontend-chatbot-ui/contracts/api-client.md | Complete |
| Quickstart | specs/005-frontend-chatbot-ui/quickstart.md | Complete |
| Tasks | specs/005-frontend-chatbot-ui/tasks.md | Pending /sp.tasks |

---

## Complexity Tracking

No violations. Implementation follows simple patterns:
- Single plugin (no complex architecture)
- No external state management library
- No additional build tooling
- No new backend endpoints

---

## Next Steps

1. Run /sp.tasks to generate implementation tasks
2. Implement plugin structure (Task 1)
3. Build chatbot components (Tasks 2-4)
4. Add text selection feature (Task 5)
5. Test all user stories (Task 6)
6. Deploy and verify (Task 7)

---

## Clarifications & Hard Rules

### Backend Ownership (Hard Rule)

The frontend MUST NOT:
- Construct prompts
- Inject system instructions
- Modify or enrich retrieved context
- Perform fallback answering

The backend (`database/chatbot.py` / agent) is the **single authority** for:
- Prompting
- Grounding rules
- Refusal behavior
- Source numbering and citation order

**The frontend renders responses verbatim.**

### API Endpoint Alignment

- The frontend MUST integrate with the existing backend by calling: `POST /ask`
- The endpoint MUST internally reuse the same pipeline as `database/chatbot.py`
- **No duplicate logic or parallel chatbot implementation is allowed in frontend code**

### Text Selection → Backend Contract

When using "Ask AI":
- Selected text MUST be sent as `context` field
- User question MUST be sent as `question` field
- The frontend MUST NOT summarize, trim, or reinterpret selected text
- **Truncation applies to UI display only, not the payload**

```javascript
// Correct: Full text sent to backend
fetch('/ask', {
  body: JSON.stringify({
    question: userInput,
    context: selectedText,  // Full text, even if > 1000 chars
    top_k: 5
  })
});
```

### Deployment Constraint

- The chatbot plugin MUST work with GitHub Pages static hosting
- Backend API URL MUST be configurable via environment variable
- No server-side rendering or build-time API calls
