# Implementation Tasks: Frontend RAG Chatbot UI Integration

**Feature**: 005-frontend-chatbot-ui
**Branch**: `005-frontend-chatbot-ui`
**Generated**: 2025-12-17
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## User Stories Summary

| Story | Priority | Description | Independent Test |
|-------|----------|-------------|------------------|
| US1 | P1 | Floating Chatbot Interaction | Open chat, ask "What is ROS 2?", verify grounded answer with citations |
| US2 | P2 | Text Selection "Ask AI" | Select text, click "Ask AI", verify context appears in chat |
| US3 | P2 | Response Rendering with Citations | Ask question, click citation [1], verify navigation to book section |
| US4 | P3 | Error and Empty State Handling | Simulate backend unavailable, verify error message with retry |

---

## Phase 1: Setup

**Goal**: Create plugin structure and foundational files

- [x] T001 Create plugin directory structure at `src/plugins/docusaurus-plugin-chatbot/`
- [x] T002 Create plugin entry point at `src/plugins/docusaurus-plugin-chatbot/index.js`
- [x] T003 Create client module registration at `src/plugins/docusaurus-plugin-chatbot/chatbot-client.js`
- [x] T004 [P] Create TypeScript type definitions at `src/plugins/docusaurus-plugin-chatbot/types.d.ts`
- [x] T005 Enable plugin in `docusaurus.config.js` with apiUrl configuration

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Implement shared hooks and utilities needed by all user stories

- [x] T006 Create API client hook at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/hooks/useApiClient.js`
- [x] T007 Create session persistence hook at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/hooks/useChatSession.js`
- [x] T008 [P] Create CSS module with base styles at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/styles.module.css`

---

## Phase 3: User Story 1 - Floating Chatbot Interaction (P1)

**Goal**: Readers can click floating icon, ask questions, and receive grounded answers with citations

**Independent Test**: Open chatbot on any page, ask "What is ROS 2?", verify response contains grounded answer with clickable source citations

### Implementation Tasks

- [x] T009 [US1] Create ChatIcon component at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/ChatIcon.jsx`
- [x] T010 [US1] Create ChatPanel container at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/ChatPanel.jsx`
- [x] T011 [US1] Create Message component with citation parsing at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/Message.jsx`
- [x] T012 [US1] Create SourceList component at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/SourceList.jsx`
- [x] T013 [US1] Create main Chatbot wrapper with BrowserOnly at `src/plugins/docusaurus-plugin-chatbot/components/Chatbot/index.jsx`
- [x] T014 [US1] Add chat input with Enter-to-submit and Escape-to-close keyboard handling in `ChatPanel.jsx`
- [x] T015 [US1] Add loading indicator during API request in `ChatPanel.jsx`
- [x] T016 [US1] Implement session persistence (conversation history) via `useChatSession.js`
- [x] T017 [US1] Add mobile responsive styles (full-screen modal < 768px) in `styles.module.css`
- [x] T018 [US1] Wire up ChatIcon click to open/close panel in `index.jsx`

### Verification
- [ ] T019 [US1] Manual test: Open chat on any page, ask "What is ROS 2?", verify grounded answer displays
- [ ] T020 [US1] Manual test: Verify citations [1], [2] are clickable and navigate to correct book sections
- [ ] T021 [US1] Manual test: Close and reopen chat, verify conversation history preserved

---

## Phase 4: User Story 2 - Text Selection "Ask AI" Context (P2)

**Goal**: Readers can select text and use "Ask AI" to query about specific passages

**Independent Test**: Select any text in main content, click "Ask AI", verify selected text appears as context in chat

**Depends On**: US1 (chat panel must exist)

### Implementation Tasks

- [x] T022 [US2] Create useTextSelection hook at `src/plugins/docusaurus-plugin-chatbot/components/TextSelection/useTextSelection.js`
- [x] T023 [US2] Create AskAIButton component at `src/plugins/docusaurus-plugin-chatbot/components/TextSelection/AskAIButton.jsx`
- [x] T024 [US2] Create TextSelection wrapper at `src/plugins/docusaurus-plugin-chatbot/components/TextSelection/index.jsx`
- [x] T025 [US2] Implement content area detection (only `.theme-doc-markdown`, `article`) in `useTextSelection.js`
- [x] T026 [US2] Position "Ask AI" button near selection using viewport coordinates in `AskAIButton.jsx`
- [x] T027 [US2] Pass selected text as context to chat panel when "Ask AI" clicked
- [x] T028 [US2] Display truncated preview (>1000 chars) in chat with full text sent to backend
- [x] T029 [US2] Hide "Ask AI" button on click outside or selection clear

### Verification
- [ ] T030 [US2] Manual test: Select text in main content area, verify "Ask AI" button appears
- [ ] T031 [US2] Manual test: Select text in nav/header/footer, verify "Ask AI" does NOT appear
- [ ] T032 [US2] Manual test: Click "Ask AI", verify chat opens with selected text as context
- [ ] T033 [US2] Manual test: Select >1000 chars, verify truncated preview shows but full text sent

---

## Phase 5: User Story 3 - Response Rendering with Citations (P2)

**Goal**: Citation markers [1], [2] are clickable links that navigate to source book sections

**Independent Test**: Ask a question, click citation [1], verify navigation to correct book section

**Depends On**: US1 (message rendering must exist)

### Implementation Tasks

- [x] T034 [US3] Enhance Message component to parse `[N]` markers with regex in `Message.jsx`
- [x] T035 [US3] Render citations as `<a>` elements with href from sources array in `Message.jsx`
- [x] T036 [US3] Style citation links (inline, underlined, distinguishable) in `styles.module.css`
- [x] T037 [US3] Add Sources section below answer with URL and content preview in `SourceList.jsx`
- [x] T038 [P] [US3] Optionally display timing info (retrieval/generation time) in `SourceList.jsx`

### Verification
- [ ] T039 [US3] Manual test: Ask question, verify [1], [2] markers render as clickable links
- [ ] T040 [US3] Manual test: Click [1] citation, verify navigation to correct book URL
- [ ] T041 [US3] Manual test: Verify Sources section shows all sources with previews

---

## Phase 6: User Story 4 - Error and Empty State Handling (P3)

**Goal**: Display appropriate feedback for errors, loading, and empty states

**Independent Test**: Simulate backend unavailable, verify error message with retry option

**Depends On**: US1 (chat panel must exist)

### Implementation Tasks

- [x] T042 [US4] Add welcome message with example questions on first open in `ChatPanel.jsx`
- [x] T043 [US4] Implement error state display with user-friendly messages in `ChatPanel.jsx`
- [x] T044 [US4] Add "Retry" button for failed requests in `ChatPanel.jsx`
- [x] T045 [US4] Disable submit button when input is empty in `ChatPanel.jsx`
- [x] T046 [US4] Handle network timeout errors (30s timeout) in `useApiClient.js`
- [x] T047 [US4] Map backend error codes to user-friendly messages per API contract

### Verification
- [ ] T048 [US4] Manual test: Open empty chat, verify welcome message and example questions
- [ ] T049 [US4] Manual test: Click example question, verify it populates input and submits
- [ ] T050 [US4] Manual test: Disconnect backend, submit question, verify error message shows
- [ ] T051 [US4] Manual test: Click Retry button, verify request retries
- [ ] T052 [US4] Manual test: Empty input field, verify submit button is disabled

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final integration, accessibility, and deployment readiness

- [x] T053 Ensure chatbot uses BrowserOnly wrapper to prevent SSR issues in `index.jsx`
- [x] T054 Add ARIA labels for accessibility (chat icon, panel, input) across all components
- [ ] T055 Test mobile responsiveness at 320px viewport width
- [x] T056 Test keyboard navigation (Tab, Enter, Escape) throughout chatbot
- [x] T057 Verify Docusaurus build succeeds with plugin enabled: `npm run build`
- [ ] T058 Verify chatbot works on GitHub Pages deployment
- [ ] T059 Update quickstart.md with final configuration instructions

---

## Dependency Graph

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Foundational)
    │
    ▼
Phase 3 (US1: Core Chat) ◄─── MVP COMPLETE
    │
    ├──────────────────┐
    ▼                  ▼
Phase 4 (US2)      Phase 5 (US3)
    │                  │
    └────────┬─────────┘
             ▼
      Phase 6 (US4)
             │
             ▼
      Phase 7 (Polish)
```

---

## Parallel Execution Opportunities

### Within Phase 1 (Setup)
- T001, T002, T003 must be sequential
- T004 (types) can run in parallel with T003

### Within Phase 2 (Foundational)
- T006, T007, T008 can all run in parallel (no dependencies)

### Within Phase 3 (US1)
- T009, T010, T011, T012 can run in parallel (independent components)
- T013 depends on T009-T012
- T014-T018 must wait for T013

### Across Phases
- Phase 4 (US2) and Phase 5 (US3) can run in parallel after Phase 3

---

## Implementation Strategy

### MVP Scope (Recommended First Delivery)
Complete **Phases 1-3** for a working MVP:
- Floating chat icon on all pages
- Chat panel with question input
- API integration with backend
- Grounded answers with citation rendering
- Session persistence

This delivers the core P1 user story and can be deployed for initial user feedback.

### Incremental Additions
1. **Phase 4 (US2)**: Add text selection "Ask AI" feature
2. **Phase 5 (US3)**: Enhance citation rendering and navigation
3. **Phase 6 (US4)**: Polish error handling and empty states
4. **Phase 7**: Final accessibility and deployment verification

---

## Task Summary

| Phase | Task Count | Parallelizable | Description |
|-------|------------|----------------|-------------|
| Phase 1 | 5 | 1 | Setup |
| Phase 2 | 3 | 3 | Foundational |
| Phase 3 | 13 | 4 | US1: Core Chat (P1) |
| Phase 4 | 12 | 0 | US2: Text Selection (P2) |
| Phase 5 | 8 | 1 | US3: Citations (P2) |
| Phase 6 | 11 | 0 | US4: Error Handling (P3) |
| Phase 7 | 7 | 0 | Polish |
| **Total** | **59** | **9** | |

---

## Hard Rules Reminder

From plan.md - these MUST be followed during implementation:

1. **Backend Ownership**: Frontend MUST NOT construct prompts, inject system instructions, or perform fallback answering
2. **Verbatim Rendering**: Frontend renders backend responses as-is without modification
3. **Full Context**: Selected text sent as full `context` field, truncation is UI-only
4. **No Duplicate Logic**: No parallel chatbot implementation in frontend code
5. **API Contract**: Use `POST /ask` with `{ question, context, top_k }`, render response verbatim
