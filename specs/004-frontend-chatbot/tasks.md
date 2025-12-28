# Tasks: Frontend Integration for Agentic RAG Chatbot

**Input**: Design documents from `/specs/004-frontend-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/, research.md, quickstart.md
**Depends On**: 001-embedding-pipeline, 002-rag-retrieval, 003-agent-rag-integration (must be complete)

**Tests**: Manual verification + browser DevTools

**Organization**: Tasks are organized in two tracks:
1. **CLI Chatbot** (Python) - Quick validation of RAG pipeline
2. **React Frontend** (TypeScript) - Web UI integration by user story

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **CLI Chatbot**: `database/chatbot.py` — NEW FILE
- **React Frontend**: `src/components/Chatbot/` — NEW DIRECTORY
- **Theme Wrapper**: `src/theme/Root.tsx` — NEW FILE
- **Backend (DO NOT MODIFY)**: `database/main.py`, `database/retrieval.py`, `database/agent.py`

---

## Phase 1: Setup (Environment Verification)

**Purpose**: Verify prerequisites and prepare development environment

- [x] T000 Verify Python dependencies using uv in `database/`
  - Run: `uv sync` to install from pyproject.toml
  - Ensure: openai, fastapi, qdrant-client, cohere, python-dotenv installed
  - Verify: `uv run python -c "import openai, fastapi, qdrant_client, cohere; print('OK')"`

- [x] T001 Verify Spec 3 backend is running: `curl http://localhost:8000/health`
- [ ] T002 Verify Docusaurus dev server works: `npm start` in project root
- [x] T003 [P] Create `src/components/Chatbot/` directory structure
- [x] T004 [P] Create `src/theme/` directory if not exists

**Checkpoint**: Both backend API and frontend dev server are running

---

## Phase 2: CLI Chatbot (Python Terminal Interface)

**Purpose**: Validate RAG pipeline end-to-end before building React UI

**Goal**: Create a terminal-based chatbot for quick testing and debugging

**Independent Test**: Run `uv run python chatbot.py` and ask "What is ROS 2?" - should return grounded answer with sources

### Implementation

- [x] T005 Create `database/chatbot.py` with module docstring and imports
  - Import: os, asyncio, dotenv, retrieval.RetrievalPipeline
  - Import: openai.OpenAI

- [x] T006 Define SYSTEM_PROMPT constant in `database/chatbot.py`
  - Same prompt as agent.py for consistency
  - Enforce grounded answers with citations

- [x] T007 Implement `format_context(results)` helper in `database/chatbot.py`
  - Format retrieved chunks as numbered sources
  - Include URL for each source

- [x] T008 Implement `generate_answer(question, context)` function in `database/chatbot.py`
  - Call OpenAI Responses API via OpenAI Agents SDK (same model and system prompt as agent.py)
  - Use GPT-4o-mini with temperature=0.3
  - Return answer text

- [x] T009 Implement `ask_question(question)` async function in `database/chatbot.py`
  - Initialize RetrievalPipeline
  - Search for relevant context
  - Generate grounded answer
  - Print answer with sources

- [x] T010 Implement interactive chat loop in `database/chatbot.py`
  - Print welcome message
  - Read user input in loop
  - Handle 'quit' and 'exit' commands
  - Call ask_question for each input

- [x] T011 Add `if __name__ == "__main__"` entry point in `database/chatbot.py`
  - Load environment variables
  - Run interactive chat loop

- [x] T012 Test CLI chatbot with sample questions in `database/`
  - Test: "What is ROS 2?"
  - Test: "How do I create a URDF file?"
  - Verify sources are displayed

**Checkpoint**: CLI chatbot works - can ask questions and get grounded answers

**Verification**:
```bash
cd database
uv run python chatbot.py
# Type: What is ROS 2?
# Should return answer with [1], [2] citations and source URLs
# Type: quit
```

---

## Phase 3: Foundational (React Setup)

**Purpose**: Set up React component infrastructure for all user stories

- [ ] T013 Create `src/components/Chatbot/types.ts` with TypeScript interfaces
  - ChatMessage, Source, ChatSession, TextSelection
  - AskRequest, AskResponse, HealthResponse
  - Component props interfaces

- [ ] T014 Create `src/components/Chatbot/api.ts` with API client
  - `askQuestion(request: AskRequest): Promise<AskResponse>`
  - `checkHealth(): Promise<HealthResponse>`
  - APIError class for typed errors
  - 30-second timeout with AbortController

- [ ] T015 Create `src/components/Chatbot/index.ts` barrel export
  - Export all components and types

- [ ] T016 Create `src/theme/Root.tsx` global wrapper
  - Import ChatProvider from Chatbot
  - Wrap children with ChatProvider
  - Export as default

**Checkpoint**: TypeScript compiles without errors - `npm run build` succeeds

---

## Phase 4: User Story 1 - Floating Chatbot Interaction (Priority: P1)

**Goal**: Core RAG functionality - floating icon, chat panel, Q&A with citations

**Independent Test**: Click chatbot icon → Type "What is ROS 2?" → See grounded answer with clickable source links

### Implementation for User Story 1

- [ ] T017 [US1] Create `src/components/Chatbot/ChatProvider.tsx` context provider
  - Define ChatState interface (messages, isOpen, isLoading, error)
  - Create ChatContext with useContext
  - Implement useReducer for state management
  - Export useChatContext hook

- [ ] T018 [US1] Create `src/components/Chatbot/ChatbotIcon.tsx` floating launcher
  - Fixed position bottom-right (20px from edges)
  - 56px circular button with chat icon
  - onClick toggles panel open/close
  - z-index: 1000

- [ ] T019 [US1] Create `src/components/Chatbot/ChatbotIcon.module.css` styles
  - Floating button styles
  - Hover and active states
  - Mobile responsive (smaller on < 768px)

- [ ] T020 [US1] Create `src/components/Chatbot/ChatMessage.tsx` message renderer
  - Props: message (ChatMessage)
  - Distinct styling for user vs assistant
  - Render source citations as clickable links
  - Format timestamps

- [ ] T021 [US1] Create `src/components/Chatbot/ChatMessage.module.css` styles
  - User message (right-aligned, primary color)
  - Assistant message (left-aligned, neutral color)
  - Source links styling
  - Timestamp styling

- [ ] T022 [US1] Create `src/components/Chatbot/ChatbotPanel.tsx` main panel
  - Props: isOpen, onClose
  - Header with title and close button
  - Message list with auto-scroll to bottom
  - Input form with submit button
  - Loading indicator during API call
  - Welcome state with example questions

- [ ] T023 [US1] Create `src/components/Chatbot/ChatbotPanel.module.css` styles
  - Panel container (fixed position, shadow, border-radius)
  - Header styles
  - Message list (scrollable)
  - Input form styles
  - Mobile full-screen on < 768px

- [ ] T024 [US1] Implement sendMessage function in ChatProvider
  - Add user message to state
  - Set loading state
  - Call askQuestion API
  - Add assistant message with sources on success
  - Set error state on failure

- [ ] T025 [US1] Add keyboard navigation to ChatbotPanel
  - Enter to submit (without Shift)
  - Escape to close panel
  - Focus management on open/close

- [ ] T026 [US1] Update `src/components/Chatbot/index.ts` to export US1 components
  - Export ChatProvider, ChatbotIcon, ChatbotPanel, ChatMessage

- [ ] T027 [US1] Update `src/theme/Root.tsx` to render chatbot components
  - Render ChatbotIcon
  - Render ChatbotPanel (conditionally based on isOpen)

**Checkpoint**: User Story 1 complete - basic Q&A chatbot works

**Verification**:
```bash
npm start
# Open http://localhost:3000/AI_Book/
# Click chatbot icon in bottom-right
# Type: "What is ROS 2?"
# Press Enter
# Should see loading indicator, then answer with [1], [2] citations
# Click a citation link - should navigate to source
# Press Escape - panel closes
```

---

## Phase 5: User Story 2 - Text Selection "Ask AI" (Priority: P2)

**Goal**: Contextual queries from selected text

**Independent Test**: Select text in book → Click "Ask AI" → Panel opens with context pre-filled

### Implementation for User Story 2

- [ ] T028 [US2] Create `src/components/Chatbot/useTextSelection.ts` custom hook
  - Listen to mouseup and selectionchange events
  - Get selection from window.getSelection()
  - Filter to article element only (book content)
  - Return TextSelection with bounding rect

- [ ] T029 [US2] Create `src/components/Chatbot/TextSelectionPopup.tsx` component
  - Props: selection (TextSelection | null), onAskAI callback
  - Position popup near selection using getBoundingClientRect
  - "Ask AI" button
  - Hide when selection is null

- [ ] T030 [US2] Create `src/components/Chatbot/TextSelectionPopup.module.css` styles
  - Absolute positioned popup
  - Button styling
  - Arrow pointing to selection
  - Mobile-friendly touch targets

- [ ] T031 [US2] Add pendingContext state to ChatProvider
  - State: pendingContext (string | null)
  - Action: SET_PENDING_CONTEXT
  - Clear after message is sent

- [ ] T032 [US2] Update ChatbotPanel to show context indicator
  - Display selected text above input when pendingContext is set
  - Truncate with "..." if > 200 characters
  - Allow clearing context

- [ ] T033 [US2] Update sendMessage to include context
  - Prepend context to question if pendingContext is set
  - Format: "Context: {selectedText}\n\nQuestion: {question}"
  - Clear pendingContext after send

- [ ] T034 [US2] Update Root.tsx to render TextSelectionPopup
  - Use useTextSelection hook
  - Pass onAskAI handler (opens panel with context)

- [ ] T035 [US2] Update `src/components/Chatbot/index.ts` to export US2 components
  - Export TextSelectionPopup, useTextSelection

**Checkpoint**: User Story 2 complete - text selection triggers "Ask AI"

**Verification**:
```bash
npm start
# Navigate to any docs page
# Select some text (e.g., a definition)
# "Ask AI" button appears near selection
# Click "Ask AI"
# Panel opens with selected text shown as context
# Add question or press Enter
# Answer should reference the selected context
```

---

## Phase 6: User Story 3 - Error and Empty State Handling (Priority: P3)

**Goal**: Graceful error handling and polished empty states

**Independent Test**: Stop backend → Submit question → See friendly error with retry button

### Implementation for User Story 3

- [ ] T036 [US3] Add error state UI to ChatbotPanel
  - Display error message prominently
  - "Retry" button to resend last question
  - Clear error on new message attempt

- [ ] T037 [US3] Implement welcome state in ChatbotPanel
  - Show when messages array is empty
  - Welcome message: "Hi! Ask me anything about Physical AI & Humanoid Robotics."
  - Example question chips: "What is ROS 2?", "How do I set up a simulation?", "What is URDF?"
  - Click chip to send that question

- [ ] T038 [US3] Add loading indicator component
  - Animated dots or spinner
  - "Thinking..." text
  - Display in message list while loading

- [ ] T039 [US3] Implement retry functionality
  - Store last failed question
  - Retry button calls sendMessage with same question
  - Clear on successful response

- [ ] T040 [US3] Add network error detection in api.ts
  - Catch fetch errors (network offline)
  - Distinguish timeout from server error
  - Return user-friendly error messages

- [ ] T041 [US3] Handle empty/no-results response
  - Check if answer indicates no relevant content
  - Display appropriately (not an error, just informational)

**Checkpoint**: User Story 3 complete - error handling and empty states work

**Verification**:
```bash
# Test 1: Error handling
# Stop backend: Ctrl+C in terminal running uvicorn
npm start
# Open chatbot, submit question
# Should see "Service temporarily unavailable" with retry button

# Test 2: Welcome state
# Open chatbot with empty history
# Should see welcome message and example questions
# Click an example question - should send it

# Test 3: Start backend again
cd database && uv run uvicorn agent:app --port 8000
# Click retry - should work now
```

---

## Phase 7: Polish & Integration

**Purpose**: Final polish, testing, and documentation

- [ ] T042 Add CSS custom properties to `src/css/custom.css`
  - Chatbot design tokens (colors, sizes, shadows)
  - Dark mode support via data-theme attribute

- [ ] T043 [P] Test on mobile viewport (320px width)
  - Verify chatbot icon visible
  - Verify panel opens full-screen
  - Verify input is accessible with keyboard

- [ ] T044 [P] Test keyboard navigation end-to-end
  - Tab navigation through chatbot
  - Enter to submit
  - Escape to close

- [ ] T045 [P] Verify all quickstart.md examples work
  - Basic Q&A flow
  - Text selection flow
  - Error handling flow

- [ ] T046 Run Docusaurus build to verify no errors: `npm run build`

- [ ] T047 Update docusaurus.config.js with API URL environment variable (if needed)

**Checkpoint**: Feature complete and ready for deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (CLI Chatbot)**: Depends on Phase 1 - validate RAG pipeline
- **Phase 3 (Foundational)**: Depends on Phase 1 - BLOCKS all React user stories
- **Phase 4 (US1)**: Depends on Phase 3 - Core chatbot functionality
- **Phase 5 (US2)**: Depends on Phase 4 - Text selection extends chatbot
- **Phase 6 (US3)**: Depends on Phase 4 - Error handling extends chatbot
- **Phase 7 (Polish)**: Depends on all user stories

### Track Independence

**CLI Track (Phases 1-2)** and **React Track (Phases 3-7)** can run in parallel after Phase 1.

```
Phase 1 (Setup)
    ├─→ Phase 2 (CLI Chatbot) ─→ [CLI Complete]
    └─→ Phase 3 (Foundational) ─→ Phase 4 (US1) ─→ Phase 5 (US2) ─→ Phase 7 (Polish)
                                              └─→ Phase 6 (US3) ─────┘
```

### Parallel Opportunities

**Within Phase 2 (CLI)**:
```bash
# T005-T007 can be written together (single file)
```

**Within Phase 4 (US1)**:
```bash
# These component files can be written in parallel:
T018 + T019: ChatbotIcon.tsx + CSS
T020 + T021: ChatMessage.tsx + CSS
T022 + T023: ChatbotPanel.tsx + CSS
```

**Within Phase 7 (Polish)**:
```bash
# These verification tasks can run in parallel:
T043: Mobile testing [P]
T044: Keyboard testing [P]
T045: Quickstart verification [P]
```

---

## Implementation Strategy

### MVP First (CLI + User Story 1)

1. Complete Phase 1: Setup (T000-T004)
2. Complete Phase 2: CLI Chatbot (T005-T012)
3. **STOP and VALIDATE**: CLI chatbot works end-to-end
4. Complete Phase 3: Foundational (T013-T016)
5. Complete Phase 4: User Story 1 (T017-T027)
6. **STOP and VALIDATE**: Web chatbot works end-to-end

### Incremental Delivery

1. CLI Chatbot → Test RAG pipeline
2. Foundational → TypeScript types and API client
3. User Story 1 → Core Q&A functionality
4. User Story 2 → Text selection enhancement
5. User Story 3 → Error handling polish
6. Polish → Mobile, keyboard, final testing

---

## Task Summary

| Phase | Tasks | Parallel Tasks | User Story |
|-------|-------|----------------|------------|
| Phase 1: Setup | 5 | 2 | - |
| Phase 2: CLI Chatbot | 8 | 0 | - |
| Phase 3: Foundational | 4 | 0 | - |
| Phase 4: US1 - Floating Chatbot | 11 | 0 | P1 |
| Phase 5: US2 - Text Selection | 8 | 0 | P2 |
| Phase 6: US3 - Error Handling | 6 | 0 | P3 |
| Phase 7: Polish | 6 | 3 | - |
| **Total** | **48** | **5** | |

---

## Notes

- **Backend code MUST NOT be modified** (Specs 1-3)
- CLI chatbot in `database/chatbot.py` reuses RetrievalPipeline from Spec 2
- React components in `src/components/Chatbot/` (isolated from existing code)
- Global wrapper in `src/theme/Root.tsx` (Docusaurus swizzling pattern)
- No new npm dependencies required
- API URL configurable via environment variable
