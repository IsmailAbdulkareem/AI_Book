# Feature Specification: Frontend RAG Chatbot UI Integration

**Feature Branch**: `005-frontend-chatbot-ui`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Integrate a UI chatbot into the book website with a floating chat icon, text selection 'Ask AI' feature, and grounded answers from book content with source citations."
**Depends On**:
- Spec 1 — URL ingestion & embeddings (Cohere + Qdrant)
- Spec 2 — RetrievalPipeline validation
- Spec 3 — Agent / OpenAI integration
- Spec 4 (CLI Track) — `database/chatbot.py` (completed)

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Floating Chatbot Interaction (Priority: P1)

A reader visiting any page of the Physical AI & Humanoid Robotics book sees a floating chat icon in the bottom-right corner. When clicked, a chat panel opens where they can type questions and receive AI-generated answers grounded strictly in the book's content, with numbered source citations linking back to relevant sections.

**Why this priority**: This is the core value proposition. Readers can get instant, accurate answers about book content without leaving their current page. It transforms the static book into an interactive learning experience.

**Independent Test**: Can be fully tested by opening the chatbot on any page, asking "What is ROS 2?", and verifying the response contains a grounded answer with clickable source citations pointing to relevant book sections.

**Acceptance Scenarios**:

1. **Given** a reader is on any page of the book website, **When** the page loads, **Then** they see a floating chatbot icon in the bottom-right corner that does not obstruct the main content.

2. **Given** a reader clicks the chatbot icon, **When** the chat panel opens, **Then** the panel slides in smoothly without triggering a page reload and the reader's scroll position is preserved.

3. **Given** the chat panel is open, **When** the reader types a question and presses Enter or clicks Send, **Then** a loading indicator appears while the system fetches the answer.

4. **Given** a question has been submitted, **When** the backend returns a response, **Then** the answer displays with numbered source citations (e.g., [1], [2]) that are clickable links to the relevant book sections.

5. **Given** the chat panel is open, **When** the reader clicks outside the panel or clicks the close button, **Then** the panel closes smoothly without page reload.

6. **Given** the reader has asked questions in the current session, **When** they close and reopen the chat panel, **Then** the conversation history is preserved within that browser session.

---

### User Story 2 - Text Selection "Ask AI" Context (Priority: P2)

A reader selects text within the book's main content area and sees an "Ask AI" button appear near the selection. Clicking it opens the chatbot with the selected text pre-filled as context, enabling focused questions about specific passages.

**Why this priority**: This feature enables contextual learning by letting readers get clarification on specific passages they're reading. It builds on US1 and provides a more seamless interaction flow.

**Independent Test**: Can be tested by selecting any text in the main content area, clicking "Ask AI", and verifying the selected text appears in the chat panel as context for the query.

**Acceptance Scenarios**:

1. **Given** a reader selects text within the main content area (not navigation, header, or footer), **When** the selection is complete (mouseup/touchend), **Then** an "Ask AI" button appears near the selection within 100ms.

2. **Given** the "Ask AI" button is visible, **When** the reader clicks it, **Then** the chatbot panel opens with the selected text displayed as context and the input field focused.

3. **Given** selected text exceeds 1000 characters, **When** the "Ask AI" button is clicked, **Then** the UI displays a truncated preview with "... [truncated]" indicator while the full text is sent to the backend.

4. **Given** the "Ask AI" button is visible, **When** the reader clicks elsewhere or clears the selection, **Then** the "Ask AI" button disappears.

5. **Given** selected text is pre-filled as context, **When** the reader types a question and submits, **Then** both the context and question are sent to the backend together.

---

### User Story 3 - Response Rendering with Citations (Priority: P2)

A reader receives an answer from the chatbot that includes numbered source citations. Each citation links to the specific book section, and clicking a citation navigates to that section in a new tab or scrolls to it on the current page.

**Why this priority**: Citations are essential for trust and verification. Readers need to verify AI-generated answers against the source material, making this a core quality requirement.

**Independent Test**: Can be tested by asking a question that retrieves multiple sources, clicking each citation link, and verifying navigation to the correct book section.

**Acceptance Scenarios**:

1. **Given** a response is received from the backend, **When** the answer contains citations like [1], [2], **Then** each citation is rendered as a clickable link.

2. **Given** a reader clicks a citation link, **When** the link destination is on the same domain, **Then** the user is navigated to that book section (new tab or same page scroll based on context).

3. **Given** a response includes a sources array, **When** the response is rendered, **Then** a "Sources" section appears below the answer listing each source with its URL and content preview.

4. **Given** response timing data is available, **When** the response is rendered, **Then** timing information (retrieval and generation time) is optionally displayed for transparency.

---

### User Story 4 - Error and Empty State Handling (Priority: P3)

When the backend is unavailable, returns an error, or no relevant content is found, the reader sees appropriate feedback messages that guide them on what to do next.

**Why this priority**: Error handling ensures a polished experience but is not required for core functionality. It prevents user confusion when issues occur.

**Independent Test**: Can be tested by simulating backend unavailability and verifying appropriate error messages appear with retry options.

**Acceptance Scenarios**:

1. **Given** the chat panel is open and the backend is unavailable, **When** the reader submits a question, **Then** they see a user-friendly error message: "Unable to reach the assistant. Please try again later."

2. **Given** the reader asks a question with no relevant content in the book, **When** the backend responds with an explicit refusal, **Then** the system displays the AI's response indicating no relevant information was found.

3. **Given** a network error occurs during question submission, **When** the error is detected, **Then** the reader sees an error message with a "Retry" button.

4. **Given** the chat panel is empty (first open), **When** the reader views the panel, **Then** they see a welcome message with 2-3 example questions they can click to ask.

5. **Given** an empty question is attempted, **When** the input field is empty, **Then** the submit button is disabled.

---

### Edge Cases

- What happens when the reader selects text outside the main content area (navigation, header, footer)?
  - The "Ask AI" button should NOT appear. Text selection detection is scoped to the main content area only.

- How does the system handle rapid consecutive questions?
  - Questions are queued and processed sequentially. A loading indicator persists until the current question is answered before accepting the next.

- What happens on mobile devices with limited screen space?
  - The chat panel should be responsive, expanding to full-screen or near full-screen on small viewports (< 768px width). The floating icon should remain visible but smaller.

- What happens when JavaScript is disabled?
  - The chatbot icon should not appear, and the site remains fully functional for reading.

- How does the chatbot handle very long responses?
  - Responses should be scrollable within the chat panel, with the latest message auto-scrolled into view.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a floating chatbot icon on all pages of the book website.
- **FR-002**: System MUST position the chatbot icon in the bottom-right corner as a fixed element.
- **FR-003**: System MUST open/close the chat panel without triggering a page reload.
- **FR-004**: System MUST preserve the reader's scroll position when opening/closing the chat panel.
- **FR-005**: System MUST detect text selection within the main content area only and display an "Ask AI" button.
- **FR-006**: System MUST send selected text as context when using "Ask AI".
- **FR-007**: System MUST render source citations as clickable links to relevant book sections.
- **FR-008**: System MUST display loading, empty, and error states appropriately.
- **FR-009**: System MUST communicate with the RAG Agent API via POST /ask endpoint.
- **FR-010**: System MUST preserve conversation history within the current browser session.
- **FR-011**: System MUST support keyboard navigation (Enter to submit, Escape to close).
- **FR-012**: System MUST be responsive and functional on mobile devices (minimum 320px width).
- **FR-013**: System MUST truncate long text selections (>1000 chars) in UI while sending full text to backend.
- **FR-014**: System MUST display grounded answers only from retrieved context (no hallucinations).
- **FR-015**: System MUST show explicit refusal message when context is insufficient to answer.

### Key Entities

- **ChatMessage**: A single message in the conversation
  - role: "user" or "assistant"
  - content: The message text
  - timestamp: When the message was sent/received
  - sources: Array of source citations (assistant messages only)

- **ChatSession**: Current conversation state
  - messages: Array of ChatMessage
  - isLoading: Whether a request is in progress
  - error: Current error state if any

- **TextSelection**: User-selected text for context
  - text: The selected content
  - truncatedPreview: Display version if text > 1000 chars
  - sourceUrl: The page URL where selection occurred

- **Source**: A citation from the backend response
  - id: Citation number (1, 2, 3...)
  - url: Link to the book section
  - contentPreview: Snippet of the source content

- **ApiResponse**: Backend response structure
  - answer: The AI-generated response with [N] citation markers
  - sources: Array of Source objects
  - timing: Object with retrieval_ms and generation_ms

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Readers can ask a question and receive a grounded answer in under 15 seconds (including network latency).
- **SC-002**: The chatbot icon is visible and accessible on 100% of book pages.
- **SC-003**: Source citations in responses link to the correct book section 100% of the time.
- **SC-004**: The chat panel opens and closes without page reload in under 300ms.
- **SC-005**: Text selection triggers the "Ask AI" button within 150ms of selection completion.
- **SC-006**: Error messages are displayed within 3 seconds of detecting a failure condition.
- **SC-007**: The chatbot is functional on mobile devices with screen widths as small as 320px.
- **SC-008**: 90% of readers can complete a basic Q&A interaction (open chat, ask question, read answer) without instructions.
- **SC-009**: Answers contain source citations that match the retrieved content 100% of the time.
- **SC-010**: The system explicitly refuses to answer when no relevant context is found, rather than hallucinating.

---

## Clarifications & Constraints

### Frontend Responsibility Boundaries

- The frontend MUST NOT perform embeddings, vector search, reranking, or any retrieval logic.
- The frontend MUST NOT communicate directly with Qdrant or any vector database.
- All grounding, retrieval, and refusal logic MUST reside exclusively in the backend (Specs 1–4).
- The frontend acts strictly as a UI layer that sends `{ question, context, top_k }` to the backend `/ask` endpoint and renders the response.

### Text Selection Scope

- Text selection detection MUST be limited to the Docusaurus main content container only (e.g., `<main>`, `.theme-doc-markdown`, or equivalent).
- Selections from navigation bars, headers, footers, sidebars, or UI chrome MUST be ignored.
- The "Ask AI" button MUST NOT appear for selections outside the main content area.

### Rendering & SEO Safety

- The chatbot UI MUST NOT interfere with Docusaurus static rendering, hydration, or SEO.
- No chatbot DOM elements may be injected into server-rendered markdown content.
- The chatbot MUST be mounted as a client-only component that loads after page hydration.

### API Contract Enforcement

The frontend MUST assume the backend response format:

```
Request: POST /ask
{
  question: string,
  context?: string,
  top_k?: number (default: 5)
}

Response:
{
  answer: string,
  sources: { id: number, url: string, contentPreview: string }[],
  timing?: { retrieval_ms: number, generation_ms: number }
}
```

- The frontend MUST render the response as-is without modification to the answer text.
- The frontend MUST handle missing optional fields gracefully (timing may be absent).
- The frontend MUST NOT attempt to parse, validate, or transform the answer content beyond rendering.

---

## Assumptions

- The RAG Agent API (via `database/chatbot.py` and agent) is running and accessible at a configurable base URL.
- The backend POST /ask endpoint accepts `{ question, context, top_k }` and returns `{ answer, sources, timing }`.
- The Docusaurus website supports custom components or plugins for UI integration.
- Backend CORS is configured to allow requests from the frontend domain.
- Session conversation history is stored client-side only (no cross-session persistence for MVP).
- The grounding and safety rules from the CLI chatbot apply: answers only from retrieved context, no hallucinations, explicit refusal if context missing, source citations required.

---

## Out of Scope

- User authentication or personalized chat history across browser sessions.
- Multi-language support for the chatbot interface.
- Voice input for questions.
- Streaming responses (full response rendered at once).
- Analytics or usage tracking for chatbot interactions.
- Backend modifications (Specs 1-4 are complete).
- Conversation export or sharing functionality.
- Dark mode theming (inherits from site theme if available).
