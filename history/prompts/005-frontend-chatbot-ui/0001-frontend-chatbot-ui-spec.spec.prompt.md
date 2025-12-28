---
id: 0001
title: Frontend Chatbot UI Spec Creation
stage: spec
date: 2025-12-17
surface: agent
model: claude-sonnet-4.5
feature: 005-frontend-chatbot-ui
branch: 005-frontend-chatbot-ui
user: IsmailAbdulkareem
command: /sp.specify
labels: ["chatbot", "ui", "frontend", "rag", "docusaurus"]
links:
  spec: specs/005-frontend-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-frontend-chatbot-ui/spec.md
  - specs/005-frontend-chatbot-ui/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

# Spec 5: Frontend RAG Chatbot Integration (UI Track)

**Spec ID**: `004-frontend-chatbot-ui`
**Status**: Ready for Implementation
**Depends On**:
- Spec 1 — URL ingestion & embeddings (Cohere + Qdrant)
- Spec 2 — RetrievalPipeline validation
- Spec 3 — Agent / OpenAI integration
- Spec 4 (CLI Track) — `database/chatbot.py` (completed)

---

## Purpose

Integrate a UI chatbot into the book website with floating chat icon, text selection "Ask AI" feature, and grounded answers from book content with source citations.

**Constraints**:
- Trigger only inside main content area
- Long selections (>1000 chars): Truncated in UI, Full text sent to backend

## API Contract

### Endpoint
POST /ask

### Request
```json
{
  "question": "What is ROS 2?",
  "context": "optional selected text",
  "top_k": 5
}
```

### Response
```json
{
  "answer": "ROS 2 is ... [1] [2]",
  "sources": [
    {
      "id": 1,
      "url": "https://ismailabdulkareem.github.io/AI_Book/...",
      "content_preview": "ROS 2 is a middleware..."
    }
  ],
  "timing": {
    "retrieval_ms": 120,
    "generation_ms": 640
  }
}
```

Frontend must render this data without modification.

## Grounding & Safety Rules
Inherited directly from CLI chatbot:
- Answers only from retrieved context
- No hallucinations
- Explicit refusal if context is missing
- Source citations required
- Same SYSTEM_PROMPT semantics

## Response snapshot

Created comprehensive specification for Frontend RAG Chatbot UI Integration:

**Branch**: `005-frontend-chatbot-ui`
**Spec File**: `specs/005-frontend-chatbot-ui/spec.md`

### Key Deliverables:
1. **4 User Stories** with acceptance scenarios:
   - US1 (P1): Floating Chatbot Interaction - core Q&A functionality
   - US2 (P2): Text Selection "Ask AI" - contextual queries
   - US3 (P2): Response Rendering with Citations - clickable sources
   - US4 (P3): Error and Empty State Handling

2. **15 Functional Requirements** (FR-001 to FR-015) covering:
   - Floating icon and chat panel behavior
   - Text selection detection in main content only
   - Citation rendering and navigation
   - Mobile responsiveness (min 320px)
   - Keyboard navigation support
   - Grounding rules (no hallucinations)

3. **10 Success Criteria** with measurable metrics:
   - Response time < 15 seconds
   - Chat panel open/close < 300ms
   - Text selection trigger < 150ms
   - 100% citation accuracy
   - Mobile support down to 320px

4. **Quality Checklist**: All items passed validation

## Outcome

- ✅ Impact: Complete specification ready for planning phase
- 🧪 Tests: N/A (specification phase - acceptance scenarios defined)
- 📁 Files: 2 files created (spec.md, checklists/requirements.md)
- 🔁 Next prompts: `/sp.plan` to create implementation plan, `/sp.tasks` to generate tasks
- 🧠 Reflection: Spec builds on completed CLI chatbot (Spec 4), reusing grounding rules and API contract

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided comprehensive requirements
- Graders run and results (PASS/FAIL): All checklist items PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase
