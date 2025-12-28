---
id: 0002
title: Frontend Chatbot UI Implementation Plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4.5
feature: 005-frontend-chatbot-ui
branch: 005-frontend-chatbot-ui
user: IsmailAbdulkareem
command: /sp.plan
labels: ["chatbot", "ui", "frontend", "docusaurus", "planning"]
links:
  spec: specs/005-frontend-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-frontend-chatbot-ui/plan.md
  - specs/005-frontend-chatbot-ui/research.md
  - specs/005-frontend-chatbot-ui/data-model.md
  - specs/005-frontend-chatbot-ui/quickstart.md
  - specs/005-frontend-chatbot-ui/contracts/api-client.md
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan

Execute the implementation planning workflow for the Frontend RAG Chatbot UI Integration feature (Spec 005).

## Response snapshot

Completed Phase 0-1 planning for Frontend RAG Chatbot UI Integration:

### Phase 0: Research (research.md)
Researched 7 key technical decisions:
1. **Plugin Architecture**: Docusaurus custom plugin with `getClientModules()`
2. **Text Selection**: Native Selection API with content area containment check
3. **Session State**: sessionStorage + React hooks (no external libs)
4. **API Integration**: Direct fetch to backend POST /ask endpoint
5. **Citation Rendering**: Regex replacement of `[N]` markers to clickable links
6. **SSR Safety**: BrowserOnly wrapper + dynamic import
7. **Mobile UX**: Full-screen modal below 768px viewport

### Phase 1: Design Artifacts
1. **data-model.md**: Defined 8 entities (ChatMessage, Source, Timing, ChatSession, TextSelection, SelectionPosition, ChatUIState, PluginConfig)
2. **contracts/api-client.md**: Full API contract with request/response schemas, error handling, retry logic
3. **quickstart.md**: Setup guide with plugin configuration, environment variables, troubleshooting

### plan.md Summary
- **Technical Stack**: React 18.x, Docusaurus 3.9.2, CSS Modules, native fetch
- **Project Structure**: `src/plugins/docusaurus-plugin-chatbot/` with Chatbot and TextSelection components
- **No New Dependencies**: All functionality with existing packages
- **Constitution Check**: All gates passed (spec-first, separation of concerns, reproducibility)

### Architecture Decision
```
<Root> (Docusaurus)
  └── <ChatbotWrapper> (BrowserOnly)
        ├── <ChatIcon />
        ├── <ChatPanel>
        │     ├── <Message /> × N
        │     └── <SourceList />
        └── <TextSelectionListener>
              └── <AskAIButton />
```

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation
- 🧪 Tests: N/A (planning phase)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, quickstart.md, api-client.md)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks
- 🧠 Reflection: Plan leverages existing Docusaurus plugin architecture; no new dependencies needed

## Evaluation notes (flywheel)

- Failure modes observed: None - all research questions resolved
- Graders run and results (PASS/FAIL): Constitution check PASS (all 6 gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation with /sp.tasks
