---
id: 0002
title: Agent RAG Implementation Plan
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-agent-rag-integration
branch: 003-agent-rag-integration
user: developer
command: /sp.plan
labels: ["rag", "agent", "openai", "fastapi", "planning"]
links:
  spec: specs/003-agent-rag-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-agent-rag-integration/plan.md
  - specs/003-agent-rag-integration/research.md
  - specs/003-agent-rag-integration/data-model.md
  - specs/003-agent-rag-integration/quickstart.md
  - specs/003-agent-rag-integration/contracts/openapi.yaml
tests:
  - Constitution check PASS (all 7 principles)
---

## Prompt

/sp.plan

Implement a RAG agent in `database/agent.py` using FastAPI + OpenAI Agents SDK.
Use existing retrieval pipeline from `database/main.py`.
Provide endpoints /ask (POST) and /health (GET).
The agent must retrieve context from Qdrant, generate grounded answers, and return sources.
Follow all functional requirements from Spec 3 plan.md.

## Response snapshot

Created comprehensive implementation plan with the following artifacts:

**Plan (plan.md)**:
- Create new `database/agent.py` (single-file design)
- FastAPI application with POST /ask and GET /health endpoints
- Import RetrievalPipeline from existing retrieval.py
- OpenAI GPT-4o-mini for answer generation
- Pydantic models for request/response validation
- Error handling for all edge cases

**Research (research.md)**:
- OpenAI SDK with function calling pattern
- FastAPI for async HTTP endpoints
- Reuse existing RetrievalPipeline from Spec 2
- System prompt enforcing grounding
- Response structure with sources and timing

**Data Model (data-model.md)**:
- AskRequest: question, top_k
- AskResponse: question, answer, sources, timing
- Source: url, content, score, position
- HealthResponse: status, dependencies
- ErrorResponse: error, message, details

**Contracts (openapi.yaml)**:
- POST /ask - Accept question, return grounded answer
- GET /health - Service health check
- Full OpenAPI 3.0 specification with examples

**Quickstart (quickstart.md)**:
- Environment setup
- Server startup command
- curl examples for all endpoints
- Python client example
- Troubleshooting guide

## Outcome

- Impact: Complete implementation plan ready for task generation
- Tests: Constitution check PASS (all 7 principles including RAG guarantees)
- Files: plan.md, research.md, data-model.md, quickstart.md, contracts/openapi.yaml
- Next prompts: `/sp.tasks` to generate implementation tasks
- Reflection: Clean separation of concerns - agent layer separate from retrieval

## Evaluation notes (flywheel)

- Failure modes observed: None - clear integration with existing codebase
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
