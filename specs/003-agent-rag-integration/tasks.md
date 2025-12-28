# Tasks: Agentic RAG with OpenAI Agents SDK + FastAPI

**Input**: Design documents from `/specs/003-agent-rag-integration/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/, research.md, quickstart.md
**Depends On**: 001-embedding-pipeline, 002-rag-retrieval (must be complete)

**Tests**: Manual API verification + quickstart.md examples

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Spec 1 (Ingestion)**: `database/main.py` — DO NOT MODIFY
- **Spec 2 (Retrieval)**: `database/retrieval.py` — DO NOT MODIFY
- **Spec 3 (Agent)**: `database/agent.py` — NEW FILE

---

## Phase 1: Setup (Environment & Dependencies)

**Purpose**: Add required dependencies and configure environment for FastAPI + OpenAI

- [x] T001 Add `fastapi>=0.109.0` to dependencies in `database/pyproject.toml`
- [x] T002 Add `uvicorn>=0.27.0` to dependencies in `database/pyproject.toml`
- [x] T003 Add `openai>=1.0.0` to dependencies in `database/pyproject.toml`
- [x] T004 Run `uv sync` to install new dependencies in `database/`
- [x] T005 Add `OPENAI_API_KEY=your_key_here` to `database/.env.example`

**Checkpoint**: Dependencies installed - `uv run python -c "import fastapi, uvicorn, openai"` succeeds

---

## Phase 2: Foundational (Data Models & Constants)

**Purpose**: Define Pydantic models and constants needed by all user stories

- [x] T006 Create `database/agent.py` with module docstring and imports
- [x] T007 [P] Add environment loading with `load_dotenv()` in `database/agent.py`
- [x] T008 [P] Define `SYSTEM_PROMPT` constant for grounded answer generation in `database/agent.py`
- [x] T009 [P] Define `AskRequest` Pydantic model in `database/agent.py` (question: str, top_k: int = 5)
- [x] T010 [P] Define `Source` Pydantic model in `database/agent.py` (url, content, score, position)
- [x] T011 [P] Define `AskResponse` Pydantic model in `database/agent.py` (question, answer, sources, timing)
- [x] T012 [P] Define `DependencyHealth` Pydantic model in `database/agent.py` (status, details)
- [x] T013 [P] Define `HealthResponse` Pydantic model in `database/agent.py` (status, dependencies)
- [x] T014 [P] Define `ErrorResponse` Pydantic model in `database/agent.py` (error, message, details)
- [x] T015 Initialize FastAPI application with title and description in `database/agent.py`

**Checkpoint**: All data models defined - `uv run python -c "from agent import *"` succeeds

---

## Phase 3: User Story 1 - Ask Question and Get Grounded Answer (Priority: P1)

**Goal**: Core RAG functionality - retrieve context, generate grounded answer with citations

**Independent Test**: `curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What is ROS 2?"}'` returns answer with sources

### Implementation for User Story 1

- [x] T016 [US1] Import `RetrievalPipeline` from `retrieval.py` in `database/agent.py`
- [x] T017 [US1] Import `OpenAI` client from `openai` package in `database/agent.py`
- [x] T018 [US1] Implement `format_context(results: List[QueryResult]) -> str` helper function in `database/agent.py`
  - Number each source [1], [2], etc.
  - Include content and source URL for each

- [x] T019 [US1] Implement `generate_answer(question: str, context: str) -> tuple[str, int]` function in `database/agent.py`
  - Initialize OpenAI client
  - Call chat completions API with system prompt and context
  - Use GPT-4o-mini model with temperature=0.3
  - Return answer text and generation time in ms

- [x] T020 [US1] Implement `query_result_to_source(qr: QueryResult) -> Source` helper in `database/agent.py`
  - Map QueryResult fields to Source model
  - Truncate content to 500 chars

- [x] T021 [US1] Implement `POST /ask` endpoint in `database/agent.py`
  - Accept AskRequest body
  - Initialize RetrievalPipeline
  - Call search with question and top_k
  - Format context from results
  - Generate answer using OpenAI
  - Return AskResponse with sources and timing

- [x] T022 [US1] Add error handling for empty retrieval results in `/ask` endpoint in `database/agent.py`
  - Return answer stating no relevant content found
  - Still return 200 status (not an error)

- [x] T023 [US1] Add error handling for OpenAI API failures in `/ask` endpoint in `database/agent.py`
  - Catch exceptions from generate_answer
  - Return 500 with ErrorResponse

- [x] T024 [US1] Add error handling for Qdrant connection failures in `/ask` endpoint in `database/agent.py`
  - Catch exceptions from RetrievalPipeline
  - Return 503 with ErrorResponse

**Checkpoint**: User Story 1 complete - basic question answering works

**Verification**:
```bash
cd database
uv run uvicorn agent:app --reload &
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
# Should return answer with sources containing "module1" URLs
```

---

## Phase 4: User Story 2 - Configure Retrieval Parameters (Priority: P2)

**Goal**: Allow users to customize top_k parameter for retrieval

**Independent Test**: Requests with different `top_k` values return corresponding number of sources

### Implementation for User Story 2

- [x] T025 [US2] Verify `AskRequest.top_k` validation (ge=1, le=20, default=5) in `database/agent.py`
- [x] T026 [US2] Add validation error handler for invalid `top_k` in `database/agent.py`
  - Return 400 with ErrorResponse for out-of-range values

- [x] T027 [US2] Verify top_k is passed correctly to `RetrievalPipeline.search()` in `database/agent.py`

**Checkpoint**: User Story 2 complete - top_k parameter works correctly

**Verification**:
```bash
# Test with top_k=3
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "top_k": 3}'
# Should return at most 3 sources

# Test default top_k
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
# Should return up to 5 sources (default)
```

---

## Phase 5: User Story 3 - Health Check and Service Status (Priority: P3)

**Goal**: Provide health check endpoint for monitoring

**Independent Test**: `curl http://localhost:8000/health` returns status with dependency info

### Implementation for User Story 3

- [x] T028 [US3] Implement `check_qdrant_health() -> DependencyHealth` helper in `database/agent.py`
  - Use RetrievalPipeline.validate_collection()
  - Return connected status with vector count if healthy
  - Return error status with details if unhealthy

- [x] T029 [US3] Implement `check_openai_health() -> DependencyHealth` helper in `database/agent.py`
  - Check if OPENAI_API_KEY is configured in environment
  - Return configured status if key exists
  - Return error status if key missing

- [x] T030 [US3] Implement `GET /health` endpoint in `database/agent.py`
  - Check all dependencies
  - Aggregate into overall status (healthy/degraded/unhealthy)
  - Return HealthResponse

- [x] T031 [US3] Add logic to determine overall health status in `database/agent.py`
  - healthy: all dependencies connected
  - degraded: some dependencies have errors
  - unhealthy: critical dependencies (qdrant) unavailable

**Checkpoint**: User Story 3 complete - health check works

**Verification**:
```bash
curl http://localhost:8000/health
# Should return {"status": "healthy", "dependencies": {...}}
```

---

## Phase 6: Polish & Integration

**Purpose**: Error handling polish, documentation, final testing

- [x] T032 Add CORS middleware to FastAPI app in `database/agent.py`
  - Allow all origins for development

- [x] T033 Add request validation error handler in `database/agent.py`
  - Return 400 with ErrorResponse for Pydantic validation errors

- [x] T034 [P] Verify all quickstart.md examples work end-to-end
- [x] T035 [P] Test error handling: empty question returns 400
- [x] T036 [P] Test error handling: invalid top_k returns 400
- [x] T037 Verify API documentation at http://localhost:8000/docs works

**Checkpoint**: Feature complete and documented

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phase 3 (US1)**: Depends on Phase 2 - Core functionality
- **Phase 4 (US2)**: Depends on Phase 3 - Extends /ask endpoint
- **Phase 5 (US3)**: Depends on Phase 2 - Independent of US1/US2
- **Phase 6 (Polish)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies - core functionality
- **User Story 2 (P2)**: Depends on US1 - adds parameter to existing endpoint
- **User Story 3 (P3)**: Independent of US1/US2 - can run in parallel after Phase 2

**Note**: US3 can be implemented in parallel with US1 after Phase 2 is complete.

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
```bash
# These model definitions can be written in parallel:
Task T007-T014: All Pydantic model definitions [P]
```

**After Phase 2 Complete (US1 and US3 parallel)**:
```bash
# US3 is independent of US1:
Task T016-T024: User Story 1 (ask endpoint)
Task T028-T031: User Story 3 (health endpoint)  # Can run in parallel!
```

**Within Phase 6 (Polish)**:
```bash
# These verification tasks can run in parallel:
Task T034: Verify quickstart examples [P]
Task T035: Test empty question error [P]
Task T036: Test invalid top_k error [P]
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T015)
3. Complete Phase 3: User Story 1 (T016-T024)
4. **STOP and VALIDATE**: Test basic question answering
5. Verify `POST /ask` returns grounded answers with sources

### Incremental Delivery

1. Setup + Foundational → FastAPI app structure ready
2. Add User Story 1 → Core RAG Q&A works
3. Add User Story 2 → Configurable retrieval
4. Add User Story 3 → Health monitoring
5. Polish → Error handling, documentation

### Single Developer Strategy

Since this is a new single file:

1. **Session 1**: T001-T015 (Setup + Foundational)
2. **Session 2**: T016-T024 (US1 - Core Q&A)
3. **Session 3**: T025-T027 (US2 - Parameters) + T028-T031 (US3 - Health)
4. **Session 4**: T032-T037 (Polish)

---

## Task Summary

| Phase | Tasks | Parallel Tasks | User Story |
|-------|-------|----------------|------------|
| Phase 1: Setup | 5 | 0 | - |
| Phase 2: Foundational | 10 | 8 | - |
| Phase 3: US1 - Ask Question | 9 | 0 | P1 |
| Phase 4: US2 - Parameters | 3 | 0 | P2 |
| Phase 5: US3 - Health Check | 4 | 0 | P3 |
| Phase 6: Polish | 6 | 3 | - |
| **Total** | **37** | **11** | |

---

## Notes

- **Spec 1 and Spec 2 code MUST NOT be modified**
- All implementation is in `database/agent.py` (new file)
- Imports `RetrievalPipeline` from `retrieval.py` for context retrieval
- Uses OpenAI GPT-4o-mini for answer generation
- FastAPI provides automatic OpenAPI documentation at `/docs`
- Environment variables: OPENAI_API_KEY (new), COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY (existing)
