# Implementation Plan: Agentic RAG with OpenAI Agents SDK + FastAPI

**Branch**: `003-agent-rag-integration` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Depends On**: 001-embedding-pipeline, 002-rag-retrieval
**Input**: Feature specification from `/specs/003-agent-rag-integration/spec.md`

## Summary

Build a RAG Agent API that accepts natural language questions via HTTP, retrieves relevant context from Qdrant using the existing retrieval pipeline, generates grounded answers using OpenAI GPT-4o-mini, and returns answers with source citations. Implementation creates a new `database/agent.py` file without modifying existing Spec 1/2 code.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, uvicorn, openai (new); cohere, qdrant-client (existing)
**Package Manager**: UV (existing)
**Storage**: Qdrant Cloud (collection: rag_embedding) - reuses existing
**Testing**: Manual verification via API + automated test scenarios
**Target Platform**: Local development / HTTP API server
**Project Type**: Single file extension (database/agent.py)
**Performance Goals**: <10 seconds response time (95th percentile)
**Constraints**: Must not modify Spec 1 or Spec 2 code
**Scale/Scope**: Single-user API, ~100 concurrent requests

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-first authoring | PASS | Full spec and plan created before implementation |
| Technical accuracy | PASS | Uses documented OpenAI/FastAPI APIs |
| Reproducibility | PASS | API documented in quickstart.md and OpenAPI spec |
| Consistency | PASS | Extends existing code patterns from Spec 2 |
| Transparent AI usage | PASS | AI-assisted development documented |
| RAG Retrieval Guarantees | PASS | Answers grounded in retrieved chunks with citations |
| Separation of Concerns | PASS | Agent layer separate from retrieval and ingestion |

## Project Structure

### Documentation (this feature)

```text
specs/003-agent-rag-integration/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology research findings
├── data-model.md        # Data structures documentation
├── quickstart.md        # API usage guide
├── contracts/
│   └── openapi.yaml     # OpenAPI specification
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
database/
├── main.py              # Spec 1: Ingestion pipeline (DO NOT MODIFY)
├── retrieval.py         # Spec 2: Retrieval pipeline (DO NOT MODIFY)
├── agent.py             # Spec 3: RAG Agent API (NEW FILE)
├── pyproject.toml       # Add fastapi, uvicorn, openai
├── .env.example         # Add OPENAI_API_KEY
└── .gitignore           # No changes needed
```

**Structure Decision**: Create new `database/agent.py` file that imports from `retrieval.py`. Maintains single-file-per-spec pattern while enabling code reuse.

---

## Function Design

### 1. FastAPI Application Setup

**Purpose**: Initialize FastAPI app with CORS and exception handlers

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="RAG Agent API",
    description="Physical AI Book RAG Chatbot",
    version="1.0.0",
)
```

---

### 2. `POST /ask` Endpoint

**Purpose**: Accept question, retrieve context, generate grounded answer

**Input**: AskRequest (question: str, top_k: int = 5)
**Output**: AskResponse (question, answer, sources, timing)

**Logic**:
1. Validate question is non-empty
2. Initialize RetrievalPipeline from retrieval.py
3. Call `pipeline.search(query=question, top_k=top_k)`
4. Format retrieved chunks as numbered context
5. Call OpenAI API with system prompt enforcing grounding
6. Parse response and extract citations
7. Return AskResponse with sources

---

### 3. `GET /health` Endpoint

**Purpose**: Check service and dependency health

**Input**: None
**Output**: HealthResponse (status, dependencies)

**Logic**:
1. Check Qdrant connectivity via `validate_collection()`
2. Check OpenAI API key is configured
3. Return aggregated health status

---

### 4. `generate_answer(question: str, context: List[QueryResult]) -> str`

**Purpose**: Generate grounded answer using OpenAI

**Input**: Question string, list of retrieved chunks
**Output**: Generated answer with source citations

**Logic**:
1. Format context as numbered sources
2. Construct system prompt enforcing grounding
3. Call OpenAI chat completion API
4. Return generated text

---

### 5. `format_context(results: List[QueryResult]) -> str`

**Purpose**: Format retrieved chunks for LLM consumption

**Input**: List of QueryResult from retrieval
**Output**: Formatted string with numbered sources

**Format**:
```
[1] Content from first chunk...
Source: https://example.com/page1

[2] Content from second chunk...
Source: https://example.com/page2
```

---

## Data Classes (Pydantic Models)

```python
from pydantic import BaseModel, Field
from typing import List, Optional

class AskRequest(BaseModel):
    question: str = Field(..., min_length=1)
    top_k: int = Field(default=5, ge=1, le=20)

class Source(BaseModel):
    url: str
    content: str
    score: float
    position: int

class AskResponse(BaseModel):
    question: str
    answer: str
    sources: List[Source]
    retrieval_time_ms: int
    generation_time_ms: int

class DependencyHealth(BaseModel):
    status: str
    details: Optional[dict] = None

class HealthResponse(BaseModel):
    status: str  # healthy, degraded, unhealthy
    dependencies: dict[str, DependencyHealth]

class ErrorResponse(BaseModel):
    error: str
    message: str
    details: Optional[dict] = None
```

---

## Error Handling

| Error | HTTP Status | Error Type | Message |
|-------|-------------|------------|---------|
| Empty question | 400 | validation_error | "Question cannot be empty" |
| Invalid top_k | 400 | validation_error | "top_k must be between 1 and 20" |
| Qdrant unavailable | 503 | service_unavailable | "Vector database is unavailable" |
| OpenAI API error | 500 | generation_error | "Failed to generate answer: {details}" |
| No context found | 200 | (not an error) | Return answer stating no relevant content found |

---

## OpenAI Integration

### System Prompt

```python
SYSTEM_PROMPT = """You are an assistant for the Physical AI & Humanoid Robotics book.

IMPORTANT RULES:
1. Answer questions ONLY using the provided context
2. If the context doesn't contain relevant information, say "I don't have information about that topic in the book"
3. Always cite sources by their number [1], [2], etc.
4. Be concise and accurate
5. Do not make up information not in the context

Context will be provided in the format:
[1] Content...
Source: URL

Use this context to answer the user's question."""
```

### API Call

```python
from openai import OpenAI

client = OpenAI()

response = client.chat.completions.create(
    model="gpt-4o-mini",
    messages=[
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": f"Context:\n{formatted_context}\n\nQuestion: {question}"}
    ],
    temperature=0.3,  # Lower temperature for factual responses
    max_tokens=1000,
)
```

---

## Dependencies to Add

```toml
# In pyproject.toml
dependencies = [
    # Existing...
    "fastapi>=0.109.0",
    "uvicorn>=0.27.0",
    "openai>=1.0.0",
]
```

---

## Environment Variables

```bash
# Existing (from Spec 1/2)
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...

# New for Spec 3
OPENAI_API_KEY=...
```

---

## Success Verification

After implementation:

1. **Health Check**: `GET /health` returns healthy status
2. **Basic Query**: `POST /ask` with "What is ROS 2?" returns grounded answer with sources
3. **Custom top_k**: `POST /ask` with top_k=3 returns at most 3 sources
4. **Error Handling**: Empty question returns 400 with clear message
5. **Grounding**: Answers cite sources by number [1], [2], etc.
6. **Timing**: Response includes retrieval_time_ms and generation_time_ms

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| OpenAI API latency | Set reasonable timeout, async handling |
| Hallucination | Strong system prompt, low temperature |
| Context too long | Truncate chunks, limit top_k |
| Rate limiting | Document in quickstart, suggest retry |

---

## Complexity Tracking

No complexity violations - single file design maintained.

---

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Add dependencies to pyproject.toml
3. Implement agent.py with FastAPI endpoints
4. Add OPENAI_API_KEY to .env.example
5. Test end-to-end
6. Validate against acceptance scenarios
