# Data Model: Agentic RAG with OpenAI Agents SDK + FastAPI

**Feature**: 003-agent-rag-integration
**Date**: 2025-12-15

## Overview

This document defines the data structures for the RAG Agent API. All structures are implemented as Pydantic models for FastAPI integration and automatic validation.

## Request Models

### 1. AskRequest

The request body for the `/ask` endpoint.

```python
from pydantic import BaseModel, Field

class AskRequest(BaseModel):
    """Request to ask a question to the RAG agent."""
    question: str = Field(
        ...,
        min_length=1,
        description="Natural language question about the book content"
    )
    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of context chunks to retrieve"
    )
```

**Validation Rules**:
- `question` must be non-empty (min_length=1)
- `top_k` must be between 1 and 20 (inclusive)

---

## Response Models

### 2. Source

A single source citation from the retrieved context.

```python
class Source(BaseModel):
    """A source citation from the book content."""
    url: str = Field(..., description="URL of the source page")
    content: str = Field(..., description="Content snippet from the source")
    score: float = Field(..., ge=0.0, le=1.0, description="Relevance score")
    position: int = Field(..., ge=0, description="Chunk position in document")
```

**Validation Rules**:
- `url` must be a valid URL string
- `score` must be between 0.0 and 1.0
- `position` must be non-negative

---

### 3. AskResponse

The complete response from the `/ask` endpoint.

```python
class AskResponse(BaseModel):
    """Response containing the grounded answer and sources."""
    question: str = Field(..., description="The original question asked")
    answer: str = Field(..., description="Generated answer grounded in sources")
    sources: List[Source] = Field(
        default_factory=list,
        description="List of source citations used"
    )
    retrieval_time_ms: int = Field(
        ...,
        ge=0,
        description="Time taken to retrieve context"
    )
    generation_time_ms: int = Field(
        ...,
        ge=0,
        description="Time taken to generate answer"
    )
```

**Computed Fields**:
- `total_time_ms` = `retrieval_time_ms` + `generation_time_ms` (can be computed client-side)

---

### 4. HealthStatus

Health status for a single dependency.

```python
class DependencyHealth(BaseModel):
    """Health status of a single dependency."""
    status: str = Field(..., description="Status: connected, configured, error")
    details: Optional[dict] = Field(
        default=None,
        description="Additional details about the dependency"
    )
```

---

### 5. HealthResponse

The response from the `/health` endpoint.

```python
class HealthResponse(BaseModel):
    """Service health check response."""
    status: str = Field(
        ...,
        description="Overall status: healthy, degraded, unhealthy"
    )
    dependencies: dict[str, DependencyHealth] = Field(
        ...,
        description="Status of each dependency"
    )
```

**Status Values**:
- `healthy`: All dependencies operational
- `degraded`: Some dependencies unavailable but service partially functional
- `unhealthy`: Critical dependencies unavailable, service non-functional

---

### 6. ErrorResponse

Standard error response format.

```python
class ErrorResponse(BaseModel):
    """Error response for failed requests."""
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(
        default=None,
        description="Additional error details"
    )
```

**Error Types**:
- `validation_error`: Invalid request data
- `retrieval_error`: Failed to search vector database
- `generation_error`: Failed to generate answer
- `service_unavailable`: Critical dependency unavailable

---

## Data Flow

```
User Question (str)
    │
    ▼
┌─────────────────┐
│  AskRequest     │  Validate question and top_k
│ (question, k)   │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  RetrievalPipeline │  From retrieval.py
│  search(query, k)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  QueryResult[]  │  score, content, url, position
│  (from Qdrant)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Format Context │  Number sources [1], [2], etc.
│  for LLM        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  OpenAI API     │  Generate grounded answer
│  (GPT-4o-mini)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  AskResponse    │  Package answer + sources
│ (answer, sources)│
└─────────────────┘
```

---

## Entity Relationships

```
AskRequest ──────────────────────────────────────┐
    │                                            │
    │ triggers                                   │
    ▼                                            │
RetrievalPipeline.search()                       │
    │                                            │
    │ returns                                    │
    ▼                                            │
QueryResult[] ──────────────────────┐            │
    │                               │            │
    │ maps to                       │            │
    ▼                               ▼            ▼
Source[] ────────────────────> AskResponse <─────┘
                                   │
                                   │ contains
                                   ▼
                              answer (str)
```

---

## Existing Entities (from Spec 2)

The agent reuses these entities from `retrieval.py`:

### QueryResult (from retrieval.py)

```python
@dataclass
class QueryResult:
    score: float
    content: str
    url: str
    position: int
    created_at: float
```

### RetrievalResponse (from retrieval.py)

```python
@dataclass
class RetrievalResponse:
    query: str
    results: List[QueryResult]
    total: int
    time_ms: int
    collection: str
```

---

## Mapping: QueryResult → Source

```python
def query_result_to_source(qr: QueryResult) -> Source:
    return Source(
        url=qr.url,
        content=qr.content[:500],  # Truncate for response
        score=qr.score,
        position=qr.position,
    )
```
