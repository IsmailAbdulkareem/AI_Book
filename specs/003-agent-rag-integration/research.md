# Research: Agentic RAG with OpenAI Agents SDK + FastAPI

**Feature**: 003-agent-rag-integration
**Date**: 2025-12-15

## Research Questions

### 1. OpenAI Agents SDK Integration Pattern

**Decision**: Use OpenAI Python SDK with function calling for RAG tool integration

**Rationale**:
- OpenAI Agents SDK provides native support for tool/function calling
- Allows defining a "retrieval" tool that the agent can invoke
- Maintains clean separation between retrieval logic and LLM generation
- Well-documented and widely adopted pattern for RAG applications

**Alternatives Considered**:
- LangChain: More abstraction layers, heavier dependency, overkill for single-tool RAG
- Direct API calls without SDK: Less structured, harder to maintain tool definitions
- Custom agent loop: More work, reinvents existing patterns

**Implementation Approach**:
```python
# Define retrieval as a tool the agent can call
tools = [{
    "type": "function",
    "function": {
        "name": "search_book_content",
        "description": "Search the Physical AI book for relevant content",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Search query"}
            }
        }
    }
}]
```

---

### 2. FastAPI Service Architecture

**Decision**: Single-file FastAPI application with async endpoints

**Rationale**:
- FastAPI provides automatic OpenAPI documentation
- Async support matches well with OpenAI API calls
- Pydantic models provide request/response validation
- Lightweight and fast startup time
- Consistent with existing database/ folder single-file pattern

**Alternatives Considered**:
- Flask: Synchronous by default, less built-in validation
- Django: Too heavy for a simple API service
- Starlette: Lower-level, FastAPI adds useful features on top

**Endpoint Structure**:
- `POST /ask` - Accept question, return grounded answer with sources
- `GET /health` - Service health check with dependency status

---

### 3. Retrieval Integration Strategy

**Decision**: Import and reuse `RetrievalPipeline` class from retrieval.py

**Rationale**:
- Existing pipeline already handles Cohere embeddings and Qdrant search
- Returns structured `RetrievalResponse` with sources
- Tested and validated in Spec 2
- Avoids code duplication and maintains single source of truth
- FR-010 requires no modification to existing Spec 1/2 code

**Integration Pattern**:
```python
from retrieval import RetrievalPipeline, RetrievalResponse

pipeline = RetrievalPipeline(collection="rag_embedding")
response = pipeline.search(query=question, top_k=top_k)
```

---

### 4. Answer Generation with Grounding

**Decision**: Use GPT-4o-mini with system prompt that enforces grounding

**Rationale**:
- GPT-4o-mini provides good balance of quality and cost
- System prompt can enforce answering only from provided context
- Retrieved chunks formatted as numbered sources for citation
- Model instructed to cite sources by number in response

**Grounding Strategy**:
```python
system_prompt = """You are an assistant for the Physical AI & Humanoid Robotics book.
Answer questions ONLY using the provided context. If the context doesn't contain
relevant information, say so. Always cite sources by their number [1], [2], etc."""

# Format context for the model
context = "\n".join([
    f"[{i+1}] {result.content}\nSource: {result.url}"
    for i, result in enumerate(retrieval_response.results)
])
```

---

### 5. Response Structure Design

**Decision**: Return structured JSON with answer, sources, and metadata

**Rationale**:
- FR-013 requires answer text, sources list, and original question
- Structured response enables downstream processing
- Sources include URL, content snippet, and relevance score
- Consistent with existing RetrievalResponse structure

**Response Schema**:
```json
{
    "question": "What is ROS 2?",
    "answer": "ROS 2 is... [1][2]",
    "sources": [
        {
            "url": "https://...",
            "content": "...",
            "score": 0.85
        }
    ],
    "retrieval_time_ms": 450,
    "generation_time_ms": 1200
}
```

---

### 6. Error Handling Strategy

**Decision**: Use FastAPI HTTPException with detailed error responses

**Rationale**:
- FastAPI HTTPException integrates with OpenAPI docs
- Provides consistent error format
- FR-006 requires clear error messages
- Can distinguish between client errors (400) and server errors (500)

**Error Categories**:
- 400: Empty question, invalid top_k
- 500: Qdrant unavailable, OpenAI API error
- 503: Service degraded (partial functionality)

---

### 7. Health Check Design

**Decision**: Check Qdrant connectivity and report component status

**Rationale**:
- FR-009 requires health check endpoint
- Should verify vector database is accessible
- OpenAI API health can be inferred from response
- Return degraded status if dependencies unavailable

**Health Response**:
```json
{
    "status": "healthy",
    "dependencies": {
        "qdrant": {"status": "connected", "collection": "rag_embedding", "vectors": 135},
        "openai": {"status": "configured"}
    }
}
```

---

## Technology Stack Summary

| Component | Choice | Version |
|-----------|--------|---------|
| Web Framework | FastAPI | 0.109+ |
| OpenAI SDK | openai | 1.0+ |
| Retrieval | Existing RetrievalPipeline | (from retrieval.py) |
| Embeddings | Cohere | (from retrieval.py) |
| Vector DB | Qdrant | (from retrieval.py) |
| Validation | Pydantic | (via FastAPI) |
| ASGI Server | uvicorn | 0.27+ |

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

## Environment Variables Required

```bash
# Existing (from Spec 1/2)
COHERE_API_KEY=...
QDRANT_URL=...
QDRANT_API_KEY=...

# New for Spec 3
OPENAI_API_KEY=...
```
