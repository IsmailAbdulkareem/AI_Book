# Research: RAG Retrieval & Pipeline Validation

**Feature**: 002-rag-retrieval
**Date**: 2025-12-15
**Status**: Complete

## Technology Decisions

### 1. Query Embedding: Cohere embed-multilingual-v3.0

**Decision**: Use the same model as ingestion (embed-multilingual-v3.0)

**Rationale**:
- Must match the model used for document embeddings (consistency requirement)
- Existing pipeline uses `embed-multilingual-v3.0` with 1024 dimensions
- `input_type="search_query"` for queries (vs `search_document` for ingestion)

**Key Configuration**:
```python
response = cohere_client.embed(
    texts=[query],
    model="embed-multilingual-v3.0",
    input_type="search_query",  # Different from ingestion!
)
```

**Alternatives Considered**:
- Different model: Would break similarity matching with existing embeddings

---

### 2. Vector Search: Qdrant Search API

**Decision**: Use Qdrant's native search with cosine similarity

**Rationale**:
- Collection already configured with `Distance.COSINE`
- Native support for top-k limiting and payload filtering
- Returns similarity scores normalized to 0.0-1.0

**Search Pattern**:
```python
results = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=top_k,
    query_filter=filter_conditions,  # Optional
    with_payload=True,
    with_vectors=False,  # Don't return vectors (saves bandwidth)
)
```

**Filter Support**:
```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Filter by URL
filter = Filter(
    must=[
        FieldCondition(key="url", match=MatchValue(value="https://..."))
    ]
)
```

---

### 3. CLI Design: argparse with Subcommands

**Decision**: Extend existing script with `query` subcommand

**Rationale**:
- Keeps all RAG functionality in single entry point
- Simple, no additional dependencies
- Clear separation: `python main.py ingest` vs `python main.py query`

**CLI Structure**:
```bash
# Ingestion (existing)
python main.py ingest

# Query (new)
python main.py query "What is ROS 2?"
python main.py query "robotics" --top-k 10
python main.py query "navigation" --url-filter "https://..."
python main.py query "sensors" --debug
```

**Arguments**:
| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| query | positional | required | Natural language query |
| --top-k | int | 5 | Number of results |
| --url-filter | str | None | Filter by source URL |
| --debug | flag | False | Show detailed diagnostics |
| --collection | str | rag_embedding | Target collection |

**Alternatives Considered**:
- Separate script: Duplicates client initialization code
- Click/Typer: Additional dependency, overkill for this scope

---

### 4. Output Format: Structured Text with JSON Option

**Decision**: Human-readable text output with optional JSON

**Rationale**:
- Text output for interactive CLI use
- JSON output for programmatic consumption (LLM integration)

**Text Format**:
```
Query: "What is ROS 2?"
Results: 5 matches (0.42s)

[1] Score: 0.89
    URL: https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-2
    Content: ROS 2 (Robot Operating System 2) is a flexible framework...
    Position: 2

[2] Score: 0.84
    ...
```

**JSON Format** (--json flag):
```json
{
  "query": "What is ROS 2?",
  "results": [
    {
      "score": 0.89,
      "content": "...",
      "url": "...",
      "position": 2
    }
  ],
  "total": 5,
  "time_ms": 420
}
```

---

### 5. Debug Mode

**Decision**: Verbose logging with diagnostic information

**Rationale**:
- Helps diagnose retrieval issues without code inspection
- Shows vector dimensions, collection stats, timing breakdown

**Debug Output**:
```
[DEBUG] Query embedding: 1024 dimensions, generated in 0.15s
[DEBUG] Collection 'rag_embedding': 523 vectors
[DEBUG] Search completed in 0.27s
[DEBUG] Score distribution: min=0.42, max=0.89, avg=0.68
```

---

### 6. Error Handling Strategy

**Decision**: Graceful degradation with actionable messages

| Error Type | Message | Action |
|------------|---------|--------|
| Missing collection | "Collection 'X' not found. Run ingestion first." | Exit with code 1 |
| Empty query | "Query cannot be empty" | Exit with code 1 |
| Embedding failure | "Failed to embed query: {error}" | Exit with code 1 |
| No results | "No matching results found" | Return empty list (success) |
| Connection timeout | "Cannot connect to Qdrant: {error}" | Exit with code 1 |

---

### 7. Test Harness Design

**Decision**: Simple assertion-based tests with known queries

**Rationale**:
- Validates end-to-end pipeline
- Catches regressions in retrieval accuracy
- No additional test framework needed

**Test Cases**:
```python
TEST_QUERIES = [
    {
        "query": "ROS 2 architecture",
        "expected_url_contains": "/module-1/",
        "min_score": 0.5,
    },
    {
        "query": "Gazebo simulation",
        "expected_url_contains": "/module-2/",
        "min_score": 0.5,
    },
]
```

**Validation**:
- Top result URL contains expected substring
- Top result score >= minimum threshold
- Results are returned within timeout

---

## Existing Codebase Integration

**Current State** (from `database/main.py`):
- Class: `DocusaurusEmbeddingPipeline`
- Collection: `rag_embedding`
- Model: `embed-multilingual-v3.0`
- Payload: `content`, `url`, `position`, `created_at`

**Extension Strategy**:
1. Add `search()` method to existing class
2. Add `embed_query()` method (uses `input_type="search_query"`)
3. Add CLI subcommand parsing in `main()`
4. Add `QueryResult` dataclass for structured results

**File Changes**:
- Modify: `database/main.py` (add retrieval methods and CLI)
- No new files needed (single-file design maintained)

---

## Dependencies

No new dependencies required - all functionality available in existing packages:
- `cohere` - Already installed (for query embedding)
- `qdrant-client` - Already installed (for search)
- `argparse` - Python standard library (for CLI)

---

## Performance Considerations

| Metric | Target | Strategy |
|--------|--------|----------|
| Query embedding | <500ms | Single text, no batching needed |
| Vector search | <200ms | Qdrant HNSW index handles 10k+ vectors |
| Total response | <2s | Well within target for expected collection size |

---

## Summary

| Component | Decision |
|-----------|----------|
| Embedding model | embed-multilingual-v3.0 (match ingestion) |
| Search method | Qdrant native search with cosine similarity |
| CLI design | argparse subcommands (query, ingest) |
| Output format | Text default, JSON optional |
| Debug mode | Verbose logging with diagnostics |
| Test harness | Assertion-based with known queries |
