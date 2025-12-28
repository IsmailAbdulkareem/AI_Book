# Implementation Plan: RAG Retrieval & Pipeline Validation

**Branch**: `002-rag-retrieval` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Depends On**: 001-embedding-pipeline (Content Ingestion Complete)
**Input**: Feature specification from `/specs/002-rag-retrieval/spec.md`

## Summary

Add semantic search retrieval capability to the existing RAG pipeline. Users can query ingested content using natural language, receive ranked results with similarity scores, filter by metadata, and debug retrieval issues. Implementation extends the existing `database/main.py` with new methods and CLI subcommands.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client (already installed)
**Package Manager**: UV (existing)
**Storage**: Qdrant Cloud (collection: rag_embedding)
**Testing**: Manual verification + automated test harness
**Target Platform**: Local development / CLI
**Project Type**: Single file extension (database/main.py)
**Performance Goals**: Query response <2 seconds
**Constraints**: Must use same embedding model as ingestion (embed-multilingual-v3.0)
**Scale/Scope**: ~500-1000 stored chunks, single-user CLI

## Constitution Check

*GATE: Must pass before implementation.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-first authoring | PASS | Full spec and plan created before implementation |
| Technical accuracy | PASS | Uses documented Cohere/Qdrant APIs |
| Reproducibility | PASS | CLI commands documented in quickstart |
| Consistency | PASS | Extends existing code patterns |
| Transparent AI usage | PASS | AI-assisted development documented |

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-retrieval/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology research findings
├── data-model.md        # Data structures documentation
├── quickstart.md        # CLI usage guide
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
database/
├── main.py              # Extended with retrieval methods
├── pyproject.toml       # No changes needed
├── .env.example         # No changes needed
└── .gitignore           # No changes needed
```

**Structure Decision**: Extend existing `database/main.py` to maintain single-file design. No new files created.

## Function Design

Extensions to `DocusaurusEmbeddingPipeline` class:

### 1. `embed_query(query: str) -> List[float]`

**Purpose**: Embed a search query using Cohere (with search_query input type)

**Input**: Natural language query string
**Output**: 1024-dimensional embedding vector

**Logic**:
```python
def embed_query(self, query: str) -> List[float]:
    response = self.cohere_client.embed(
        texts=[query],
        model=COHERE_EMBED_MODEL,
        input_type="search_query",  # Key difference from document embedding
    )
    return response.embeddings[0]
```

---

### 2. `search(query: str, top_k: int = 5, url_filter: str = None) -> RetrievalResponse`

**Purpose**: Execute semantic search against stored embeddings

**Input**: Query string, result limit, optional URL filter
**Output**: RetrievalResponse with ranked results

**Logic**:
1. Embed query using `embed_query()`
2. Build Qdrant filter if url_filter provided
3. Execute Qdrant search with query vector
4. Convert ScoredPoints to QueryResult objects
5. Package into RetrievalResponse with timing

---

### 3. `validate_collection(collection: str) -> bool`

**Purpose**: Check if target collection exists and has vectors

**Input**: Collection name
**Output**: True if valid, False otherwise

**Logic**:
1. Get collection info from Qdrant
2. Check points_count > 0
3. Return validation result

---

### 4. `format_results(response: RetrievalResponse, json_output: bool = False) -> str`

**Purpose**: Format retrieval results for display

**Input**: RetrievalResponse, output format flag
**Output**: Formatted string (text or JSON)

---

### 5. `run_test_harness() -> bool`

**Purpose**: Execute automated retrieval tests

**Input**: None (uses predefined test cases)
**Output**: True if all tests pass, False otherwise

**Test Cases**:
- Known queries return expected URL patterns
- Empty queries are handled gracefully
- Missing collection fails clearly

---

## CLI Design

### Subcommand Structure

```bash
# Existing functionality (renamed)
python main.py ingest

# New functionality
python main.py query "your search query"
python main.py test
```

### Query Command Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| query | positional | required | Natural language query |
| --top-k, -k | int | 5 | Number of results |
| --url-filter, -u | str | None | Filter by URL substring |
| --debug, -d | flag | False | Show diagnostics |
| --json, -j | flag | False | Output as JSON |
| --collection, -c | str | rag_embedding | Target collection |

### Examples

```bash
# Basic query
python main.py query "What is ROS 2?"

# Limit results
python main.py query "robotics" --top-k 3

# Filter by URL
python main.py query "simulation" --url-filter "/module-2/"

# Debug mode
python main.py query "sensors" --debug

# JSON output (for LLM integration)
python main.py query "navigation" --json

# Run tests
python main.py test
```

---

## Data Classes

Add to `database/main.py`:

```python
@dataclass
class QueryResult:
    score: float
    content: str
    url: str
    position: int
    created_at: float

@dataclass
class RetrievalResponse:
    query: str
    results: List[QueryResult]
    total: int
    time_ms: int
    collection: str
```

---

## Error Handling

| Error | Message | Exit Code |
|-------|---------|-----------|
| Empty query | "Error: Query cannot be empty" | 1 |
| Missing collection | "Error: Collection 'X' not found. Run 'python main.py ingest' first." | 1 |
| Embedding failure | "Error: Failed to embed query: {details}" | 1 |
| Connection failure | "Error: Cannot connect to Qdrant: {details}" | 1 |
| No results | (Not an error - return empty results with message) | 0 |

---

## Test Harness

Predefined test cases:

```python
TEST_CASES = [
    {
        "query": "ROS 2 robot operating system",
        "expect_url_contains": "module-1",
        "min_score": 0.4,
    },
    {
        "query": "Gazebo simulation environment",
        "expect_url_contains": "module-2",
        "min_score": 0.4,
    },
    {
        "query": "Isaac Sim NVIDIA",
        "expect_url_contains": "module-3",
        "min_score": 0.4,
    },
]
```

Test execution:
1. Run each query
2. Check top result matches expected URL pattern
3. Check top result score >= minimum
4. Report pass/fail for each
5. Return overall success/failure

---

## Complexity Tracking

No complexity violations - single file design maintained.

## Success Verification

After implementation:

1. **Basic Query**: `python main.py query "What is ROS 2?"` returns relevant results
2. **Filter**: `--url-filter` correctly narrows results
3. **Debug**: `--debug` shows embedding time, collection stats
4. **JSON**: `--json` outputs valid JSON
5. **Tests**: `python main.py test` passes all cases
6. **Errors**: Invalid inputs produce clear messages

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Model mismatch | Use exact same model constant as ingestion |
| Empty collection | Validate collection before search |
| Slow queries | Qdrant HNSW handles 10k+ vectors efficiently |
| API rate limits | Single query, no batching needed |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Add dataclasses to main.py
3. Implement search methods
4. Add CLI argument parsing
5. Implement test harness
6. Test end-to-end
