# Quickstart: RAG Retrieval & Pipeline Validation

**Feature**: 002-rag-retrieval
**Date**: 2025-12-15
**Prerequisite**: Embedding pipeline (Spec 1) must have ingested content

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Content already ingested into Qdrant (run `python main.py` from Spec 1)
- Valid API credentials in `.env` file

## Verify Prerequisites

```bash
cd database

# Check .env file exists with credentials
cat .env

# Should contain:
# COHERE_API_KEY=your_key
# QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
# QDRANT_API_KEY=your_key
```

## Basic Usage

### 1. Simple Query

```bash
cd database
uv run python main.py query "What is ROS 2?"
```

**Expected Output**:
```
Query: "What is ROS 2?"
Collection: rag_embedding
Results: 5 matches (420ms)

[1] Score: 0.89
    URL: https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-2
    Position: 2
    Content: ROS 2 (Robot Operating System 2) is a flexible framework...

[2] Score: 0.84
    URL: https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-1
    Position: 0
    Content: Introduction to Physical AI and robotics fundamentals...
```

### 2. Limit Results

```bash
uv run python main.py query "robotics" --top-k 3
```

### 3. Filter by URL

```bash
# Only search within module-2 content
uv run python main.py query "simulation" --url-filter "/module-2/"
```

### 4. Debug Mode

```bash
uv run python main.py query "sensors" --debug
```

**Debug Output**:
```
[DEBUG] Embedding query: "sensors"
[DEBUG] Query vector: 1024 dimensions (152ms)
[DEBUG] Collection 'rag_embedding': 523 vectors
[DEBUG] Searching with top_k=5, filter=None
[DEBUG] Search completed in 268ms
[DEBUG] Score distribution: min=0.42, max=0.89, avg=0.68

Query: "sensors"
...
```

### 5. JSON Output (for LLM Integration)

```bash
uv run python main.py query "navigation" --json
```

**JSON Output**:
```json
{
  "query": "navigation",
  "collection": "rag_embedding",
  "results": [
    {
      "score": 0.85,
      "content": "...",
      "url": "...",
      "position": 1,
      "created_at": 1734278400.0
    }
  ],
  "total": 5,
  "time_ms": 380
}
```

### 6. Run Test Harness

```bash
uv run python main.py test
```

**Expected Output**:
```
Running RAG Retrieval Tests...

[1/3] Query: "ROS 2 robot operating system"
      Expected URL contains: module-1
      Result: PASS (score=0.87, url matches)

[2/3] Query: "Gazebo simulation environment"
      Expected URL contains: module-2
      Result: PASS (score=0.82, url matches)

[3/3] Query: "Isaac Sim NVIDIA"
      Expected URL contains: module-3
      Result: PASS (score=0.79, url matches)

Test Summary: 3/3 PASSED
```

## CLI Reference

```bash
python main.py query <query> [options]
python main.py test
python main.py ingest  # (existing ingestion command)
```

### Query Options

| Option | Short | Default | Description |
|--------|-------|---------|-------------|
| `--top-k` | `-k` | 5 | Number of results to return |
| `--url-filter` | `-u` | None | Filter results by URL substring |
| `--debug` | `-d` | False | Show detailed diagnostics |
| `--json` | `-j` | False | Output as JSON |
| `--collection` | `-c` | rag_embedding | Target collection name |

## Troubleshooting

### "Collection not found" Error

**Cause**: Content hasn't been ingested yet

**Solution**:
```bash
# Run the ingestion pipeline first
uv run python main.py ingest
```

### "Failed to embed query" Error

**Cause**: Cohere API issue or invalid key

**Solution**:
1. Check `COHERE_API_KEY` in `.env`
2. Verify API key is valid at https://dashboard.cohere.com
3. Check for rate limiting (trial keys have limits)

### "Cannot connect to Qdrant" Error

**Cause**: Network or credential issue

**Solution**:
1. Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Verify Qdrant cluster is running
3. Check network connectivity

### No Results Returned

**Cause**: Query may not match indexed content

**Solution**:
1. Try more general queries
2. Use `--debug` to see collection stats
3. Verify content was indexed: check Qdrant dashboard

### Low Relevance Scores

**Cause**: Query phrasing differs significantly from content

**Solution**:
1. Rephrase query to match documentation terminology
2. Use `--debug` to see score distribution
3. Try different query formulations

## Integration with LLMs

For programmatic use with LLM agents:

```python
import subprocess
import json

def retrieve_context(query: str, top_k: int = 5) -> list:
    """Retrieve relevant context for RAG."""
    result = subprocess.run(
        ["uv", "run", "python", "main.py", "query", query,
         "--top-k", str(top_k), "--json"],
        capture_output=True,
        text=True,
        cwd="database"
    )

    if result.returncode != 0:
        raise RuntimeError(result.stderr)

    response = json.loads(result.stdout)
    return response["results"]

# Usage
contexts = retrieve_context("What is ROS 2?", top_k=3)
for ctx in contexts:
    print(f"[{ctx['score']:.2f}] {ctx['content'][:100]}...")
```

## Next Steps

After verifying retrieval works:

1. Integrate with your LLM/Agent system
2. Fine-tune `top_k` based on your use case
3. Add URL filters to scope searches
4. Set up periodic re-ingestion if content changes
