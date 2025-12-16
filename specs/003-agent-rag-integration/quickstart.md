# Quickstart: RAG Agent API

**Feature**: 003-agent-rag-integration
**Date**: 2025-12-15
**Prerequisite**: Specs 1 and 2 must be complete (content ingested into Qdrant)

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Content already ingested into Qdrant (run Spec 1 pipeline)
- Valid API credentials in `.env` file

## Environment Setup

```bash
cd database

# Check .env file has all required keys
cat .env

# Should contain:
# COHERE_API_KEY=your_key
# QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
# QDRANT_API_KEY=your_key
# OPENAI_API_KEY=your_key  # NEW for Spec 3
```

## Install Dependencies

```bash
cd database

# Add new dependencies
uv add fastapi uvicorn openai

# Sync environment
uv sync
```

## Start the Server

```bash
cd database

# Start the RAG agent API
uv run uvicorn agent:app --reload --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345]
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

## Basic Usage

### 1. Health Check

```bash
curl http://localhost:8000/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "dependencies": {
    "qdrant": {
      "status": "connected",
      "details": {
        "collection": "rag_embedding",
        "vectors": 135
      }
    },
    "openai": {
      "status": "configured",
      "details": null
    }
  }
}
```

### 2. Ask a Question

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected Response**:
```json
{
  "question": "What is ROS 2?",
  "answer": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software [1]. It is middleware specifically designed for robotics, providing tools, libraries, and conventions for building robot applications [2]. ROS 2 is NOT an operating system - it runs on Linux, Windows, and macOS. It's also not a programming language, as it works with Python, C++, Java, and other languages [1].",
  "sources": [
    {
      "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module1/module1-chapter2",
      "content": "ROS 2 is middleware specifically designed for robotics...",
      "score": 0.66,
      "position": 1
    },
    {
      "url": "https://ismailabdulkareem.github.io/AI_Book/docs/module1/module1-chapter2",
      "content": "Module 1: The Robotic Nervous System (ROS 2)...",
      "score": 0.64,
      "position": 0
    }
  ],
  "retrieval_time_ms": 1043,
  "generation_time_ms": 2150
}
```

### 3. Ask with Custom top_k

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "How does Gazebo simulation work?", "top_k": 3}'
```

### 4. Ask About Isaac Sim

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Isaac Sim and how is it used for robotics?"}'
```

## API Reference

### POST /ask

Ask a question and receive a grounded answer with sources.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `question` | string | required | Natural language question |
| `top_k` | integer | 5 | Number of context chunks (1-20) |

### GET /health

Check service health and dependency status.

Returns:
- `healthy`: All systems operational
- `degraded`: Some dependencies unavailable
- `unhealthy`: Critical failure

## Interactive API Docs

FastAPI provides automatic interactive documentation:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Troubleshooting

### "Collection not found" Error

**Cause**: Content hasn't been ingested yet

**Solution**:
```bash
# Run the ingestion pipeline first (Spec 1)
cd database
uv run python main.py
```

### "OpenAI API key not configured" Error

**Cause**: Missing OPENAI_API_KEY in environment

**Solution**:
1. Get an API key from https://platform.openai.com/api-keys
2. Add to `.env`:
   ```
   OPENAI_API_KEY=sk-...
   ```

### "Cannot connect to Qdrant" Error

**Cause**: Network or credential issue

**Solution**:
1. Check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Verify Qdrant cluster is running
3. Check network connectivity

### Empty or Low-Quality Answers

**Cause**: Insufficient context or off-topic question

**Solutions**:
1. Try increasing `top_k` for more context
2. Rephrase question to match book terminology
3. Check if topic is covered in the book
4. Use `/health` to verify vector count

### Slow Response Times

**Cause**: Network latency or large context

**Solutions**:
1. Reduce `top_k` for faster responses
2. Check network connectivity to Qdrant and OpenAI
3. Consider using a different OpenAI model

## Python Client Example

```python
import httpx

def ask_question(question: str, top_k: int = 5) -> dict:
    """Ask a question to the RAG agent."""
    response = httpx.post(
        "http://localhost:8000/ask",
        json={"question": question, "top_k": top_k},
        timeout=30.0,
    )
    response.raise_for_status()
    return response.json()

# Usage
result = ask_question("What is ROS 2?")
print(f"Answer: {result['answer']}")
print(f"Sources: {len(result['sources'])} citations")
for i, source in enumerate(result['sources'], 1):
    print(f"  [{i}] {source['url']} (score: {source['score']:.2f})")
```

## Next Steps

After verifying the agent works:

1. **Frontend Integration**: Connect to a chat UI (Spec 4)
2. **Production Deployment**: Add authentication, rate limiting
3. **Performance Tuning**: Optimize top_k and model selection
4. **Monitoring**: Add logging and metrics collection
