# RAG Query Skill

A reusable skill for querying the RAG (Retrieval-Augmented Generation) system to answer questions about the Physical AI & Humanoid Robotics book.

## Purpose
This skill enables Claude to query the book's vector database and generate grounded answers with citations.

## When to Use
- When user asks questions about ROS 2, robotics, simulation, or Physical AI
- When user wants to find specific information from the book
- When testing the RAG pipeline functionality

## API Endpoint
- **Production**: `https://ai-book-h6kj.onrender.com/ask`
- **Local**: `http://localhost:8000/ask`

## Request Format
```json
{
  "question": "string (required)",
  "context": "string (optional - selected text)",
  "top_k": 5
}
```

## Response Format
```json
{
  "question": "string",
  "answer": "string with [N] citations",
  "sources": [
    {"url": "string", "content": "string"}
  ],
  "retrieval_time_ms": number,
  "generation_time_ms": number
}
```

## Example Usage

### Query the RAG system:
```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "top_k": 3}'
```

### With context:
```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain this concept", "context": "Selected text from the book...", "top_k": 5}'
```

## Integration Points
- Frontend chatbot at `/docs/*` pages
- CLI chatbot at `database/chatbot.py`
- API server at `database/app.py`

## Error Handling
- 503: Vector database unavailable
- 500: Retrieval or generation failed
- 422: Invalid request format
