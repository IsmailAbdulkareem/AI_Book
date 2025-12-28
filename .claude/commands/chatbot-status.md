# Chatbot Status Command

Quick status check for all chatbot system components.

## Usage
```
/chatbot-status
```

## Instructions

You are a System Status Agent. Your task is to provide a comprehensive status report.

### Execute All Checks

Run these checks in parallel for efficiency:

```bash
# 1. Backend API health
curl -s https://ai-book-h6kj.onrender.com/health

# 2. Frontend availability
curl -s -o /dev/null -w "%{http_code}" https://ismailabdulkareem.github.io/AI_Book/

# 3. Test RAG query
curl -s -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "test", "top_k": 1}' | head -c 200
```

### Vector Database Status

```bash
cd database
uv run python -c "
from dotenv import load_dotenv
load_dotenv()
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
try:
    info = client.get_collection('rag_embedding')
    print(f'Qdrant: OK ({info.points_count} vectors)')
except Exception as e:
    print(f'Qdrant: ERROR - {e}')
"
```

### Git Status

```bash
git status --short
git log -1 --oneline
```

### Generate Status Report

Format output as:

```
## RAG Chatbot System Status

| Component | Status | Details |
|-----------|--------|---------|
| Frontend | OK/ERROR | HTTP status, URL |
| Backend API | OK/ERROR | Health check result |
| Vector DB | OK/ERROR | Vector count |
| Git | CLEAN/DIRTY | Last commit |

**Last Updated**: [timestamp]
**Environment**: Production
```

### Alert Conditions

Flag as WARNING if:
- Backend response > 5 seconds (cold start)
- Vector count < 100
- Uncommitted changes exist

Flag as ERROR if:
- Any component returns error
- HTTP status not 200
- Health check fails
