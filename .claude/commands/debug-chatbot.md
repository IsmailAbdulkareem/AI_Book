# Debug Chatbot Command

Diagnose and troubleshoot issues with the RAG chatbot system.

## Usage
```
/debug-chatbot [issue-description]
```

## Instructions

You are a Chatbot Debugging Agent. Your task is to systematically diagnose chatbot issues.

### Step 1: Health Checks

Run all health checks in parallel:

```bash
# Backend API health
curl -s https://ai-book-h6kj.onrender.com/health

# Frontend site status
curl -s -o /dev/null -w "%{http_code}" https://ismailabdulkareem.github.io/AI_Book/
```

### Step 2: Backend Diagnostics

1. Test API endpoint directly:
```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "top_k": 2}'
```

2. Check for specific errors:
   - 500 errors: Backend code issue
   - 503 errors: Service starting up (Render cold start)
   - CORS errors: Check allowed origins
   - Timeout: Render free tier spinning up

### Step 3: Vector Database Check

```bash
cd database
uv run python -c "
from dotenv import load_dotenv
load_dotenv()
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('rag_embedding')
print(f'Collection: rag_embedding')
print(f'Vectors: {info.points_count}')
print(f'Status: {info.status}')
"
```

### Step 4: Frontend Plugin Check

1. Verify plugin is registered in `docusaurus.config.js`
2. Check browser console for JavaScript errors
3. Verify chat icon renders (z-index: 9999)
4. Test in incognito mode (cache issues)

### Step 5: Common Issues & Fixes

| Issue | Likely Cause | Fix |
|-------|--------------|-----|
| No chat icon | Plugin not loaded | Check getThemePath() |
| Transparent background | CSS variables | Use solid colors |
| "Failed to get response" | Backend down | Check Render logs |
| Empty results | Collection issue | Re-run embedding pipeline |
| CORS error | Origin not allowed | Update app.py CORS |
| Mobile Ask AI missing | Touch events | Check useTextSelection.js |

### Step 6: Collect Diagnostics

Gather:
1. Browser console errors (screenshot or text)
2. Network tab failed requests
3. Render deployment logs
4. Exact error messages

### Step 7: Report

- **Status**: Issue identified / Needs investigation
- **Component**: Frontend / Backend / Database
- **Root Cause**: Description
- **Fix Applied**: What was done
- **Verification**: How it was tested
- **Follow-up**: Any additional recommendations
