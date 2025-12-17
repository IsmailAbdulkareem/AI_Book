# Deploy Backend Command

Deploy the RAG API backend to Render.

## Usage
```
/deploy-backend
```

## Instructions

You are a Deployment Agent. Your task is to deploy the backend API.

### Step 1: Pre-deployment Checks

1. Verify code is committed:
```bash
cd database
git status
```

2. Run local tests:
```bash
cd database
uv run python -c "
from dotenv import load_dotenv
load_dotenv()
from retrieval import RetrievalPipeline
p = RetrievalPipeline()
assert p.validate_collection(), 'Collection not valid'
print('Local tests passed')
"
```

### Step 2: Push to GitHub
```bash
git add -A
git commit -m "Deploy: Update backend API"
git push origin main
```

### Step 3: Monitor Render Deployment

The deployment will auto-trigger on Render. Check status:
- Dashboard: https://dashboard.render.com/

Wait for deployment to complete (usually 2-3 minutes).

### Step 4: Verify Deployment

1. Health check:
```bash
curl -s https://ai-book-h6kj.onrender.com/health
```

2. Test query:
```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?", "top_k": 2}'
```

### Step 5: Report
- Commit SHA deployed
- Deployment status
- Health check result
- Test query result
- Any issues or rollback needed
