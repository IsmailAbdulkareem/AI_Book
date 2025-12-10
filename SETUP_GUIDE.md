# 🚀 AI-Native Skills Architecture Setup Guide

Complete step-by-step guide to get the AI chatbot pipeline running.

## Prerequisites

- Python 3.8+
- Docker & Docker Compose (for Qdrant)
- Node.js 22+ (already installed for Docusaurus)
- Git

## Step 1: Environment Setup

### 1.1 Create and activate virtual environment

```bash
# Windows PowerShell
python -m venv venv
.\venv\Scripts\Activate.ps1

# macOS/Linux
python3 -m venv venv
source venv/bin/activate
```

### 1.2 Install Python dependencies

```bash
pip install -r requirements.txt
```

This installs:
- **FastAPI & Uvicorn** – Backend framework
- **OpenAI** – LLM for chat responses
- **Cohere** – Embeddings for RAG
- **Qdrant** – Vector database client
- **Pydantic** – Request validation
- **Python-dotenv** – Environment variable management
- And ~15 other supporting packages

## Step 2: Configure Environment Variables

### 2.1 Create `.env` file

```bash
cp .env.example .env
```

Or manually create `.env` in project root:

```env
# LLM Configuration
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4
OPENAI_TEMPERATURE=0.7

# Embeddings
COHERE_API_KEY=...
COHERE_MODEL=embed-english-v3.0

# Vector Database
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=default_key
QDRANT_COLLECTION=docs

# Backend
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
DEBUG=true

# Docusaurus
DOCS_PATH=./docs
```

### 2.2 Get API Keys

**OpenAI:**
1. Visit https://platform.openai.com/api-keys
2. Create new API key
3. Copy to `OPENAI_API_KEY`

**Cohere:**
1. Visit https://dashboard.cohere.com/api-keys
2. Create new API key
3. Copy to `COHERE_API_KEY`

## Step 3: Start Qdrant Vector Database

### 3.1 Using Docker Compose (Recommended)

```bash
docker-compose -f docker-compose.qdrant.yml up -d
```

Verify it's running:
```bash
curl http://localhost:6333/health
```

Expected response:
```json
{
  "title": "Qdrant",
  "version": "0.x.x"
}
```

### 3.2 Alternative: Local Qdrant Binary

Download from: https://qdrant.tech/documentation/quick-start/

## Step 4: Ingest Documents

### 4.1 Run the complete workflow

```bash
python examples/setup_workflow.py
```

This will:
✅ Check Python environment
✅ Validate virtual environment
✅ Verify `.env` configuration
✅ Connect to Qdrant
✅ Ingest all documents from `docs/` folder
✅ Test chat with sample query
✅ Display available skills

### 4.2 Manual ingestion (if needed)

```bash
python -c "
from specify.skills.rag_ingestor import RagIngestor
ingestor = RagIngestor()
result = ingestor.ingest_pipeline('docs')
print(result)
"
```

## Step 5: Start the Backend API

### 5.1 Run FastAPI server

```bash
python -m backend.main
```

Or with auto-reload:

```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

### 5.2 Test the API

Visit http://localhost:8000/docs for interactive API documentation.

Test health check:
```bash
curl http://localhost:8000/health
```

Test chat endpoint:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

## Step 6: Start Docusaurus Documentation Site

```bash
npm start
```

The site will open at http://localhost:3000

## Step 7: Integrate ChatWidget (Optional)

To add the AI chatbot to Docusaurus homepage:

```bash
# Copy ChatWidget to Docusaurus components
cp src/components/ChatWidget.js src/components/

# Update src/pages/index.js to include ChatWidget
# See examples/chat-widget-integration.md
```

## Step 8: Full System Test

All three services running:
1. **Backend API** – http://localhost:8000
2. **Docusaurus Site** – http://localhost:3000
3. **Qdrant Database** – http://localhost:6333

### Test the complete flow:

```bash
# In PowerShell
$response = Invoke-WebRequest -Uri "http://localhost:8000/chat" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"query":"What is Physical AI?"}'
$response.Content | ConvertFrom-Json | Format-Table
```

## Troubleshooting

### ❌ `ModuleNotFoundError: No module named 'specify.skills'`

**Solution:**
```bash
# Ensure you're running from project root
cd d:\spec-driven-dev\ai-native

# Add project to PYTHONPATH
$env:PYTHONPATH = (Get-Location).Path + ";$env:PYTHONPATH"

# Verify structure
ls .specify/skills/
```

### ❌ Qdrant connection refused

**Solution:**
```bash
# Check if Docker is running
docker ps

# Start Qdrant container
docker-compose -f docker-compose.qdrant.yml up -d

# Verify running
curl http://localhost:6333/health
```

### ❌ `OpenAI API key not found`

**Solution:**
```bash
# Verify .env file exists
ls .env

# Check OPENAI_API_KEY is set
cat .env | Select-String "OPENAI_API_KEY"

# Test import
python -c "from dotenv import load_dotenv; load_dotenv(); import os; print(os.getenv('OPENAI_API_KEY'))"
```

### ❌ Slow document ingestion

**Solution:**
- Reduce `CHUNK_SIZE` in rag_ingestor.py (default: 800 tokens)
- Disable Cohere embeddings: use local embeddings instead
- Process documents in batches

### ❌ Chat returns empty response

**Solution:**
```bash
# Verify documents were ingested
curl http://localhost:8000/collections

# Check collection has data
curl http://localhost:8000/collections/docs

# Re-ingest documents
python examples/setup_workflow.py
```

## Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│           Frontend (Docusaurus + React)             │
│  Homepage → ChatWidget → /chat API (localhost:8000) │
└──────────────┬──────────────────────────────────────┘
               │ HTTP POST
               ↓
┌─────────────────────────────────────────────────────┐
│          Backend (FastAPI + Uvicorn)                │
│          ┌──────────────────────────┐               │
│          │   Chat Engine            │               │
│          │ (RAG + LLM Integration)  │               │
│          └──────────┬───────────────┘               │
│                     │                               │
│                     ↓                               │
│          ┌──────────────────────────┐               │
│          │ Vector DB Handler        │               │
│          │ (Similarity Search)      │               │
│          └──────────┬───────────────┘               │
└─────────────────────┼──────────────────────────────┘
                      │ (gRPC)
                      ↓
         ┌──────────────────────────┐
         │  Qdrant Vector DB        │
         │ (Document Embeddings)    │
         │ (localhost:6333)         │
         └──────────────────────────┘
```

## Skill Orchestration

The **SkillRegistry** enables orchestration of all 10 specialized skills:

```python
from specify.skills.registry import SkillRegistry

registry = SkillRegistry()

# Invoke any skill action
result = registry.invoke("chat_engine", "chat", query="What is ROS2?")
print(result)

# List available skills
skills = registry.list_skills()
for skill, actions in skills.items():
    print(f"{skill}: {actions}")
```

## Available Skills

1. **env_manager** – Environment setup, venv, .env validation
2. **docusaurus_builder** – Doc creation, sidebar management
3. **rag_ingestor** – Document processing, chunking, embedding
4. **vector_db_handler** – Qdrant CRUD and search operations
5. **chat_engine** – LLM queries with RAG context retrieval
6. **fastapi_builder** – Backend endpoint scaffolding
7. **cli_runner** – Terminal command execution
8. **chat_ui_builder** – React component generation
9. **debugger** – Error diagnosis and validation
10. **repo_manager** – Git operations and version control

## Next Steps

- [ ] Configure ChatWidget on homepage
- [ ] Add more LLM models (Claude, Llama)
- [ ] Implement user authentication
- [ ] Add conversation memory/history
- [ ] Deploy to production (Railway, Heroku, AWS)
- [ ] Set up CI/CD pipeline
- [ ] Monitor API performance

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review skill method docstrings: `python -c "from specify.skills.chat_engine import ChatEngine; help(ChatEngine.chat)"`
3. Check backend logs: tail -f backend_logs.txt
4. Review Qdrant docs: https://qdrant.tech/documentation/

## References

- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Qdrant Vector DB](https://qdrant.tech/documentation/)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [Cohere Embeddings](https://docs.cohere.com/docs/embeddings)
- [Docusaurus Docs](https://docusaurus.io/)
