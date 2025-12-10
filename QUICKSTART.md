# Quick Start Guide: AI-Native Skills Architecture

**Complete system running in 10 minutes**

## 🎯 What You're Setting Up

A full RAG (Retrieval-Augmented Generation) chatbot system with:
- **Frontend:** Docusaurus documentation site with ChatWidget
- **Backend:** FastAPI server with RAG capabilities
- **Database:** Qdrant vector database for semantic search
- **Skills:** 10 specialized AI modules orchestrated via SkillRegistry

## ⚡ Prerequisites (2 min)

```bash
# Check Python version (3.8+)
python --version

# Check Node.js (22+)
node --version

# Check Docker (for Qdrant)
docker --version

# Check Git
git --version
```

## 🚀 Setup (8 min)

### Step 1: Virtual Environment (1 min)

```bash
# PowerShell
python -m venv venv
.\venv\Scripts\Activate.ps1

# macOS/Linux
python -m venv venv
source venv/bin/activate
```

### Step 2: Install Dependencies (2 min)

```bash
pip install -r requirements.txt
```

Wait for `Successfully installed` message.

### Step 3: Configure API Keys (2 min)

```bash
# Copy template
copy .env.example .env

# Open in editor and add:
# 1. OPENAI_API_KEY from https://platform.openai.com/api-keys
# 2. COHERE_API_KEY from https://dashboard.cohere.com/api-keys
```

**Don't have keys?**
- OpenAI: Sign up free, get $5 credit
- Cohere: Free tier includes embeddings

### Step 4: Start Qdrant (1 min)

```bash
# Terminal 1: Start vector database
docker-compose -f docker-compose.qdrant.yml up -d

# Verify
curl http://localhost:6333/health
```

### Step 5: Ingest Documents (1 min)

```bash
# Terminal 2: Process all docs
python examples/setup_workflow.py
```

Wait for: `✅ All systems initialized!`

### Step 6: Start Backend (1 min)

```bash
# Terminal 3: Start FastAPI server
python -m backend.main
```

Wait for: `Application startup complete`

### Step 7: Start Frontend (1 min)

```bash
# Terminal 4: Start Docusaurus
npm start
```

Wait for: `Local: http://localhost:3000`

## ✅ Verification (1 min)

Open browser and test:

1. **Frontend:** http://localhost:3000
   - See documentation site with 4 module cards
   - Homepage loads without errors

2. **Backend API Docs:** http://localhost:8000/docs
   - See Swagger UI with all endpoints
   - Green "200" responses

3. **Test Chat Query:**
   ```bash
   curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"query":"What is ROS2?"}'
   ```
   - Should return AI response with sources

4. **Vector DB:** http://localhost:6333/health
   - Should return Qdrant version info

## 📦 What's Running

| Service | URL | Terminal |
|---------|-----|----------|
| Qdrant (Vector DB) | http://localhost:6333 | 1 |
| FastAPI (Backend) | http://localhost:8000 | 3 |
| Docusaurus (Frontend) | http://localhost:3000 | 4 |

## 🎮 First Chat Interaction

### Via Browser (if ChatWidget enabled)
1. Open http://localhost:3000
2. Look for chat widget
3. Type: "What is Physical AI?"
4. Get AI response with sources

### Via API (always works)
```bash
# PowerShell
$response = Invoke-WebRequest -Uri "http://localhost:8000/chat" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"query":"What is Gazebo?"}'

$response.Content | ConvertFrom-Json | Select-Object response,sources
```

### Via Python
```python
import requests

response = requests.post(
    "http://localhost:8000/chat",
    json={"query": "What is URDF?"}
)
print(response.json()["response"])
```

## 📚 Available Skills

All 10 skills ready to orchestrate:

```python
from specify.skills.registry import SkillRegistry

registry = SkillRegistry()

# List all
skills = registry.list_skills()
for skill_name, methods in skills.items():
    print(f"{skill_name}: {len(methods)} methods")

# Invoke any skill
result = registry.invoke("chat_engine", "chat", query="What is ROS2?")
print(result)
```

**The 10 Skills:**
1. `env_manager` – Python env management
2. `docusaurus_builder` – Doc site operations
3. `rag_ingestor` – Document processing
4. `vector_db_handler` – Qdrant operations
5. `chat_engine` – LLM with RAG
6. `fastapi_builder` – Backend scaffolding
7. `cli_runner` – Terminal commands
8. `chat_ui_builder` – React components
9. `debugger` – Error diagnosis
10. `repo_manager` – Git operations

## 🛑 Stop Everything

```bash
# Terminal 1: Stop Qdrant
docker-compose -f docker-compose.qdrant.yml down

# Terminal 2-4: Press Ctrl+C
```

## 🔄 Restart in Future

```bash
# Terminal 1: Qdrant
docker-compose -f docker-compose.qdrant.yml up -d

# Terminal 2: Activate venv + Backend
.\venv\Scripts\Activate.ps1
python -m backend.main

# Terminal 3: Frontend
npm start
```

## ❌ Troubleshooting

### "ModuleNotFoundError: No module named 'specify.skills'"
```bash
# Ensure you're in project root
cd d:\spec-driven-dev\ai-native

# Verify venv is activated (see (venv) in prompt)
.\venv\Scripts\Activate.ps1

# Reinstall
pip install -r requirements.txt
```

### "Qdrant connection refused"
```bash
# Start Docker (if not running)
docker-compose -f docker-compose.qdrant.yml up -d

# Verify
docker ps | Select-String qdrant
curl http://localhost:6333/health
```

### "OpenAI API key not valid"
```bash
# Verify .env file exists
test-path .env

# Check key is set
cat .env | Select-String OPENAI_API_KEY

# Get valid key from https://platform.openai.com/api-keys
```

### "No documents in collection"
```bash
# Re-ingest
python examples/setup_workflow.py

# Or manually
python -c "from specify.skills.rag_ingestor import RagIngestor; r = RagIngestor(); print(r.ingest_pipeline('docs'))"

# Check
curl http://localhost:8000/collections/docs
```

### "Chat returns empty response"
```bash
# Verify Cohere API key
cat .env | Select-String COHERE_API_KEY

# Check embeddings model
python -c "import cohere; c = cohere.Client(); print(c.embed(model='embed-english-v3.0', texts=['test']))"

# Re-ingest with embeddings
python examples/setup_workflow.py
```

## 📖 Next Steps

1. **Read Documentation:** http://localhost:3000
2. **Explore API:** http://localhost:8000/docs
3. **Add ChatWidget:** See `src/components/ChatWidget.js`
4. **Deploy:** See `SETUP_GUIDE.md` deployment section
5. **Customize:** Modify skills in `.specify/skills/`
6. **Monitor:** Check logs in `backend/logs/`

## 🎓 Learning Paths

### Path 1: Customize Chatbot
1. Modify chat engine prompts in `specify/skills/chat_engine.py`
2. Change LLM model in `.env`: `OPENAI_MODEL=gpt-4-turbo`
3. Adjust response temperature: `OPENAI_TEMPERATURE=0.5`
4. Test via API docs

### Path 2: Add More Documents
1. Create new `.md` files in `docs/` folder
2. Re-run: `python examples/setup_workflow.py`
3. Chat queries automatically use new docs

### Path 3: Extend Skills
1. Create new skill in `specify/skills/new_skill.py`
2. Add to `registry.py` SkillRegistry
3. Invoke: `registry.invoke("new_skill", "method_name")`

### Path 4: Deploy to Cloud
1. Follow `SETUP_GUIDE.md` Deployment section
2. Choose platform: Railway, Heroku, AWS, etc.
3. Set environment variables on platform
4. Deploy with one command

## 📚 Architecture Overview

```
User Browser                    Your Computer                Qdrant Server
     |                               |                            |
     |-- http://localhost:3000       |                            |
     |       (Docusaurus)            |                            |
     |                               |                            |
     |-- POST /chat -------> FastAPI Backend                      |
     |       (localhost:8000)        |                            |
     |                          Chat Engine                       |
     |                          (RAG + LLM)                       |
     |                               |                            |
     |                               |--- Similarity Search ---> Vector DB
     |                               |    (localhost:6333)         |
     |                               |                            |
     |<-- Response JSON -------------|<-- Top K Results ----------|
     |
```

## 🔐 Security Notes

⚠️ **For Development Only:**
- `.env` contains API keys (never commit!)
- CORS allows all origins
- Debug mode enabled
- Qdrant has no authentication

✅ **For Production:**
- Use secrets manager (AWS Secrets Manager, Heroku Secrets)
- Restrict CORS origins to your domain
- Set `DEBUG=false`
- Enable Qdrant authentication
- Use HTTPS
- Add request rate limiting

## 💡 Pro Tips

1. **Speed up ingestion:** Reduce `CHUNK_SIZE` in `.env`
2. **Better responses:** Use `gpt-4` instead of `gpt-3.5-turbo`
3. **Faster responses:** Use streaming endpoint `/chat/stream`
4. **Monitor performance:** Use `/status` endpoint
5. **Reset data:** `docker-compose down && docker volume prune`
6. **View logs:** `uvicorn backend.main:app --log-level debug`
7. **Interactive testing:** Visit http://localhost:8000/docs
8. **Hot reload:** Use `--reload` flag with uvicorn

## ❓ Questions?

- **Setup issues?** → See SETUP_GUIDE.md
- **API docs?** → See backend/README.md
- **Skill usage?** → Check docstrings: `python -c "from specify.skills.chat_engine import ChatEngine; help(ChatEngine.chat)"`
- **Copilot instructions?** → See .github/copilot-instructions.md

## 🎉 Congratulations!

Your AI-native skills architecture is running. You now have:
- ✅ Full-stack RAG chatbot
- ✅ 10 orchestrated AI skills
- ✅ Production-ready backend
- ✅ Documentation website
- ✅ Vector semantic search

**Next:** Add ChatWidget to homepage, customize responses, deploy to cloud!

---

**Time to completion:** ~10 minutes  
**System ready:** ✅ Yes  
**Chat working:** ✅ Yes  
**Skills accessible:** ✅ Yes
