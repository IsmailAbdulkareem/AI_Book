# 📚 Documentation Index

Complete guide to all documentation and resources in this project.

## 🚀 START HERE

### For First-Time Setup
👉 **[QUICKSTART.md](./QUICKSTART.md)** (10 minutes)
- Prerequisites check
- 7-step setup process
- Verification & testing
- First chat interaction
- Troubleshooting quick fixes

**→ This is your entry point**

---

## 📖 Complete Documentation

### Setup & Installation
- **[QUICKSTART.md](./QUICKSTART.md)** – 10-minute quick start
- **[SETUP_GUIDE.md](./SETUP_GUIDE.md)** – Comprehensive 8-step setup guide with troubleshooting

### API & Backend
- **[backend/README.md](./backend/README.md)** – Complete API reference with examples
- **[backend/main.py](./backend/main.py)** – FastAPI application source code

### Project Overview
- **[SYSTEM_SUMMARY.md](./SYSTEM_SUMMARY.md)** – Complete system overview and capabilities
- **[README.md](./README.md)** – Project root readme

### Agent & Contributor Guidance
- **[.github/copilot-instructions.md](./.github/copilot-instructions.md)** – Guidelines for AI agents
- **[CLAUDE.md](./CLAUDE.md)** – Agent conventions and workflows

### Examples & Workflows
- **[examples/setup_workflow.py](./examples/setup_workflow.py)** – Complete workflow demonstration
- **[.env.example](./.env.example)** – Environment configuration template

### Configuration & Infrastructure
- **[docker-compose.qdrant.yml](./docker-compose.qdrant.yml)** – Qdrant vector database setup
- **[requirements.txt](./requirements.txt)** – Python dependencies
- **[docusaurus.config.js](./docusaurus.config.js)** – Site configuration
- **[sidebars.js](./sidebars.js)** – Documentation navigation

---

## 🎯 Find What You Need

### "I want to..."

#### Get the system running quickly
→ **[QUICKSTART.md](./QUICKSTART.md)** (10 minutes)

#### Understand the complete setup process
→ **[SETUP_GUIDE.md](./SETUP_GUIDE.md)** (30 minutes)

#### Learn the API endpoints
→ **[backend/README.md](./backend/README.md)** (API reference)

#### Understand the system architecture
→ **[SYSTEM_SUMMARY.md](./SYSTEM_SUMMARY.md)** (Complete overview)

#### Deploy to production
→ **[SETUP_GUIDE.md#deployment](./SETUP_GUIDE.md)** (Deployment section)

#### Troubleshoot issues
→ **[SETUP_GUIDE.md#troubleshooting](./SETUP_GUIDE.md)** (Troubleshooting section)

#### Use the AI skills
→ Check **[.specify/skills/](./specify/skills/)** docstrings
```bash
python -c "from specify.skills.chat_engine import ChatEngine; help(ChatEngine.chat)"
```

#### Write an agent script
→ **[.github/copilot-instructions.md](./.github/copilot-instructions.md)** (Agent guidance)

#### View example code
→ **[examples/setup_workflow.py](./examples/setup_workflow.py)** (Complete workflow demo)

#### Configure environment variables
→ **[.env.example](./.env.example)** (Configuration template)

---

## 📁 Directory Structure

```
project-root/
├── 📚 QUICKSTART.md              ← Start here (10 min)
├── 📚 SETUP_GUIDE.md             ← Complete setup guide
├── 📚 SYSTEM_SUMMARY.md          ← System overview
├── 📚 README.md                  ← Project readme
│
├── 🔧 backend/
│   ├── main.py                   (FastAPI application)
│   └── 📚 README.md              (API reference)
│
├── 📝 .specify/skills/
│   ├── chat_engine.py            (LLM + RAG)
│   ├── vector_db_handler.py      (Qdrant operations)
│   ├── rag_ingestor.py           (Document processing)
│   ├── registry.py               (Skill orchestration)
│   └── [7 more skills...]
│
├── 💻 examples/
│   └── setup_workflow.py         (Complete demo)
│
├── 🐳 docker-compose.qdrant.yml  (Vector DB setup)
├── .env.example                  (Config template)
├── requirements.txt              (Python packages)
│
├── 🤖 .github/
│   └── 📚 copilot-instructions.md (Agent guide)
│
├── 📚 CLAUDE.md                  (Agent conventions)
│
├── 📖 docs/
│   ├── intro.md
│   ├── glossary.md
│   ├── module1/
│   ├── module2/
│   ├── module3/
│   └── module4/
│
├── 🎨 src/
│   ├── components/
│   ├── css/
│   └── pages/
│
└── history/prompts/
    └── 📝 PHRs (Prompt History Records)
```

---

## 🎓 Reading Paths

### Path 1: Complete Beginner (No Setup Experience)
1. [QUICKSTART.md](./QUICKSTART.md) – 10 minutes
2. [SETUP_GUIDE.md](./SETUP_GUIDE.md) – 30 minutes (detailed)
3. [backend/README.md](./backend/README.md) – API reference
4. Run examples and experiment!

**Time:** ~1 hour
**Result:** System running and API tested

### Path 2: I Know Python & APIs
1. [QUICKSTART.md](./QUICKSTART.md) – 5 minutes (skim)
2. [backend/README.md](./backend/README.md) – API reference
3. [SYSTEM_SUMMARY.md](./SYSTEM_SUMMARY.md) – Architecture overview
4. Explore skills in `.specify/skills/`

**Time:** ~30 minutes
**Result:** Understanding complete system architecture

### Path 3: Deployment Ready
1. [SETUP_GUIDE.md#deployment](./SETUP_GUIDE.md) – Deployment section
2. [docker-compose.qdrant.yml](./docker-compose.qdrant.yml) – Infrastructure
3. [.env.example](./.env.example) – Configuration
4. Choose cloud platform and deploy

**Time:** ~2 hours
**Result:** System deployed to production

### Path 4: Agent/Contributor
1. [.github/copilot-instructions.md](./.github/copilot-instructions.md) – Agent conventions
2. [CLAUDE.md](./CLAUDE.md) – Workflow guidelines
3. [examples/setup_workflow.py](./examples/setup_workflow.py) – Code patterns
4. Explore skills in `.specify/skills/`

**Time:** ~1 hour
**Result:** Ready to extend system

---

## ⚡ Quick Commands

### Setup (First Time)
```bash
# 1. Virtual environment
python -m venv venv
.\venv\Scripts\Activate.ps1

# 2. Dependencies
pip install -r requirements.txt

# 3. Config
cp .env.example .env
# Edit .env with API keys

# 4. Qdrant (Terminal 1)
docker-compose -f docker-compose.qdrant.yml up -d

# 5. Setup (Terminal 2)
python examples/setup_workflow.py

# 6. Backend (Terminal 3)
python -m backend.main

# 7. Frontend (Terminal 4)
npm start
```

### View Documentation
```bash
# Open in browser
start http://localhost:3000          # Frontend
start http://localhost:8000/docs     # API Docs
start http://localhost:6333/health   # Qdrant

# Or text editor
code QUICKSTART.md
code SETUP_GUIDE.md
code backend/README.md
code SYSTEM_SUMMARY.md
```

### Test API
```bash
# Health check
curl http://localhost:8000/health

# Chat query
curl -X POST http://localhost:8000/chat `
  -H "Content-Type: application/json" `
  -d '{"query":"What is ROS2?"}'

# Collections
curl http://localhost:8000/collections
```

### Check Skills
```bash
python -c "from specify.skills.registry import SkillRegistry; r = SkillRegistry(); print(r.list_skills())"
```

---

## 📊 Documentation Overview

| Document | Purpose | Read Time | Audience |
|----------|---------|-----------|----------|
| QUICKSTART.md | Fast setup | 10 min | Everyone |
| SETUP_GUIDE.md | Complete guide | 30 min | Users |
| SYSTEM_SUMMARY.md | Architecture | 20 min | Developers |
| backend/README.md | API reference | 30 min | Developers/Users |
| .github/copilot-instructions.md | Agent guide | 10 min | AI Agents |
| CLAUDE.md | Conventions | 5 min | AI Agents |
| examples/setup_workflow.py | Code example | 20 min | Developers |
| .env.example | Configuration | 5 min | Users |

**Total:** ~2 hours to read all documentation

---

## ✅ Using This Index

### If you're stuck:
1. Check "Find What You Need" section above
2. Find relevant document
3. Read the section that matches your problem
4. If still stuck, check Troubleshooting sections

### If you're new:
1. Start with QUICKSTART.md
2. Follow step-by-step instructions
3. When done, explore SYSTEM_SUMMARY.md
4. Then read API reference as needed

### If you're a developer:
1. Read SYSTEM_SUMMARY.md (architecture)
2. Check backend/README.md (API)
3. Explore `.specify/skills/` (skill code)
4. Use examples/setup_workflow.py (patterns)

### If you're an agent:
1. Read .github/copilot-instructions.md (guidance)
2. Check CLAUDE.md (conventions)
3. Follow project patterns
4. Create PHRs for all work

---

## 🔗 Navigation

**Quick Links:**
- 🚀 [Get Started in 10 Minutes](./QUICKSTART.md)
- 📚 [Complete Setup Guide](./SETUP_GUIDE.md)
- 🏗️ [System Architecture](./SYSTEM_SUMMARY.md)
- 🔌 [API Reference](./backend/README.md)
- 🤖 [Agent Guidelines](./.github/copilot-instructions.md)
- 💻 [Example Code](./examples/setup_workflow.py)

**Resources:**
- 📁 [Skills Directory](./.specify/skills/)
- 🐳 [Docker Setup](./docker-compose.qdrant.yml)
- ⚙️ [Configuration](./.env.example)
- 📖 [Project README](./README.md)

---

## 📞 Help & Support

### Before asking for help:
1. Check [SETUP_GUIDE.md#troubleshooting](./SETUP_GUIDE.md)
2. Check [backend/README.md#troubleshooting](./backend/README.md)
3. Read relevant documentation above
4. Check Python docstrings: `help(SkillClass.method)`

### Common issues:
- **Setup problems** → See [SETUP_GUIDE.md](./SETUP_GUIDE.md)
- **API issues** → See [backend/README.md](./backend/README.md)
- **Architecture questions** → See [SYSTEM_SUMMARY.md](./SYSTEM_SUMMARY.md)
- **Configuration** → See [.env.example](./.env.example)

---

## 📈 Next Steps

After reading this documentation:

1. ✅ Follow QUICKSTART.md (10 minutes)
2. ✅ Test API endpoints
3. ✅ Chat with AI
4. ✅ Explore skills
5. ✅ Deploy to cloud
6. ✅ Monitor in production

**Expected time to full deployment:** ~2 hours

---

**This documentation is current as of:** 2024  
**Last Updated:** Complete system integration  
**Status:** ✅ Ready for use  

**Start here:** [QUICKSTART.md](./QUICKSTART.md) → 10 minutes to running system
