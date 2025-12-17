# Claude Code Subagents & Skills

This directory contains reusable intelligence components for the AI Book RAG Chatbot project. These enable Claude Code to perform specialized tasks efficiently.

## Overview

```
.claude/
├── commands/           # Slash commands (Subagents)
│   ├── rag-test.md        # Test RAG pipeline
│   ├── reindex.md         # Re-run embedding pipeline
│   ├── deploy-backend.md  # Deploy API to Render
│   ├── deploy-frontend.md # Deploy site to GitHub Pages
│   ├── add-docs.md        # Add new documentation
│   ├── debug-chatbot.md   # Troubleshoot issues
│   └── chatbot-status.md  # System status check
│
└── skills/             # Reusable knowledge (Skills)
    ├── rag-query.md           # RAG system knowledge
    ├── embedding-pipeline.md  # Embedding pipeline patterns
    └── docusaurus-plugin.md   # Plugin development patterns
```

## Slash Commands (Subagents)

Slash commands create specialized agents that execute specific workflows. Use them by typing the command in Claude Code.

### `/rag-test [question]`
Test the RAG pipeline end-to-end with a query.

**Example:**
```
/rag-test What is ROS 2?
```

**What it does:**
1. Health check on API
2. Execute test query
3. Analyze response quality
4. Report pass/fail status

---

### `/reindex`
Re-run the embedding pipeline to update vectors.

**When to use:**
- After adding new documentation
- After content updates
- When retrieval quality degrades

**What it does:**
1. Check prerequisites (API keys)
2. Run embedding pipeline
3. Verify new vector count
4. Test retrieval

---

### `/deploy-backend`
Deploy the FastAPI backend to Render.

**What it does:**
1. Pre-deployment checks
2. Git commit and push
3. Monitor Render deployment
4. Verify health endpoint
5. Test with sample query

---

### `/deploy-frontend`
Deploy Docusaurus site to GitHub Pages.

**What it does:**
1. Build verification
2. Git commit and push
3. Monitor GitHub Actions
4. Verify live site

---

### `/add-docs [topic]`
Add new documentation and update search index.

**What it does:**
1. Create markdown file
2. Update sidebar
3. Build and test
4. Deploy and reindex

---

### `/debug-chatbot [issue]`
Systematically diagnose chatbot issues.

**What it does:**
1. Run health checks
2. Test all components
3. Check common issues
4. Provide fix recommendations

---

### `/chatbot-status`
Quick status check of all system components.

**Output:**
```
| Component | Status | Details |
|-----------|--------|---------|
| Frontend  | OK     | HTTP 200 |
| Backend   | OK     | healthy  |
| Vector DB | OK     | 135 vectors |
```

---

## Skills

Skills are knowledge documents that inform Claude Code about project-specific patterns and practices.

### `rag-query.md`
Knowledge about the RAG (Retrieval-Augmented Generation) system:
- API endpoints and parameters
- Response format
- Error handling
- Performance characteristics

### `embedding-pipeline.md`
Knowledge about the embedding pipeline:
- URL discovery from sitemap
- Content extraction patterns
- Chunking strategy
- Cohere embedding model
- Qdrant vector storage

### `docusaurus-plugin.md`
Plugin development patterns:
- Plugin structure and registration
- Theme Root wrapper pattern
- BrowserOnly for SSR safety
- CSS Modules usage
- Common issues and fixes

---

## How It Works

### Slash Commands
When you type `/command-name`, Claude Code:
1. Reads the command file from `.claude/commands/`
2. Follows the instructions step-by-step
3. Uses appropriate tools (Bash, Read, Write, etc.)
4. Provides a structured report

### Skills
Skills are automatically loaded as context when relevant. They help Claude Code:
- Understand project architecture
- Follow established patterns
- Make informed decisions
- Avoid common mistakes

---

## Adding New Commands

Create a new file in `.claude/commands/`:

```markdown
# Command Name

Brief description.

## Usage
\`\`\`
/command-name [args]
\`\`\`

## Instructions

You are a [Role] Agent. Your task is to [goal].

### Step 1: [First Step]
[Instructions and commands]

### Step 2: [Second Step]
[Instructions and commands]

### Report
- [What to report]
```

---

## Adding New Skills

Create a new file in `.claude/skills/`:

```markdown
# Skill Name

Brief description of what this skill covers.

## Purpose
When and why to use this skill.

## Key Concepts
- Concept 1
- Concept 2

## Patterns
[Code examples and patterns]

## Troubleshooting
[Common issues and fixes]
```

---

## Project Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    GitHub Pages                          │
│              (Docusaurus Frontend)                       │
│         ismailabdulkareem.github.io/AI_Book              │
└─────────────────────┬───────────────────────────────────┘
                      │ HTTP POST /ask
                      ▼
┌─────────────────────────────────────────────────────────┐
│                   Render (Backend)                       │
│                FastAPI + RAG Pipeline                    │
│            ai-book-h6kj.onrender.com                     │
└─────────────────────┬───────────────────────────────────┘
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
┌─────────────────┐     ┌─────────────────┐
│  Qdrant Cloud   │     │   Cohere API    │
│  (Vector DB)    │     │  (Embeddings &  │
│ rag_embedding   │     │   Generation)   │
└─────────────────┘     └─────────────────┘
```

---

## Quick Reference

| Task | Command |
|------|---------|
| Test chatbot | `/rag-test` |
| Check status | `/chatbot-status` |
| Deploy backend | `/deploy-backend` |
| Deploy frontend | `/deploy-frontend` |
| Update vectors | `/reindex` |
| Fix issues | `/debug-chatbot` |
| Add docs | `/add-docs` |
