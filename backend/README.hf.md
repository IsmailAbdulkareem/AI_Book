---
title: Physical AI RAG Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics - RAG Chatbot API

FastAPI backend for the Physical AI & Humanoid Robotics book chatbot.

## Features

- RAG (Retrieval-Augmented Generation) using Qdrant vector database
- OpenAI GPT-4 for response generation
- Cohere embeddings for document retrieval
- Intent detection (greetings, meta questions, content questions)
- Session management with PostgreSQL

## API Endpoints

- `GET /health` - Health check
- `POST /ask` - Ask a question about the book

## Environment Variables

Required secrets (set in Hugging Face Space settings):

```
OPENAI_API_KEY=your-openai-api-key
COHERE_API_KEY=your-cohere-api-key
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=AI-book
DATABASE_URL=your-postgresql-connection-string
```

## Usage

```bash
curl -X POST https://your-space.hf.space/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?", "session_id": "test-123"}'
```

## Local Development

```bash
pip install -r requirements.txt
python app.py
```

Visit http://localhost:8000/docs for API documentation.
