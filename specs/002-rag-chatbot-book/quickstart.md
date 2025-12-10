# Quickstart Guide: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

**Feature**: 002-rag-chatbot-book
**Created**: 2025-12-10
**Status**: Draft

## Overview

This guide provides a quick setup and usage guide for the RAG Chatbot system integrated with the Physical AI & Humanoid Robotics book.

## Prerequisites

- Python 3.10 or higher
- pip package manager
- Git for version control
- Access to required API keys (Qdrant, Cohere, OpenAI, Neon Postgres)

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install fastapi uvicorn[standard] qdrant-client cohere openai python-dotenv python-frontmatter pyyaml psycopg[binary]
```

### 4. Configure Environment Variables

Create a `.env` file in the project root:

```env
# Qdrant
QDRANT_URL=https://<your-qdrant-endpoint>.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=physical_ai_book

# Cohere
COHERE_API_KEY=your_cohere_api_key_here
COHERE_EMBED_MODEL=embed-english-v3.0

# OpenAI
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4o-mini

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:password@host/dbname
```

## Running the Application

### 1. Start the Backend API

```bash
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

### 2. Initialize Vector Store

Run the content ingestion script to populate Qdrant with book content:

```bash
python scripts/ingest_content.py
```

### 3. Test the API

Send a test request to the chat endpoint:

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '{
    "message": "What is physical AI?",
    "session_id": "test-session-123"
  }'
```

## API Usage Examples

### General Q&A

Ask a question about the book content:

```json
{
  "message": "Explain the key principles of humanoid robotics",
  "session_id": "session-uuid-here"
}
```

### Selected Text Q&A

Ask about specific selected text:

```json
{
  "message": "Can you elaborate on this?",
  "selected_text": "Humanoid robots are designed to mimic human form and behavior...",
  "page_url": "/docs/chapter-5/humanoid-design",
  "session_id": "session-uuid-here"
}
```

## Frontend Integration

### 1. Docusaurus Component

The chat widget is integrated as a React component in the Docusaurus theme. The component will automatically appear on all book pages.

### 2. Text Selection Feature

Users can select text on any page and click the chat icon to ask questions specifically about that selection.

## Content Management

### Adding New Content

To add new book content to the vector store:

1. Add the new content to the Docusaurus markdown files
2. Run the ingestion script: `python scripts/ingest_content.py`
3. The new content will be available for Q&A within seconds

### Updating Existing Content

When book content is updated:

1. Update the Docusaurus markdown files
2. Run the update script: `python scripts/update_content.py`
3. The vector store will be updated with the new content while preserving existing embeddings

## Monitoring and Analytics

### Accessing Chat Logs

Chat logs are stored in the Neon Postgres database. Access them via the admin interface or direct database query.

### Performance Metrics

Monitor API performance through the built-in FastAPI metrics endpoint at `/metrics`.

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Verify all API keys in `.env` file are correct and have proper permissions
2. **Vector Search Slow**: Check Qdrant Cloud plan and ensure collection is properly indexed
3. **Empty Responses**: Verify content has been ingested into the vector store

### Getting Help

- Check the logs at `logs/app.log`
- Verify API connectivity with: `python scripts/test_connections.py`
- Review the full documentation in the `/docs` directory