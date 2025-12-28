# AI-Native Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-10

## Active Technologies

- Backend Framework: FastAPI (Python)
- Vector Database: Qdrant Cloud
- Embeddings: Cohere API
- Language Model: OpenAI API
- Database: Neon Serverless Postgres
- Frontend Integration: Docusaurus React components
- Environment Management: python-dotenv

## Project Structure

```text
specs/002-rag-chatbot-book/
├── spec.md
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
└── contracts/
    └── chat-api.yaml
```

## Commands

### Python/Backend
- `uvicorn main:app --reload --port 8000` - Start the FastAPI development server
- `python scripts/ingest_content.py` - Ingest book content into vector store
- `python scripts/update_content.py` - Update existing content in vector store
- `python scripts/test_connections.py` - Test all API connections

### Environment
- `python -m venv .venv && source .venv/bin/activate` - Create and activate virtual environment
- `pip install fastapi uvicorn[standard] qdrant-client cohere openai python-dotenv python-frontmatter pyyaml psycopg[binary]` - Install dependencies

### API Testing
- `curl -X POST http://localhost:8000/api/chat` - Test chat endpoint

## Code Style

### Python
- Use snake_case for function and variable names
- Use PascalCase for class names
- Follow PEP 8 guidelines
- Use type hints for all function parameters and return values
- Use docstrings for all public functions and classes

### FastAPI
- Use Pydantic models for request/response validation
- Use dependency injection for common operations
- Use proper HTTP status codes
- Include comprehensive API documentation

### Database
- Use async database operations where possible
- Implement proper connection pooling
- Use parameterized queries to prevent injection attacks

## Recent Changes

- Feature 002-rag-chatbot-book: Added RAG chatbot system with Qdrant vector store, Cohere embeddings, OpenAI integration, and Neon Postgres for chat logs
- Feature 001-book-sitemap-generation: Added sitemap generation functionality for Docusaurus book
- Feature 1-physical-ai-robotics-book: Initial Physical AI & Humanoid Robotics book structure

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->