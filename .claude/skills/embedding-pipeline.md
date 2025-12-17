# Embedding Pipeline Skill

A reusable skill for managing the document embedding pipeline that powers the RAG chatbot.

## Purpose
This skill handles crawling documentation, generating embeddings via Cohere, and storing vectors in Qdrant.

## When to Use
- When new documentation is added to the book
- When re-indexing is needed after content updates
- When setting up the system for the first time
- When debugging retrieval issues

## Components

### 1. URL Discovery
- Parses `sitemap.xml` from the documentation site
- Falls back to crawling if sitemap unavailable
- Filters for internal documentation URLs only

### 2. Content Extraction
- Uses BeautifulSoup to parse HTML
- Targets `.theme-doc-markdown`, `article`, `.markdown` selectors
- Removes scripts and styles
- Normalizes whitespace

### 3. Text Chunking
- Default chunk size: 1000 characters
- Overlap: 100 characters
- Preserves context across chunks

### 4. Embedding Generation
- Model: `embed-multilingual-v3.0` (Cohere)
- Vector size: 1024 dimensions
- Input type: `search_document`

### 5. Vector Storage
- Database: Qdrant Cloud
- Collection: `rag_embedding`
- Distance metric: Cosine similarity

## Commands

### Run full pipeline:
```bash
cd database
uv run python main.py
```

### Check collection status:
```bash
cd database
uv run python -c "
from retrieval import RetrievalPipeline
p = RetrievalPipeline()
print('Valid:', p.validate_collection())
"
```

### Test retrieval:
```bash
cd database
uv run python retrieval.py query "What is ROS 2?" --top-k 3
```

## Environment Variables Required
- `COHERE_API_KEY`: Cohere API key for embeddings
- `QDRANT_URL`: Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Qdrant Cloud API key

## File Locations
- Pipeline: `database/main.py`
- Retrieval: `database/retrieval.py`
- Config: `database/.env`

## Troubleshooting

### No results found
1. Check if collection exists and has vectors
2. Verify the target URL is accessible
3. Re-run the embedding pipeline

### Embedding failures
1. Check Cohere API key validity
2. Verify rate limits not exceeded
3. Check network connectivity
