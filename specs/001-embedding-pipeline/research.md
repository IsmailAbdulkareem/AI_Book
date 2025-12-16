# Research: Embedding Pipeline Setup

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-15
**Status**: Complete

## Technology Decisions

### 1. Package Manager: UV

**Decision**: Use UV for Python project initialization and dependency management

**Rationale**:
- Fast, modern Python package manager from Astral (creators of Ruff)
- Creates standard `pyproject.toml` configuration
- Handles virtual environment creation automatically
- Supports dependency locking with `uv.lock`

**Alternatives Considered**:
- pip + venv: Manual virtual environment management, slower resolution
- Poetry: More complex, slower than UV
- PDM: Less ecosystem adoption

**Key Commands**:
```bash
uv init                    # Initialize project
uv add <package>           # Add dependency
uv run python main.py      # Run with managed environment
uv sync                    # Sync dependencies
```

---

### 2. Embedding Service: Cohere embed-english-v3.0

**Decision**: Use Cohere embed-english-v3.0 model

**Rationale**:
- 1024 dimensions (good balance of quality and storage efficiency)
- High-quality semantic embeddings optimized for search
- Supports `input_type` parameter for document vs query differentiation
- Batch API reduces API calls

**Specifications**:
| Parameter | Value |
|-----------|-------|
| Dimensions | 1024 |
| Max Tokens | 512 (~2000 characters) |
| Max Batch Size | 96 texts per request |
| Rate Limit (Trial) | 5 calls/minute |
| Rate Limit (Production) | 1000 calls/minute |

**Input Types**:
- `search_document`: For indexing (use when generating embeddings for storage)
- `search_query`: For retrieval (use when embedding user queries)

**Alternatives Considered**:
- OpenAI text-embedding-3-small: Higher cost, 1536 dimensions
- Sentence Transformers (local): Requires GPU for optimal performance
- Voyage AI: Less established ecosystem

---

### 3. Vector Database: Qdrant Cloud

**Decision**: Use Qdrant Cloud with collection name "AI-Book"

**Rationale**:
- Native Python client with excellent async support
- Cloud-hosted eliminates infrastructure management
- Fast similarity search with HNSW indexing
- Rich metadata filtering capabilities

**Configuration**:
```python
# Collection: AI-Book
vectors_config = VectorParams(
    size=1024,           # Cohere embed-english-v3.0 dimensions
    distance=Distance.COSINE  # Recommended for Cohere
)
```

**Alternatives Considered**:
- Pinecone: Higher cost, more complex pricing
- Weaviate: More complex setup
- ChromaDB: Less suitable for production scale
- Local Qdrant Docker: Requires infrastructure management

---

### 4. HTTP Client: httpx + BeautifulSoup

**Decision**: Use httpx for HTTP requests and BeautifulSoup with lxml for HTML parsing

**Rationale**:
- httpx supports async/await for concurrent fetching
- BeautifulSoup provides robust HTML parsing
- lxml backend is fast and handles malformed HTML well

**Docusaurus Content Extraction**:
```python
# Primary content selector
article = soup.find('article')
# Fallback
content_div = soup.find('div', class_='theme-doc-markdown')
```

**Elements to Remove**:
- Navigation: `nav.theme-doc-breadcrumbs`
- Sidebar: `aside.theme-doc-sidebar-container`
- Table of Contents: `div.tocCollapsible_*`

**Alternatives Considered**:
- requests: No async support
- aiohttp: Less intuitive API than httpx
- Scrapy: Overkill for this use case

---

### 5. Text Chunking Strategy

**Decision**: Character-based chunking with overlap

**Rationale**:
- Cohere v3 handles up to 512 tokens (~2000 characters)
- Overlap ensures context is preserved across chunk boundaries
- Simple to implement without NLP dependencies

**Parameters**:
| Parameter | Value |
|-----------|-------|
| Chunk Size | 1500 characters |
| Overlap | 200 characters |
| Min Chunk | 100 characters |

**Alternatives Considered**:
- Semantic chunking (langchain): Adds dependency complexity
- Sentence-based: Inconsistent chunk sizes
- Paragraph-based: May exceed token limits

---

## Dependencies

```toml
[project]
name = "embedding-pipeline"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "httpx>=0.27.0",
    "beautifulsoup4>=4.12.0",
    "lxml>=5.0.0",
    "python-dotenv>=1.0.0",
]
```

---

## Target Site Configuration

**Site URL**: https://ismailabdulkareem.github.io/AI_Book/
**Sitemap**: https://ismailabdulkareem.github.io/AI_Book/sitemap.xml

**Content Structure** (from sitemap analysis):
- `/docs/intro` - Introduction
- `/docs/module-*/` - Module content pages
- `/docs/appendix/` - Appendix materials
- `/docs/glossary` - Glossary
- `/blog/` - Blog posts (lower priority)

---

## Rate Limiting Strategy

**Cohere (Trial Key)**:
- 5 calls/minute limit
- 12-second delay between batches
- Exponential backoff on 429 errors

**Web Scraping**:
- 0.5-second delay between page fetches
- Respect robots.txt
- User-Agent identification

---

## Error Handling

| Error Type | Strategy |
|------------|----------|
| Network timeout | Retry 3 times with exponential backoff |
| HTTP 429 (Rate limit) | Wait and retry with longer delay |
| HTTP 5xx | Retry 3 times |
| HTTP 4xx | Log and skip |
| Empty content | Log warning, skip |
| Embedding failure | Retry batch, then skip failed items |
| Qdrant connection | Retry 3 times |
