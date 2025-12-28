# Implementation Plan: Embedding Pipeline Setup

**Branch**: `001-embedding-pipeline` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-embedding-pipeline/spec.md`

## Summary

Build a content ingestion pipeline that extracts text from the deployed Docusaurus site (https://ismailabdulkareem.github.io/AI_Book/), generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval. Implementation is contained in a single `main.py` file within a `database/` backend folder, using UV for package management.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, httpx, beautifulsoup4, lxml, python-dotenv
**Package Manager**: UV
**Storage**: Qdrant Cloud (collection: AI-Book)
**Testing**: Manual verification via Qdrant dashboard + test queries
**Target Platform**: Local development / CI/CD scripts
**Project Type**: Single file (main.py)
**Performance Goals**: Process 100-page site in <30 minutes
**Constraints**: Cohere trial key (5 calls/min), single-threaded for simplicity
**Scale/Scope**: ~50-100 documentation pages, ~500-1000 chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-first authoring | PASS | Full spec created before implementation |
| Technical accuracy | PASS | Research validates Cohere/Qdrant configurations |
| Reproducibility | PASS | UV + .env enables reproducible setup |
| Consistency | PASS | Single file design, consistent data structures |
| Transparent AI usage | PASS | AI-assisted development documented |

## Project Structure

### Documentation (this feature)

```text
specs/001-embedding-pipeline/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology research findings
├── data-model.md        # Data structures documentation
├── quickstart.md        # Setup and run instructions
├── contracts/           # (Empty - no external API exposed)
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
database/
├── main.py              # Single-file embedding pipeline
├── pyproject.toml       # UV project configuration
├── uv.lock              # Dependency lock file
└── .env.example         # Environment variable template
```

**Structure Decision**: Single `database/` folder with one `main.py` file as explicitly requested. This keeps the pipeline simple and self-contained for initial development.

## Function Design

The `main.py` file contains these functions in order:

### 1. `get_all_urls(sitemap_url: str) -> list[SitemapEntry]`

**Purpose**: Fetch and parse sitemap.xml to discover all documentation URLs

**Input**: Sitemap URL (e.g., `https://ismailabdulkareem.github.io/AI_Book/sitemap.xml`)
**Output**: List of SitemapEntry objects with loc, lastmod, priority

**Logic**:
1. Fetch sitemap.xml using httpx
2. Parse XML with BeautifulSoup (lxml-xml parser)
3. Extract all `<url>` entries
4. Filter to include only `/docs/` URLs (exclude blog, etc.)
5. Return list of SitemapEntry dataclasses

---

### 2. `extract_text_from_url(url: str) -> PageContent | None`

**Purpose**: Fetch a page and extract clean text content

**Input**: Documentation page URL
**Output**: PageContent dataclass or None if extraction fails

**Logic**:
1. Fetch page HTML using httpx
2. Parse with BeautifulSoup (lxml parser)
3. Find main content: `<article>` or `div.theme-doc-markdown`
4. Remove navigation, sidebar, TOC elements
5. Extract title from `<h1>` or `<title>`
6. Get text content with `get_text(separator='\n')`
7. Return PageContent or None if content too short (<100 chars)

---

### 3. `chunk_text(page: PageContent) -> list[Chunk]`

**Purpose**: Split page content into embedding-sized chunks

**Input**: PageContent dataclass
**Output**: List of Chunk dataclasses

**Parameters**:
- chunk_size: 1500 characters
- overlap: 200 characters
- min_chunk: 100 characters

**Logic**:
1. If content < chunk_size, return single chunk
2. Split by paragraph boundaries where possible
3. Create overlapping chunks of ~1500 chars
4. Attach metadata (url, title, index) to each chunk
5. Filter out chunks < min_chunk

---

### 4. `embed(chunks: list[Chunk]) -> list[EmbeddedChunk]`

**Purpose**: Generate embeddings for chunks using Cohere

**Input**: List of Chunk dataclasses
**Output**: List of EmbeddedChunk dataclasses

**Logic**:
1. Initialize Cohere client with API key from env
2. Batch chunks (max 96 per request)
3. For each batch:
   - Call `cohere.embed()` with `input_type="search_document"`
   - Handle rate limiting (12s delay for trial keys)
   - Retry on transient errors
4. Pair embeddings with source chunks
5. Return EmbeddedChunk list

---

### 5. `create_collection(client: QdrantClient, collection_name: str) -> None`

**Purpose**: Create Qdrant collection if it doesn't exist

**Input**: Qdrant client, collection name ("AI-Book")
**Output**: None (creates collection as side effect)

**Logic**:
1. Check if collection exists
2. If exists, optionally recreate (configurable)
3. Create collection with:
   - vectors_config: size=1024, distance=COSINE
4. Log success/failure

---

### 6. `save_chunk_to_qdrant(client: QdrantClient, embedded_chunks: list[EmbeddedChunk], collection_name: str) -> None`

**Purpose**: Upsert embedded chunks to Qdrant

**Input**: Qdrant client, embedded chunks, collection name
**Output**: None (upserts as side effect)

**Logic**:
1. Convert EmbeddedChunk to PointStruct:
   - Generate UUID for id
   - Set vector from embedding
   - Build payload with metadata + timestamp
2. Batch upsert (100 points per batch)
3. Log progress and any errors

---

### 7. `main() -> None`

**Purpose**: Orchestrate the full pipeline

**Logic**:
```python
def main():
    # 1. Load environment variables
    load_dotenv()

    # 2. Initialize clients
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # 3. Create collection
    create_collection(qdrant_client, "AI-Book")

    # 4. Get all URLs from sitemap
    sitemap_url = "https://ismailabdulkareem.github.io/AI_Book/sitemap.xml"
    urls = get_all_urls(sitemap_url)
    print(f"Found {len(urls)} URLs")

    # 5. Process each URL
    all_chunks = []
    for entry in urls:
        page = extract_text_from_url(entry.loc)
        if page:
            chunks = chunk_text(page)
            all_chunks.extend(chunks)
        time.sleep(0.5)  # Rate limit scraping

    print(f"Created {len(all_chunks)} chunks")

    # 6. Generate embeddings
    embedded_chunks = embed(all_chunks)
    print(f"Generated {len(embedded_chunks)} embeddings")

    # 7. Save to Qdrant
    save_chunk_to_qdrant(qdrant_client, embedded_chunks, "AI-Book")
    print("Pipeline complete!")

if __name__ == "__main__":
    main()
```

## Environment Variables

```bash
# .env.example
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key
```

## Dependencies (pyproject.toml)

```toml
[project]
name = "embedding-pipeline"
version = "0.1.0"
description = "RAG content ingestion pipeline for AI Book"
requires-python = ">=3.11"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "httpx>=0.27.0",
    "beautifulsoup4>=4.12.0",
    "lxml>=5.0.0",
    "python-dotenv>=1.0.0",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

## Complexity Tracking

No complexity violations to justify - single file design is the simplest approach.

## Success Verification

After running the pipeline:

1. **Qdrant Dashboard**: Verify collection "AI-Book" exists with points
2. **Point Count**: Should have 500-1000 points (depends on content)
3. **Sample Search**: Run similarity search to verify embeddings work
4. **Metadata Check**: Verify payload contains url, title, text fields

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Cohere rate limiting | 12s delay between batches, exponential backoff |
| Site structure changes | CSS selectors handle both article and div layouts |
| Large pages overflow tokens | Chunking limits to 1500 chars (<512 tokens) |
| Network failures | Retry logic with 3 attempts |

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Create `database/` folder structure
3. Initialize UV project
4. Implement functions in order
5. Test with full sitemap ingestion
