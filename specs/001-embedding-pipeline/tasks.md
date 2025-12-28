# Tasks: Embedding Pipeline Setup

**Input**: Design documents from `/specs/001-embedding-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, research.md, quickstart.md

**Tests**: No automated tests requested - manual verification via Qdrant dashboard

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single file project**: `database/main.py` at repository root
- Configuration: `database/pyproject.toml`, `database/.env.example`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Create database folder, initialize UV project, configure dependencies

- [x] T001 Create `database/` folder at repository root
- [x] T002 Initialize UV project by running `uv init` in `database/` folder
- [x] T003 Add dependencies to `database/pyproject.toml`: cohere, qdrant-client, httpx, beautifulsoup4, lxml, python-dotenv
- [x] T004 Create `database/.env.example` with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY placeholders
- [x] T005 Run `uv sync` to generate `database/uv.lock` file

**Checkpoint**: UV project initialized with all dependencies locked

---

## Phase 2: Foundational (Data Structures)

**Purpose**: Define all dataclasses that all user stories depend on in `database/main.py`

**CRITICAL**: These data structures must be complete before any function implementation

- [x] T006 Create `database/main.py` with standard imports (os, time, uuid, dataclasses, typing)
- [x] T007 Add third-party imports to `database/main.py` (cohere, qdrant_client, httpx, bs4, dotenv)
- [x] T008 [P] Define `SitemapEntry` dataclass in `database/main.py` (loc, lastmod, priority)
- [x] T009 [P] Define `PageContent` dataclass in `database/main.py` (url, title, content, headings)
- [x] T010 [P] Define `Chunk` dataclass in `database/main.py` (text, url, title, chunk_index, total_chunks)
- [x] T011 [P] Define `EmbeddedChunk` dataclass in `database/main.py` (chunk, embedding)

**Checkpoint**: All data structures defined - function implementation can begin

---

## Phase 3: User Story 1 - Ingest Content from Documentation Site (Priority: P1)

**Goal**: Crawl and extract text content from all pages of the deployed Docusaurus site

**Independent Test**: Run `get_all_urls()` and `extract_text_from_url()` functions, verify URLs are discovered and content is extracted with title and clean text

### Implementation for User Story 1

- [x] T012 [US1] Implement `get_all_urls(sitemap_url: str) -> list[SitemapEntry]` in `database/main.py`
  - Fetch sitemap.xml using httpx
  - Parse XML with BeautifulSoup (lxml-xml parser)
  - Extract all `<url>` entries with loc, lastmod, priority
  - Filter to include only `/docs/` URLs
  - Return list of SitemapEntry dataclasses

- [x] T013 [US1] Implement `extract_text_from_url(url: str) -> PageContent | None` in `database/main.py`
  - Fetch page HTML using httpx with timeout
  - Parse with BeautifulSoup (lxml parser)
  - Find main content: `<article>` or `div.theme-doc-markdown`
  - Remove navigation, sidebar, TOC elements
  - Extract title from `<h1>` or `<title>`
  - Get text content with `get_text(separator='\n')`
  - Return None if content < 100 chars

- [x] T014 [US1] Add error handling to `get_all_urls()` for network failures and invalid XML in `database/main.py`

- [x] T015 [US1] Add error handling to `extract_text_from_url()` for HTTP errors and parsing failures in `database/main.py`

- [x] T016 [US1] Add logging statements to track URL discovery and content extraction progress in `database/main.py`

**Checkpoint**: User Story 1 complete - can discover URLs from sitemap and extract page content

**Verification**:
```python
# Test US1 independently
urls = get_all_urls("https://ismailabdulkareem.github.io/AI_Book/sitemap.xml")
print(f"Found {len(urls)} URLs")
page = extract_text_from_url(urls[0].loc)
print(f"Title: {page.title}, Content length: {len(page.content)}")
```

---

## Phase 4: User Story 2 - Generate Embeddings for Extracted Content (Priority: P2)

**Goal**: Split content into chunks and generate vector embeddings using Cohere

**Independent Test**: Provide test chunks and verify embeddings are generated with 1024 dimensions

### Implementation for User Story 2

- [x] T017 [US2] Implement `chunk_text(page: PageContent) -> list[Chunk]` in `database/main.py`
  - Parameters: chunk_size=1500, overlap=200, min_chunk=100
  - If content < chunk_size, return single chunk
  - Create overlapping chunks respecting character limits
  - Attach metadata (url, title, chunk_index, total_chunks)
  - Filter out chunks < min_chunk characters

- [x] T018 [US2] Implement `embed(chunks: list[Chunk]) -> list[EmbeddedChunk]` in `database/main.py`
  - Initialize Cohere client with API key from environment
  - Batch chunks (max 96 per request per Cohere limits)
  - Call `cohere.embed()` with model="embed-english-v3.0", input_type="search_document"
  - Handle rate limiting with 12-second delay between batches (trial key)
  - Pair embeddings with source chunks
  - Return list of EmbeddedChunk

- [x] T019 [US2] Add retry logic with exponential backoff for Cohere API failures in `database/main.py`

- [x] T020 [US2] Add progress logging for chunking and embedding generation in `database/main.py`

**Checkpoint**: User Story 2 complete - can chunk content and generate embeddings

**Verification**:
```python
# Test US2 independently (requires US1 output)
chunks = chunk_text(page)
print(f"Created {len(chunks)} chunks")
embedded = embed(chunks[:5])  # Test with small batch
print(f"Embedding dimensions: {len(embedded[0].embedding)}")  # Should be 1024
```

---

## Phase 5: User Story 3 - Store Embeddings in Vector Database (Priority: P3)

**Goal**: Create Qdrant collection and store embeddings with metadata

**Independent Test**: Provide test embeddings and verify they are stored in Qdrant with correct metadata

### Implementation for User Story 3

- [x] T021 [US3] Implement `create_collection(client: QdrantClient, collection_name: str) -> None` in `database/main.py`
  - Check if collection "AI-Book" exists
  - If exists, delete and recreate (full re-ingestion mode)
  - Create collection with vectors_config: size=1024, distance=COSINE
  - Log collection creation status

- [x] T022 [US3] Implement `save_chunk_to_qdrant(client: QdrantClient, embedded_chunks: list[EmbeddedChunk], collection_name: str) -> None` in `database/main.py`
  - Convert EmbeddedChunk to PointStruct with UUID
  - Build payload with url, title, text, chunk_index, total_chunks, ingested_at
  - Batch upsert (100 points per batch)
  - Log progress for each batch

- [x] T023 [US3] Add error handling for Qdrant connection failures and upsert errors in `database/main.py`

- [x] T024 [US3] Add final summary logging (total points saved, collection stats) in `database/main.py`

**Checkpoint**: User Story 3 complete - can create collection and store embeddings

**Verification**:
```python
# Test US3 independently (requires Qdrant credentials)
from qdrant_client import QdrantClient
client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
create_collection(client, "AI-Book")
save_chunk_to_qdrant(client, embedded[:10], "AI-Book")  # Test with small batch
# Check Qdrant dashboard for points
```

---

## Phase 6: Pipeline Orchestration

**Purpose**: Implement main() function to orchestrate the full pipeline

- [x] T025 Implement `main()` function in `database/main.py` that orchestrates:
  1. Load environment variables with `load_dotenv()`
  2. Initialize Cohere and Qdrant clients
  3. Create "AI-Book" collection
  4. Get all URLs from sitemap
  5. Extract content from each URL (with 0.5s rate limiting)
  6. Chunk all extracted content
  7. Generate embeddings for all chunks
  8. Save all embeddings to Qdrant
  9. Print final summary

- [x] T026 Add `if __name__ == "__main__": main()` entry point in `database/main.py`

- [x] T027 Add overall pipeline timing and statistics logging in `database/main.py`

**Checkpoint**: Full pipeline executable with `uv run python main.py`

---

## Phase 7: Polish & Verification

**Purpose**: Final testing and documentation

- [ ] T028 [P] Run full pipeline against https://ismailabdulkareem.github.io/AI_Book/sitemap.xml
- [ ] T029 [P] Verify "AI-Book" collection exists in Qdrant dashboard with expected point count
- [ ] T030 [P] Test similarity search in Qdrant dashboard with sample query
- [ ] T031 Update `database/.env.example` with any additional configuration discovered during testing
- [ ] T032 Validate quickstart.md instructions work end-to-end

**Checkpoint**: Pipeline verified working, documentation complete

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phase 3 (US1)**: Depends on Phase 2 - Content ingestion
- **Phase 4 (US2)**: Depends on Phase 2 - Can start in parallel with US1 (different functions)
- **Phase 5 (US3)**: Depends on Phase 2 - Can start in parallel with US1/US2
- **Phase 6 (Orchestration)**: Depends on US1, US2, US3 completion
- **Phase 7 (Polish)**: Depends on Phase 6 completion

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies on other stories - produces PageContent
- **User Story 2 (P2)**: Uses PageContent from US1 - produces EmbeddedChunk
- **User Story 3 (P3)**: Uses EmbeddedChunk from US2 - produces stored points

**Note**: While data flows US1 → US2 → US3, the function implementations can be done in parallel since they work on different code blocks within the same file.

### Within Each User Story

- Main function implementation before error handling
- Error handling before logging
- All functions must be complete before orchestration phase

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
```bash
# These dataclass definitions can be written in parallel:
Task T008: Define SitemapEntry dataclass
Task T009: Define PageContent dataclass
Task T010: Define Chunk dataclass
Task T011: Define EmbeddedChunk dataclass
```

**Across User Stories** (different functions, same file):
```bash
# These implementations are in different functions, can be done in parallel:
Task T012: get_all_urls() [US1]
Task T017: chunk_text() [US2]
Task T021: create_collection() [US3]
```

**Within Phase 7 (Polish)**:
```bash
# These verification tasks can run in parallel:
Task T028: Run full pipeline
Task T029: Check Qdrant dashboard
Task T030: Test similarity search
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T011)
3. Complete Phase 3: User Story 1 (T012-T016)
4. **STOP and VALIDATE**: Test URL discovery and content extraction
5. Verify sitemap parsing works, content is clean

### Incremental Delivery

1. Setup + Foundational → Project structure ready
2. Add User Story 1 → Can crawl and extract content
3. Add User Story 2 → Can chunk and generate embeddings
4. Add User Story 3 → Can store in Qdrant
5. Add Orchestration → Full pipeline working
6. Polish → Production ready

### Single Developer Strategy

Since this is a single-file project, recommended sequence:

1. **Day 1**: T001-T011 (Setup + Foundational)
2. **Day 2**: T012-T016 (US1 - Content Ingestion)
3. **Day 3**: T017-T020 (US2 - Embeddings)
4. **Day 4**: T021-T027 (US3 + Orchestration)
5. **Day 5**: T028-T032 (Polish + Verification)

---

## Task Summary

| Phase | Tasks | Parallel Tasks |
|-------|-------|----------------|
| Phase 1: Setup | 5 | 0 |
| Phase 2: Foundational | 6 | 4 |
| Phase 3: US1 - Content Ingestion | 5 | 0 |
| Phase 4: US2 - Embeddings | 4 | 0 |
| Phase 5: US3 - Vector Storage | 4 | 0 |
| Phase 6: Orchestration | 3 | 0 |
| Phase 7: Polish | 5 | 3 |
| **Total** | **32** | **7** |

---

## Notes

- All implementation is in `database/main.py` (single file as requested)
- No automated tests - manual verification via Qdrant dashboard
- Rate limiting: 12s between Cohere batches, 0.5s between page fetches
- Target site: https://ismailabdulkareem.github.io/AI_Book/
- Collection name: "AI-Book"
- Embedding model: Cohere embed-english-v3.0 (1024 dimensions)
