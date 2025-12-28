# Tasks: RAG Retrieval & Pipeline Validation

**Input**: Design documents from `/specs/002-rag-retrieval/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, research.md, quickstart.md
**Depends On**: 001-embedding-pipeline (content must be ingested)

**Tests**: Test harness included (US4) - validates retrieval accuracy

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Spec 1 (Ingestion)**: `database/main.py` — URL discovery, embedding, Qdrant ingestion (DO NOT MODIFY)
- **Spec 2 (Retrieval)**: `database/retrieval.py` — retrieval, query CLI, test harness (NEW FILE)
- Retrieval imports shared clients/config from `main.py`

---

## Phase 1: Setup (CLI Infrastructure)

**Purpose**: Add argparse CLI structure with subcommands to new retrieval.py

- [X] T001 Add `argparse` import and `dataclass` import to `database/retrieval.py`
- [X] T002 Add `json` import for JSON output support in `database/retrieval.py`
- [X] T003 Create argument parser with subcommands (ingest, query, test) in `database/retrieval.py`
- [X] T004 Refactor existing `main()` to become `run_ingest()` command in `database/retrieval.py`
- [X] T005 Create new `main()` that dispatches to subcommands based on args in `database/retrieval.py`

**Checkpoint**: CLI structure ready - `python retrieval.py ingest` works as before ✓

---

## Phase 2: Foundational (Data Structures)

**Purpose**: Define dataclasses needed by all user stories

- [X] T006 [P] Define `QueryResult` dataclass in `database/retrieval.py` (score, content, url, position, created_at)
- [X] T007 [P] Define `RetrievalResponse` dataclass in `database/retrieval.py` (query, results, total, time_ms, collection)
- [X] T008 Add Qdrant filter imports (`Filter`, `FieldCondition`, `MatchText`) to `database/retrieval.py`

**Checkpoint**: All data structures defined - retrieval implementation can begin ✓

---

## Phase 3: User Story 1 - Semantic Retrieval (Priority: P1)

**Goal**: Retrieve relevant content chunks using natural language queries

**Independent Test**: Run `python retrieval.py query "What is ROS 2?"` and verify ranked results are returned

### Implementation for User Story 1

- [X] T009 [US1] Implement `embed_query(query: str) -> List[float]` method in `RetrievalPipeline` class in `database/retrieval.py`
  - Use `input_type="search_query"` (different from document embedding)
  - Reuse existing Cohere client
  - Return 1024-dimensional vector

- [X] T010 [US1] Implement `validate_collection(collection: str) -> bool` method in `database/retrieval.py`
  - Check if collection exists in Qdrant
  - Check if collection has vectors (points_count > 0)
  - Return True/False

- [X] T011 [US1] Implement `search(query: str, top_k: int = 5, url_filter: str = None) -> RetrievalResponse` method in `database/retrieval.py`
  - Call `embed_query()` to get query vector
  - Execute Qdrant `query_points()` with query vector and limit
  - Convert ScoredPoints to QueryResult objects
  - Package into RetrievalResponse with timing

- [X] T012 [US1] Implement `format_results(response: RetrievalResponse, json_output: bool = False) -> str` method in `database/retrieval.py`
  - Text format: numbered results with score, URL, position, content
  - JSON format: structured JSON output

- [X] T013 [US1] Add `run_query(args)` function to handle query subcommand in `database/retrieval.py`
  - Parse query from positional argument
  - Call `search()` with provided options
  - Format and print results

- [X] T014 [US1] Add error handling for empty query in `database/retrieval.py`
  - Check query is not empty/whitespace
  - Print clear error message and exit with code 1

- [X] T015 [US1] Add error handling for missing collection in `database/retrieval.py`
  - Call `validate_collection()` before search
  - Print actionable error message if collection missing

- [X] T016 [US1] Add error handling for embedding failures in `database/retrieval.py`
  - Catch Cohere API errors in `embed_query()`
  - Print descriptive error message

**Checkpoint**: User Story 1 complete - basic semantic search works ✓

**Verification**:
```bash
uv run python retrieval.py query "What is ROS 2?"
# Should return ranked results with scores ✓
```

---

## Phase 4: User Story 2 - Debug & Inspect Retrieval (Priority: P2)

**Goal**: Provide detailed diagnostic information for debugging retrieval issues

**Independent Test**: Run `python retrieval.py query "test" --debug` and verify debug output is displayed

### Implementation for User Story 2

- [X] T017 [US2] Add `--debug` / `-d` flag to query subcommand parser in `database/retrieval.py`

- [X] T018 [US2] Add debug logging for query embedding in `embed_query()` method in `database/retrieval.py`
  - Log query text being embedded
  - Log embedding dimensions
  - Log embedding time

- [X] T019 [US2] Add debug logging for collection stats in `search()` method in `database/retrieval.py`
  - Log collection name
  - Log total vector count in collection
  - Log search parameters (top_k, filter)

- [X] T020 [US2] Add debug logging for search results in `search()` method in `database/retrieval.py`
  - Log search time
  - Log score distribution (min, max, avg)
  - Log number of results found

- [X] T021 [US2] Add warning for empty results in debug mode in `database/retrieval.py`
  - When no results found, log explanation
  - Suggest possible causes (empty collection, query mismatch)

**Checkpoint**: User Story 2 complete - debug mode provides diagnostics ✓

**Verification**:
```bash
uv run python retrieval.py query "test query" --debug
# Should show [DEBUG] messages before results ✓
```

---

## Phase 5: User Story 3 - Metadata Filtering (Priority: P3)

**Goal**: Filter results by URL and limit result count

**Independent Test**: Run `python retrieval.py query "test" --url-filter "module2" --top-k 3` and verify filters are applied

### Implementation for User Story 3

- [X] T022 [US3] Add `--top-k` / `-k` argument to query subcommand parser in `database/retrieval.py`
  - Type: int
  - Default: 5

- [X] T023 [US3] Add `--url-filter` / `-u` argument to query subcommand parser in `database/retrieval.py`
  - Type: str
  - Default: None

- [X] T024 [US3] Implement URL filter conversion to Qdrant Filter in `search()` method in `database/retrieval.py`
  - Use `MatchText` for substring matching
  - Build `Filter` with `FieldCondition` on "url" field
  - Auto-create text index on URL field if needed

- [X] T025 [US3] Pass filter to Qdrant `query_points()` call in `database/retrieval.py`
  - Add `query_filter` parameter to search call
  - Only apply filter if url_filter is provided

- [X] T026 [US3] Add `--collection` / `-c` argument to query subcommand parser in `database/retrieval.py`
  - Type: str
  - Default: DEFAULT_COLLECTION ("rag_embedding")

**Checkpoint**: User Story 3 complete - filtering works correctly ✓

**Verification**:
```bash
uv run python retrieval.py query "simulation" --url-filter "module2" --top-k 3
# Should return max 3 results, all from URLs containing "module2" ✓
```

---

## Phase 6: User Story 4 - Retrieval Test Harness (Priority: P4)

**Goal**: Automated tests to validate retrieval accuracy and prevent regressions

**Independent Test**: Run `python retrieval.py test` and verify pass/fail summary

### Implementation for User Story 4

- [X] T027 [US4] Define `TEST_CASES` list with known queries in `database/retrieval.py`
  - Query: "ROS 2 robot operating system", expect URL contains "module1"
  - Query: "Gazebo simulation environment", expect URL contains "module2"
  - Query: "Isaac Sim NVIDIA", expect URL contains "module3"
  - Each case: query, expected_url_contains, min_score

- [X] T028 [US4] Implement `run_test_harness() -> bool` function in `database/retrieval.py`
  - Loop through TEST_CASES
  - Execute search for each query
  - Check top result URL contains expected substring
  - Check top result score >= min_score
  - Track pass/fail for each test

- [X] T029 [US4] Add test result formatting in `run_test_harness()` in `database/retrieval.py`
  - Print test case number and query
  - Print expected vs actual
  - Print PASS/FAIL status
  - Print summary at end (X/Y passed)

- [X] T030 [US4] Add `run_test(args)` function to handle test subcommand in `database/retrieval.py`
  - Initialize pipeline
  - Call `run_test_harness()`
  - Exit with code 0 if all pass, 1 if any fail

- [X] T031 [US4] Add test for missing collection error handling in test harness in `database/retrieval.py`
  - Test that missing collection produces clear error
  - Not a search test, but validates error handling

**Checkpoint**: User Story 4 complete - test harness validates pipeline ✓

**Verification**:
```bash
uv run python retrieval.py test
# Should show pass/fail for each test case and summary ✓
# Test Summary: 3/3 PASSED
```

---

## Phase 7: Polish & Integration

**Purpose**: JSON output, final testing, documentation validation

- [X] T032 Add `--json` / `-j` flag to query subcommand parser in `database/retrieval.py`
- [X] T033 Implement JSON output in `format_results()` method in `database/retrieval.py`
  - Use `json.dumps()` with indent=2
  - Include all RetrievalResponse fields
- [X] T034 [P] Run full test suite: `python retrieval.py test` ✓ (3/3 PASSED)
- [X] T035 [P] Test JSON output: `python retrieval.py query "test" --json` ✓
- [X] T036 Validate quickstart.md examples work end-to-end ✓

**Checkpoint**: Feature complete and documented ✓

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately ✓
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories ✓
- **Phase 3 (US1)**: Depends on Phase 2 - Core retrieval ✓
- **Phase 4 (US2)**: Depends on Phase 3 - Debug extends basic retrieval ✓
- **Phase 5 (US3)**: Depends on Phase 3 - Filtering extends basic retrieval ✓
- **Phase 6 (US4)**: Depends on Phase 3 - Test harness uses retrieval ✓
- **Phase 7 (Polish)**: Depends on all user stories ✓

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies - core functionality ✓
- **User Story 2 (P2)**: Depends on US1 - adds debug to existing search ✓
- **User Story 3 (P3)**: Depends on US1 - adds filtering to existing search ✓
- **User Story 4 (P4)**: Depends on US1 - uses search for automated tests ✓

**Note**: US2 and US3 can be implemented in parallel after US1 is complete.

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
```bash
# Dataclass definitions can be written in parallel:
Task T006: Define QueryResult dataclass ✓
Task T007: Define RetrievalResponse dataclass ✓
```

**After US1 Complete (US2 and US3 parallel)**:
```bash
# These user stories are independent of each other:
Task T017-T021: Debug mode (US2) ✓
Task T022-T026: Filtering (US3) ✓
```

**Within Phase 7 (Polish)**:
```bash
# These verification tasks can run in parallel:
Task T034: Run test suite ✓
Task T035: Test JSON output ✓
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005) ✓
2. Complete Phase 2: Foundational (T006-T008) ✓
3. Complete Phase 3: User Story 1 (T009-T016) ✓
4. **STOP and VALIDATE**: Test basic query functionality ✓
5. Verify `python retrieval.py query "test"` returns results ✓

### Incremental Delivery

1. Setup + Foundational → CLI structure ready ✓
2. Add User Story 1 → Basic semantic search works ✓
3. Add User Story 2 → Debug diagnostics available ✓
4. Add User Story 3 → Filtering enhances precision ✓
5. Add User Story 4 → Automated regression prevention ✓
6. Polish → JSON output, documentation validation ✓

### Single Developer Strategy

Since this is a new file separate from Spec 1:

1. **Session 1**: T001-T008 (Setup + Foundational) ✓
2. **Session 2**: T009-T016 (US1 - Core Retrieval) ✓
3. **Session 3**: T017-T021 (US2 - Debug Mode) ✓
4. **Session 4**: T022-T026 (US3 - Filtering) ✓
5. **Session 5**: T027-T036 (US4 + Polish) ✓

---

## Task Summary

| Phase | Tasks | Parallel Tasks | Status |
|-------|-------|----------------|--------|
| Phase 1: Setup | 5 | 0 | ✓ Complete |
| Phase 2: Foundational | 3 | 2 | ✓ Complete |
| Phase 3: US1 - Semantic Retrieval | 8 | 0 | ✓ Complete |
| Phase 4: US2 - Debug Mode | 5 | 0 | ✓ Complete |
| Phase 5: US3 - Filtering | 5 | 0 | ✓ Complete |
| Phase 6: US4 - Test Harness | 5 | 0 | ✓ Complete |
| Phase 7: Polish | 5 | 2 | ✓ Complete |
| **Total** | **36** | **4** | **✓ All Complete** |

---

## Notes

- **Spec 1 and Spec 2 are separated by file**:
  - `database/main.py` — Ingestion pipeline (Spec 1) - DO NOT MODIFY
  - `database/retrieval.py` — Retrieval pipeline (Spec 2) - NEW FILE
- Retrieval imports shared configuration from `main.py` but does not mutate it
- Uses same Cohere and Qdrant credentials from `.env`
- Test harness validates retrieval works with ingested content
- Collection name: "rag_embedding" (from Spec 1 ingestion)
- Embedding model: embed-multilingual-v3.0 (must match ingestion)

---

## Implementation Complete

**Date**: 2025-12-15
**Status**: All 36 tasks completed successfully
**Test Results**: 3/3 PASSED
