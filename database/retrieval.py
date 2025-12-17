"""
RAG Retrieval Pipeline - Spec 2

Semantic search retrieval for the Docusaurus RAG chatbot.
Queries ingested content from Qdrant and returns ranked results.

Usage:
    python retrieval.py query "What is ROS 2?"
    python retrieval.py query "robotics" --top-k 3
    python retrieval.py query "simulation" --url-filter "/module-2/"
    python retrieval.py query "test" --debug
    python retrieval.py query "test" --json
    python retrieval.py test
    python retrieval.py ingest
"""

import os
import sys
import time
import argparse
import json
from dataclasses import dataclass, asdict
from typing import List, Optional

from dotenv import load_dotenv

import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchText, TextIndexParams, TokenizerType

# Import ingestion pipeline for 'ingest' subcommand
from main import DocusaurusEmbeddingPipeline, main as run_ingestion_main


# -----------------------------------------------------------------------------
# Environment
# -----------------------------------------------------------------------------

load_dotenv()


# -----------------------------------------------------------------------------
# Constants (must match main.py ingestion)
# -----------------------------------------------------------------------------

DEFAULT_COLLECTION = os.getenv("QDRANT_COLLECTION", "rag_embedding")
COHERE_EMBED_MODEL = "embed-multilingual-v3.0"
VECTOR_SIZE = 1024


# -----------------------------------------------------------------------------
# Data Classes (Phase 2: T006-T007)
# -----------------------------------------------------------------------------

@dataclass
class QueryResult:
    """Single retrieval result from vector database."""
    score: float
    content: str
    url: str
    position: int
    created_at: float


@dataclass
class RetrievalResponse:
    """Complete response from a search query."""
    query: str
    results: List[QueryResult]
    total: int
    time_ms: int
    collection: str


# -----------------------------------------------------------------------------
# Test Cases (Phase 6: T027)
# -----------------------------------------------------------------------------

TEST_CASES = [
    {
        "query": "ROS 2 robot operating system",
        "expect_url_contains": "module1",
        "min_score": 0.4,
    },
    {
        "query": "Gazebo simulation environment",
        "expect_url_contains": "module2",
        "min_score": 0.4,
    },
    {
        "query": "Isaac Sim NVIDIA",
        "expect_url_contains": "module3",
        "min_score": 0.4,
    },
]


# -----------------------------------------------------------------------------
# Retrieval Pipeline (Phase 3: T009-T016)
# -----------------------------------------------------------------------------

class RetrievalPipeline:
    """Semantic search retrieval for RAG pipeline."""

    def __init__(self, collection: str = DEFAULT_COLLECTION, debug: bool = False):
        self.collection = collection
        self.debug = debug

        self.cohere_client = cohere.Client(
            api_key=os.getenv("COHERE_API_KEY")
        )

        self.qdrant_client = self._init_qdrant()

    def _init_qdrant(self) -> QdrantClient:
        """Initialize Qdrant client."""
        url = os.getenv("QDRANT_URL", "http://localhost:6333")
        api_key = os.getenv("QDRANT_API_KEY")

        return (
            QdrantClient(url=url, api_key=api_key)
            if api_key
            else QdrantClient(url=url)
        )

    # -------------------------------------------------------------------------
    # Query Embedding (T009)
    # -------------------------------------------------------------------------

    def embed_query(self, query: str) -> List[float]:
        """
        Embed a search query using Cohere.

        Uses input_type="search_query" (different from document embedding).
        Returns 1024-dimensional vector.
        """
        start_time = time.time()

        try:
            response = self.cohere_client.embed(
                texts=[query],
                model=COHERE_EMBED_MODEL,
                input_type="search_query",  # Key difference from document embedding
            )

            embedding = response.embeddings[0]
            elapsed_ms = int((time.time() - start_time) * 1000)

            # Debug logging (T018)
            if self.debug:
                print(f"[DEBUG] Embedding query: \"{query}\"")
                print(f"[DEBUG] Query vector: {len(embedding)} dimensions ({elapsed_ms}ms)")

            return embedding

        except Exception as e:
            # Error handling (T016)
            print(f"Error: Failed to embed query: {e}", file=sys.stderr)
            sys.exit(1)

    # -------------------------------------------------------------------------
    # Collection Validation (T010)
    # -------------------------------------------------------------------------

    def validate_collection(self, collection: str = None) -> bool:
        """
        Check if target collection exists and has vectors.

        Returns True if valid, False otherwise.
        """
        collection = collection or self.collection

        try:
            collections = {
                col.name for col in self.qdrant_client.get_collections().collections
            }

            if collection not in collections:
                return False

            info = self.qdrant_client.get_collection(collection)
            return info.points_count > 0

        except Exception:
            return False

    def ensure_text_index(self, collection: str = None) -> None:
        """
        Ensure text index exists on URL field for filtering.

        Creates index if it doesn't exist.
        """
        collection = collection or self.collection

        try:
            self.qdrant_client.create_payload_index(
                collection_name=collection,
                field_name="url",
                field_schema=TextIndexParams(
                    type="text",
                    tokenizer=TokenizerType.WORD,
                    min_token_len=2,
                    max_token_len=20,
                    lowercase=True,
                ),
            )
            if self.debug:
                print(f"[DEBUG] Created text index on 'url' field")
        except Exception as e:
            # Index may already exist - that's fine
            if self.debug:
                print(f"[DEBUG] Text index check: {e}")

    # -------------------------------------------------------------------------
    # Search (T011, T024, T025)
    # -------------------------------------------------------------------------

    def search(
        self,
        query: str,
        top_k: int = 5,
        url_filter: str = None
    ) -> RetrievalResponse:
        """
        Execute semantic search against stored embeddings.

        Args:
            query: Natural language query string
            top_k: Maximum number of results to return
            url_filter: Optional URL substring filter

        Returns:
            RetrievalResponse with ranked results
        """
        start_time = time.time()

        # Get query embedding
        query_vector = self.embed_query(query)

        # Build Qdrant filter if url_filter provided (T024)
        query_filter = None
        if url_filter:
            # Ensure text index exists for URL filtering
            self.ensure_text_index()

            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="url",
                        match=MatchText(text=url_filter)
                    )
                ]
            )

        # Debug logging for collection stats (T019)
        if self.debug:
            try:
                info = self.qdrant_client.get_collection(self.collection)
                print(f"[DEBUG] Collection '{self.collection}': {info.points_count} vectors")
            except Exception:
                pass
            print(f"[DEBUG] Searching with top_k={top_k}, filter={url_filter}")

        # Execute Qdrant search (T025)
        search_response = self.qdrant_client.query_points(
            collection_name=self.collection,
            query=query_vector,
            limit=top_k,
            query_filter=query_filter,
        )

        elapsed_ms = int((time.time() - start_time) * 1000)

        # Convert ScoredPoints to QueryResult objects
        results = []
        for point in search_response.points:
            payload = point.payload
            results.append(QueryResult(
                score=point.score,
                content=payload.get("content", ""),
                url=payload.get("url", ""),
                position=payload.get("position", 0),
                created_at=payload.get("created_at", 0.0),
            ))

        # Debug logging for search results (T020)
        if self.debug:
            print(f"[DEBUG] Search completed in {elapsed_ms}ms")
            if results:
                scores = [r.score for r in results]
                print(f"[DEBUG] Score distribution: min={min(scores):.2f}, max={max(scores):.2f}, avg={sum(scores)/len(scores):.2f}")
            print(f"[DEBUG] Results found: {len(results)}")

        # Warning for empty results in debug mode (T021)
        if self.debug and not results:
            print("[DEBUG] WARNING: No results found")
            print("[DEBUG] Possible causes: empty collection, query mismatch, or filter too restrictive")

        return RetrievalResponse(
            query=query,
            results=results,
            total=len(results),
            time_ms=elapsed_ms,
            collection=self.collection,
        )

    # -------------------------------------------------------------------------
    # Result Formatting (T012, T033)
    # -------------------------------------------------------------------------

    def format_results(
        self,
        response: RetrievalResponse,
        json_output: bool = False
    ) -> str:
        """
        Format retrieval results for display.

        Args:
            response: RetrievalResponse from search
            json_output: If True, output as JSON; otherwise human-readable text

        Returns:
            Formatted string
        """
        if json_output:
            # JSON format (T033)
            output = {
                "query": response.query,
                "collection": response.collection,
                "results": [asdict(r) for r in response.results],
                "total": response.total,
                "time_ms": response.time_ms,
            }
            return json.dumps(output, indent=2)

        # Text format
        lines = [
            f'Query: "{response.query}"',
            f"Collection: {response.collection}",
            f"Results: {response.total} matches ({response.time_ms}ms)",
            "",
        ]

        if not response.results:
            lines.append("No results found.")
        else:
            for idx, result in enumerate(response.results, start=1):
                content_preview = result.content[:200] + "..." if len(result.content) > 200 else result.content
                lines.extend([
                    f"[{idx}] Score: {result.score:.2f}",
                    f"    URL: {result.url}",
                    f"    Position: {result.position}",
                    f"    Content: {content_preview}",
                    "",
                ])

        return "\n".join(lines)


# -----------------------------------------------------------------------------
# Test Harness (Phase 6: T028-T031)
# -----------------------------------------------------------------------------

def run_test_harness(pipeline: RetrievalPipeline) -> bool:
    """
    Execute automated retrieval tests.

    Returns True if all tests pass, False otherwise.
    """
    print("Running RAG Retrieval Tests...\n")

    passed = 0
    failed = 0
    total = len(TEST_CASES)

    for idx, test_case in enumerate(TEST_CASES, start=1):
        query = test_case["query"]
        expect_url_contains = test_case["expect_url_contains"]
        min_score = test_case["min_score"]

        print(f"[{idx}/{total}] Query: \"{query}\"")
        print(f"      Expected URL contains: {expect_url_contains}")

        try:
            response = pipeline.search(query, top_k=1)

            if not response.results:
                print(f"      Result: FAIL (no results returned)")
                failed += 1
                print()
                continue

            top_result = response.results[0]
            url_match = expect_url_contains in top_result.url
            score_match = top_result.score >= min_score

            if url_match and score_match:
                print(f"      Result: PASS (score={top_result.score:.2f}, url matches)")
                passed += 1
            else:
                reasons = []
                if not url_match:
                    reasons.append(f"URL '{top_result.url}' does not contain '{expect_url_contains}'")
                if not score_match:
                    reasons.append(f"score {top_result.score:.2f} < min {min_score}")
                print(f"      Result: FAIL ({', '.join(reasons)})")
                failed += 1

        except Exception as e:
            print(f"      Result: FAIL (error: {e})")
            failed += 1

        print()

    # Print summary (T029)
    print(f"Test Summary: {passed}/{total} PASSED")

    if failed > 0:
        print(f"             {failed}/{total} FAILED")

    return failed == 0


# -----------------------------------------------------------------------------
# CLI Command Handlers (Phase 1: T003-T005, Phase 3: T013)
# -----------------------------------------------------------------------------

def run_ingest(args) -> int:
    """Handle ingest subcommand - runs existing ingestion pipeline."""
    run_ingestion_main()
    return 0


def run_query(args) -> int:
    """Handle query subcommand."""
    # Set UTF-8 encoding for stdout to handle Unicode characters
    if sys.stdout.encoding != 'utf-8':
        sys.stdout.reconfigure(encoding='utf-8')

    query = args.query

    # Error handling for empty query (T014)
    if not query or not query.strip():
        print("Error: Query cannot be empty", file=sys.stderr)
        return 1

    # Initialize pipeline
    pipeline = RetrievalPipeline(
        collection=args.collection,
        debug=args.debug,
    )

    # Error handling for missing collection (T015)
    if not pipeline.validate_collection():
        print(
            f"Error: Collection '{args.collection}' not found. "
            f"Run 'python retrieval.py ingest' first.",
            file=sys.stderr
        )
        return 1

    # Execute search
    response = pipeline.search(
        query=query.strip(),
        top_k=args.top_k,
        url_filter=args.url_filter,
    )

    # Format and print results
    output = pipeline.format_results(response, json_output=args.json)
    print(output)

    return 0


def run_test(args) -> int:
    """Handle test subcommand."""
    pipeline = RetrievalPipeline(
        collection=args.collection if hasattr(args, 'collection') else DEFAULT_COLLECTION,
        debug=False,
    )

    # Error handling for missing collection (T031)
    if not pipeline.validate_collection():
        print(
            f"Error: Collection '{pipeline.collection}' not found. "
            f"Run 'python retrieval.py ingest' first.",
            file=sys.stderr
        )
        return 1

    success = run_test_harness(pipeline)
    return 0 if success else 1


# -----------------------------------------------------------------------------
# Argument Parser (Phase 1: T003, Phase 4-5: T017, T022-T023, T026, T032)
# -----------------------------------------------------------------------------

def create_parser() -> argparse.ArgumentParser:
    """Create argument parser with subcommands."""
    parser = argparse.ArgumentParser(
        prog="retrieval.py",
        description="RAG Retrieval Pipeline - Query ingested content using semantic search",
    )

    subparsers = parser.add_subparsers(
        dest="command",
        title="commands",
        description="Available commands",
    )

    # Ingest subcommand
    ingest_parser = subparsers.add_parser(
        "ingest",
        help="Run the ingestion pipeline to populate the vector database",
    )
    ingest_parser.set_defaults(func=run_ingest)

    # Query subcommand
    query_parser = subparsers.add_parser(
        "query",
        help="Search ingested content using natural language",
    )
    query_parser.add_argument(
        "query",
        type=str,
        help="Natural language query",
    )
    query_parser.add_argument(
        "--top-k", "-k",
        type=int,
        default=5,
        help="Number of results to return (default: 5)",
    )
    query_parser.add_argument(
        "--url-filter", "-u",
        type=str,
        default=None,
        help="Filter results by URL substring",
    )
    query_parser.add_argument(
        "--debug", "-d",
        action="store_true",
        help="Show detailed diagnostics",
    )
    query_parser.add_argument(
        "--json", "-j",
        action="store_true",
        help="Output results as JSON",
    )
    query_parser.add_argument(
        "--collection", "-c",
        type=str,
        default=DEFAULT_COLLECTION,
        help=f"Target collection name (default: {DEFAULT_COLLECTION})",
    )
    query_parser.set_defaults(func=run_query)

    # Test subcommand
    test_parser = subparsers.add_parser(
        "test",
        help="Run automated retrieval tests",
    )
    test_parser.add_argument(
        "--collection", "-c",
        type=str,
        default=DEFAULT_COLLECTION,
        help=f"Target collection name (default: {DEFAULT_COLLECTION})",
    )
    test_parser.set_defaults(func=run_test)

    return parser


# -----------------------------------------------------------------------------
# Main Entry Point (Phase 1: T005)
# -----------------------------------------------------------------------------

def main() -> int:
    """Main entry point - dispatch to subcommands."""
    parser = create_parser()
    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 1

    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
