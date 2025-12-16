"""
RAG Agent API - Agentic RAG with OpenAI + FastAPI

Spec 3: 003-agent-rag-integration
Purpose: HTTP API for question answering using RAG with source citations

This module provides:
- POST /ask: Accept questions, retrieve context, generate grounded answers
- GET /health: Check service and dependency health

Dependencies: Spec 1 (ingestion), Spec 2 (retrieval)
"""

import os
import time
from typing import Optional

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from openai import OpenAI
from pydantic import BaseModel, Field

from retrieval import QueryResult, RetrievalPipeline

# Load environment variables
load_dotenv()

# =============================================================================
# Constants
# =============================================================================

SYSTEM_PROMPT = """You are an assistant for the Physical AI & Humanoid Robotics book.

IMPORTANT RULES:
1. Answer questions ONLY using the provided context
2. If the context doesn't contain relevant information, say "I don't have information about that topic in the book"
3. Always cite sources by their number [1], [2], etc.
4. Be concise and accurate
5. Do not make up information not in the context

Context will be provided in the format:
[1] Content...
Source: URL

Use this context to answer the user's question."""

# =============================================================================
# Pydantic Models
# =============================================================================


class AskRequest(BaseModel):
    """Request model for POST /ask endpoint."""

    question: str = Field(..., min_length=1, description="The question to ask")
    top_k: int = Field(
        default=5, ge=1, le=20, description="Number of sources to retrieve"
    )


class Source(BaseModel):
    """A source citation from the retrieved context."""

    url: str = Field(..., description="Source URL")
    content: str = Field(..., description="Relevant content snippet")
    score: float = Field(..., description="Relevance score")
    position: int = Field(..., description="Position in results (1-indexed)")


class AskResponse(BaseModel):
    """Response model for POST /ask endpoint."""

    question: str = Field(..., description="The original question")
    answer: str = Field(..., description="Generated answer with citations")
    sources: list[Source] = Field(..., description="Source citations")
    retrieval_time_ms: int = Field(..., description="Time spent on retrieval")
    generation_time_ms: int = Field(..., description="Time spent on generation")


class DependencyHealth(BaseModel):
    """Health status of a dependency."""

    status: str = Field(..., description="Status: connected, configured, error")
    details: Optional[dict] = Field(default=None, description="Additional details")


class HealthResponse(BaseModel):
    """Response model for GET /health endpoint."""

    status: str = Field(..., description="Overall status: healthy, degraded, unhealthy")
    dependencies: dict[str, DependencyHealth] = Field(
        ..., description="Health of each dependency"
    )


class ErrorResponse(BaseModel):
    """Standard error response model."""

    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(default=None, description="Additional error details")


# =============================================================================
# FastAPI Application
# =============================================================================

app = FastAPI(
    title="RAG Agent API",
    description="Physical AI Book RAG Chatbot - Question answering with source citations",
    version="1.0.0",
)


# =============================================================================
# Helper Functions
# =============================================================================


def format_context(results: list[QueryResult]) -> str:
    """Format retrieved results as numbered context for the LLM.

    Args:
        results: List of QueryResult from retrieval pipeline

    Returns:
        Formatted string with numbered sources
    """
    if not results:
        return ""

    context_parts = []
    for i, result in enumerate(results, 1):
        context_parts.append(f"[{i}] {result.content}\nSource: {result.url}")

    return "\n\n".join(context_parts)


def generate_answer(question: str, context: str) -> tuple[str, int]:
    """Generate a grounded answer using OpenAI.

    Args:
        question: The user's question
        context: Formatted context from retrieval

    Returns:
        Tuple of (answer text, generation time in ms)
    """
    client = OpenAI()

    start_time = time.time()

    if not context:
        user_content = f"Question: {question}\n\nNote: No relevant context was found in the book."
    else:
        user_content = f"Context:\n{context}\n\nQuestion: {question}"

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_content},
        ],
        temperature=0.3,
        max_tokens=1000,
    )

    generation_time_ms = int((time.time() - start_time) * 1000)
    answer = response.choices[0].message.content or ""

    return answer, generation_time_ms


def query_result_to_source(qr: QueryResult, position: int) -> Source:
    """Convert a QueryResult to a Source model.

    Args:
        qr: QueryResult from retrieval pipeline
        position: Position in results (1-indexed)

    Returns:
        Source model with truncated content
    """
    # Truncate content to 500 chars
    content = qr.content[:500] + "..." if len(qr.content) > 500 else qr.content

    return Source(
        url=qr.url,
        content=content,
        score=qr.score,
        position=position,
    )


def check_qdrant_health() -> DependencyHealth:
    """Check Qdrant vector database health.

    Returns:
        DependencyHealth with connection status
    """
    try:
        pipeline = RetrievalPipeline()
        is_valid = pipeline.validate_collection()

        if is_valid:
            # Get vector count from collection info
            info = pipeline.qdrant_client.get_collection(pipeline.collection)
            return DependencyHealth(
                status="connected",
                details={"vector_count": info.points_count},
            )
        else:
            return DependencyHealth(
                status="error",
                details={"message": "Collection validation failed or empty"},
            )
    except Exception as e:
        return DependencyHealth(
            status="error",
            details={"message": str(e)},
        )


def check_openai_health() -> DependencyHealth:
    """Check OpenAI API configuration.

    Returns:
        DependencyHealth with configuration status
    """
    api_key = os.getenv("OPENAI_API_KEY")

    if api_key and api_key != "your_openai_api_key_here":
        return DependencyHealth(
            status="configured",
            details={"key_prefix": api_key[:8] + "..."},
        )
    else:
        return DependencyHealth(
            status="error",
            details={"message": "OPENAI_API_KEY not configured"},
        )


# =============================================================================
# API Endpoints
# =============================================================================


@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    """Ask a question and get a grounded answer with source citations.

    Args:
        request: AskRequest with question and optional top_k

    Returns:
        AskResponse with answer and sources
    """
    # Initialize retrieval pipeline
    try:
        pipeline = RetrievalPipeline()
    except Exception as e:
        raise HTTPException(
            status_code=503,
            detail=ErrorResponse(
                error="service_unavailable",
                message="Vector database is unavailable",
                details={"exception": str(e)},
            ).model_dump(),
        )

    # Retrieve context
    try:
        start_time = time.time()
        response = pipeline.search(query=request.question, top_k=request.top_k)
        retrieval_time_ms = int((time.time() - start_time) * 1000)
        results = response.results
    except Exception as e:
        raise HTTPException(
            status_code=503,
            detail=ErrorResponse(
                error="service_unavailable",
                message="Failed to retrieve context from vector database",
                details={"exception": str(e)},
            ).model_dump(),
        )

    # Format context
    context = format_context(results)

    # Generate answer
    try:
        answer, generation_time_ms = generate_answer(request.question, context)
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error="generation_error",
                message="Failed to generate answer",
                details={"exception": str(e)},
            ).model_dump(),
        )

    # Build sources
    sources = [
        query_result_to_source(qr, i + 1) for i, qr in enumerate(results)
    ]

    return AskResponse(
        question=request.question,
        answer=answer,
        sources=sources,
        retrieval_time_ms=retrieval_time_ms,
        generation_time_ms=generation_time_ms,
    )


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check service health and dependency status.

    Returns:
        HealthResponse with overall status and dependency details
    """
    qdrant_health = check_qdrant_health()
    openai_health = check_openai_health()

    dependencies = {
        "qdrant": qdrant_health,
        "openai": openai_health,
    }

    # Determine overall status
    statuses = [d.status for d in dependencies.values()]

    if all(s in ("connected", "configured") for s in statuses):
        overall_status = "healthy"
    elif qdrant_health.status == "error":
        overall_status = "unhealthy"
    else:
        overall_status = "degraded"

    return HealthResponse(
        status=overall_status,
        dependencies=dependencies,
    )


# =============================================================================
# Error Handlers
# =============================================================================


@app.exception_handler(422)
async def validation_exception_handler(request, exc):
    """Handle Pydantic validation errors."""
    return JSONResponse(
        status_code=400,
        content=ErrorResponse(
            error="validation_error",
            message="Request validation failed",
            details={"errors": exc.errors() if hasattr(exc, "errors") else str(exc)},
        ).model_dump(),
    )


# =============================================================================
# CORS Middleware
# =============================================================================

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# =============================================================================
# Main Entry Point
# =============================================================================

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
