"""
FastAPI Server - HTTP API for RAG Chatbot

Spec 5: 005-frontend-chatbot-ui
Purpose: HTTP API endpoint for the frontend chatbot UI

This module provides:
- POST /ask endpoint for questions
- GET /health endpoint for health checks
- CORS support for frontend access

Usage:
    cd database
    uv run uvicorn api:app --reload --port 8000
"""

import os
import time
from typing import List, Optional

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from openai import OpenAI

from retrieval import QueryResult, RetrievalPipeline

# Load environment variables
load_dotenv()

# =============================================================================
# FastAPI App
# =============================================================================

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI & Humanoid Robotics Book Chatbot",
    version="1.0.0",
)

# CORS - Allow frontend to access the API
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================================================
# Models
# =============================================================================


class AskRequest(BaseModel):
    """Request model for /ask endpoint."""
    question: str = Field(..., min_length=1, description="The question to ask")
    context: Optional[str] = Field(None, description="Optional selected text context")
    top_k: int = Field(5, ge=1, le=10, description="Number of sources to retrieve")


class Source(BaseModel):
    """A source citation."""
    url: str
    content: str


class AskResponse(BaseModel):
    """Response model for /ask endpoint."""
    question: str
    answer: str
    sources: List[Source]
    retrieval_time_ms: int
    generation_time_ms: int


class HealthResponse(BaseModel):
    """Response model for /health endpoint."""
    status: str
    message: str


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
# Helper Functions
# =============================================================================


def format_context(results: List[QueryResult], user_context: Optional[str] = None) -> str:
    """Format retrieved results as numbered context for the LLM."""
    context_parts = []

    # Add user-provided context first if available
    if user_context:
        context_parts.append(f"[User Selected Text]\n{user_context}")

    # Add retrieved results
    for i, result in enumerate(results, 1):
        context_parts.append(f"[{i}] {result.content}\nSource: {result.url}")

    return "\n\n".join(context_parts) if context_parts else ""


def generate_answer(question: str, context: str) -> tuple[str, int]:
    """Generate a grounded answer using OpenAI."""
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


# =============================================================================
# Endpoints
# =============================================================================


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint."""
    return HealthResponse(status="ok", message="RAG Chatbot API is running")


@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    """
    Ask a question about the book content.

    The API will:
    1. Search for relevant content in the vector database
    2. Generate a grounded answer using OpenAI
    3. Return the answer with source citations
    """
    try:
        # Initialize retrieval pipeline
        pipeline = RetrievalPipeline()
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Vector database unavailable: {str(e)}")

    # Search for relevant context
    # If user provided selected text context, combine it with the question for better retrieval
    try:
        start_time = time.time()

        # Build search query: include context for better semantic search
        if request.context:
            # Combine selected text with question for more relevant retrieval
            search_query = f"{request.context}\n\nQuestion: {request.question}"
        else:
            search_query = request.question

        response = pipeline.search(query=search_query, top_k=request.top_k)
        retrieval_time_ms = int((time.time() - start_time) * 1000)
        results = response.results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Retrieval failed: {str(e)}")

    # Format context (include user-provided context if any)
    context = format_context(results, request.context)

    # Generate answer
    try:
        answer, generation_time_ms = generate_answer(request.question, context)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Generation failed: {str(e)}")

    # Build sources list
    sources = [
        Source(url=r.url, content=r.content[:200])  # Truncate content for response
        for r in results
    ]

    return AskResponse(
        question=request.question,
        answer=answer,
        sources=sources,
        retrieval_time_ms=retrieval_time_ms,
        generation_time_ms=generation_time_ms,
    )


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    import uvicorn

    # Verify required environment variables
    if not os.getenv("OPENAI_API_KEY"):
        print("❌ Error: OPENAI_API_KEY environment variable not set.")
        exit(1)

    if not os.getenv("QDRANT_URL"):
        print("⚠️  Warning: QDRANT_URL not set, using default localhost:6333")

    print("🚀 Starting RAG Chatbot API server...")
    uvicorn.run(app, host="0.0.0.0", port=8000)
