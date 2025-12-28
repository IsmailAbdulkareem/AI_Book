"""
FastAPI Server - HTTP API for RAG Chatbot

Spec 5: 005-frontend-chatbot-ui
Spec 6: 006-user-auth-profiling (user context personalization)

Purpose: HTTP API endpoint for the frontend chatbot UI

This module provides:
- POST /ask endpoint for questions (with user context for personalization)
- GET /health endpoint for health checks
- CORS support for frontend access

Usage:
    cd database
    uv run uvicorn api:app --reload --port 8000
"""

import os
import re
import time
from typing import List, Optional
from enum import Enum

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from openai import OpenAI

from retrieval import QueryResult, RetrievalPipeline
from models import UserProfile


# =============================================================================
# Intent Detection
# =============================================================================

class MessageIntent(str, Enum):
    """Classification of user message intent."""
    GREETING = "greeting"
    META = "meta"  # Questions about the chatbot itself
    CONTENT = "content"  # Actual book-related questions


# Greeting patterns (case-insensitive)
GREETING_PATTERNS = [
    r"^hi$", r"^hello$", r"^hey$", r"^howdy$",
    r"^hi[!.,\s]", r"^hello[!.,\s]", r"^hey[!.,\s]",
    r"^good\s*(morning|afternoon|evening|day)",
    r"^greetings",
    r"^thanks?$", r"^thank\s*you",
    r"^ok$", r"^okay$", r"^sure$", r"^yes$", r"^no$",
    r"^cool$", r"^nice$", r"^great$", r"^good$", r"^awesome$",
    r"^bye$", r"^goodbye$", r"^see\s*you",
    r"^sup$", r"^what'?s\s*up",
]

# Meta question patterns (questions about the chatbot)
META_PATTERNS = [
    r"what\s+can\s+you\s+do",
    r"how\s+do(es)?\s+(this|you)\s+work",
    r"what\s+are\s+you",
    r"who\s+are\s+you",
    r"what\s+is\s+this",
    r"help\s*$", r"^help\s+me",
    r"what\s+topics?\s+(can|do)\s+you",
    r"what\s+questions?\s+can\s+i\s+ask",
    r"how\s+can\s+you\s+help",
    r"what\s+do\s+you\s+know",
    r"tell\s+me\s+about\s+(yourself|you)",
]

def detect_intent(message: str) -> MessageIntent:
    """Detect the intent of a user message.

    Args:
        message: The user's message text

    Returns:
        MessageIntent indicating if it's a greeting, meta question, or content question
    """
    # Normalize message
    normalized = message.lower().strip()

    # Check for greetings (short messages or greeting patterns)
    if len(normalized) <= 15:  # Short messages are likely greetings
        for pattern in GREETING_PATTERNS:
            if re.search(pattern, normalized, re.IGNORECASE):
                return MessageIntent.GREETING

    # Check for meta questions about the chatbot
    for pattern in META_PATTERNS:
        if re.search(pattern, normalized, re.IGNORECASE):
            return MessageIntent.META

    # Default to content question (will use RAG)
    return MessageIntent.CONTENT


def generate_conversational_response(message: str, intent: MessageIntent) -> tuple[str, int]:
    """Generate a natural conversational response using OpenAI for greetings/meta questions.

    Args:
        message: The user's message
        intent: The detected intent (GREETING or META)

    Returns:
        Tuple of (response text, generation time in ms)
    """
    client = OpenAI()
    start_time = time.time()

    if intent == MessageIntent.GREETING:
        system_prompt = """You are a friendly assistant for the Physical AI & Humanoid Robotics book.

The user is greeting you or making casual conversation. Respond naturally and warmly, then invite them to ask questions about the book.

Keep your response brief (1-2 sentences). Be friendly and conversational.
Remember: You can ONLY answer questions about this specific book's content."""
    else:  # META
        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics book.

The user is asking about your capabilities or how you work. Explain clearly:
- You answer questions ONLY about this specific book
- Your answers are grounded in the book content with citations
- You cannot answer questions outside the book's scope
- You cannot make up information

Be helpful and encourage them to ask questions about the book."""

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message},
        ],
        temperature=0.7,  # Slightly more creative for conversational responses
        max_tokens=200,
    )

    generation_time_ms = int((time.time() - start_time) * 1000)
    answer = response.choices[0].message.content or ""

    return answer, generation_time_ms

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
    """
    Request model for /ask endpoint.

    Spec 006: Extended with user_id and user_profile for personalization.
    user_id and user_profile are REQUIRED for authenticated requests.
    """
    question: str = Field(..., min_length=1, description="The question to ask")
    context: Optional[str] = Field(None, description="Optional selected text context")
    top_k: int = Field(5, ge=1, le=10, description="Number of sources to retrieve")
    # User context for personalization (Spec 006)
    user_id: str = Field(..., description="Authenticated user ID from Better Auth session")
    user_profile: UserProfile = Field(..., description="User profile for response personalization")


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
6. When [User Selected Text] is provided, the user is asking about THAT specific text. Commands like "explain", "summarize", "what is this" refer to the selected text.

Context will be provided in the format:
[User Selected Text] (optional - text the user highlighted on the page)
The selected text...

[1] Content from the book...
Source: URL

Use this context to answer the user's question. If the user selected text and asks to "explain" or similar, explain that selected text using the book context."""


def build_personalized_system_prompt(user_profile: UserProfile) -> str:
    """
    Build a personalized system prompt based on user profile.

    RAG SAFETY GUARDRAIL (Non-Negotiable):
    - Profile affects ONLY tone, explanation depth, and examples
    - Profile MUST NOT add facts, change retrieval, or override citations
    - ALL facts must come from the retrieved context

    Args:
        user_profile: User profile containing programming level, technologies, hardware access

    Returns:
        Extended system prompt with personalization instructions
    """
    technologies_str = ", ".join(user_profile.technologies) if user_profile.technologies else "None specified"

    personalization = f"""

PERSONALIZATION CONTEXT (affects tone only, NOT facts):
- User programming level: {user_profile.programming_level}
- Familiar technologies: {technologies_str}
- Hardware access: {user_profile.hardware_access}

Adjust your explanations based on user level:
- For beginners: Use simpler language, provide more step-by-step detail, explain terminology
- For intermediate: Balance detail with conciseness, assume basic familiarity
- For advanced: Be concise, assume familiarity with concepts, focus on specifics

Adjust examples based on hardware access:
- For none: Focus on conceptual understanding
- For simulator_only: Prioritize simulation-based workflows and examples
- For real_robots: Include practical hardware deployment considerations

CRITICAL RAG SAFETY RULE:
- Do NOT invent information based on user profile
- ALL facts MUST come from the provided context
- If unsure, default to the retrieved context only
- Never say "since you have X hardware, you can try Y" unless Y is in the context"""

    return SYSTEM_PROMPT + personalization


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


def generate_answer(
    question: str,
    context: str,
    has_user_selection: bool = False,
    user_profile: Optional[UserProfile] = None
) -> tuple[str, int]:
    """
    Generate a grounded answer using OpenAI.

    Args:
        question: User's question
        context: Retrieved context from RAG
        has_user_selection: Whether user selected text
        user_profile: Optional user profile for personalization (Spec 006)

    Returns:
        Tuple of (answer, generation_time_ms)
    """
    client = OpenAI()

    start_time = time.time()

    if not context:
        user_content = f"Question: {question}\n\nNote: No relevant context was found in the book."
    else:
        # If user selected text and asked a short command, make the intent clear
        if has_user_selection and len(question.split()) <= 3:
            user_content = f"Context:\n{context}\n\nThe user selected some text and wants you to: {question}"
        else:
            user_content = f"Context:\n{context}\n\nQuestion: {question}"

    # Build system prompt with personalization if user profile provided
    system_prompt = (
        build_personalized_system_prompt(user_profile)
        if user_profile
        else SYSTEM_PROMPT
    )

    response = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role": "system", "content": system_prompt},
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

    Spec 006: Extended with user context for personalized responses.

    This endpoint includes intent detection to handle:
    - Greetings: Returns friendly responses without RAG
    - Meta questions: Explains the chatbot's capabilities
    - Content questions: Uses full RAG pipeline with personalization

    Personalization affects TONE ONLY, not factual content.
    RAG grounding is preserved - all facts come from retrieved context.
    """
    # Spec 006: Validate user context
    if not request.user_id:
        raise HTTPException(status_code=401, detail="user_id is required")
    if not request.user_profile:
        raise HTTPException(status_code=401, detail="user_profile is required")

    # ==========================================================================
    # Step 1: Intent Detection - Route greetings and meta questions
    # ==========================================================================
    intent = detect_intent(request.question)

    # Handle greetings and meta questions - use LLM but no RAG
    if intent in (MessageIntent.GREETING, MessageIntent.META):
        try:
            answer, generation_time_ms = generate_conversational_response(
                request.question, intent
            )
            return AskResponse(
                question=request.question,
                answer=answer,
                sources=[],
                retrieval_time_ms=0,
                generation_time_ms=generation_time_ms,
            )
        except Exception as e:
            # Fallback if OpenAI fails for conversational response
            raise HTTPException(status_code=500, detail=f"Generation failed: {str(e)}")

    # ==========================================================================
    # Step 2: Content questions - Use full RAG pipeline
    # ==========================================================================

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

    # Generate answer with personalization (Spec 006)
    try:
        answer, generation_time_ms = generate_answer(
            request.question,
            context,
            has_user_selection=bool(request.context),
            user_profile=request.user_profile  # Pass profile for personalization
        )
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
