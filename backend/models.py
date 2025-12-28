"""
Pydantic Models for RAG Chatbot API

Spec 006: Authentication + User Profiling
Purpose: Define request/response models with user context for personalization
"""

from typing import List, Literal, Optional

from pydantic import BaseModel, Field


# ============================================================================
# User Profile Models (from Better Auth session)
# ============================================================================

class UserProfile(BaseModel):
    """
    User profile for personalization.

    These fields affect response TONE ONLY, not factual content.
    RAG grounding is preserved - all facts come from Qdrant context.

    Fields match Better Auth additionalFields (camelCase in DB → snake_case in API).
    """
    programming_level: Literal["beginner", "intermediate", "advanced"] = Field(
        ...,
        description="User's programming experience level"
    )
    technologies: List[str] = Field(
        default_factory=list,
        description="Technologies the user is familiar with"
    )
    hardware_access: Literal["none", "simulator_only", "real_robots"] = Field(
        ...,
        description="User's hardware access level"
    )


# ============================================================================
# API Request/Response Models
# ============================================================================

class AskRequest(BaseModel):
    """
    Request model for /ask endpoint.

    Extended with user context for Spec 006 (Authentication + Profiling).
    user_id and user_profile are REQUIRED for authenticated requests.
    """
    question: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="The question to ask"
    )
    context: Optional[str] = Field(
        None,
        max_length=5000,
        description="Optional selected text context from the book"
    )
    top_k: int = Field(
        5,
        ge=1,
        le=10,
        description="Number of sources to retrieve"
    )
    # User context (required for authenticated requests)
    user_id: str = Field(
        ...,
        description="Authenticated user ID from Better Auth session"
    )
    user_profile: UserProfile = Field(
        ...,
        description="User profile for response personalization"
    )


class Source(BaseModel):
    """A source citation from the RAG retrieval."""
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
