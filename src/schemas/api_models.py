from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID
import uuid
from datetime import datetime

class ChatRequest(BaseModel):
    """
    Request model for chat endpoint
    """
    message: str = Field(..., description="The user's message/question", example="What is physical AI?")
    session_id: Optional[str] = Field(None, description="The session ID (optional, will be created if not provided)")
    page_url: Optional[str] = Field(None, description="The URL of the current page (for context)", example="/docs/chapter-3/physical-ai-principles")
    selected_text: Optional[str] = Field(None, description="Text selected by the user (optional, for context-specific questions)")


class SelectedTextChatRequest(BaseModel):
    """
    Request model for chat/selected endpoint
    """
    message: str = Field(..., description="The user's question about the selected text", example="Can you explain this concept more clearly?")
    selected_text: str = Field(..., description="The text selected by the user", example="Physical AI is the integration of artificial intelligence with physical systems...")
    session_id: Optional[str] = Field(None, description="The session ID (optional, will be created if not provided)", example="550e8400-e29b-41d4-a716-446655440000")
    page_url: Optional[str] = Field(None, description="The URL of the page where text was selected", example="/docs/chapter-3/physical-ai-principles")


class Source(BaseModel):
    """
    Model for sources used in responses
    """
    section_id: str = Field(..., description="Unique identifier for the content section", example="chapter-3-section-2")
    title: str = Field(..., description="Title of the source content", example="Introduction to Physical AI")
    page_url: str = Field(..., description="URL to the source page", example="/docs/chapter-3/section-2")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score of the source to the query", example=0.92)


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint
    """
    response: str = Field(..., description="The chatbot's response to the user's message", example="Physical AI is the integration of artificial intelligence with physical systems...")
    session_id: str = Field(..., description="The session ID", example="550e8400-e29b-41d4-a716-446655440000")
    sources: List[Source] = Field(default_factory=list, description="List of sources used to generate the response")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="The timestamp of the response")
    confidence_score: Optional[float] = Field(None, ge=0.0, le=1.0, description="Confidence score of the response", example=0.85)


class Message(BaseModel):
    """
    Model for individual messages in a session
    """
    id: UUID = Field(default_factory=uuid.uuid4, description="Unique identifier for the message")
    role: str = Field(..., description="The role of the message sender", example="user")
    content: str = Field(..., description="The content of the message", example="What is the main principle of physical AI?")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the message was created")
    sources: Optional[List[Source]] = Field(default_factory=list, description="Sources used in the assistant's response")


class SessionHistoryResponse(BaseModel):
    """
    Response model for session history endpoint
    """
    session_id: str = Field(..., description="The session ID", example="550e8400-e29b-41d4-a716-446655440000")
    messages: List[Message] = Field(default_factory=list, description="List of messages in the session")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the session was created")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="When the session was last updated")


class SearchResult(BaseModel):
    """
    Model for search results
    """
    id: str = Field(..., description="Unique identifier for the content", example="550e8400-e29b-41d4-a716-446655440002")
    title: str = Field(..., description="Title of the content", example="Principles of Physical AI")
    content_preview: str = Field(..., description="Preview of the content (first 200 characters)", example="Physical AI is the integration of artificial intelligence with physical systems...")
    page_url: str = Field(..., description="URL to the full content", example="/docs/chapter-3/section-2")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Relevance score of the content to the query", example=0.92)
    content_type: str = Field(..., description="Type of content", example="section")


class SearchResponse(BaseModel):
    """
    Response model for search endpoint
    """
    results: List[SearchResult] = Field(default_factory=list, description="Search results")
    query: str = Field(..., description="The original search query", example="physical AI principles")
    total_count: int = Field(..., description="Total number of results available", example=25)


class ErrorResponse(BaseModel):
    """
    Model for error responses
    """
    error: str = Field(..., description="Error code", example="INVALID_INPUT")
    message: str = Field(..., description="Human-readable error message", example="Message content is required")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details (optional)")


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint
    """
    status: str = Field(default="healthy", description="Health status of the service")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Timestamp of the health check")
    version: str = Field(default="1.0.0", description="API version")


class AnalyticsRequest(BaseModel):
    """
    Request model for analytics endpoints
    """
    record_type: str = Field(..., description="Type of analytics record", example="query")
    action: str = Field(..., description="The specific action taken", example="question_asked")
    user_id: Optional[str] = Field(None, description="User ID if available")
    session_id: Optional[str] = Field(None, description="Session ID")
    target_id: Optional[str] = Field(None, description="ID of the target entity")
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Additional context like timing, success metrics")


class AnalyticsResponse(BaseModel):
    """
    Response model for analytics endpoints
    """
    success: bool = Field(..., description="Whether the analytics record was created successfully")
    record_id: Optional[str] = Field(None, description="ID of the created record")
    message: str = Field(..., description="Status message", example="Analytics record created successfully")