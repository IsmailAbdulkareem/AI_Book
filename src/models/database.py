from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, JSON, ARRAY
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid
from datetime import datetime

Base = declarative_base()

class ChatMessage(Base):
    """
    Represents a single message in a conversation between user and system.
    """
    __tablename__ = "chat_messages"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("user_sessions.id"), nullable=False)
    role = Column(String(20), nullable=False)  # "user" or "assistant"
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
    metadata_ = Column("metadata", JSON)  # Additional context like page_url, selected_text
    source_chunks = Column(ARRAY(PG_UUID(as_uuid=True)))  # References to BookContent chunks used in response

    # Validation
    __table_args__ = (
        # Check constraint to ensure role is either "user" or "assistant"
        # Note: SQLAlchemy doesn't support check constraints directly in column definitions
        # This validation should be handled in the application layer
    )

class UserSession(Base):
    """
    Represents a user's chat session with conversation history and context.
    """
    __tablename__ = "user_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(PG_UUID(as_uuid=True), nullable=True)  # For registered users
    session_token = Column(String(255), unique=True, nullable=True)  # For anonymous users
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    is_active = Column(Boolean, default=True, nullable=False)
    metadata_ = Column("metadata", JSON)  # User preferences, current page, etc.

    # Validation: Either user_id or session_token must be present
    # This validation should be handled in the application layer

class TextSelection(Base):
    """
    Represents a user's selected text on a page when asking context-specific questions.
    """
    __tablename__ = "text_selections"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("user_sessions.id"), nullable=False)
    selected_text = Column(Text, nullable=False)
    page_url = Column(String(500), nullable=False)
    selection_bounds = Column(JSON)  # Start/end positions, coordinates
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
    query_id = Column(PG_UUID(as_uuid=True), ForeignKey("chat_messages.id"))  # Foreign key to ChatMessage that used this selection

class AnalyticsRecord(Base):
    """
    Represents aggregated data about user interactions for educational insights.
    """
    __tablename__ = "analytics_records"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    record_type = Column(String(50), nullable=False)  # "chat_session", "query", "engagement", "feedback"
    user_id = Column(PG_UUID(as_uuid=True), nullable=True)
    session_id = Column(PG_UUID(as_uuid=True), nullable=True)
    action = Column(String(100), nullable=False)  # The specific action taken
    target_id = Column(PG_UUID(as_uuid=True))  # ID of the target entity (e.g., ChatMessage ID)
    metadata_ = Column("metadata", JSON)  # Additional context like timing, success metrics
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Validation: Either user_id or session_id must be present
    # This validation should be handled in the application layer