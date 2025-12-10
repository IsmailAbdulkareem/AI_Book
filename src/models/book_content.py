from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field
from uuid import UUID
import uuid
from datetime import datetime

class BookContent(BaseModel):
    """
    Represents the book's content that has been processed and stored in the vector database for retrieval.
    This is a Pydantic model for API requests/responses and data validation.
    """
    id: UUID = Field(default_factory=uuid.uuid4)
    section_id: str = Field(..., description="Identifier for the section/chapter")
    title: str = Field(..., description="Title of the section")
    content: str = Field(..., description="The actual content")
    page_url: str = Field(..., description="URL of the page in the book")
    content_type: str = Field(..., description="Type of content: chapter, section, subsection, figure, table")
    embedding_vector: List[float] = Field(..., description="Cohere embedding vector")
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict, description="Additional information like tags, difficulty level")
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        # Allow extra fields for flexibility
        extra = "allow"

        # Enable ORM mode for SQLAlchemy compatibility
        from_attributes = True

class BookContentChunk(BaseModel):
    """
    Represents a chunk of book content with its embedding and metadata
    """
    id: UUID = Field(default_factory=uuid.uuid4)
    content_id: UUID
    chunk_text: str
    chunk_index: int
    embedding_vector: List[float]
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)
    page_url: str
    section_id: str
    created_at: datetime = Field(default_factory=datetime.utcnow)

class VectorStorageInterface:
    """
    Interface for vector storage operations
    This class provides methods to interact with the vector store (Qdrant)
    """

    @staticmethod
    def validate_embedding_vector(vector: List[float], expected_size: int = 1024) -> bool:
        """
        Validate that the embedding vector has the correct size
        """
        return len(vector) == expected_size

    @staticmethod
    def normalize_embedding_vector(vector: List[float]) -> List[float]:
        """
        Normalize the embedding vector to unit length
        """
        import math
        magnitude = math.sqrt(sum(x * x for x in vector))
        if magnitude == 0:
            return vector
        return [x / magnitude for x in vector]

    @staticmethod
    def create_payload_for_content(book_content: BookContent) -> Dict[str, Any]:
        """
        Create a payload for storing in the vector database
        """
        return {
            "id": str(book_content.id),
            "section_id": book_content.section_id,
            "title": book_content.title,
            "page_url": book_content.page_url,
            "content_type": book_content.content_type,
            "content": book_content.content[:500],  # Store first 500 chars as preview
            "metadata": book_content.metadata or {}
        }

    @staticmethod
    def create_payload_for_chunk(chunk: BookContentChunk) -> Dict[str, Any]:
        """
        Create a payload for storing a content chunk in the vector database
        """
        return {
            "id": str(chunk.id),
            "content_id": str(chunk.content_id),
            "chunk_index": chunk.chunk_index,
            "page_url": chunk.page_url,
            "section_id": chunk.section_id,
            "chunk_text": chunk.chunk_text[:500],  # Store first 500 chars as preview
            "metadata": chunk.metadata or {}
        }