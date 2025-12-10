from typing import List, Optional
from sqlalchemy.orm import Session
from uuid import UUID
from src.database.repositories import ChatMessageRepository
from src.models.database import ChatMessage

class MessageService:
    def __init__(self):
        pass  # We'll instantiate repositories as needed

    def create_message(self, db: Session, session_id: UUID, role: str, content: str,
                      metadata: dict = None, source_chunks: List[UUID] = None) -> ChatMessage:
        """
        Create a new chat message
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.create(
            role=role,
            content=content,
            session_id=session_id,
            metadata=metadata,
            source_chunks=source_chunks
        )

    def get_message_by_id(self, db: Session, message_id: UUID) -> Optional[ChatMessage]:
        """
        Get a message by its ID
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.get_by_id(message_id)

    def get_messages_by_session(self, db: Session, session_id: UUID, limit: int = 50, offset: int = 0) -> List[ChatMessage]:
        """
        Get all messages for a session
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.get_by_session(session_id, limit=limit, offset=offset)

    def update_message(self, db: Session, message_id: UUID, **kwargs) -> Optional[ChatMessage]:
        """
        Update a message
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.update(message_id, **kwargs)

    def delete_message(self, db: Session, message_id: UUID) -> bool:
        """
        Delete a message
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.delete(message_id)

    def get_conversation_context(self, db: Session, session_id: UUID, limit: int = 10) -> List[ChatMessage]:
        """
        Get the most recent messages for conversation context
        """
        message_repo = ChatMessageRepository(db)
        messages = message_repo.get_by_session(session_id, limit=limit)
        # Return messages in chronological order (oldest first)
        return list(reversed(messages))

# Global instance of MessageService
message_service = MessageService()