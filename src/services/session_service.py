from typing import Optional
from sqlalchemy.orm import Session
from uuid import UUID
from src.database.repositories import UserSessionRepository, ChatMessageRepository
from src.models.database import UserSession, ChatMessage
from datetime import datetime

class SessionService:
    def __init__(self):
        pass  # We'll instantiate repositories as needed

    def get_or_create_session(self, db: Session, session_token: Optional[str] = None, user_id: Optional[UUID] = None):
        """
        Get existing session or create a new one
        """
        session_repo = UserSessionRepository(db)

        # If session token is provided, try to find existing session
        if session_token:
            session = session_repo.get_by_token(session_token)
            if session and session.is_active:
                # Update the session's last accessed time
                session_repo.update(session.id, updated_at=datetime.utcnow())
                return session

        # Create new session
        session = session_repo.create(user_id=user_id, session_token=session_token)
        return session

    def get_session_by_id(self, db: Session, session_id: UUID) -> Optional[UserSession]:
        """
        Get session by ID
        """
        session_repo = UserSessionRepository(db)
        return session_repo.get_by_id(session_id)

    def update_session(self, db: Session, session_id: UUID, **kwargs) -> Optional[UserSession]:
        """
        Update session properties
        """
        session_repo = UserSessionRepository(db)
        return session_repo.update(session_id, **kwargs)

    def delete_session(self, db: Session, session_id: UUID) -> bool:
        """
        Delete a session and all its messages
        """
        session_repo = UserSessionRepository(db)
        message_repo = ChatMessageRepository(db)

        # First delete all messages associated with this session
        # Note: In a real implementation, you might want to use foreign key constraints
        # to automatically delete messages when session is deleted
        messages = message_repo.get_by_session(session_id)
        for message in messages:
            message_repo.delete(message.id)

        # Then delete the session
        return session_repo.delete(session_id)

    def end_session(self, db: Session, session_id: UUID) -> bool:
        """
        End a session by marking it as inactive
        """
        session_repo = UserSessionRepository(db)
        updated_session = session_repo.update(session_id, is_active=False)
        return updated_session is not None

    def get_session_messages(self, db: Session, session_id: UUID) -> list[ChatMessage]:
        """
        Get all messages for a session
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.get_by_session(session_id)

    def add_message_to_session(self, db: Session, session_id: UUID, role: str, content: str,
                              metadata: dict = None, source_chunks: list = None) -> ChatMessage:
        """
        Add a message to a session
        """
        message_repo = ChatMessageRepository(db)
        return message_repo.create(
            role=role,
            content=content,
            session_id=session_id,
            metadata=metadata,
            source_chunks=source_chunks
        )

# Global instance of SessionService
session_service = SessionService()