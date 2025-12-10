from sqlalchemy.orm import Session
from sqlalchemy import and_, or_, func
from typing import List, Optional
from uuid import UUID
from src.models.database import ChatMessage, UserSession, TextSelection, AnalyticsRecord
from datetime import datetime

class ChatMessageRepository:
    def __init__(self, db_session: Session):
        self.db_session = db_session

    def create(self, role: str, content: str, session_id: UUID, metadata: dict = None, source_chunks: List[UUID] = None) -> ChatMessage:
        db_message = ChatMessage(
            role=role,
            content=content,
            session_id=session_id,
            metadata_=metadata,
            source_chunks=source_chunks or []
        )
        self.db_session.add(db_message)
        self.db_session.commit()
        self.db_session.refresh(db_message)
        return db_message

    def get_by_session(self, session_id: UUID, limit: int = 50, offset: int = 0) -> List[ChatMessage]:
        return self.db_session.query(ChatMessage)\
            .filter(ChatMessage.session_id == session_id)\
            .order_by(ChatMessage.timestamp.desc())\
            .offset(offset).limit(limit).all()

    def get_by_id(self, message_id: UUID) -> Optional[ChatMessage]:
        return self.db_session.query(ChatMessage).filter(ChatMessage.id == message_id).first()

    def update(self, message_id: UUID, **kwargs) -> Optional[ChatMessage]:
        message = self.get_by_id(message_id)
        if message:
            for key, value in kwargs.items():
                setattr(message, key, value)
            self.db_session.commit()
            self.db_session.refresh(message)
        return message

    def delete(self, message_id: UUID) -> bool:
        message = self.get_by_id(message_id)
        if message:
            self.db_session.delete(message)
            self.db_session.commit()
            return True
        return False


class UserSessionRepository:
    def __init__(self, db_session: Session):
        self.db_session = db_session

    def create(self, user_id: Optional[UUID] = None, session_token: Optional[str] = None, metadata: dict = None) -> UserSession:
        db_session = UserSession(
            user_id=user_id,
            session_token=session_token,
            metadata_=metadata
        )
        self.db_session.add(db_session)
        self.db_session.commit()
        self.db_session.refresh(db_session)
        return db_session

    def get_by_id(self, session_id: UUID) -> Optional[UserSession]:
        return self.db_session.query(UserSession).filter(UserSession.id == session_id).first()

    def get_by_token(self, session_token: str) -> Optional[UserSession]:
        return self.db_session.query(UserSession).filter(UserSession.session_token == session_token).first()

    def get_by_user_id(self, user_id: UUID) -> List[UserSession]:
        return self.db_session.query(UserSession).filter(UserSession.user_id == user_id).all()

    def update(self, session_id: UUID, **kwargs) -> Optional[UserSession]:
        session = self.get_by_id(session_id)
        if session:
            for key, value in kwargs.items():
                setattr(session, key, value)
            self.db_session.commit()
            self.db_session.refresh(session)
        return session

    def delete(self, session_id: UUID) -> bool:
        session = self.get_by_id(session_id)
        if session:
            self.db_session.delete(session)
            self.db_session.commit()
            return True
        return False


class TextSelectionRepository:
    def __init__(self, db_session: Session):
        self.db_session = db_session

    def create(self, session_id: UUID, selected_text: str, page_url: str, selection_bounds: dict = None, query_id: Optional[UUID] = None) -> TextSelection:
        db_selection = TextSelection(
            session_id=session_id,
            selected_text=selected_text,
            page_url=page_url,
            selection_bounds=selection_bounds,
            query_id=query_id
        )
        self.db_session.add(db_selection)
        self.db_session.commit()
        self.db_session.refresh(db_selection)
        return db_selection

    def get_by_session(self, session_id: UUID) -> List[TextSelection]:
        return self.db_session.query(TextSelection).filter(TextSelection.session_id == session_id).all()

    def get_by_id(self, selection_id: UUID) -> Optional[TextSelection]:
        return self.db_session.query(TextSelection).filter(TextSelection.id == selection_id).first()

    def get_by_query_id(self, query_id: UUID) -> Optional[TextSelection]:
        return self.db_session.query(TextSelection).filter(TextSelection.query_id == query_id).first()


class AnalyticsRecordRepository:
    def __init__(self, db_session: Session):
        self.db_session = db_session

    def create(self, record_type: str, action: str, user_id: Optional[UUID] = None,
               session_id: Optional[UUID] = None, target_id: Optional[UUID] = None,
               metadata: dict = None) -> AnalyticsRecord:
        db_record = AnalyticsRecord(
            record_type=record_type,
            user_id=user_id,
            session_id=session_id,
            action=action,
            target_id=target_id,
            metadata_=metadata
        )
        self.db_session.add(db_record)
        self.db_session.commit()
        self.db_session.refresh(db_record)
        return db_record

    def get_by_session(self, session_id: UUID) -> List[AnalyticsRecord]:
        return self.db_session.query(AnalyticsRecord).filter(AnalyticsRecord.session_id == session_id).all()

    def get_by_user(self, user_id: UUID) -> List[AnalyticsRecord]:
        return self.db_session.query(AnalyticsRecord).filter(AnalyticsRecord.user_id == user_id).all()

    def get_by_type(self, record_type: str) -> List[AnalyticsRecord]:
        return self.db_session.query(AnalyticsRecord).filter(AnalyticsRecord.record_type == record_type).all()

    def get_by_date_range(self, start_date: datetime, end_date: datetime) -> List[AnalyticsRecord]:
        return self.db_session.query(AnalyticsRecord)\
            .filter(AnalyticsRecord.timestamp >= start_date, AnalyticsRecord.timestamp <= end_date)\
            .all()

    def get_by_action(self, action: str) -> List[AnalyticsRecord]:
        return self.db_session.query(AnalyticsRecord).filter(AnalyticsRecord.action == action).all()