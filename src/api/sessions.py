from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import uuid
from src.database.connection import get_db
from src.schemas.api_models import SessionHistoryResponse
from src.database.repositories import ChatMessageRepository
from src.utils.validation import validate_uuid
from src.logging_config import get_logger

router = APIRouter()
logger = get_logger(__name__)

@router.get("/sessions/{session_id}", response_model=SessionHistoryResponse)
async def get_session_history(
    session_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve the history of messages in a chat session
    """
    try:
        # Validate session ID
        if not validate_uuid(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        session_uuid = uuid.UUID(session_id)

        # Get chat messages for the session
        message_repo = ChatMessageRepository(db)
        messages = message_repo.get_by_session(session_uuid)

        # Convert to response format
        formatted_messages = []
        for msg in messages:
            formatted_messages.append({
                "id": msg.id,
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp,
                "sources": []  # In a real implementation, you'd retrieve sources
            })

        response = SessionHistoryResponse(
            session_id=session_id,
            messages=formatted_messages,
            created_at=min([msg.timestamp for msg in messages]) if messages else None,
            updated_at=max([msg.timestamp for msg in messages]) if messages else None
        )

        logger.info(f"Session history retrieved for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error retrieving session history: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")