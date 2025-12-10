from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
import uuid
from src.database.connection import get_db
from src.schemas.api_models import ChatRequest, ChatResponse, SelectedTextChatRequest
from src.services.rag_service import rag_service
from src.services.session_service import session_service
from src.utils.validation import validate_message_content, sanitize_text
from src.middleware.rate_limit import rate_limit
from src.logging_config import get_logger

router = APIRouter()
logger = get_logger(__name__)

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Endpoint for general questions about book content
    """
    try:
        # Validate the request
        is_valid, error_msg = validate_message_content(request.message)
        if not is_valid:
            raise HTTPException(status_code=400, detail=error_msg)

        # Sanitize the input
        sanitized_message = sanitize_text(request.message)

        # Get or create session
        session_id = request.session_id or str(uuid.uuid4())

        # If selected_text is provided, use the selected text flow
        if request.selected_text:
            # Sanitize selected text
            sanitized_selected_text = sanitize_text(request.selected_text)

            # Process with selected text
            result = rag_service.query_with_selected_text(
                query=sanitized_message,
                selected_text=sanitized_selected_text,
                session_id=uuid.UUID(session_id) if is_valid_uuid(session_id) else None
            )
        else:
            # Process general query
            result = rag_service.query(
                query=sanitized_message,
                session_id=uuid.UUID(session_id) if is_valid_uuid(session_id) else None,
                filters={"page_url": request.page_url} if request.page_url else None
            )

        # Create response
        response = ChatResponse(
            response=result["response"],
            session_id=result["session_id"],
            sources=result["sources"],
            timestamp=result["timestamp"],
            confidence_score=result["confidence_score"]
        )

        logger.info(f"Chat response generated for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/chat/selected", response_model=ChatResponse)
async def chat_selected_endpoint(
    request: SelectedTextChatRequest,
    db: Session = Depends(get_db)
):
    """
    Endpoint for questions about selected text
    """
    try:
        # Validate the request
        is_valid, error_msg = validate_message_content(request.message)
        if not is_valid:
            raise HTTPException(status_code=400, detail=error_msg)

        # Validate selected text length
        from src.utils.validation import validate_selected_text_length
        is_valid, error_msg = validate_selected_text_length(request.selected_text)
        if not is_valid:
            raise HTTPException(status_code=400, detail=error_msg)

        # Sanitize inputs
        sanitized_message = sanitize_text(request.message)
        sanitized_selected_text = sanitize_text(request.selected_text)

        # Get or create session
        session_id = request.session_id or str(uuid.uuid4())

        # Process with selected text
        result = rag_service.query_with_selected_text(
            query=sanitized_message,
            selected_text=sanitized_selected_text,
            session_id=uuid.UUID(session_id) if is_valid_uuid(session_id) else None
        )

        # Create response
        response = ChatResponse(
            response=result["response"],
            session_id=result["session_id"],
            sources=result["sources"],
            timestamp=result["timestamp"],
            confidence_score=result["confidence_score"]
        )

        logger.info(f"Selected text chat response generated for session {session_id}")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in selected text chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


def is_valid_uuid(uuid_string: str) -> bool:
    """
    Check if a string is a valid UUID
    """
    try:
        uuid.UUID(uuid_string)
        return True
    except ValueError:
        return False