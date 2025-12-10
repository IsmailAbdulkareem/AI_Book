from typing import List, Dict, Any, Optional
from src.services.content_retrieval import content_retrieval_service
from src.schemas.api_models import ChatResponse
from src.logging_config import get_logger
import uuid
from datetime import datetime

logger = get_logger(__name__)

class ErrorHandlingService:
    def __init__(self):
        self.content_retrieval = content_retrieval_service

    def handle_no_content_found(self, query: str, session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle the case where no relevant content is found in the book
        """
        logger.warning(f"No content found for query: {query}")

        # Create a response indicating that no relevant content was found
        response_text = (
            "I couldn't find any relevant information in the book about your query. "
            "The Physical AI & Humanoid Robotics book may not contain information on this specific topic. "
            "Please try rephrasing your question or consult other resources for this information."
        )

        # Return a ChatResponse with no sources and low confidence
        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.1
        )

    def handle_low_relevance_content(self, query: str, retrieved_content: List[Dict[str, Any]],
                                   min_score_threshold: float = 0.3,
                                   session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle the case where content is found but has low relevance scores
        """
        logger.info(f"Low relevance content found for query: {query}")

        # Determine the highest relevance score
        max_score = max([item.get('score', 0) for item in retrieved_content]) if retrieved_content else 0

        response_text = (
            f"I found some content related to your query, but the relevance is low (highest score: {max_score:.2f}). "
            f"The information might not be fully accurate or comprehensive. "
            f"Please verify with the book directly or try a different search term."
        )

        # Return a response with low confidence and the low-relevance sources
        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],  # In a real implementation, you might still include sources but with a warning
            timestamp=datetime.utcnow(),
            confidence_score=max_score
        )

    def handle_query_processing_error(self, query: str, error: Exception,
                                    session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle errors that occur during query processing
        """
        logger.error(f"Error processing query '{query}': {str(error)}")

        # Create a response indicating that an error occurred
        response_text = (
            "An error occurred while processing your query. "
            "Please try again later, or rephrase your question. "
            "If the problem persists, the system may need maintenance."
        )

        # Return a response with no sources and low confidence
        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.05
        )

    def handle_api_connection_error(self, service_name: str,
                                  session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle errors connecting to external APIs (Cohere, OpenAI, Qdrant)
        """
        logger.error(f"Connection error with {service_name} API")

        response_text = (
            f"The system is currently unable to connect to the {service_name} service. "
            f"This may be due to network issues or service unavailability. "
            f"Please try again later."
        )

        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.0
        )

    def handle_database_error(self, operation: str,
                            session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle errors that occur during database operations
        """
        logger.error(f"Database error during {operation}")

        response_text = (
            f"A database error occurred while processing your request. "
            f"The system is temporarily unable to access stored information. "
            f"Please try again later."
        )

        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.05
        )

    def handle_invalid_input_error(self, input_text: str,
                                 error_details: str,
                                 session_id: Optional[str] = None) -> ChatResponse:
        """
        Handle errors caused by invalid user input
        """
        logger.warning(f"Invalid input: {input_text}, error: {error_details}")

        response_text = (
            f"Your input appears to be invalid: {error_details}. "
            f"Please check your input and try again with a valid question or text selection."
        )

        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.1
        )

    def validate_content_before_response(self, query: str, content: List[Dict[str, Any]],
                                       min_content_length: int = 50) -> tuple[bool, str, List[Dict[str, Any]]]:
        """
        Validate retrieved content before generating a response
        """
        if not content:
            return False, "no_content", []

        # Check if content is relevant and substantial
        valid_content = []
        for item in content:
            payload = item.get('payload', {})
            content_text = payload.get('content', '')
            relevance_score = item.get('score', 0)

            # Check if content is substantial and relevant
            if len(content_text) >= min_content_length and relevance_score > 0.1:
                valid_content.append(item)

        if not valid_content:
            return False, "low_quality_content", content

        return True, "valid", valid_content

    def create_fallback_response(self, original_query: str,
                               fallback_reason: str,
                               session_id: Optional[str] = None) -> ChatResponse:
        """
        Create a fallback response when normal processing fails
        """
        logger.info(f"Using fallback response for query: {original_query}, reason: {fallback_reason}")

        fallback_responses = {
            "no_content": "I couldn't find specific information about this topic in the book. The Physical AI & Humanoid Robotics book may not cover this particular subject in detail.",
            "low_relevance": "I found some related information, but it might not fully address your question. The content relevance is low.",
            "processing_error": "I encountered an issue while processing your request. Please try rephrasing your question.",
            "api_error": "I'm having trouble connecting to the services needed to answer your question. Please try again later."
        }

        response_text = fallback_responses.get(fallback_reason, fallback_responses["processing_error"])

        return ChatResponse(
            response=response_text,
            session_id=session_id or str(uuid.uuid4()),
            sources=[],
            timestamp=datetime.utcnow(),
            confidence_score=0.2 if fallback_reason != "api_error" else 0.0
        )

# Global instance of ErrorHandlingService
error_handling_service = ErrorHandlingService()