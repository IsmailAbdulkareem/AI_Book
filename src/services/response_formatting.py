from typing import List, Dict, Any, Optional
from src.schemas.api_models import ChatResponse, Source
from src.services.citation_service import citation_service
import uuid
from datetime import datetime

class ResponseFormattingService:
    def __init__(self):
        self.citation_service = citation_service

    def format_response(self, raw_response: str, sources: List[Dict[str, Any]],
                       confidence_score: float = 0.8, session_id: Optional[str] = None) -> ChatResponse:
        """
        Format the raw response from OpenAI into the required API response model
        """
        # Format sources using the citation service
        formatted_sources = self.citation_service.format_sources(sources)

        # Create Source objects for the API response
        api_sources = []
        for source in formatted_sources:
            api_source = Source(
                section_id=source['section_id'],
                title=source['title'],
                page_url=source['page_url'],
                relevance_score=source['relevance_score']
            )
            api_sources.append(api_source)

        # Create the formatted response
        response = ChatResponse(
            response=raw_response,
            session_id=session_id or str(uuid.uuid4()),
            sources=api_sources,
            timestamp=datetime.utcnow(),
            confidence_score=confidence_score
        )

        return response

    def format_sources_for_display(self, sources: List[Dict[str, Any]],
                                  style: str = "structured") -> List[Dict[str, str]]:
        """
        Format sources for different display styles
        """
        if style == "structured":
            return [
                {
                    "section_id": source['section_id'],
                    "title": source['title'],
                    "page_url": source['page_url'],
                    "relevance": f"{source['relevance_score']:.2f}"
                }
                for source in sources
            ]
        elif style == "compact":
            return [
                {
                    "title": source['title'],
                    "url": source['page_url']
                }
                for source in sources
            ]
        else:  # default to minimal
            return [
                {
                    "section_id": source['section_id'],
                    "title": source['title']
                }
                for source in sources
            ]

    def enhance_response_with_citations(self, response: str, sources: List[Dict[str, Any]]) -> str:
        """
        Enhance the response text with in-line citations
        """
        if not sources:
            return response

        # Add numbered citations to the response
        enhanced_response = response
        for i, source in enumerate(sources, 1):
            # This is a simplified approach - in a real implementation,
            # you would identify where in the response each source is referenced
            if i == 1:
                enhanced_response += f"\n\nBased on information from the book:"

            enhanced_response += f"\n[{i}] {source['title']} (Section {source['section_id']})"

        return enhanced_response

    def calculate_confidence_score(self, sources: List[Dict[str, Any]],
                                  query_relevance_threshold: float = 0.5) -> float:
        """
        Calculate a confidence score based on source quality and relevance
        """
        if not sources:
            return 0.1  # Low confidence if no sources

        # Calculate average relevance score
        total_score = sum(source.get('relevance_score', 0) for source in sources)
        avg_score = total_score / len(sources)

        # Count highly relevant sources
        high_relevance_count = sum(1 for source in sources
                                  if source.get('relevance_score', 0) >= query_relevance_threshold)

        # Calculate confidence based on both average relevance and number of high-quality sources
        relevance_factor = avg_score
        coverage_factor = min(1.0, high_relevance_count / 3)  # Cap at 1.0, assuming 3+ good sources is excellent

        # Weighted confidence calculation
        confidence = (relevance_factor * 0.7) + (coverage_factor * 0.3)

        # Ensure confidence is between 0 and 1
        return max(0.0, min(1.0, confidence))

    def validate_response_format(self, response: ChatResponse) -> bool:
        """
        Validate that the response is properly formatted
        """
        # Check that required fields are present
        if not response.response or not response.session_id or not response.timestamp:
            return False

        # Check that sources have required fields if present
        for source in response.sources:
            if not source.section_id or not source.title or not source.page_url:
                return False

        # Check confidence score range
        if response.confidence_score is not None:
            if not (0.0 <= response.confidence_score <= 1.0):
                return False

        return True

    def create_condensed_response(self, full_response: ChatResponse,
                                 max_length: int = 1000) -> ChatResponse:
        """
        Create a condensed version of the response that fits within length limits
        """
        if len(full_response.response) <= max_length:
            return full_response

        # Create a new response with a truncated response text
        condensed_response = ChatResponse(
            response=full_response.response[:max_length] + "... [Response truncated]",
            session_id=full_response.session_id,
            sources=full_response.sources[:3],  # Limit to top 3 sources
            timestamp=full_response.timestamp,
            confidence_score=full_response.confidence_score
        )

        return condensed_response

    def add_metadata_to_response(self, response: ChatResponse, metadata: Dict[str, Any]) -> ChatResponse:
        """
        Add additional metadata to the response
        """
        # In this implementation, we're not adding metadata directly to the ChatResponse object
        # since the Pydantic model doesn't have a metadata field. This is more of a conceptual
        # function that shows how metadata could be handled in a more flexible implementation
        return response

# Global instance of ResponseFormattingService
response_formatting_service = ResponseFormattingService()