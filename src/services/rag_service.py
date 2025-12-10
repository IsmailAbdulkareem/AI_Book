from typing import List, Dict, Any, Optional
from src.services.embedding_service import embedding_service
from src.ai.openai_client import openai_client
from src.vector_store.qdrant_client import qdrant_manager
from src.models.database import ChatMessage
from src.database.repositories import ChatMessageRepository
from src.config import settings
from uuid import UUID
import uuid

class RAGService:
    def __init__(self):
        self.embedding_service = embedding_service
        self.openai_client = openai_client
        self.vector_store = qdrant_manager

    def retrieve_context(self, query: str, limit: int = 5, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the vector store based on the query
        """
        try:
            # Search for similar content in the vector store
            results = self.embedding_service.search_similar_content(
                query=query,
                limit=limit,
                filters=filters
            )

            return results
        except Exception as e:
            print(f"Error retrieving context: {e}")
            raise

    def generate_answer(self, query: str, context: List[Dict[str, Any]],
                       session_id: Optional[UUID] = None,
                       selected_text: Optional[str] = None) -> Dict[str, Any]:
        """
        Generate an answer based on the query and retrieved context
        """
        try:
            # Prepare the context for the LLM
            context_text = ""
            sources = []

            for item in context:
                payload = item.get('payload', {})
                content_preview = payload.get('content', '')[:500]  # Get first 500 chars
                context_text += f"Section: {payload.get('section_id', 'N/A')}\n"
                context_text += f"Content: {content_preview}\n\n"

                # Add source information
                sources.append({
                    "section_id": payload.get('section_id', 'N/A'),
                    "title": payload.get('title', 'N/A'),
                    "page_url": payload.get('page_url', 'N/A'),
                    "relevance_score": item.get('score', 0.0)
                })

            # If selected text is provided, use it as the primary context
            if selected_text:
                context_text = f"Selected text for context:\n{selected_text}\n\n"
                context_text += "Additional context from book:\n"
                for item in context:
                    payload = item.get('payload', {})
                    content_preview = payload.get('content', '')[:300]
                    context_text += f"Section: {payload.get('section_id', 'N/A')}\n"
                    context_text += f"Content: {content_preview}\n\n"

            # Generate the answer using OpenAI
            response = self.openai_client.generate_response(
                prompt=query,
                context=context_text,
                sources=sources
            )

            return response
        except Exception as e:
            print(f"Error generating answer: {e}")
            raise

    def query(self, query: str, session_id: Optional[UUID] = None,
              selected_text: Optional[str] = None,
              filters: Optional[Dict] = None) -> Dict[str, Any]:
        """
        Main RAG query method that retrieves context and generates an answer
        """
        try:
            # Retrieve relevant context from the vector store
            context = self.retrieve_context(query, filters=filters)

            # Generate answer based on context
            response = self.generate_answer(query, context, session_id, selected_text)

            # Prepare the final response
            result = {
                "response": response["response"],
                "sources": response["sources"],
                "confidence_score": response.get("confidence_score", 0.8),
                "session_id": str(session_id) if session_id else str(uuid.uuid4()),
                "timestamp": "2025-12-10T10:30:00Z"  # This would be dynamically set in real implementation
            }

            return result
        except Exception as e:
            print(f"Error in RAG query: {e}")
            raise

    def query_with_selected_text(self, query: str, selected_text: str, session_id: Optional[UUID] = None) -> Dict[str, Any]:
        """
        Query method specifically for questions about selected text
        """
        try:
            # When selected text is provided, we focus on that text
            # But still retrieve additional context for a more comprehensive answer
            context = self.retrieve_context(selected_text + " " + query, limit=3)

            # Generate answer with selected text as primary context
            response = self.generate_answer(query, context, session_id, selected_text)

            result = {
                "response": response["response"],
                "sources": response["sources"],
                "confidence_score": response.get("confidence_score", 0.8),
                "session_id": str(session_id) if session_id else str(uuid.uuid4()),
                "timestamp": "2025-12-10T10:30:00Z"  # This would be dynamically set in real implementation
            }

            return result
        except Exception as e:
            print(f"Error in RAG query with selected text: {e}")
            raise

# Global instance of RAGService
rag_service = RAGService()