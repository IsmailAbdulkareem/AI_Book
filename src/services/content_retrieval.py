from typing import List, Dict, Any, Optional
from src.vector_store.qdrant_client import qdrant_manager
from src.services.embedding_service import embedding_service
from src.models.book_content import VectorStorageInterface
from uuid import UUID

class ContentRetrievalService:
    def __init__(self):
        self.vector_store = qdrant_manager
        self.embedding_service = embedding_service
        self.vector_interface = VectorStorageInterface()

    def retrieve_content_by_query(self, query: str, limit: int = 5,
                                  filters: Optional[Dict[str, Any]] = None,
                                  min_score: float = 0.3) -> List[Dict[str, Any]]:
        """
        Retrieve content from vector store based on a query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embedding_service.embed_query(query)

            # Search in vector store with filters
            results = self.vector_store.search_vectors(
                query_vector=query_embedding,
                limit=limit,
                filters=filters
            )

            # Filter results by minimum score
            filtered_results = [
                result for result in results
                if result.get('score', 0) >= min_score
            ]

            return filtered_results

        except Exception as e:
            print(f"Error retrieving content by query: {e}")
            raise

    def retrieve_content_by_section(self, section_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Retrieve content by section ID
        """
        filters = {"section_id": section_id}
        return self.retrieve_content_by_query("retrieval placeholder", limit=limit, filters=filters)

    def retrieve_content_by_page_url(self, page_url: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Retrieve content by page URL
        """
        filters = {"page_url": page_url}
        return self.retrieve_content_by_query("retrieval placeholder", limit=limit, filters=filters)

    def retrieve_content_by_content_type(self, content_type: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Retrieve content by content type (chapter, section, etc.)
        """
        filters = {"content_type": content_type}
        return self.retrieve_content_by_query("retrieval placeholder", limit=limit, filters=filters)

    def retrieve_similar_content(self, content_id: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve content similar to a given content item
        """
        try:
            # Get the original content vector
            original_content = self.vector_store.get_by_id(content_id)
            if not original_content:
                return []

            # Search for similar content using the original's vector
            results = self.vector_store.search_vectors(
                query_vector=original_content['vector'],
                limit=limit + 1  # +1 to exclude the original content
            )

            # Filter out the original content and return similar ones
            similar_results = [
                result for result in results
                if result['id'] != content_id
            ][:limit]

            return similar_results

        except Exception as e:
            print(f"Error retrieving similar content: {e}")
            raise

    def retrieve_content_by_ids(self, content_ids: List[str]) -> List[Dict[str, Any]]:
        """
        Retrieve specific content items by their IDs
        """
        results = []
        for content_id in content_ids:
            content = self.vector_store.get_by_id(content_id)
            if content:
                results.append(content)
        return results

    def search_content(self, search_term: str, content_types: Optional[List[str]] = None,
                      limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for content using a search term with optional content type filtering
        """
        filters = {}
        if content_types:
            filters["content_type"] = content_types

        return self.retrieve_content_by_query(search_term, limit=limit, filters=filters)

    def get_content_relevance_scores(self, query: str, content_list: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Calculate relevance scores for a list of content against a query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embedding_service.embed_query(query)

            scored_content = []
            for content in content_list:
                # Calculate similarity score (this is a simplified approach)
                # In a real implementation, you might want to embed the content and calculate similarity
                content_text = content.get('payload', {}).get('content', '')[:500]

                # For now, we'll use the existing score from the vector store
                # In a more sophisticated implementation, we might recalculate
                scored_content.append({
                    **content,
                    "calculated_score": content.get('score', 0)
                })

            # Sort by score in descending order
            scored_content.sort(key=lambda x: x['calculated_score'], reverse=True)
            return scored_content

        except Exception as e:
            print(f"Error calculating content relevance scores: {e}")
            raise

# Global instance of ContentRetrievalService
content_retrieval_service = ContentRetrievalService()