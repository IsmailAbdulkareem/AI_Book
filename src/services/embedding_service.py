from typing import List, Dict, Any, Optional
from src.ai.cohere_client import cohere_client
from src.models.book_content import BookContent, VectorStorageInterface
from src.vector_store.qdrant_client import qdrant_manager
from src.config import settings
import uuid

class EmbeddingService:
    def __init__(self):
        self.cohere_client = cohere_client
        self.vector_interface = VectorStorageInterface()
        self.vector_store = qdrant_manager

    def embed_document_content(self, content: str, section_id: str = None, page_url: str = None) -> List[float]:
        """
        Generate embedding for a document content
        """
        try:
            # Use Cohere to generate embedding
            embedding = self.cohere_client.embed_text(content, input_type="search_document")

            # Validate the embedding
            if not self.vector_interface.validate_embedding_vector(embedding):
                raise ValueError(f"Invalid embedding vector size. Expected {self.vector_interface.get_expected_dimension()}, got {len(embedding)}")

            # Normalize the embedding vector
            normalized_embedding = self.vector_interface.normalize_embedding_vector(embedding)

            return normalized_embedding
        except Exception as e:
            print(f"Error embedding document content: {e}")
            raise

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a query
        """
        try:
            # Use Cohere to generate embedding for query
            embedding = self.cohere_client.embed_query(query, input_type="search_query")

            # Validate the embedding
            if not self.vector_interface.validate_embedding_vector(embedding):
                raise ValueError(f"Invalid embedding vector size. Expected {self.vector_interface.get_expected_dimension()}, got {len(embedding)}")

            # Normalize the embedding vector
            normalized_embedding = self.vector_interface.normalize_embedding_vector(embedding)

            return normalized_embedding
        except Exception as e:
            print(f"Error embedding query: {e}")
            raise

    def embed_multiple_documents(self, contents: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple documents at once
        """
        try:
            # Use Cohere to generate embeddings for multiple texts
            embeddings = self.cohere_client.embed_documents(contents, input_type="search_document")

            # Validate and normalize each embedding
            normalized_embeddings = []
            for embedding in embeddings:
                if not self.vector_interface.validate_embedding_vector(embedding):
                    raise ValueError(f"Invalid embedding vector size. Expected {self.vector_interface.get_expected_dimension()}, got {len(embedding)}")

                normalized_embedding = self.vector_interface.normalize_embedding_vector(embedding)
                normalized_embeddings.append(normalized_embedding)

            return normalized_embeddings
        except Exception as e:
            print(f"Error embedding multiple documents: {e}")
            raise

    def process_book_content(self, book_content: BookContent) -> Dict[str, Any]:
        """
        Process a book content item by generating its embedding and preparing for storage
        """
        try:
            # Generate embedding for the content
            embedding = self.embed_document_content(
                content=book_content.content,
                section_id=book_content.section_id,
                page_url=book_content.page_url
            )

            # Update the book content with the embedding
            book_content.embedding_vector = embedding

            # Create payload for vector storage
            payload = self.vector_interface.create_payload_for_content(book_content)

            return {
                "id": str(book_content.id),
                "vector": embedding,
                "payload": payload
            }
        except Exception as e:
            print(f"Error processing book content: {e}")
            raise

    def search_similar_content(self, query: str, limit: int = 10, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Search for content similar to the query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embed_query(query)

            # Search in vector store
            results = self.vector_store.search_vectors(
                query_vector=query_embedding,
                limit=limit,
                filters=filters
            )

            return results
        except Exception as e:
            print(f"Error searching for similar content: {e}")
            raise

    def store_content_embeddings(self, content_embeddings: List[Dict[str, Any]]):
        """
        Store multiple content embeddings in the vector store
        """
        try:
            # Store in vector store
            self.vector_store.upsert_vectors(content_embeddings)
        except Exception as e:
            print(f"Error storing content embeddings: {e}")
            raise

# Global instance of EmbeddingService
embedding_service = EmbeddingService()