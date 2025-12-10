from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Any, Optional
from uuid import UUID
import uuid
from src.config import settings

class QdrantManager:
    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Using HTTP API
        )
        self.collection_name = settings.qdrant_collection

    def create_collection(self, vector_size: int = 1024, distance: Distance = Distance.COSINE):
        """
        Create a collection for storing book content embeddings
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=distance),
                )

                # Create payload index for efficient filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="page_url",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                print(f"Collection '{self.collection_name}' created successfully")
            else:
                print(f"Collection '{self.collection_name}' already exists")

        except Exception as e:
            print(f"Error creating collection: {e}")
            raise

    def upsert_vectors(self, vectors: List[Dict[str, Any]]):
        """
        Upsert vectors to the collection
        Each vector dict should contain:
        - id: UUID or string id
        - vector: embedding vector
        - payload: dict with metadata
        """
        points = []
        for item in vectors:
            point = PointStruct(
                id=item.get('id', str(uuid.uuid4())),
                vector=item['vector'],
                payload=item.get('payload', {})
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search_vectors(self, query_vector: List[float], limit: int = 10, filters: Optional[Dict] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors
        """
        search_filter = None
        if filters:
            conditions = []
            for key, value in filters.items():
                if isinstance(value, list):
                    conditions.append(models.FieldCondition(
                        key=key,
                        match=models.MatchAny(any=value)
                    ))
                else:
                    conditions.append(models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    ))

            if conditions:
                search_filter = models.Filter(must=conditions)

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            query_filter=search_filter
        )

        # Format results to include id, payload, and score
        formatted_results = []
        for result in results:
            formatted_results.append({
                'id': result.id,
                'payload': result.payload,
                'score': result.score,
                'vector': result.vector
            })

        return formatted_results

    def get_by_id(self, vector_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific vector by ID
        """
        results = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[vector_id]
        )

        if results:
            result = results[0]
            return {
                'id': result.id,
                'payload': result.payload,
                'vector': result.vector
            }

        return None

    def delete_vectors(self, vector_ids: List[str]):
        """
        Delete vectors by IDs
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=vector_ids
        )

    def delete_collection(self):
        """
        Delete the entire collection
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            print(f"Collection '{self.collection_name}' deleted successfully")
        except Exception as e:
            print(f"Error deleting collection: {e}")

    def get_collection_info(self):
        """
        Get information about the collection
        """
        return self.client.get_collection(collection_name=self.collection_name)

# Global instance of QdrantManager
qdrant_manager = QdrantManager()

# Function to initialize the collection when the module is imported
def initialize_qdrant():
    """
    Initialize Qdrant collection with proper schema
    """
    qdrant_manager.create_collection()