"""Skill 4: Vector DB Handler - Qdrant collection management, search, delete"""

from typing import Dict, Any, List, Optional


class VectorDbHandler:
    """Manages Qdrant vector database: create collections, search, delete, troubleshoot."""

    def __init__(self, host: str = "localhost", port: int = 6333):
        self.host = host
        self.port = port
        self.client = None

    def connect(self) -> Dict[str, Any]:
        """Connect to Qdrant instance."""
        try:
            from qdrant_client import QdrantClient
            self.client = QdrantClient(self.host, port=self.port)
            info = self.client.get_collections()
            return {"status": "success", "collections": len(info.collections)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def create_collection(self, name: str, vector_size: int = 1024) -> Dict[str, Any]:
        """Create a new vector collection."""
        try:
            from qdrant_client.models import VectorParams, Distance
            
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            self.client.create_collection(
                collection_name=name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
            return {"status": "success", "collection": name}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def similarity_search(self, query_vector: List[float], collection_name: str, limit: int = 5) -> Dict[str, Any]:
        """Run similarity search on a collection."""
        try:
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit
            )
            
            return {
                "status": "success",
                "results": [
                    {"id": r.id, "score": r.score, "payload": r.payload}
                    for r in results
                ]
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def delete_collection(self, name: str) -> Dict[str, Any]:
        """Delete a collection."""
        try:
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            self.client.delete_collection(name)
            return {"status": "success", "message": f"Deleted {name}"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def list_collections(self) -> Dict[str, Any]:
        """List all collections."""
        try:
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            info = self.client.get_collections()
            return {
                "status": "success",
                "collections": [c.name for c in info.collections]
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def get_collection_info(self, name: str) -> Dict[str, Any]:
        """Get info about a collection (point count, etc.)."""
        try:
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            info = self.client.get_collection(name)
            return {
                "status": "success",
                "name": name,
                "point_count": info.points_count,
                "vector_count": info.vectors_count,
                "indexed": info.indexed_vectors_count
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def health_check(self) -> Dict[str, Any]:
        """Check Qdrant server health."""
        try:
            if not self.client:
                return {"status": "error", "message": "Not connected"}
            
            self.client.http_client.session.head(f"http://{self.host}:{self.port}/health")
            return {"status": "success", "qdrant_healthy": True}
        except Exception as e:
            return {"status": "error", "message": f"Qdrant unhealthy: {e}"}

    def troubleshoot(self) -> Dict[str, Any]:
        """Diagnose common Qdrant issues."""
        diagnostics = {}
        
        # Check connection
        diagnostics["connection"] = self.connect()
        
        # Check collections
        if diagnostics["connection"]["status"] == "success":
            diagnostics["collections"] = self.list_collections()
            diagnostics["health"] = self.health_check()
        
        return diagnostics
