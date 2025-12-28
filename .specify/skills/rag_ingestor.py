"""Skill 3: RAG Ingestor - Document chunking, embedding, Qdrant upsert"""

import re
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass


@dataclass
class Chunk:
    """A text chunk with metadata."""
    text: str
    source: str
    chunk_id: str
    start_pos: int
    end_pos: int


class RagIngestor:
    """Handles document ingestion: read, chunk, embed, upsert to vector DB."""

    def __init__(self, repo_root: str = ".", chunk_size: int = 800, overlap: int = 100):
        self.repo_root = Path(repo_root)
        self.docs_dir = self.repo_root / "docs"
        self.chunk_size = chunk_size
        self.overlap = overlap
        self.cohere_client = None  # Lazy init

    def read_documents(self, pattern: str = "**/*.md") -> Dict[str, Any]:
        """Read all markdown files matching pattern."""
        try:
            files = list(self.docs_dir.glob(pattern))
            docs = {}
            for file in files:
                docs[str(file)] = file.read_text()
            return {"status": "success", "file_count": len(docs), "files": list(docs.keys())}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def chunk_documents(self, docs: Dict[str, str], chunk_size: Optional[int] = None) -> Dict[str, Any]:
        """Split documents into overlapping chunks."""
        chunk_size = chunk_size or self.chunk_size
        chunks = []

        for source, text in docs.items():
            # Remove frontmatter
            text = re.sub(r'^---\n.*?\n---\n', '', text, flags=re.DOTALL)
            
            # Split by sentences/paragraphs
            paragraphs = text.split('\n\n')
            current_chunk = ""
            pos = 0

            for para in paragraphs:
                if len(current_chunk) + len(para) > chunk_size and current_chunk:
                    chunks.append(Chunk(
                        text=current_chunk.strip(),
                        source=source,
                        chunk_id=f"{source}:chunk_{len(chunks)}",
                        start_pos=pos,
                        end_pos=pos + len(current_chunk)
                    ))
                    current_chunk = para[-self.overlap:] if len(para) > self.overlap else para
                    pos += len(current_chunk)
                else:
                    current_chunk += "\n\n" + para if current_chunk else para

            if current_chunk:
                chunks.append(Chunk(
                    text=current_chunk.strip(),
                    source=source,
                    chunk_id=f"{source}:chunk_{len(chunks)}",
                    start_pos=pos,
                    end_pos=pos + len(current_chunk)
                ))

        return {"status": "success", "chunk_count": len(chunks), "chunks": chunks}

    def generate_embeddings(self, chunks: List[Chunk], model: str = "embed-english-v3.0") -> Dict[str, Any]:
        """Generate embeddings using Cohere API."""
        try:
            import cohere
            
            client = cohere.ClientV2(api_key=self._get_cohere_key())
            texts = [c.text for c in chunks]
            
            response = client.embed(
                texts=texts,
                model=model,
                input_type="search_document"
            )
            
            embeddings = response.embeddings
            return {
                "status": "success",
                "embedding_count": len(embeddings),
                "model": model,
                "embeddings": embeddings
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def upsert_to_qdrant(self, chunks: List[Chunk], embeddings: List[List[float]], collection_name: str = "docs") -> Dict[str, Any]:
        """Upsert chunks + embeddings to Qdrant."""
        try:
            from qdrant_client import QdrantClient
            from qdrant_client.models import PointStruct
            
            client = QdrantClient("localhost", port=6333)
            
            points = [
                PointStruct(
                    id=int(i),
                    vector=embeddings[i],
                    payload={
                        "text": chunks[i].text,
                        "source": chunks[i].source,
                        "chunk_id": chunks[i].chunk_id
                    }
                )
                for i in range(len(chunks))
            ]
            
            client.upsert(collection_name, points=points)
            return {"status": "success", "points_upserted": len(points)}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def ingest_pipeline(self, collection_name: str = "docs") -> Dict[str, Any]:
        """Full pipeline: read → chunk → embed → upsert."""
        try:
            step1 = self.read_documents()
            if step1["status"] != "success":
                return step1
            
            docs_dict = {f["split"]()[0]: f for f in step1["files"]}  # Simplified
            step2 = self.chunk_documents(docs_dict)
            if step2["status"] != "success":
                return step2
            
            chunks = step2["chunks"]
            step3 = self.generate_embeddings(chunks)
            if step3["status"] != "success":
                return step3
            
            embeddings = step3["embeddings"]
            step4 = self.upsert_to_qdrant(chunks, embeddings, collection_name)
            return step4
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _get_cohere_key(self) -> str:
        """Get Cohere API key from .env or environment."""
        import os
        return os.getenv("COHERE_API_KEY", "")
