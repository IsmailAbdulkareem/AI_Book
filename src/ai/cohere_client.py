import cohere
from typing import List, Dict, Any
from src.config import settings

class CohereEmbeddingClient:
    def __init__(self):
        # Initialize Cohere client with API key from settings
        self.client = cohere.Client(api_key=settings.cohere_api_key)
        self.model = settings.cohere_embed_model

    def embed_documents(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of documents
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=input_type
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            print(f"Error generating document embeddings: {e}")
            raise

    def embed_query(self, query: str, input_type: str = "search_query") -> List[float]:
        """
        Generate embedding for a single query
        """
        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type=input_type
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            print(f"Error generating query embedding: {e}")
            raise

    def embed_text(self, text: str, input_type: str = "classification") -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type=input_type
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            print(f"Error generating text embedding: {e}")
            raise

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current embedding model
        """
        # Note: Cohere API doesn't have a direct endpoint to get model info
        # This returns basic info based on the configured model
        return {
            "model": self.model,
            "expected_dimension": self.get_expected_dimension()
        }

    def get_expected_dimension(self) -> int:
        """
        Get the expected embedding dimension for the current model
        """
        # Based on Cohere documentation, embed-english-v3.0 returns 1024 dimensions
        # This could be expanded to handle other models
        if "v3" in self.model:
            return 1024
        elif "light" in self.model:
            return 384  # embed-english-light-v3.0 has 384 dimensions
        else:
            return 1024  # Default to 1024 for most Cohere models

# Global instance of CohereEmbeddingClient
cohere_client = CohereEmbeddingClient()