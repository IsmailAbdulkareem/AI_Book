"""Skill 5: Chat Engine - LLM + RAG, streaming, context injection"""

from typing import Dict, Any, List, Optional, Generator


class ChatEngine:
    """Handles the chatbot: query embedding, retrieval, LLM generation, streaming."""

    def __init__(self, vector_db_handler=None, openai_api_key: Optional[str] = None):
        self.vector_db = vector_db_handler
        self.openai_key = openai_api_key or self._get_openai_key()
        self.client = None

    def initialize(self) -> Dict[str, Any]:
        """Initialize OpenAI client."""
        try:
            import openai
            openai.api_key = self.openai_key
            self.client = openai.OpenAI(api_key=self.openai_key)
            return {"status": "success", "message": "OpenAI initialized"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def embed_query(self, query: str, model: str = "embed-english-v3.0") -> Dict[str, Any]:
        """Generate embedding for user query using Cohere."""
        try:
            import cohere
            
            client = cohere.ClientV2(api_key=self._get_cohere_key())
            response = client.embed(
                texts=[query],
                model=model,
                input_type="search_query"
            )
            
            embedding = response.embeddings[0]
            return {"status": "success", "embedding": embedding}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def retrieve_context(self, query: str, collection_name: str = "docs", limit: int = 5) -> Dict[str, Any]:
        """Retrieve relevant docs from vector DB."""
        if not self.vector_db:
            return {"status": "error", "message": "Vector DB not connected"}
        
        # Step 1: Embed query
        embed_result = self.embed_query(query)
        if embed_result["status"] != "success":
            return embed_result
        
        # Step 2: Search
        search_result = self.vector_db.similarity_search(
            embed_result["embedding"],
            collection_name,
            limit
        )
        
        if search_result["status"] != "success":
            return search_result
        
        context = "\n\n".join([
            r["payload"].get("text", "")
            for r in search_result["results"]
        ])
        
        return {
            "status": "success",
            "context": context,
            "sources": [r["payload"]["source"] for r in search_result["results"]]
        }

    def generate_response(self, query: str, context: str, model: str = "gpt-4") -> Dict[str, Any]:
        """Generate LLM response using retrieved context."""
        try:
            if not self.client:
                init = self.initialize()
                if init["status"] != "success":
                    return init
            
            system_prompt = """You are an expert AI tutor for Physical AI and Humanoid Robotics.
Use the provided context to answer user questions accurately and pedagogically.
Always cite your sources and be clear about limitations."""
            
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
            ]
            
            response = self.client.chat.completions.create(
                model=model,
                messages=messages,
                max_tokens=2048,
                temperature=0.7
            )
            
            return {
                "status": "success",
                "response": response.choices[0].message.content,
                "model": model,
                "usage": response.usage.model_dump()
            }
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def chat(self, query: str, collection_name: str = "docs") -> Dict[str, Any]:
        """End-to-end chat: retrieve context → generate response."""
        try:
            # Retrieve context
            context_result = self.retrieve_context(query, collection_name)
            if context_result["status"] != "success":
                return context_result
            
            # Generate response
            response_result = self.generate_response(query, context_result["context"])
            response_result["sources"] = context_result["sources"]
            
            return response_result
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def stream_response(self, query: str, context: str, model: str = "gpt-4") -> Generator[str, None, None]:
        """Stream LLM response token-by-token."""
        try:
            if not self.client:
                self.initialize()
            
            system_prompt = """You are an expert AI tutor for Physical AI and Humanoid Robotics.
Use the provided context to answer user questions accurately and pedagogically."""
            
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
            ]
            
            with self.client.chat.completions.stream(
                model=model,
                messages=messages,
                max_tokens=2048,
                temperature=0.7
            ) as stream:
                for text in stream.text_stream:
                    yield text
        except Exception as e:
            yield f"Error: {str(e)}"

    def _get_openai_key(self) -> str:
        """Get OpenAI key from environment."""
        import os
        return os.getenv("OPENAI_API_KEY", "")

    def _get_cohere_key(self) -> str:
        """Get Cohere key from environment."""
        import os
        return os.getenv("COHERE_API_KEY", "")
