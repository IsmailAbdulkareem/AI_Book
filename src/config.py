from pydantic_settings import BaseSettings
from typing import Optional, List
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    # Qdrant settings
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection: str = os.getenv("QDRANT_COLLECTION", "physical_ai_book")

    # Cohere settings
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    cohere_embed_model: str = os.getenv("COHERE_MODEL", "embed-english-v3.0")

    # OpenAI settings
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    openai_model: str = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

    # Database settings
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")

    # Application settings
    debug: bool = os.getenv("DEBUG", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # CORS settings
    cors_origins: List[str] = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

    class Config:
        env_file = ".env"

# Create a global settings instance
settings = Settings()