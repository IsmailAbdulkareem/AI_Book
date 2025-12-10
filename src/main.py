from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv
from src.logging_config import setup_logging, get_logger
from src.config import settings
from src.vector_store.qdrant_client import initialize_qdrant

# Load environment variables
load_dotenv()

# Setup logging
setup_logging(log_level=settings.log_level, log_file=os.getenv("LOG_FILE"))

# Import API routes
from src.api.health import router as health_router
from src.api.chat import router as chat_router
from src.api.sessions import router as sessions_router

app = FastAPI(
    title="RAG Chatbot API for Physical AI & Humanoid Robotics Book",
    description="API for interacting with the RAG chatbot system",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(health_router, prefix="/health", tags=["health"])
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(sessions_router, prefix="/api", tags=["sessions"])

# Application startup and shutdown events
@app.on_event("startup")
async def startup_event():
    logger = get_logger(__name__)
    logger.info("Application startup initiated")

    # Initialize Qdrant collection
    try:
        initialize_qdrant()
        logger.info("Qdrant collection initialized successfully")
    except Exception as e:
        logger.error(f"Error initializing Qdrant: {e}")
        raise

    logger.info("Application startup complete")

@app.on_event("shutdown")
async def shutdown_event():
    logger = get_logger(__name__)
    logger.info("Application shutdown initiated")
    # Add any cleanup code here if needed
    logger.info("Application shutdown complete")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=os.getenv("BACKEND_HOST", "0.0.0.0"),
        port=int(os.getenv("BACKEND_PORT", 8000)),
        reload=os.getenv("DEBUG", "false").lower() == "true"
    )