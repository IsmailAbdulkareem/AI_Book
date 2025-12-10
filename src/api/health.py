from fastapi import APIRouter, Depends
from datetime import datetime
from src.schemas.api_models import HealthResponse
from src.config import settings

router = APIRouter()

@router.get("/", response_model=HealthResponse)
async def health_check():
    """
    Basic health check endpoint
    """
    return HealthResponse(
        status="healthy",
        timestamp=datetime.utcnow(),
        version="1.0.0"
    )

@router.get("/detailed")
async def detailed_health_check():
    """
    Detailed health check with system information
    """
    import platform
    import psutil

    health_info = {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "version": "1.0.0",
        "environment": {
            "debug": settings.debug,
            "log_level": settings.log_level
        },
        "system": {
            "platform": platform.system(),
            "platform_version": platform.version(),
            "processor": platform.processor(),
            "cpu_percent": psutil.cpu_percent(),
            "memory_percent": psutil.virtual_memory().percent,
        }
    }

    return health_info