import time
from typing import Dict, Optional
from fastapi import Request, HTTPException
from collections import defaultdict, deque
from src.config import settings

class InMemoryRateLimiter:
    def __init__(self, requests: int = 10, window: int = 60):
        """
        Initialize rate limiter
        :param requests: Number of requests allowed per window
        :param window: Time window in seconds
        """
        self.requests = requests
        self.window = window
        self.requests_log: Dict[str, deque] = defaultdict(deque)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed
        :param identifier: Unique identifier for the requester (e.g., IP address)
        :return: True if request is allowed, False otherwise
        """
        current_time = time.time()

        # Remove old requests outside the time window
        while (self.requests_log[identifier] and
               current_time - self.requests_log[identifier][0] > self.window):
            self.requests_log[identifier].popleft()

        # Check if the number of requests is within the limit
        if len(self.requests_log[identifier]) < self.requests:
            # Add current request to the log
            self.requests_log[identifier].append(current_time)
            return True

        return False

# Global rate limiter instance with default settings (10 requests per minute per IP)
rate_limiter = InMemoryRateLimiter(
    requests=int(settings.get('RATE_LIMIT_REQUESTS', 10)),
    window=int(settings.get('RATE_LIMIT_WINDOW', 60))
)

def get_client_ip(request: Request) -> str:
    """
    Get the client IP address from the request
    """
    # Check for forwarded IP headers first (for when behind a proxy/load balancer)
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        # x-forwarded-for can contain multiple IPs, take the first one
        return forwarded_for.split(",")[0].strip()

    real_ip = request.headers.get("x-real-ip")
    if real_ip:
        return real_ip.strip()

    # Fallback to client host
    return request.client.host

async def rate_limit_middleware(request: Request, call_next):
    """
    Rate limiting middleware function
    """
    client_ip = get_client_ip(request)

    # Skip rate limiting for health checks and other internal endpoints
    if request.url.path in ["/health", "/docs", "/redoc"]:
        response = await call_next(request)
        return response

    # Check if the request is allowed
    if not rate_limiter.is_allowed(client_ip):
        raise HTTPException(
            status_code=429,
            detail={
                "error": "RATE_LIMIT_EXCEEDED",
                "message": "Rate limit exceeded. Please try again later."
            }
        )

    response = await call_next(request)
    return response

# Alternative implementation using decorator approach
from functools import wraps
from fastapi import Depends

def rate_limit(requests: int = 10, window: int = 60):
    """
    Decorator for rate limiting specific endpoints
    """
    limiter = InMemoryRateLimiter(requests, window)

    def rate_limit_dependency(request: Request):
        client_ip = get_client_ip(request)
        if not limiter.is_allowed(client_ip):
            raise HTTPException(
                status_code=429,
                detail={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": f"Rate limit exceeded ({requests} requests per {window} seconds)"
                }
            )
        return True

    return Depends(rate_limit_dependency)