import time
from typing import Dict, Any, Optional, Callable
from functools import wraps
from datetime import datetime, timedelta
import threading
from collections import deque, defaultdict
from src.logging_config import get_logger
from src.database.repositories import AnalyticsRecordRepository
from sqlalchemy.orm import Session

logger = get_logger(__name__)

class PerformanceMonitoringService:
    def __init__(self):
        # Store performance metrics in memory (in production, you'd use a database or Redis)
        self.response_times = deque(maxlen=1000)  # Keep last 1000 response times
        self.error_counts = defaultdict(int)
        self.request_counts = defaultdict(int)
        self.lock = threading.Lock()  # Thread safety for shared metrics

    def record_response_time(self, response_time_ms: float, endpoint: str = "unknown"):
        """
        Record a response time for performance monitoring
        """
        with self.lock:
            self.response_times.append({
                "timestamp": datetime.utcnow(),
                "response_time_ms": response_time_ms,
                "endpoint": endpoint
            })

    def record_error(self, error_type: str, endpoint: str = "unknown"):
        """
        Record an error for monitoring
        """
        with self.lock:
            self.error_counts[(endpoint, error_type)] += 1

    def record_request(self, endpoint: str = "unknown"):
        """
        Record a request for monitoring
        """
        with self.lock:
            self.request_counts[endpoint] += 1

    def get_average_response_time(self, minutes: int = 5) -> float:
        """
        Get average response time for the last N minutes
        """
        with self.lock:
            if not self.response_times:
                return 0.0

            cutoff_time = datetime.utcnow() - timedelta(minutes=minutes)
            recent_responses = [
                resp for resp in self.response_times
                if resp["timestamp"] > cutoff_time
            ]

            if not recent_responses:
                return 0.0

            avg_time = sum(resp["response_time_ms"] for resp in recent_responses) / len(recent_responses)
            return avg_time

    def get_p95_response_time(self, minutes: int = 5) -> float:
        """
        Get 95th percentile response time for the last N minutes
        """
        with self.lock:
            if not self.response_times:
                return 0.0

            cutoff_time = datetime.utcnow() - timedelta(minutes=minutes)
            recent_responses = [
                resp["response_time_ms"] for resp in self.response_times
                if resp["timestamp"] > cutoff_time
            ]

            if not recent_responses:
                return 0.0

            # Calculate 95th percentile
            sorted_times = sorted(recent_responses)
            index = int(0.95 * len(sorted_times))
            return sorted_times[min(index, len(sorted_times) - 1)]

    def get_error_rate(self, minutes: int = 5) -> float:
        """
        Get error rate as percentage for the last N minutes
        """
        with self.lock:
            if not self.response_times:
                return 0.0

            cutoff_time = datetime.utcnow() - timedelta(minutes=minutes)
            recent_responses = [
                resp for resp in self.response_times
                if resp["timestamp"] > cutoff_time
            ]

            if not recent_responses:
                return 0.0

            total_requests = len(recent_responses)
            error_requests = sum(1 for resp in recent_responses if resp.get("is_error", False))

            return (error_requests / total_requests) * 100

    def get_request_rate(self, minutes: int = 1) -> float:
        """
        Get requests per minute
        """
        with self.lock:
            if not self.response_times:
                return 0.0

            cutoff_time = datetime.utcnow() - timedelta(minutes=minutes)
            recent_requests = [
                resp for resp in self.response_times
                if resp["timestamp"] > cutoff_time
            ]

            return len(recent_requests) / minutes

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get a summary of performance metrics
        """
        return {
            "timestamp": datetime.utcnow().isoformat(),
            "average_response_time_ms": self.get_average_response_time(),
            "p95_response_time_ms": self.get_p95_response_time(),
            "error_rate_percent": self.get_error_rate(),
            "requests_per_minute": self.get_request_rate(),
            "total_requests": sum(self.request_counts.values()),
            "total_errors": sum(self.error_counts.values()),
            "recent_response_times": list(self.response_times)[-10:]  # Last 10 response times
        }

    def time_endpoint(self, endpoint_name: str):
        """
        Decorator to time endpoint performance
        """
        def decorator(func: Callable) -> Callable:
            @wraps(func)
            def wrapper(*args, **kwargs):
                start_time = time.perf_counter()
                self.record_request(endpoint_name)

                try:
                    result = func(*args, **kwargs)
                    response_time = (time.perf_counter() - start_time) * 1000  # Convert to milliseconds
                    self.record_response_time(response_time, endpoint_name)
                    return result
                except Exception as e:
                    response_time = (time.perf_counter() - start_time) * 1000
                    self.record_response_time(response_time, endpoint_name)
                    self.record_error(type(e).__name__, endpoint_name)
                    raise
            return wrapper
        return decorator

    def log_performance_metrics(self):
        """
        Log performance metrics at regular intervals
        """
        summary = self.get_performance_summary()
        logger.info(f"Performance Summary: {summary}")

    def store_analytics_record(self, db: Session, record_type: str, action: str,
                             user_id: Optional[str] = None, session_id: Optional[str] = None,
                             target_id: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None):
        """
        Store analytics record in the database
        """
        try:
            analytics_repo = AnalyticsRecordRepository(db)
            record = analytics_repo.create(
                record_type=record_type,
                action=action,
                user_id=user_id,
                session_id=session_id,
                target_id=target_id,
                metadata=metadata
            )
            return record
        except Exception as e:
            logger.error(f"Error storing analytics record: {e}")
            return None

    def track_user_interaction(self, db: Session, interaction_type: str, query: str,
                             response_time: float, session_id: str, user_id: Optional[str] = None):
        """
        Track a user interaction with performance metrics
        """
        metadata = {
            "query_length": len(query),
            "response_time_ms": response_time,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.store_analytics_record(
            db=db,
            record_type="user_interaction",
            action=interaction_type,
            session_id=session_id,
            user_id=user_id,
            metadata=metadata
        )

    def track_api_performance(self, db: Session, endpoint: str, response_time: float,
                            status_code: int, session_id: Optional[str] = None):
        """
        Track API performance metrics
        """
        metadata = {
            "response_time_ms": response_time,
            "status_code": status_code,
            "endpoint": endpoint,
            "timestamp": datetime.utcnow().isoformat()
        }

        self.store_analytics_record(
            db=db,
            record_type="api_performance",
            action=f"api_call_{endpoint}",
            session_id=session_id,
            metadata=metadata
        )

# Global instance of PerformanceMonitoringService
performance_monitoring_service = PerformanceMonitoringService()