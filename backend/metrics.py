"""
Performance metrics collection module for the RAG service.
"""

import time
import threading
from collections import defaultdict, deque
from typing import Dict, List, Optional
from datetime import datetime, timedelta
import logging


logger = logging.getLogger(__name__)


class MetricsCollector:
    """
    Collects and manages performance metrics for the RAG service.
    """

    def __init__(self, max_samples: int = 1000):
        self.max_samples = max_samples
        self.start_time = time.time()

        # Request tracking
        self.request_count = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.request_history = deque(maxlen=max_samples)

        # Timing metrics
        self.response_times = deque(maxlen=max_samples)
        self.avg_response_time = 0.0

        # Error tracking
        self.error_count = 0
        self.error_history = deque(maxlen=max_samples)

        # Concurrent request tracking
        self.active_requests = 0
        self.max_concurrent_requests = 0

        # Lock for thread safety
        self._lock = threading.Lock()

    def record_request_start(self) -> str:
        """
        Record the start of a request and return a request ID.

        Returns:
            Unique request ID
        """
        with self._lock:
            request_id = f"req_{int(time.time() * 1000000)}"
            self.active_requests += 1
            self.max_concurrent_requests = max(self.max_concurrent_requests, self.active_requests)

            # Record request start time
            self.request_history.append({
                'id': request_id,
                'start_time': time.time(),
                'status': 'started'
            })

            return request_id

    def record_request_complete(self, request_id: str, success: bool = True, response_time: float = None):
        """
        Record the completion of a request.

        Args:
            request_id: ID of the request
            success: Whether the request was successful
            response_time: Time taken to process the request in seconds
        """
        with self._lock:
            self.request_count += 1
            self.active_requests -= 1

            if success:
                self.successful_requests += 1
            else:
                self.failed_requests += 1
                self.error_count += 1

            # Find the request in history and update it
            for req in self.request_history:
                if req['id'] == request_id:
                    req['end_time'] = time.time()
                    req['success'] = success
                    req['response_time'] = response_time or (time.time() - req['start_time'])

                    if response_time is not None:
                        self.response_times.append(response_time)
                    else:
                        self.response_times.append(req['response_time'])

                    # Update average response time
                    if self.response_times:
                        self.avg_response_time = sum(self.response_times) / len(self.response_times)
                    break

    def record_error(self, error_type: str, error_message: str = ""):
        """
        Record an error occurrence.

        Args:
            error_type: Type of error that occurred
            error_message: Error message
        """
        with self._lock:
            self.error_count += 1
            self.error_history.append({
                'timestamp': time.time(),
                'type': error_type,
                'message': error_message
            })

    def get_metrics(self) -> Dict:
        """
        Get current performance metrics.

        Returns:
            Dictionary containing current metrics
        """
        with self._lock:
            # Calculate rates
            total_time = time.time() - self.start_time
            requests_per_second = self.request_count / total_time if total_time > 0 else 0
            success_rate = (self.successful_requests / self.request_count * 100) if self.request_count > 0 else 100
            error_rate = (self.error_count / self.request_count * 100) if self.request_count > 0 else 0

            # Calculate percentiles for response times
            response_times_list = list(self.response_times)
            response_times_list.sort()

            p50_response_time = self._get_percentile(response_times_list, 50) if response_times_list else 0
            p95_response_time = self._get_percentile(response_times_list, 95) if response_times_list else 0
            p99_response_time = self._get_percentile(response_times_list, 99) if response_times_list else 0

            return {
                'timestamp': datetime.utcnow().isoformat(),
                'uptime_seconds': total_time,
                'request_metrics': {
                    'total_requests': self.request_count,
                    'successful_requests': self.successful_requests,
                    'failed_requests': self.failed_requests,
                    'requests_per_second': round(requests_per_second, 2),
                    'success_rate_percent': round(success_rate, 2),
                    'error_rate_percent': round(error_rate, 2)
                },
                'timing_metrics': {
                    'avg_response_time_seconds': round(self.avg_response_time, 3),
                    'p50_response_time_seconds': round(p50_response_time, 3),
                    'p95_response_time_seconds': round(p95_response_time, 3),
                    'p99_response_time_seconds': round(p99_response_time, 3),
                    'min_response_time_seconds': round(min(self.response_times), 3) if self.response_times else 0,
                    'max_response_time_seconds': round(max(self.response_times), 3) if self.response_times else 0
                },
                'error_metrics': {
                    'total_errors': self.error_count,
                    'recent_errors': list(self.error_history)[-10:]  # Last 10 errors
                },
                'concurrency_metrics': {
                    'current_active_requests': self.active_requests,
                    'max_concurrent_requests': self.max_concurrent_requests
                }
            }

    def get_detailed_metrics(self) -> Dict:
        """
        Get detailed metrics including historical data.

        Returns:
            Dictionary containing detailed metrics
        """
        base_metrics = self.get_metrics()

        with self._lock:
            base_metrics['historical_data'] = {
                'recent_requests': list(self.request_history)[-50:],  # Last 50 requests
                'response_time_samples': list(self.response_times)[-100:]  # Last 100 samples
            }

        return base_metrics

    def _get_percentile(self, sorted_list: List[float], percentile: int) -> float:
        """
        Calculate percentile from sorted list.

        Args:
            sorted_list: Sorted list of values
            percentile: Percentile to calculate (e.g., 50, 95, 99)

        Returns:
            Calculated percentile value
        """
        if not sorted_list:
            return 0

        index = (percentile / 100) * (len(sorted_list) - 1)
        lower_index = int(index)
        upper_index = lower_index + 1

        if upper_index >= len(sorted_list):
            return sorted_list[lower_index]

        # Interpolate between values
        fraction = index - lower_index
        lower_value = sorted_list[lower_index]
        upper_value = sorted_list[upper_index]

        return lower_value + fraction * (upper_value - lower_value)


# Global metrics collector instance
metrics_collector = MetricsCollector()


def get_metrics_collector() -> MetricsCollector:
    """
    Get the global metrics collector instance.

    Returns:
        MetricsCollector instance
    """
    return metrics_collector


# Context manager for measuring request performance
class RequestTimer:
    """
    Context manager to time requests and record metrics.
    """

    def __init__(self, request_description: str = ""):
        self.request_description = request_description
        self.start_time = None
        self.request_id = None

    def __enter__(self):
        self.start_time = time.time()
        self.request_id = metrics_collector.record_request_start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        response_time = time.time() - self.start_time
        success = exc_type is None

        metrics_collector.record_request_complete(
            request_id=self.request_id,
            success=success,
            response_time=response_time
        )

        if exc_type is not None:
            metrics_collector.record_error(
                error_type=str(exc_type.__name__),
                error_message=str(exc_val)
            )


# Decorator for automatic metrics collection
def collect_metrics(func):
    """
    Decorator to automatically collect metrics for function calls.

    Args:
        func: Function to wrap

    Returns:
        Wrapped function with metrics collection
    """
    def wrapper(*args, **kwargs):
        with RequestTimer(f"{func.__name__}"):
            return func(*args, **kwargs)
    return wrapper


# Initialize metrics collection
logger.info("Performance metrics collection initialized")