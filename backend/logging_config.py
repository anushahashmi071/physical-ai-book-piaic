"""
Logging configuration for the RAG service.
"""

import logging
import sys
import os
from datetime import datetime
from logging.handlers import RotatingFileHandler


def setup_logging(log_level: str = "INFO", log_file: str = None):
    """
    Set up comprehensive logging for the RAG service.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file to log to (default: None, logs to console only)
    """
    # Convert string level to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(funcName)s - %(message)s'
    )

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)

    # Clear existing handlers
    root_logger.handlers.clear()

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # File handler if specified
    if log_file:
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(log_file)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        # Create rotating file handler
        file_handler = RotatingFileHandler(
            log_file,
            maxBytes=10*1024*1024,  # 10 MB
            backupCount=5
        )
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)

    # Set specific loggers to WARNING level to reduce noise
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("fastapi").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.INFO)  # Keep Qdrant logs for debugging


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.

    Args:
        name: Name of the logger

    Returns:
        Logger instance
    """
    return logging.getLogger(name)


def log_request_details(logger_instance: logging.Logger, request_id: str, user_id: str = None, query: str = None):
    """
    Log details about a request for monitoring and debugging.

    Args:
        logger_instance: Logger instance to use
        request_id: Unique request identifier
        user_id: User identifier (optional)
        query: User query (optional)
    """
    logger_instance.info(f"Processing request {request_id}" +
                         (f" for user {user_id}" if user_id else "") +
                         (f" with query: '{query[:100]}...'" if query else ""))


def log_response_details(logger_instance: logging.Logger, request_id: str, response_time: float, success: bool = True):
    """
    Log details about a response for monitoring and debugging.

    Args:
        logger_instance: Logger instance to use
        request_id: Unique request identifier
        response_time: Time taken to process the request in seconds
        success: Whether the request was successful
    """
    status = "SUCCESS" if success else "FAILED"
    logger_instance.info(f"Request {request_id} completed with {status} in {response_time:.3f}s")


def log_error_details(logger_instance: logging.Logger, error: Exception, context: str = ""):
    """
    Log detailed error information for debugging.

    Args:
        logger_instance: Logger instance to use
        error: Exception that occurred
        context: Additional context about where the error occurred
    """
    logger_instance.error(f"Error in {context}: {str(error)}", exc_info=True)


# Initialize logging when module is imported
setup_logging(
    log_level=os.getenv("LOG_LEVEL", "INFO"),
    log_file=os.getenv("LOG_FILE", "logs/rag-service.log")
)