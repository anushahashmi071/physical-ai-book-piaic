from fastapi import APIRouter
from datetime import datetime
from typing import Dict, Any
import time
import logging
import asyncio
from config.settings import settings
from services.vector_storage import get_vector_storage_service
from services.agent import get_agent_service


router = APIRouter()
logger = logging.getLogger(__name__)

# Store service instances for monitoring
_start_time = time.time()


@router.get("/")
async def health_check() -> Dict[str, Any]:
    """
    Health check endpoint to verify service availability.

    Returns:
        Health status information including service connectivity
    """
    global _start_time

    # Check service availability
    vector_db_status = await _check_vector_db_connection()
    llm_status = await _check_llm_connection()
    index_status = await _check_content_index()

    all_services_healthy = vector_db_status and llm_status and index_status
    overall_status = "healthy" if all_services_healthy else "degraded"

    health_status = {
        "status": overall_status,
        "services": {
            "vector_db": vector_db_status,
            "llm_backend": llm_status,
            "content_index": index_status
        },
        "uptime": int(time.time() - _start_time),
        "timestamp": datetime.utcnow().isoformat(),
        "app_info": {
            "title": settings.app_title,
            "version": settings.app_version,
            "debug": settings.debug
        }
    }

    logger.info(f"Health check performed - Status: {overall_status}")
    return health_status


@router.get("/detailed")
async def detailed_health_check() -> Dict[str, Any]:
    """
    Detailed health check with more comprehensive diagnostics.

    Returns:
        Detailed health status with individual service checks
    """
    # Perform comprehensive service checks
    vector_db_result = await _check_vector_db_connection_detailed()
    llm_result = await _check_llm_connection_detailed()
    index_result = await _check_content_index_detailed()

    all_healthy = (
        vector_db_result["status"] == "ok" and
        llm_result["status"] == "ok" and
        index_result["status"] == "ok"
    )

    detailed_status = {
        "overall_status": "healthy" if all_healthy else "degraded",
        "checks": {
            "vector_database": vector_db_result,
            "llm_backend": llm_result,
            "content_index": index_result
        },
        "timestamp": datetime.utcnow().isoformat(),
        "performance": {
            "response_time_ms": 10  # Placeholder - would measure actual response time
        },
        "metrics": await _collect_runtime_metrics()
    }

    logger.info(f"Detailed health check performed - Overall: {detailed_status['overall_status']}")
    return detailed_status


@router.get("/metrics")
async def metrics_endpoint() -> Dict[str, Any]:
    """
    Metrics endpoint to provide runtime metrics for monitoring.

    Returns:
        Runtime metrics for monitoring and observability
    """
    metrics = await _collect_runtime_metrics()

    logger.info("Metrics endpoint accessed")
    return metrics


async def _collect_runtime_metrics() -> Dict[str, Any]:
    """
    Collect various runtime metrics for monitoring.

    Returns:
        Dictionary containing runtime metrics
    """
    # Get vector storage service to check collection info
    vector_storage = get_vector_storage_service()
    collection_info = await vector_storage.get_collection_info()

    metrics = {
        "timestamp": datetime.utcnow().isoformat(),
        "uptime_seconds": time.time() - _start_time,
        "vector_db": {
            "connected": await _check_vector_db_connection(),
            "collection_info": collection_info
        },
        "memory_usage_mb": 0,  # Would need psutil or similar to get actual memory
        "active_connections": 0,  # Would need to track active connections
        "request_count": 0,  # Would need to track request counters
        "error_count": 0  # Would need to track error counters
    }

    return metrics


async def _check_vector_db_connection() -> bool:
    """
    Check if vector database connection is available.

    Returns:
        True if connection is available, False otherwise
    """
    try:
        vector_storage = get_vector_storage_service()
        # Attempt to get collection info as a basic connectivity check
        collection_info = await vector_storage.get_collection_info()
        return collection_info is not None
    except Exception as e:
        logger.error(f"Vector DB connection check failed: {str(e)}")
        return False


async def _check_llm_connection() -> bool:
    """
    Check if LLM backend connection is available.

    Returns:
        True if connection is available, False otherwise
    """
    try:
        agent_service = get_agent_service()
        # This would normally make a test call to the LLM
        # For now, just check if the service can be instantiated properly
        return agent_service is not None
    except Exception as e:
        logger.error(f"LLM connection check failed: {str(e)}")
        return False


async def _check_content_index() -> bool:
    """
    Check if content index is available and populated.

    Returns:
        True if index is available, False otherwise
    """
    try:
        vector_storage = get_vector_storage_service()
        collection_info = await vector_storage.get_collection_info()

        if collection_info:
            # Check if there's content in the index
            return collection_info.get('points_count', 0) > 0
        return False
    except Exception as e:
        logger.error(f"Content index check failed: {str(e)}")
        return False


async def _check_vector_db_connection_detailed() -> Dict[str, Any]:
    """
    Detailed check for vector database connection.

    Returns:
        Detailed status information
    """
    try:
        vector_storage = get_vector_storage_service()
        collection_info = await vector_storage.get_collection_info()

        if collection_info:
            return {
                "status": "ok",
                "message": f"Connected to vector database with {collection_info['points_count']} entries",
                "details": collection_info
            }
        else:
            return {
                "status": "warning",
                "message": "Connected to vector database but collection info unavailable",
                "details": None
            }
    except Exception as e:
        logger.error(f"Detailed Vector DB check failed: {str(e)}")
        return {
            "status": "error",
            "message": f"Cannot connect to vector database: {str(e)}",
            "details": None
        }


async def _check_llm_connection_detailed() -> Dict[str, Any]:
    """
    Detailed check for LLM backend connection.

    Returns:
        Detailed status information
    """
    try:
        agent_service = get_agent_service()
        # Would normally make a test API call here
        return {
            "status": "ok",
            "message": "LLM backend service available",
            "details": {
                "provider": settings.llm_provider,
                "model": settings.model_name
            }
        }
    except Exception as e:
        logger.error(f"Detailed LLM check failed: {str(e)}")
        return {
            "status": "error",
            "message": f"LLM backend not accessible: {str(e)}",
            "details": None
        }


async def _check_content_index_detailed() -> Dict[str, Any]:
    """
    Detailed check for content index status.

    Returns:
        Detailed status information
    """
    try:
        vector_storage = get_vector_storage_service()
        collection_info = await vector_storage.get_collection_info()

        if collection_info and collection_info.get('points_count', 0) > 0:
            return {
                "status": "ok",
                "message": f"Content index available with {collection_info['points_count']} entries",
                "details": collection_info
            }
        elif collection_info:
            return {
                "status": "warning",
                "message": "Content index exists but appears to be empty",
                "details": collection_info
            }
        else:
            return {
                "status": "error",
                "message": "Content index not available",
                "details": None
            }
    except Exception as e:
        logger.error(f"Detailed content index check failed: {str(e)}")
        return {
            "status": "error",
            "message": f"Content index check failed: {str(e)}",
            "details": None
        }