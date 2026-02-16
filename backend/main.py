from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from config.settings import settings
from routes.health import router as health_router
from routes.chat import router as chat_router
import logging
from logging_config import setup_logging


# Set up logging
setup_logging(
    log_level=settings.log_level if hasattr(settings, 'log_level') else 'INFO',
    log_file="logs/rag-service.log"
)

logger = logging.getLogger(__name__)


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application.
    """
    app = FastAPI(
        title=settings.app_title,
        version=settings.app_version,
        debug=settings.debug,
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include routers
    app.include_router(health_router, prefix="/health", tags=["health"])
    app.include_router(chat_router, prefix="/chat", tags=["chat"])

    @app.get("/")
    def read_root():
        logger.info("Root endpoint accessed")
        return {
            "message": "RAG Chatbot Service for Physical AI Book",
            "version": settings.app_version,
            "status": "running"
        }

    return app


app = create_app()


if __name__ == "__main__":
    import os
    logger.info("Starting RAG Chatbot Service")
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=port,
        reload=True,
        log_level="info"
    )