from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # Qdrant Configuration
    qdrant_url: str = "http://localhost:6333"  # Default for local development
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "textbook_content"
    qdrant_port: int = 6333

    # Embedding Configuration
    embedding_model: str = "all-MiniLM-L6-v2"
    embedding_dimension: int = 384

    # LLM Configuration
    llm_provider: str = "openai"  # or "gemini"
    openai_api_key: Optional[str] = None
    gemini_api_key: Optional[str] = None
    model_name: str = "gpt-3.5-turbo"  # or gemini model name

    # Application Configuration
    app_title: str = "RAG Chatbot for Physical AI Book"
    app_version: str = "0.1.0"
    debug: bool = False

    # Logging Configuration
    log_level: str = "INFO"
    log_file: str = "logs/rag-service.log"

    # Performance Configuration
    retrieval_top_k: int = 5
    response_timeout_seconds: int = 30
    max_concurrent_users: int = 50

    class Config:
        env_file = ".env"
        case_sensitive = False


settings = Settings()