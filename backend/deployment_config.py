"""
Production deployment configuration for the RAG service.
"""

import os
from typing import Dict, Any, Optional
from pydantic import BaseModel, Field


class DeploymentConfig(BaseModel):
    """
    Configuration for deploying the RAG service in production.
    """

    # Server configuration
    host: str = Field(default="0.0.0.0", description="Host to bind to")
    port: int = Field(default=8000, description="Port to listen on")
    workers: int = Field(default=4, description="Number of worker processes")
    timeout: int = Field(default=120, description="Request timeout in seconds")
    keepalive: int = Field(default=5, description="Keep-alive timeout in seconds")

    # Logging configuration
    log_level: str = Field(default="INFO", description="Logging level")
    log_file: Optional[str] = Field(default="logs/production.log", description="Log file path")
    log_rotation_max_bytes: int = Field(default=10*1024*1024, description="Max log file size in bytes")
    log_rotation_backup_count: int = Field(default=5, description="Number of backup log files")

    # Database configuration
    qdrant_url: str = Field(description="Qdrant vector database URL")
    qdrant_api_key: Optional[str] = Field(default=None, description="Qdrant API key")
    qdrant_collection_name: str = Field(default="textbook_content", description="Qdrant collection name")
    qdrant_port: int = Field(default=6333, description="Qdrant port")

    # Embedding configuration
    embedding_model: str = Field(default="all-MiniLM-L6-v2", description="Sentence transformer model")
    embedding_dimension: int = Field(default=384, description="Dimension of embeddings")

    # LLM configuration
    llm_provider: str = Field(default="openai", description="LLM provider (openai, gemini)")
    llm_model: str = Field(default="gpt-3.5-turbo", description="LLM model name")
    llm_api_key: str = Field(description="LLM API key")
    llm_temperature: float = Field(default=0.3, description="LLM temperature parameter")
    llm_max_tokens: int = Field(default=1000, description="Maximum tokens in response")

    # Performance configuration
    retrieval_top_k: int = Field(default=5, description="Number of top results to retrieve")
    response_timeout: int = Field(default=30, description="Response timeout in seconds")
    max_concurrent_users: int = Field(default=50, description="Maximum concurrent users")
    cache_enabled: bool = Field(default=True, description="Enable response caching")
    cache_ttl_seconds: int = Field(default=3600, description="Cache TTL in seconds")

    # Security configuration
    cors_allow_origins: list = Field(default=["*"], description="Allowed CORS origins")
    request_rate_limit: int = Field(default=100, description="Requests per minute per IP")
    input_validation_enabled: bool = Field(default=True, description="Enable input validation")
    sanitize_user_inputs: bool = Field(default=True, description="Sanitize user inputs")

    # Monitoring and health check
    health_check_interval: int = Field(default=30, description="Health check interval in seconds")
    metrics_collection_enabled: bool = Field(default=True, description="Enable metrics collection")
    performance_monitoring: bool = Field(default=True, description="Enable performance monitoring")

    # Resource limits
    max_request_size: int = Field(default=10*1024*1024, description="Max request size in bytes (10MB)")
    memory_limit_mb: int = Field(default=1024, description="Memory limit in MB")

    class Config:
        env_file = ".env.production"
        case_sensitive = False


def get_production_config() -> DeploymentConfig:
    """
    Get production configuration from environment variables or defaults.

    Returns:
        DeploymentConfig instance with production settings
    """
    # Set default production values
    config_dict = {
        "qdrant_url": os.getenv("QDRANT_URL", "http://qdrant:6333"),
        "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
        "llm_api_key": os.getenv("LLM_API_KEY", os.getenv("OPENAI_API_KEY", os.getenv("GEMINI_API_KEY"))),
        "host": os.getenv("HOST", "0.0.0.0"),
        "port": int(os.getenv("PORT", "8000")),
        "workers": int(os.getenv("WORKERS", "4")),
        "log_level": os.getenv("LOG_LEVEL", "INFO"),
        "cors_allow_origins": os.getenv("CORS_ALLOW_ORIGINS", "*").split(","),
        "llm_provider": os.getenv("LLM_PROVIDER", "openai"),
        "llm_model": os.getenv("LLM_MODEL", "gpt-3.5-turbo"),
    }

    # Create config from environment variables
    return DeploymentConfig(**config_dict)


def get_development_config() -> DeploymentConfig:
    """
    Get development configuration with relaxed settings.

    Returns:
        DeploymentConfig instance with development settings
    """
    config_dict = {
        "qdrant_url": os.getenv("QDRANT_URL", "http://localhost:6333"),
        "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
        "llm_api_key": os.getenv("LLM_API_KEY", os.getenv("OPENAI_API_KEY", os.getenv("GEMINI_API_KEY"))),
        "host": os.getenv("HOST", "127.0.0.1"),
        "port": int(os.getenv("PORT", "8000")),
        "workers": int(os.getenv("WORKERS", "1")),  # Single worker for development
        "log_level": os.getenv("LOG_LEVEL", "DEBUG"),  # More verbose in dev
        "cors_allow_origins": ["*"],  # Allow all in development
        "cache_enabled": False,  # Disable cache for development
        "request_rate_limit": 1000,  # Higher rate limit in development
        "llm_provider": os.getenv("LLM_PROVIDER", "openai"),
        "llm_model": os.getenv("LLM_MODEL", "gpt-3.5-turbo"),
    }

    return DeploymentConfig(**config_dict)


def get_testing_config() -> DeploymentConfig:
    """
    Get testing configuration with isolated settings.

    Returns:
        DeploymentConfig instance with testing settings
    """
    config_dict = {
        "qdrant_url": os.getenv("QDRANT_URL", "http://localhost:6333"),
        "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
        "llm_api_key": "test-key",  # Use test key for testing
        "host": "127.0.0.1",
        "port": 0,  # Use random port for testing
        "workers": 1,
        "log_level": "ERROR",  # Less verbose in tests
        "cors_allow_origins": ["http://localhost:3000"],  # More restrictive for testing
        "cache_enabled": False,
        "request_rate_limit": 10000,  # Very high for testing
        "retrieval_top_k": 2,  # Smaller results for faster tests
        "llm_provider": os.getenv("LLM_PROVIDER", "openai"),
        "llm_model": os.getenv("LLM_MODEL", "gpt-3.5-turbo"),
    }

    return DeploymentConfig(**config_dict)


def get_current_config() -> DeploymentConfig:
    """
    Get configuration based on current environment.

    Returns:
        Appropriate DeploymentConfig instance for current environment
    """
    env = os.getenv("ENVIRONMENT", "development").lower()

    if env == "production":
        return get_production_config()
    elif env == "testing":
        return get_testing_config()
    else:
        return get_development_config()


# Create a global config instance
config = get_current_config()


# Docker-related configuration
DOCKER_CONFIG = {
    "image_name": "rag-chatbot-service",
    "container_name": "rag-chatbot-container",
    "ports": {
        "host": config.port,
        "container": config.port
    },
    "volumes": [
        "./logs:/app/logs",
        "./data:/app/data"
    ],
    "environment": {
        "ENVIRONMENT": os.getenv("ENVIRONMENT", "development"),
        "QDRANT_URL": config.qdrant_url,
        "LOG_LEVEL": config.log_level,
    },
    "resources": {
        "mem_limit": f"{config.memory_limit_mb}m",
        "cpu_quota": 100000  # 1 CPU
    }
}


# Kubernetes deployment configuration
K8S_DEPLOYMENT_CONFIG = {
    "apiVersion": "apps/v1",
    "kind": "Deployment",
    "metadata": {
        "name": "rag-chatbot-deployment",
        "labels": {
            "app": "rag-chatbot"
        }
    },
    "spec": {
        "replicas": 2,
        "selector": {
            "matchLabels": {
                "app": "rag-chatbot"
            }
        },
        "template": {
            "metadata": {
                "labels": {
                    "app": "rag-chatbot"
                }
            },
            "spec": {
                "containers": [
                    {
                        "name": "rag-chatbot",
                        "image": DOCKER_CONFIG["image_name"],
                        "ports": [
                            {
                                "containerPort": config.port
                            }
                        ],
                        "env": [
                            {"name": "ENVIRONMENT", "value": os.getenv("ENVIRONMENT", "production")},
                            {"name": "QDRANT_URL", "value": config.qdrant_url},
                            {"name": "LOG_LEVEL", "value": config.log_level}
                        ],
                        "resources": {
                            "requests": {
                                "memory": f"{config.memory_limit_mb // 2}Mi",
                                "cpu": "250m"
                            },
                            "limits": {
                                "memory": f"{config.memory_limit_mb}Mi",
                                "cpu": "500m"
                            }
                        }
                    }
                ]
            }
        }
    }
}


# Environment-specific configurations
ENV_CONFIGS = {
    "production": {
        "workers": 4,
        "log_level": "INFO",
        "cache_enabled": True,
        "request_rate_limit": 100,
        "cors_allow_origins": ["https://yourdomain.com", "https://www.yourdomain.com"]
    },
    "staging": {
        "workers": 2,
        "log_level": "INFO",
        "cache_enabled": True,
        "request_rate_limit": 200,
        "cors_allow_origins": ["https://staging.yourdomain.com"]
    },
    "development": {
        "workers": 1,
        "log_level": "DEBUG",
        "cache_enabled": False,
        "request_rate_limit": 1000,
        "cors_allow_origins": ["*"]
    }
}


def update_config_for_environment(config_obj: DeploymentConfig, env: str = None) -> DeploymentConfig:
    """
    Update configuration based on environment-specific settings.

    Args:
        config_obj: Base configuration object to update
        env: Environment name (production, staging, development)

    Returns:
        Updated configuration object
    """
    if env is None:
        env = os.getenv("ENVIRONMENT", "development")

    if env in ENV_CONFIGS:
        env_settings = ENV_CONFIGS[env]
        for key, value in env_settings.items():
            if hasattr(config_obj, key):
                setattr(config_obj, key, value)

    return config_obj


# Export the final configuration
final_config = update_config_for_environment(config)