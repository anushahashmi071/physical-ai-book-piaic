# RAG Chatbot Service for Physical AI Book

This service implements a Retrieval-Augmented Generation (RAG) system for the Physical AI & Humanoid Robotics textbook. It allows students to ask questions about the textbook content and receive accurate, cited answers grounded in the textbook.

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [API Endpoints](#api-endpoints)
- [Development](#development)
- [Deployment](#deployment)
- [Troubleshooting](#troubleshooting)

## Features

- **Intelligent Question Answering**: Students can ask questions about robotics, AI, and Physical AI concepts
- **Citation System**: All answers include citations to specific textbook sections
- **Selected Text Queries**: Students can select text in the textbook and ask clarifying questions
- **Chapter-Specific Context**: Answers are appropriate to the student's current learning level
- **Fast Response Times**: Optimized for interactive learning with responses under 3 seconds
- **Content Grounding**: Answers are strictly based on textbook content with hallucination prevention

## Prerequisites

- Python 3.8+
- Docker and Docker Compose (for containerized deployment)
- Access to OpenAI or Gemini API
- Qdrant Cloud account (or local Qdrant instance)

## Installation

### Option 1: Local Installation

1. Clone the repository:
```bash
git clone https://github.com/your-org/physical-ai-book.git
cd backend/rag-service
```

2. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

### Option 2: Docker Installation

1. Clone the repository:
```bash
git clone https://github.com/your-org/physical-ai-book.git
cd backend/rag-service
```

2. Build and run with Docker Compose:
```bash
docker-compose up --build
```

## Configuration

### Environment Variables

Create a `.env` file in the backend/rag-service directory with the following variables:

```bash
# LLM Configuration
OPENAI_API_KEY=your-openai-api-key
# OR for Gemini:
GEMINI_API_KEY=your-gemini-api-key
LLM_PROVIDER=openai  # or "gemini"
MODEL_NAME=gpt-3.5-turbo  # or "gemini-pro"

# Qdrant Vector Database Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key  # if using cloud
QDRANT_COLLECTION_NAME=textbook_content

# Application Configuration
ENVIRONMENT=development  # or "production"
LOG_LEVEL=INFO
HOST=0.0.0.0
PORT=8000
```

### Settings Configuration

The application uses `config/settings.py` for additional configuration:

- `retrieval_top_k`: Number of top results to retrieve (default: 5)
- `response_timeout_seconds`: Timeout for responses (default: 30)
- `max_concurrent_users`: Maximum concurrent users (default: 50)
- `embedding_model`: Model used for embeddings (default: "all-MiniLM-L6-v2")

## Usage

### Starting the Service

#### Local Development
```bash
python main.py
```

Or using uvicorn:
```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

#### Production
```bash
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

### Ingesting Textbook Content

Before the RAG system can answer questions, you need to ingest the textbook content:

```python
from services.ingestion import get_ingestion_service
from models.content import Document

# Example of ingesting content
ingestion_service = get_ingestion_service()

# Create documents from textbook content
documents = [
    Document(
        id="doc1",
        content="Textbook content here...",
        metadata={"chapter": "Chapter 1", "section": "Introduction"}
    )
]

# Ingest the documents
result = await ingestion_service.ingest_documents(documents)
```

### Making Queries

Once the service is running and content is ingested, you can make queries to the API:

```bash
curl -X POST "http://localhost:8000/chat/" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "selected_text": null,
    "context": {}
  }'
```

## API Endpoints

### Chat Endpoint
- **Endpoint**: `POST /chat/`
- **Description**: Process natural language queries and return textbook-grounded responses
- **Request Body**:
  ```json
  {
    "query": "string (required, 1-1000 chars)",
    "selected_text": "string (optional, up to 2000 chars)",
    "context": {
      "chapter": "string (optional)",
      "section": "string (optional)",
      "conversation_id": "string (optional)"
    }
  }
  ```
- **Response**:
  ```json
  {
    "response_id": "string",
    "answer": "string",
    "citations": [
      {
        "chapter": "string",
        "section": "string",
        "relevance_score": "number (0-1)",
        "text_snippet": "string"
      }
    ],
    "confidence": "number (0-1)",
    "conversation_id": "string",
    "timestamp": "ISO 8601 datetime"
  }
  ```

### Health Check Endpoints
- **Endpoint**: `GET /health/`
- **Description**: Basic health check
- **Response**: Health status information

- **Endpoint**: `GET /health/detailed`
- **Description**: Detailed health check with service status
- **Response**: Comprehensive health status

- **Endpoint**: `GET /health/metrics`
- **Description**: Performance and runtime metrics
- **Response**: Runtime metrics

## Development

### Running Tests

```bash
# Run unit tests
python -m pytest tests/

# Run integration tests
python test_integration.py

# Run with coverage
python -m pytest --cov=services tests/
```

### Code Style

- Follow PEP 8 guidelines
- Use type hints for all function signatures
- Write docstrings for all public functions and classes
- Use Pydantic for data validation

### Adding New Features

1. Create new service files in the `services/` directory
2. Add new route files in the `routes/` directory
3. Update the main application to include new routes
4. Write unit and integration tests
5. Update documentation

## Deployment

### Production Deployment with Docker

1. Build the production image:
```bash
docker build -t rag-chatbot-service .
```

2. Run with Docker Compose:
```bash
docker-compose -f docker-compose.prod.yml up -d
```

### Kubernetes Deployment

Apply the Kubernetes configuration:
```bash
kubectl apply -f k8s-deployment.yaml
```

### Environment-Specific Configurations

- **Development**: Permissive CORS, detailed logging, single worker
- **Staging**: Limited CORS, info logging, multiple workers
- **Production**: Restricted CORS, info logging, multiple workers, resource limits

## Troubleshooting

### Common Issues

**Issue**: Service won't start due to missing dependencies
**Solution**: Ensure all dependencies are installed with `pip install -r requirements.txt`

**Issue**: Qdrant connection fails
**Solution**: Verify Qdrant URL and API key in environment variables

**Issue**: LLM API returns errors
**Solution**: Check API key and ensure sufficient quota is available

**Issue**: Slow response times
**Solution**: Check vector database performance and consider optimizing embeddings

### Logging

The service logs to both console and file (configured in `logging_config.py`):
- **Development**: Logs to console with DEBUG level
- **Production**: Logs to file with INFO level

### Monitoring

The service provides several endpoints for monitoring:
- `/health/` - Basic health check
- `/health/detailed` - Detailed service status
- `/health/metrics` - Performance metrics

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests for your changes
5. Run all tests (`python -m pytest`)
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For support, please open an issue in the GitHub repository or contact the development team.

---

Built with ❤️ for the Physical AI & Humanoid Robotics Community