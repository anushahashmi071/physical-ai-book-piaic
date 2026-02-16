# API Contracts: RAG Chatbot for Physical AI Book

**Date**: 2026-01-23
**Feature**: 005-rag-chatbot

## Core API Endpoints

### Chat Endpoint
**Path**: `POST /chat`
**Purpose**: Process natural language queries and return textbook-grounded responses

**Request**:
```json
{
  "query": "string (required)",
  "selected_text": "string (optional)",
  "context": {
    "chapter": "string (optional)",
    "section": "string (optional)",
    "conversation_id": "string (optional)"
  }
}
```

**Response**:
```json
{
  "response_id": "string",
  "answer": "string",
  "citations": [
    {
      "chapter": "string",
      "section": "string",
      "relevance_score": "number (0-1)"
    }
  ],
  "confidence": "number (0-1)",
  "conversation_id": "string",
  "timestamp": "ISO 8601 datetime"
}
```

**Success Criteria**: Returns grounded response within 3 seconds with proper citations
**Error Responses**:
- 400: Invalid query format
- 500: Processing error or service unavailable

### Content Ingestion Endpoint
**Path**: `POST /ingest`
**Purpose**: Add textbook content to the vector database for retrieval

**Request**:
```json
{
  "documents": [
    {
      "id": "string (required)",
      "content": "string (required)",
      "metadata": {
        "source": "string (chapter/section reference)",
        "type": "string (text, code, diagram reference)",
        "tags": ["string"]
      }
    }
  ],
  "options": {
    "chunk_size": "number (default: 512)",
    "overlap": "number (default: 64)",
    "reindex": "boolean (default: false)"
  }
}
```

**Response**:
```json
{
  "job_id": "string",
  "status": "string (queued, processing, completed, failed)",
  "processed_documents": "number",
  "failed_documents": "number",
  "timestamp": "ISO 8601 datetime"
}
```

**Success Criteria**: Successfully processes and indexes all valid documents
**Error Responses**:
- 400: Invalid document format
- 500: Ingestion service error

### Health Check Endpoint
**Path**: `GET /health`
**Purpose**: Check the health and availability of the RAG service

**Request**: No body required

**Response**:
```json
{
  "status": "string (healthy, degraded, unavailable)",
  "services": {
    "vector_db": "boolean",
    "llm_backend": "boolean",
    "content_index": "boolean"
  },
  "uptime": "number (seconds)",
  "timestamp": "ISO 8601 datetime"
}
```

**Success Criteria**: Returns health status within 1 second
**Error Responses**: 500 if service is completely unavailable

## Service Contracts

### Content Retrieval Service
**Interface**: `retrieve_relevant_content(query: str, selected_text: Optional[str] = None, top_k: int = 5) -> List[ContentChunk]`

**Contract**:
- Takes natural language query and optional selected text context
- Returns top-k most relevant content chunks from textbook
- Each chunk includes text, source reference, and similarity score
- Must complete within 2 seconds for 95% of requests
- Must return chunks with semantic relevance > 0.6

### Response Generation Service
**Interface**: `generate_response(retrieved_context: List[ContentChunk], query: str) -> GeneratedResponse`

**Contract**:
- Takes retrieved content chunks and original query
- Generates response strictly grounded in provided context
- Returns response with confidence score and citations
- Must complete within 3 seconds for 95% of requests
- Must not generate content outside of provided context (>95% accuracy)

### Content Ingestion Service
**Interface**: `ingest_documents(documents: List[Document]) -> IngestionResult`

**Contract**:
- Takes list of textbook documents with metadata
- Processes documents through chunking and embedding pipeline
- Stores embeddings in vector database with metadata
- Returns processing status and statistics
- Must maintain 99% data integrity during ingestion

## Data Contracts

### Content Chunk Schema
```json
{
  "chunk_id": "string (required)",
  "content": "string (required, max 1000 chars)",
  "source": {
    "document_id": "string (required)",
    "chapter": "string (required)",
    "section": "string",
    "page_reference": "string"
  },
  "embedding": "array of numbers (required, dimensions vary by model)",
  "metadata": {
    "tags": ["string"],
    "concepts": ["string"],
    "difficulty_level": "string (beginner, intermediate, advanced)"
  },
  "created_at": "ISO 8601 datetime"
}
```

### Query Request Schema
```json
{
  "query_id": "string (optional, auto-generated if not provided)",
  "query_text": "string (required, min 3 chars, max 500 chars)",
  "selected_text": "string (optional, max 1000 chars)",
  "user_context": {
    "current_chapter": "string (optional)",
    "knowledge_level": "string (optional, beginner/intermediate/advanced)",
    "conversation_history": [
      {
        "query": "string",
        "response_id": "string",
        "timestamp": "ISO 8601 datetime"
      }
    ]
  },
  "timestamp": "ISO 8601 datetime"
}
```

### Generated Response Schema
```json
{
  "response_id": "string (required)",
  "query_id": "string (required)",
  "answer": "string (required)",
  "citations": [
    {
      "source_document": "string (required)",
      "chapter": "string (required)",
      "section": "string",
      "text_snippet": "string (max 200 chars)",
      "similarity_score": "number (0-1, required)"
    }
  ],
  "confidence_score": "number (0-1, required)",
  "processing_time_ms": "number (required)",
  "grounding_accuracy": "number (0-1, required)",
  "timestamp": "ISO 8601 datetime"
}
```

## Performance Contracts

### Response Time Requirements
- **Query Processing**: 95% of requests completed within 3 seconds
- **Content Retrieval**: 95% of retrieval operations within 1.5 seconds
- **Response Generation**: 95% of generation operations within 1.5 seconds

### Accuracy Requirements
- **Content Retrieval**: 90%+ semantic relevance for top-5 results
- **Response Grounding**: 95%+ of responses must be grounded in provided context
- **Citation Accuracy**: 95%+ of citations must correctly reference textbook content

### Availability Requirements
- **Service Uptime**: 99%+ availability during educational hours
- **Content Index**: 100% of textbook content must be searchable
- **Concurrent Users**: Support up to 50 concurrent queries with maintained performance

## Error Handling Contracts

### Graceful Degradation
- If LLM service unavailable: Return message indicating temporary unavailability
- If vector database unavailable: Return error with suggestion to try again later
- If content retrieval fails: Acknowledge inability to answer from book content
- If query is out of scope: Politely redirect to relevant textbook sections

### Fallback Mechanisms
- Provide general guidance when specific content cannot be retrieved
- Maintain conversation state even during partial service failures
- Log errors for system improvement while preserving user privacy