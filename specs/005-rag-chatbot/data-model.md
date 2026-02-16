# Data Model: RAG Chatbot for Physical AI Book

**Date**: 2026-01-23
**Feature**: 005-rag-chatbot

## Key Entities

### Textbook Content Chunk
**Definition**: A semantically coherent segment of textbook content that has been processed for vector storage
**Attributes**:
- chunk_id: Unique identifier for the content chunk
- content_text: The actual text content of the chunk
- source_document: Reference to the original document/chapter
- source_section: Specific section within the document
- embedding_vector: High-dimensional vector representation of the content
- metadata: Additional information like chapter, section, concepts covered
- validation_rules: Content accuracy and relevance requirements

### Query Request
**Definition**: A user's natural language question or request for information
**Attributes**:
- query_id: Unique identifier for the query
- query_text: The natural language text of the query
- selected_text: Optional user-selected text for context-specific queries
- timestamp: Time the query was submitted
- user_context: Optional context about the user's current location in the textbook
- validation_rules: Query format and content requirements

### Retrieved Context
**Definition**: Relevant content chunks retrieved based on similarity to the user's query
**Attributes**:
- query_id: Reference to the original query
- retrieved_chunks: List of relevant content chunks
- similarity_scores: Confidence/relevance scores for each chunk
- context_window: Amount of context provided to the generation system
- ranking_metadata: Information about how chunks were ranked
- validation_rules: Relevance and accuracy requirements for retrieved content

### Generated Response
**Definition**: AI-generated answer to the user's query, grounded in retrieved context
**Attributes**:
- response_id: Unique identifier for the response
- query_id: Reference to the original query
- response_text: The generated answer text
- cited_sections: References to specific textbook sections used
- confidence_score: Confidence in the accuracy of the response
- generation_timestamp: Time the response was generated
- validation_rules: Accuracy and grounding requirements

### Vector Database Entry
**Definition**: Storage record in the vector database containing embeddings and metadata
**Attributes**:
- entry_id: Unique identifier for the database entry
- embedding: High-dimensional vector representation
- payload: Associated metadata and original content
- similarity_threshold: Minimum similarity required for retrieval
- validation_rules: Embedding quality and storage requirements

### Agent Configuration
**Definition**: Settings and parameters for the OpenAI Agent system
**Attributes**:
- agent_id: Unique identifier for the agent configuration
- llm_provider: Backend LLM provider (e.g., "gemini")
- model_name: Specific model to use
- temperature: Generation randomness parameter
- max_tokens: Maximum tokens in response
- grounding_constraints: Rules to prevent hallucination
- validation_rules: Configuration safety and performance requirements

### Conversation Context
**Definition**: Maintained context for multi-turn conversations
**Attributes**:
- conversation_id: Unique identifier for the conversation
- query_history: Previous queries and responses in the conversation
- user_state: Current state of the user's learning journey
- context_retention_time: How long to maintain conversation history
- validation_rules: Privacy and data retention requirements

### Content Ingestion Job
**Definition**: Process for importing and preparing textbook content for RAG
**Attributes**:
- job_id: Unique identifier for the ingestion job
- source_files: List of files to be ingested
- status: Current status of the job (processing, completed, failed)
- completion_percentage: Progress of the ingestion process
- error_log: Any errors encountered during ingestion
- validation_rules: Data integrity and completeness requirements

### User Interaction Log
**Definition**: Record of user interactions with the RAG system for analysis
**Attributes**:
- interaction_id: Unique identifier for the interaction
- query: Original user query
- response: System response provided
- timestamp: Time of interaction
- user_satisfaction: Optional user feedback on response quality
- validation_rules: Privacy and logging compliance requirements

## Entity Relationships

```
[Query Request] --(triggers)--> [Retrieved Context] --(used by)--> [Generated Response]
[Textbook Content Chunk] --(stored as)--> [Vector Database Entry]
[Generated Response] --(part of)--> [Conversation Context]
[Content Ingestion Job] --(creates)--> [Textbook Content Chunks]
[Query Request] --(processed by)--> [Agent Configuration]
[User Interaction Log] --(records)--> [Query Request + Generated Response]
```

## State Transitions

### Content Ingestion States
- **Queued**: Ingestion job awaiting processing
- **Processing**: Content being parsed, chunked, and embedded
- **Embedding**: Vector embeddings being generated
- **Storing**: Content being stored in vector database
- **Completed**: Content successfully ingested and indexed
- **Failed**: Ingestion process encountered an error
- **state_validations**: Ensure proper data integrity throughout process

### Agent Processing States
- **Idle**: Agent awaiting new queries
- **Retrieving**: Searching vector database for relevant content
- **Generating**: Creating response based on retrieved context
- **Validating**: Checking response for grounding and accuracy
- **Responding**: Returning response to user
- **Error**: Handling processing failures
- **state_validations**: Ensure proper transition and error handling

## Data Flow Patterns

### RAG Query Flow
Query Request → Content Retrieval → Context Assembly → Response Generation → Validation → Response Delivery

### Content Ingestion Flow
Source Documents → Parsing → Chunking → Embedding Generation → Vector Storage → Indexing → Available for Retrieval

### Response Validation Flow
Generated Response → Grounding Check → Citation Verification → Quality Validation → User Delivery

## Validation Rules

- **Response Accuracy**: All responses must be grounded in retrieved textbook content with 90%+ precision
- **Latency**: System must respond to queries within 3 seconds for 95%+ of requests
- **Reliability**: System must maintain 99%+ uptime with graceful degradation during outages
- **Safety**: All responses must pass hallucination checks and safety validation
- **Concurrency**: System must handle up to 50 concurrent user queries as specified in requirements
- **Citation Quality**: 95%+ of responses must contain proper citations to specific textbook sections
- **Content Integrity**: All textbook content must be accurately represented in the vector database
- **Privacy**: User interactions must comply with educational privacy requirements