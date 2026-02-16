# Research: RAG Chatbot for Physical AI Book

**Date**: 2026-01-23
**Feature**: 005-rag-chatbot

## RAG System Architecture

### Data Flow Components
The RAG system follows a three-stage architecture: Ingestion → Retrieval → Generation. During ingestion, textbook content is parsed, chunked, and embedded for storage in a vector database. During retrieval, user queries are processed to find the most relevant content chunks. During generation, an LLM creates responses grounded in the retrieved context.

**Decision**: Implement three-stage pipeline with clear separation of concerns
**Rationale**: This provides modularity, making each component independently testable and optimizable
**Alternatives considered**: End-to-end neural models, direct neural retrieval, monolithic architectures

### OpenAI Agents SDK Integration
The OpenAI Agents SDK provides orchestration capabilities for connecting various components in the RAG system. Though the underlying LLM will be Gemini, the SDK provides a consistent interface for agent creation, tool integration, and response generation.

**Decision**: Use OpenAI Agents SDK as the orchestration layer with Gemini as the backend
**Rationale**: Meets hackathon requirements while providing robust agent capabilities
**Alternatives considered**: LangChain, LlamaIndex, custom orchestration frameworks

### Frontend-Backend Separation
The RAG system separates concerns between the Docusaurus frontend that provides the user interface and the FastAPI backend that handles the RAG processing. This allows for independent scaling and development of each component.

**Decision**: Implement clear API boundary between frontend and backend components
**Rationale**: Enables independent development, testing, and deployment of components
**Alternatives considered**: Monolithic architecture, client-side RAG processing

## Content Ingestion Pipeline

### Markdown Parsing Strategy
The system needs to parse the textbook's markdown content while preserving semantic structure and metadata. This includes handling headers, code blocks, lists, and other markdown elements that provide context for the content.

**Decision**: Use markdown libraries that preserve structure and convert to plain text for chunking
**Rationale**: Maintains content integrity while providing clean text for embedding
**Alternatives considered**: Direct text extraction, HTML conversion, custom parsing

### Chunking Strategy
Content chunking is critical for effective retrieval. Chunks must be large enough to contain complete concepts but small enough to maintain precision in retrieval.

**Decision**: Implement semantic chunking with overlap to preserve context across boundaries
**Rationale**: Balances retrieval precision with context preservation
**Alternatives considered**: Fixed-length chunking, sentence-level chunking, document-level chunking

### Embedding Generation
Generating high-quality embeddings is essential for effective retrieval. The embeddings must capture semantic meaning while being efficient to compute and store.

**Decision**: Use sentence-transformers with a model optimized for domain-specific content
**Rationale**: Good balance of semantic understanding and computational efficiency
**Alternatives considered**: OpenAI embeddings, custom neural embeddings, keyword-based indexing

## Retrieval Layer

### Similarity Search Workflow
The retrieval process involves converting user queries to embeddings, searching the vector database for similar content, and ranking results by relevance. This requires careful tuning of similarity thresholds and ranking algorithms.

**Decision**: Use cosine similarity with configurable thresholds and reranking
**Rationale**: Cosine similarity provides good semantic matching for text embeddings
**Alternatives considered**: Euclidean distance, dot product, specialized ranking models

### Context Filtering and Ranking
Not all retrieved content may be relevant to the specific query. Context filtering ensures that only the most relevant content is provided to the generation stage.

**Decision**: Implement multi-stage filtering with initial similarity followed by semantic relevance scoring
**Rationale**: Improves answer quality by reducing noise from marginally relevant content
**Alternatives considered**: Simple threshold filtering, re-ranking models, keyword matching

### Selected-Text Override Mechanism
For selected-text queries, the system must prioritize the selected content over general retrieval results. This requires special handling of user-provided context.

**Decision**: Implement context boosting where selected text is given higher priority in generation
**Rationale**: Ensures selected-text queries are properly grounded in the user-provided context
**Alternatives considered**: Separate processing pipeline, context replacement, hybrid approach

## Agent-Based Answer Generation

### OpenAI Agent Configuration
The agent needs to be configured to work with the RAG system, using the retrieved context as tools or knowledge sources. The agent should be constrained to only use information from the textbook content.

**Decision**: Configure agent with strict grounding rules and context injection
**Rationale**: Prevents hallucination while enabling flexible question answering
**Alternatives considered**: Fine-tuning, prompt engineering, retrieval-augmented models

### Gemini Backend Integration
Using Gemini as the LLM backend through the OpenAI Agents SDK requires proper configuration of API keys and model parameters to ensure optimal performance.

**Decision**: Use Gemini Pro with appropriate parameters for educational content generation
**Rationale**: Good balance of capability and cost for educational applications
**Alternatives considered**: Other Gemini models, alternative LLMs, open-source models

### Context-Grounded Response Generation
The generation process must ensure that responses are strictly grounded in the retrieved context without introducing external knowledge.

**Decision**: Implement grounding validation and response filtering
**Rationale**: Maintains the requirement that answers come only from textbook content
**Alternatives considered**: Model fine-tuning, retrieval-only responses, hybrid approaches

### Hallucination Prevention
Preventing the generation of content not present in the textbook is critical for educational integrity.

**Decision**: Implement multiple validation layers including grounding checks and citation verification
**Rationale**: Multiple layers provide defense against hallucination while maintaining response quality
**Alternatives considered**: Output filtering, model constraints, human validation

## Technical Decisions

### Vector Database Selection
Selected Qdrant Cloud for the vector database due to:
- Strong similarity search capabilities with configurable algorithms
- Good performance with educational-scale datasets
- Cloud-hosted solution reducing infrastructure complexity
- Active development and community support
- Free tier availability supporting the project constraints

**Rationale**: Best balance of features, performance, and cost for the educational use case
**Alternatives considered**: Pinecone, Weaviate, Chroma, local solutions

### Backend Framework Selection
Selected FastAPI for the backend service due to:
- High performance with async support
- Automatic API documentation generation
- Strong typing with Pydantic integration
- Excellent for ML/AI service deployment
- Good integration with Python ML ecosystem

**Rationale**: Optimal for the RAG service requirements with performance and developer experience benefits
**Alternatives considered**: Flask, Django, Express.js, other frameworks

### Frontend Integration
Selected Docusaurus integration for the chatbot UI due to:
- Existing framework for the textbook
- Good plugin ecosystem for custom components
- Static site generation with client-side interactivity
- Markdown compatibility with existing content
- React-based allowing for rich interactive components

**Rationale**: Seamless integration with existing textbook structure and deployment model
**Alternatives considered**: Standalone React app, separate frontend framework, iframe embedding