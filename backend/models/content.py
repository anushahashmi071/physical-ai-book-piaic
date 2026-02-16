from pydantic import BaseModel, Field
from typing import List, Dict, Any, Optional
from datetime import datetime


class ContentSource(BaseModel):
    """Source information for content chunks"""
    document_id: str
    chapter: str
    section: Optional[str] = None
    page_reference: Optional[str] = None


class ContentMetadata(BaseModel):
    """Metadata for content chunks"""
    tags: List[str] = []
    concepts: List[str] = []
    difficulty_level: str = Field(default="beginner", pattern=r"^(beginner|intermediate|advanced)$")


class ContentChunk(BaseModel):
    """Model for content chunks stored in the vector database"""
    chunk_id: str
    content: str = Field(..., max_length=2000)  # Increased to accommodate longer content chunks
    source: ContentSource
    embedding: Optional[List[float]] = None  # Will be populated after embedding generation
    metadata: ContentMetadata = Field(default_factory=ContentMetadata)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class RetrievedChunk(ContentChunk):
    """Model for retrieved content chunks with similarity scores"""
    similarity_score: float = Field(ge=0.0, le=1.0)


class Document(BaseModel):
    """Model for documents to be ingested"""
    id: str
    content: str
    metadata: Dict[str, Any] = {}


class IngestionOptions(BaseModel):
    """Options for content ingestion"""
    chunk_size: int = 512
    overlap: int = 64
    reindex: bool = False


class IngestionRequest(BaseModel):
    """Request model for content ingestion"""
    documents: List[Document]
    options: IngestionOptions = Field(default_factory=IngestionOptions)


class IngestionResult(BaseModel):
    """Result model for content ingestion"""
    job_id: str
    status: str  # queued, processing, completed, failed
    processed_documents: int
    failed_documents: int
    timestamp: datetime = Field(default_factory=datetime.utcnow)