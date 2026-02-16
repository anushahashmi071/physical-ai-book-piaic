from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime


class UserContext(BaseModel):
    """User context for the query"""
    current_chapter: Optional[str] = None
    knowledge_level: Optional[str] = None  # beginner, intermediate, advanced
    conversation_history: Optional[List[Dict[str, Any]]] = []


class QueryRequest(BaseModel):
    """Request model for the query endpoint"""
    query_id: Optional[str] = None
    query_text: str = Field(..., min_length=3, max_length=500)
    selected_text: Optional[str] = Field(None, max_length=1000)
    context: Optional[Dict[str, Any]] = {}
    user_context: Optional[UserContext] = None
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    def __init__(self, **data):
        super().__init__(**data)
        if not self.query_id:
            self.query_id = str(uuid4())


class QueryResponse(BaseModel):
    """Response model for the query endpoint"""
    response_id: str
    query_id: str
    answer: str
    citations: List[Dict[str, Any]]
    confidence: float = Field(ge=0.0, le=1.0)
    conversation_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class ChatRequest(BaseModel):
    """Request model for the chat endpoint"""
    query: str = Field(..., min_length=1, max_length=500)
    selected_text: Optional[str] = Field(None, max_length=1000)
    context: Optional[Dict[str, Any]] = {
        "chapter": None,
        "section": None,
        "conversation_id": None
    }


class ChatResponse(BaseModel):
    """Response model for the chat endpoint"""
    response_id: str
    answer: str
    citations: List[Dict[str, Any]]
    confidence: float = Field(ge=0.0, le=1.0)
    conversation_id: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)