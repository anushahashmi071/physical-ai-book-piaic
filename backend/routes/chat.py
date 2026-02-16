from fastapi import APIRouter, HTTPException
from typing import Dict, Any
import logging
from models.query import ChatRequest, ChatResponse
from uuid import uuid4
from datetime import datetime
from services.agent import get_agent_service
from services.conversation import get_conversation_service


router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/")
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint to process natural language queries and return textbook-grounded responses.

    Args:
        request: Chat request with query and optional context

    Returns:
        Chat response with answer and citations
    """
    # Generate response ID
    response_id = str(uuid4())
    conversation_service = get_conversation_service()

    # Get or create conversation ID
    conversation_id = request.context.get("conversation_id")
    if not conversation_id:
        conversation_id = await conversation_service.create_new_conversation()
    else:
        # Ensure conversation exists
        if not await _conversation_exists(conversation_id):
            conversation_id = await conversation_service.create_new_conversation()

    try:
        # Get the agent service to process the query
        agent_service = get_agent_service()

        # Prepare context filters from the request
        context_filters = {}
        if request.context and request.context.get("chapter"):
            context_filters["chapter"] = request.context["chapter"]
        if request.context and request.context.get("section"):
            context_filters["section"] = request.context["section"]

        # Process the query through the RAG pipeline
        result = await agent_service.process_query(
            query=request.query,
            selected_text=request.selected_text,
            context_filters=context_filters
        )

        if not result.get("processing_successful", False):
            # Handle case where processing failed
            raise HTTPException(status_code=500, detail=result.get("error", "Processing failed"))

        # Create the response object
        response = ChatResponse(
            response_id=response_id,
            answer=result["answer"],
            citations=result["citations"],
            confidence=result["confidence"],
            conversation_id=conversation_id,
            timestamp=datetime.utcnow()
        )

        # Add to conversation history
        await conversation_service.add_interaction(
            conversation_id=conversation_id,
            query=request.query,
            response=result["answer"]
        )

        logger.info(f"Processed chat request {request.query[:50]}... with response ID {response_id}")

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


async def _conversation_exists(conversation_id: str) -> bool:
    """
    Check if a conversation exists in the conversation service.

    Args:
        conversation_id: ID of the conversation to check

    Returns:
        True if conversation exists, False otherwise
    """
    conversation_service = get_conversation_service()
    context = await conversation_service.get_conversation_context(conversation_id)
    return len(context) > 0


@router.post("/conversation")
async def chat_with_context_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Chat endpoint that maintains conversation context for follow-up questions.

    Args:
        request: Chat request with query and conversation context

    Returns:
        Chat response with answer and citations
    """
    response_id = str(uuid4())
    conversation_id = request.context.get("conversation_id") or str(uuid4())

    try:
        # Get the agent service to process the query with conversation context
        agent_service = get_agent_service()

        # Prepare context filters
        context_filters = {}
        if request.context and request.context.get("chapter"):
            context_filters["chapter"] = request.context["chapter"]
        if request.context and request.context.get("section"):
            context_filters["section"] = request.context["section"]

        # Process the query through the RAG pipeline
        result = await agent_service.process_query(
            query=request.query,
            selected_text=request.selected_text,
            context_filters=context_filters
        )

        if not result.get("processing_successful", False):
            raise HTTPException(status_code=500, detail=result.get("error", "Processing failed"))

        response = ChatResponse(
            response_id=response_id,
            answer=result["answer"],
            citations=result["citations"],
            confidence=result["confidence"],
            conversation_id=conversation_id,
            timestamp=datetime.utcnow()
        )

        logger.info(f"Processed contextual chat request {request.query[:50]}... with conversation ID {conversation_id}")

        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing contextual chat request: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")