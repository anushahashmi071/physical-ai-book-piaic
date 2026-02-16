from typing import Dict, Any, List, Optional
import logging
from openai import AsyncOpenAI
from config.settings import settings
from models.content import RetrievedChunk
from models.query import QueryRequest
from services.retrieval import get_retrieval_service
from services.generation import get_generation_service
from services.conversation import get_conversation_service


logger = logging.getLogger(__name__)


class OpenAIAgentService:
    """
    Service that orchestrates the RAG pipeline using OpenAI-compatible interface
    with Gemini backend to ensure responses are grounded in textbook content.
    """

    def __init__(self):
        # Initialize OpenAI client with appropriate backend
        try:
            if settings.llm_provider == "gemini" and settings.gemini_api_key:
                self.client = AsyncOpenAI(
                    api_key=settings.gemini_api_key,
                    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
                )
                self.model_name = settings.model_name or "gemini-1.5-pro-latest"
            elif settings.llm_provider == "openai" and settings.openai_api_key:
                self.client = AsyncOpenAI(api_key=settings.openai_api_key)
                self.model_name = settings.model_name or "gpt-4-turbo"
            else:
                # Handle missing API keys
                logger.warning("No LLM API key configured. Using mock client.")
                self.client = None
                self.model_name = "mock-model"
        except Exception as e:
            logger.warning(f"Failed to initialize LLM client: {str(e)}. Using mock client.")
            self.client = None
            self.model_name = "mock-model"

        # Initialize dependencies
        self.retrieval_service = get_retrieval_service()
        self.generation_service = get_generation_service()
        self.conversation_service = get_conversation_service()

    async def process_query(
        self,
        query: str,
        selected_text: Optional[str] = None,
        context_filters: Optional[Dict[str, Any]] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Process a user query through the complete RAG pipeline.

        Args:
            query: User's natural language query
            selected_text: Optional selected text for context-specific queries
            context_filters: Optional filters for content retrieval (chapter, section, etc.)
            conversation_history: Optional conversation history for context

        Returns:
            Dictionary containing the response and metadata
        """
        try:
            # Step 1: Retrieve relevant content based on the query
            retrieved_chunks = await self.retrieval_service.retrieve_relevant_content(
                query=query,
                selected_text=selected_text,
                top_k=settings.retrieval_top_k,
                filters=context_filters
            )

            # Check if we have any relevant content
            if not retrieved_chunks or len(retrieved_chunks) == 0:
                logger.warning(f"No relevant content found for query: {query[:50]}...")

                # Return a specific fallback response when no content is found
                return {
                    "query": query,
                    "answer": (
                        "I couldn't find relevant content in the textbook to answer your question: "
                        f"'{query}'. This might mean the topic isn't covered in the current textbook "
                        "sections, or the question is phrased in a way that doesn't match the textbook content. "
                        "I recommend reviewing the relevant textbook chapters or trying to rephrase your question."
                    ),
                    "citations": [],
                    "confidence": 0.1,  # Very low confidence when no content found
                    "grounding_validation": {"is_properly_grounded": False, "reason": "no_content_found"},
                    "retrieved_chunks_count": 0,
                    "processing_successful": True  # This is still a successful processing, just no content found
                }

            # Determine user knowledge level from context
            user_knowledge_level = "beginner"  # Default
            if context_filters and "knowledge_level" in context_filters:
                user_knowledge_level = context_filters.get("knowledge_level", "beginner")

            # Step 2: Generate response based on retrieved content
            response_data = await self.generation_service.generate_response(
                query=query,
                retrieved_context=retrieved_chunks,
                selected_text=selected_text,
                user_knowledge_level=user_knowledge_level,
                temperature=0.3
            )

            # Step 3: Validate grounding in textbook content
            validation_result = await self.validate_grounding(
                response_data["answer"],
                retrieved_chunks,
                selected_text
            )

            # Step 4: Compile final response
            result = {
                "query": query,
                "answer": response_data["answer"],
                "citations": response_data["citations"],
                "confidence": response_data["confidence"],
                "grounding_validation": validation_result,
                "retrieved_chunks_count": len(retrieved_chunks),
                "processing_successful": True
            }

            logger.info(f"Successfully processed query: {query[:50]}...")
            return result

        except Exception as e:
            logger.error(f"Error processing query '{query[:50]}...': {str(e)}")

            # Return a fallback response for any exception
            return {
                "query": query,
                "answer": (
                    "I apologize, but I'm currently unable to process your query due to a service issue. "
                    "This could be due to:\n"
                    "- Temporary unavailability of the AI service\n"
                    "- Issues with content retrieval\n"
                    "- Network connectivity problems\n\n"
                    "Please try again later, or consult the relevant textbook sections directly. "
                    "If the problem persists, please contact support."
                ),
                "citations": [],
                "confidence": 0.0,
                "grounding_validation": {"is_properly_grounded": False, "error": str(e)},
                "retrieved_chunks_count": 0,
                "processing_successful": False,
                "error": str(e)
            }

    async def validate_grounding(self, response: str, retrieved_chunks: List[RetrievedChunk], selected_text: str = None) -> Dict[str, Any]:
        """
        Validate that the response is properly grounded in the retrieved content.

        Args:
            response: Generated response to validate
            retrieved_chunks: Chunks that were used to generate the response
            selected_text: Optional selected text for context-specific validation

        Returns:
            Validation results
        """
        if not retrieved_chunks:
            return {
                "is_properly_grounded": False,
                "reason": "No context provided for response generation"
            }

        # Check if the response makes references to content in the retrieved chunks
        response_lower = response.lower()
        context_text = " ".join([chunk.content.lower() for chunk in retrieved_chunks])

        # Simple validation: check if key terms from context appear in response
        context_words = set(context_text.split())
        response_words = set(response_lower.split())

        overlap = len(context_words.intersection(response_words))
        total_context_words = len(context_words)

        # Calculate grounding score based on vocabulary overlap and other heuristics
        if total_context_words == 0:
            grounding_score = 0.0
        else:
            # Weight vocabulary overlap but also consider other factors
            vocab_overlap_ratio = overlap / total_context_words

            # Additional heuristic: check if citations reference the provided chunks
            has_citations = len([chunk for chunk in retrieved_chunks if chunk.source.chapter]) > 0

            grounding_score = min(vocab_overlap_ratio * 1.5, 1.0)  # Boost ratio slightly
            if has_citations:
                grounding_score = min(grounding_score + 0.2, 1.0)  # Bonus for citations

        # Additional validation for selected text queries
        selected_text_compliance = 1.0  # Default value
        if selected_text:
            # Check if the response addresses the selected text appropriately
            selected_text_lower = selected_text.lower()

            # Check if key concepts from selected text appear in the response
            selected_words = set(selected_text_lower.split())
            response_words = set(response_lower.split())

            selected_overlap = len(selected_words.intersection(response_words))
            selected_coverage = selected_overlap / len(selected_words) if selected_words else 1.0

            # Adjust compliance based on how well the response addresses the selected text
            selected_text_compliance = min(selected_coverage * 2.0, 1.0)  # Boost the coverage slightly but cap at 1.0

        # Overall validation considering both general grounding and selected text compliance
        if selected_text:
            # Combine grounding score with selected text compliance
            combined_score = (grounding_score * 0.7 + selected_text_compliance * 0.3)
            is_properly_grounded = combined_score > 0.7
        else:
            is_properly_grounded = grounding_score > 0.3  # Adjust threshold as needed

        return {
            "is_properly_grounded": is_properly_grounded,
            "grounding_score": grounding_score,
            "selected_text_compliance": selected_text_compliance,
            "combined_score": grounding_score if not selected_text else (grounding_score * 0.7 + selected_text_compliance * 0.3),
            "vocabulary_overlap": overlap,
            "total_context_words": total_context_words,
            "has_citations": has_citations,
            "selected_text_addressed": bool(selected_text) and selected_text_compliance > 0.5
        }

    async def process_follow_up_query(
        self,
        query: str,
        conversation_context: List[Dict[str, str]]
    ) -> Dict[str, Any]:
        """
        Process a follow-up query that should consider the conversation history.

        Args:
            query: User's follow-up query
            conversation_context: History of the conversation

        Returns:
            Dictionary containing the response and metadata
        """
        # For follow-up queries, we might want to retrieve content related to the conversation
        # For now, we'll use the regular process but with conversation context
        context_str = " ".join([
            f"Previous question: {item['query']}. Previous answer: {item['answer']}."
            for item in conversation_context[-2:]  # Last 2 exchanges
        ])

        full_query = f"{context_str} Current question: {query}" if context_str else query

        return await self.process_query(full_query)

    def get_agent_config(self) -> Dict[str, Any]:
        """
        Get the current agent configuration.

        Returns:
            Dictionary containing agent configuration
        """
        return {
            "model_name": self.model_name,
            "provider": settings.llm_provider,
            "temperature": 0.3,
            "max_tokens": 1000,
            "retrieval_top_k": settings.retrieval_top_k,
            "grounding_constraints": {
                "prevent_hallucination": True,
                "require_citations": True,
                "validate_context_use": True
            }
        }


# Global instance for easy access
agent_service = OpenAIAgentService()


def get_agent_service() -> OpenAIAgentService:
    """
    Get the agent service instance.

    Returns:
        OpenAIAgentService instance
    """
    return agent_service