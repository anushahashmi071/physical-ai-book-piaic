from typing import List, Dict, Any
import logging
from openai import AsyncOpenAI
from config.settings import settings
from models.content import RetrievedChunk
from models.query import QueryRequest
from utils.embedding import get_embedding_generator
import asyncio
import json


logger = logging.getLogger(__name__)


class ResponseGenerationService:
    """
    Service for generating responses using LLM based on retrieved context.
    Ensures responses are grounded in textbook content and properly cited.
    """

    def __init__(self):
        # Initialize OpenAI client with error handling for missing API keys
        try:
            if settings.llm_provider == "openai":
                if settings.openai_api_key:
                    self.client = AsyncOpenAI(api_key=settings.openai_api_key)
                    self.model_name = settings.model_name or "gpt-4-turbo"
                else:
                    # Handle missing API key
                    logger.warning("OpenAI API key not configured. Using mock client.")
                    self.client = None
                    self.model_name = "mock-model"
            else:
                # For Gemini, we'll use the OpenAI-compatible interface
                if settings.gemini_api_key:
                    self.client = AsyncOpenAI(
                        api_key=settings.gemini_api_key,
                        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
                    )
                    self.model_name = settings.model_name or "gemini-1.5-pro-latest"
                else:
                    # Handle missing API key
                    logger.warning("Gemini API key not configured. Using mock client.")
                    self.client = None
                    self.model_name = "mock-model"
        except Exception as e:
            logger.warning(f"Failed to initialize LLM client: {str(e)}. Using mock client.")
            self.client = None
            self.model_name = "mock-model"

    async def generate_response(
        self,
        query: str,
        retrieved_context: List[RetrievedChunk],
        selected_text: str = None,
        user_knowledge_level: str = "beginner",  # beginner, intermediate, advanced
        temperature: float = 0.3
    ) -> Dict[str, Any]:
        """
        Generate a response based on the query and retrieved context.

        Args:
            query: User's original query
            retrieved_context: List of retrieved content chunks with similarity scores
            selected_text: Optional selected text for context-specific queries
            user_knowledge_level: User's knowledge level (beginner, intermediate, advanced)
            temperature: Temperature parameter for response generation

        Returns:
            Dictionary containing the generated response and metadata
        """
        try:
            # Prepare the context for the LLM
            context_text = self._prepare_context_for_llm(retrieved_context)

            # Build the system message to ground the response in textbook content
            system_message = self._build_system_message(selected_text, user_knowledge_level)

            # Build the user message with query and context
            user_message = self._build_user_message(query, context_text)

            # Check if client is available
            if self.client is None:
                # Use a mock response for development/testing
                logger.warning("LLM client not available, using mock response")

                # Generate a simple response based on the context
                answer = f"Based on the textbook content: {context_text[:500]}..."
                citations = self._generate_citations(retrieved_context)
                confidence = self._calculate_confidence(retrieved_context)

                return {
                    "answer": answer,
                    "citations": citations,
                    "confidence": confidence,
                    "retrieved_chunks_used": len(retrieved_context),
                    "processing_time": 0
                }

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_message}
                ],
                temperature=temperature,
                max_tokens=1000,
                timeout=30
            )

            # Extract the generated answer
            answer = response.choices[0].message.content.strip()

            # Generate citations from the retrieved context
            citations = self._generate_citations(retrieved_context)

            # Calculate confidence based on similarity scores
            confidence = self._calculate_confidence(retrieved_context)

            logger.info(f"Generated response for query: {query[:50]}...")

            return {
                "answer": answer,
                "citations": citations,
                "confidence": confidence,
                "retrieved_chunks_used": len(retrieved_context),
                "processing_time": response.usage.total_tokens if hasattr(response, 'usage') else 0
            }

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return {
                "answer": "I apologize, but I'm unable to generate a response at the moment. Please try again later.",
                "citations": [],
                "confidence": 0.0,
                "error": str(e)
            }

    def _prepare_context_for_llm(self, retrieved_context: List[RetrievedChunk]) -> str:
        """
        Prepare the retrieved context in a format suitable for the LLM.

        Args:
            retrieved_context: List of retrieved content chunks

        Returns:
            Formatted context string
        """
        if not retrieved_context:
            return "No relevant content was found in the textbook to answer this question."

        context_parts = ["Relevant textbook content:"]
        for i, chunk in enumerate(retrieved_context, 1):
            context_parts.append(
                f"\n{i}. From {chunk.source.chapter}"
                f"{f', Section {chunk.source.section}' if chunk.source.section else ''}:"
                f"\n{chunk.content}\n"
                f"(Similarity Score: {chunk.similarity_score:.2f})"
            )

        return "\n".join(context_parts)

    def _build_system_message(self, selected_text: str = None, user_knowledge_level: str = "beginner") -> str:
        """
        Build the system message to guide the LLM's behavior.

        Args:
            selected_text: Optional selected text for context-specific queries
            user_knowledge_level: User's knowledge level (beginner, intermediate, advanced)

        Returns:
            System message string
        """
        # Define instructions based on user knowledge level
        if user_knowledge_level == "beginner":
            level_instructions = (
                "Provide clear, simple explanations with basic terminology. "
                "Avoid advanced concepts that the user may not have learned yet. "
                "Use analogies and simple examples to explain complex concepts."
            )
        elif user_knowledge_level == "intermediate":
            level_instructions = (
                "Provide detailed explanations with appropriate technical terminology. "
                "Include relevant concepts that build on foundational knowledge. "
                "Balance depth with clarity."
            )
        else:  # advanced
            level_instructions = (
                "Provide comprehensive, technically detailed explanations. "
                "Include advanced concepts and nuanced details as appropriate. "
                "Assume solid foundational knowledge of the subject matter."
            )

        base_instructions = (
            "You are an AI assistant for a Physical AI & Humanoid Robotics textbook. "
            "Your role is to answer questions based strictly on the provided textbook content. "
            f"{level_instructions} "
            "Do not fabricate information or provide knowledge from outside sources. "
            "If the provided context doesn't contain enough information to answer the question, "
            "acknowledge this limitation and suggest the user refer to relevant textbook sections."
        )

        if selected_text:
            context_instruction = (
                f"The user has selected the following text for context: '{selected_text}'. "
                "Please focus your answer on this selected text when possible, "
                "while supplementing with other relevant textbook content if needed."
            )
            return f"{base_instructions} {context_instruction}"
        else:
            return base_instructions

    def _build_user_message(self, query: str, context_text: str) -> str:
        """
        Build the user message containing the query and context.

        Args:
            query: User's original query
            context_text: Prepared context from retrieved content

        Returns:
            User message string
        """
        return (
            f"Question: {query}\n\n"
            f"Textbook Context:\n{context_text}\n\n"
            "Please provide a clear, concise answer based only on the textbook content provided above. "
            "If the answer cannot be found in the provided context, say so explicitly. "
            "Include citations to specific chapters and sections where the information comes from."
        )

    def _generate_citations(self, retrieved_context: List[RetrievedChunk]) -> List[Dict[str, Any]]:
        """
        Generate citations from the retrieved context.

        Args:
            retrieved_context: List of retrieved content chunks

        Returns:
            List of citation dictionaries
        """
        citations = []
        for chunk in retrieved_context:
            if chunk.similarity_score > 0.3:  # Only include chunks with reasonable similarity
                citation = {
                    "chapter": chunk.source.chapter,
                    "section": chunk.source.section,
                    "relevance_score": chunk.similarity_score,
                    "text_snippet": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
                }
                citations.append(citation)

        # Sort by relevance score (descending)
        citations.sort(key=lambda x: x["relevance_score"], reverse=True)
        return citations

    def _calculate_confidence(self, retrieved_context: List[RetrievedChunk]) -> float:
        """
        Calculate confidence in the response based on the retrieved context.

        Args:
            retrieved_context: List of retrieved content chunks

        Returns:
            Confidence score between 0 and 1
        """
        if not retrieved_context:
            return 0.0

        # Calculate average similarity score
        avg_similarity = sum(chunk.similarity_score for chunk in retrieved_context) / len(retrieved_context)

        # Calculate confidence based on both quantity and quality of retrieved content
        num_high_quality = sum(1 for chunk in retrieved_context if chunk.similarity_score > 0.6)

        # Weight the confidence based on both factors
        quality_weight = avg_similarity
        quantity_weight = min(len(retrieved_context) / 5.0, 1.0)  # Up to 5 chunks maximum weight

        confidence = (quality_weight * 0.7 + quantity_weight * 0.3)
        return min(confidence, 1.0)  # Ensure confidence is between 0 and 1

    async def validate_grounding(self, response: str, retrieved_context: List[RetrievedChunk], selected_text: str = None) -> Dict[str, Any]:
        """
        Validate that the response is properly grounded in the retrieved context.

        Args:
            response: Generated response to validate
            retrieved_context: Context used to generate the response
            selected_text: Optional selected text for context-specific validation

        Returns:
            Validation results
        """
        # This would typically involve more sophisticated validation
        # For now, we'll implement basic validation

        context_text = " ".join([chunk.content.lower() for chunk in retrieved_context])
        response_lower = response.lower()

        # Count how much of the response is supported by context
        words_in_context = 0
        total_words = 0

        for word in response_lower.split():
            if word in context_text:
                words_in_context += 1
            total_words += 1

        grounding_score = words_in_context / total_words if total_words > 0 else 0.0

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
            is_properly_grounded = grounding_score > 0.7

        return {
            "grounding_score": grounding_score,
            "selected_text_compliance": selected_text_compliance,
            "combined_score": grounding_score if not selected_text else (grounding_score * 0.7 + selected_text_compliance * 0.3),
            "words_supported": words_in_context,
            "total_words": total_words,
            "is_properly_grounded": is_properly_grounded,
            "selected_text_addressed": bool(selected_text) and selected_text_compliance > 0.5
        }


# Global instance for easy access
generation_service = ResponseGenerationService()


def get_generation_service() -> ResponseGenerationService:
    """
    Get the response generation service instance.

    Returns:
        ResponseGenerationService instance
    """
    return generation_service