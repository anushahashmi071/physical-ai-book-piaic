from typing import List, Optional, Dict, Any
import logging
from models.content import RetrievedChunk, ContentChunk
from models.query import QueryRequest
from utils.embedding import get_embedding_generator
from services.vector_storage import get_vector_storage_service
from config.settings import settings


logger = logging.getLogger(__name__)


class ContentRetrievalService:
    """
    Service for retrieving relevant content based on user queries.
    Handles query processing, embedding generation, and similarity search.
    """

    def __init__(self):
        self.embedding_gen = get_embedding_generator(settings.embedding_model)
        self.vector_storage = get_vector_storage_service()

    async def retrieve_relevant_content(
        self,
        query: str,
        selected_text: Optional[str] = None,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant content chunks based on the query.

        Args:
            query: User's natural language query
            selected_text: Optional selected text for context-specific queries
            top_k: Number of top results to return
            filters: Optional filters to apply to the search

        Returns:
            List of relevant content chunks with similarity scores
        """
        try:
            # If selected text is provided, use context boosting
            if selected_text:
                retrieved_chunks = await self._retrieve_with_context_boosting(
                    query, selected_text, top_k, filters
                )
            else:
                # Standard retrieval for general queries
                retrieved_chunks = await self._retrieve_standard(query, top_k, filters)

            logger.info(f"Retrieved {len(retrieved_chunks)} relevant chunks for query: {query[:50]}...")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving content for query '{query[:50]}...': {str(e)}")
            return []

    async def _retrieve_standard(
        self,
        query: str,
        top_k: int,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[RetrievedChunk]:
        """
        Standard retrieval without context boosting.

        Args:
            query: User's natural language query
            top_k: Number of top results to return
            filters: Optional filters to apply to the search

        Returns:
            List of relevant content chunks with similarity scores
        """
        # Generate embedding for the query
        query_embedding = self.embedding_gen.generate_embedding(query)

        # Perform similarity search
        retrieved_chunks = await self.vector_storage.retrieve_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            filters=filters
        )

        return retrieved_chunks

    async def _retrieve_with_context_boosting(
        self,
        query: str,
        selected_text: str,
        top_k: int,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[RetrievedChunk]:
        """
        Retrieve content with context boosting based on selected text.

        Args:
            query: User's natural language query
            selected_text: Selected text for context-specific queries
            top_k: Number of top results to return
            filters: Optional filters to apply to the search

        Returns:
            List of relevant content chunks with similarity scores
        """
        # Generate embeddings for both query and selected text
        query_embedding = self.embedding_gen.generate_embedding(query)
        selected_text_embedding = self.embedding_gen.generate_embedding(selected_text)

        # Retrieve results based on the query
        query_results = await self.vector_storage.retrieve_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            filters=filters
        )

        # Retrieve results based on the selected text
        selected_results = await self.vector_storage.retrieve_similar(
            query_embedding=selected_text_embedding,
            top_k=top_k,
            filters=filters
        )

        # Combine and rank results with boosting for selected text relevance
        combined_results = self._combine_and_boost_results(
            query_results, selected_results, selected_text
        )

        # Return top_k results after boosting
        return combined_results[:top_k]

    def _combine_and_boost_results(
        self,
        query_results: List[RetrievedChunk],
        selected_results: List[RetrievedChunk],
        selected_text: str
    ) -> List[RetrievedChunk]:
        """
        Combine query results and selected text results with boosting.

        Args:
            query_results: Results from query-based search
            selected_results: Results from selected text-based search
            selected_text: The selected text for context

        Returns:
            Combined and re-ranked results with boosting applied
        """
        # Create a dictionary to store unique chunks with potentially updated scores
        combined_dict = {}

        # Add query results with original scores
        for chunk in query_results:
            combined_dict[chunk.chunk_id] = chunk

        # Add selected text results, boosting scores if they're also in query results
        for chunk in selected_results:
            if chunk.chunk_id in combined_dict:
                # Boost the score for chunks that appear in both results
                existing_chunk = combined_dict[chunk.chunk_id]
                # Average the scores with a bias toward selected text relevance
                boosted_score = (existing_chunk.similarity_score * 0.4 + chunk.similarity_score * 0.6)
                existing_chunk.similarity_score = min(boosted_score, 1.0)
            else:
                # Add new chunk from selected text search
                combined_dict[chunk.chunk_id] = chunk

        # Convert back to list and sort by score
        combined_list = list(combined_dict.values())
        combined_list.sort(key=lambda x: x.similarity_score, reverse=True)

        return combined_list

    async def retrieve_by_chapter(
        self,
        query: str,
        chapter: str,
        top_k: int = 5
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant content chunks from a specific chapter.

        Args:
            query: User's natural language query
            chapter: Chapter to search within
            top_k: Number of top results to return

        Returns:
            List of relevant content chunks with similarity scores
        """
        filters = {"chapter": chapter}
        return await self.retrieve_relevant_content(query, top_k=top_k, filters=filters)

    async def retrieve_by_section(
        self,
        query: str,
        chapter: str,
        section: str,
        top_k: int = 5
    ) -> List[RetrievedChunk]:
        """
        Retrieve relevant content chunks from a specific section.

        Args:
            query: User's natural language query
            chapter: Chapter to search within
            section: Section to search within
            top_k: Number of top results to return

        Returns:
            List of relevant content chunks with similarity scores
        """
        filters = {
            "chapter": chapter,
            "section": section
        }
        return await self.retrieve_relevant_content(query, top_k=top_k, filters=filters)

    async def find_related_chunks(
        self,
        chunk: ContentChunk,
        top_k: int = 3
    ) -> List[RetrievedChunk]:
        """
        Find content chunks related to a given chunk.

        Args:
            chunk: Content chunk to find related content for
            top_k: Number of top results to return

        Returns:
            List of related content chunks with similarity scores
        """
        if not chunk.embedding:
            logger.warning(f"Chunk {chunk.chunk_id} has no embedding for related content search")
            return []

        try:
            # Use the chunk's embedding to find similar content
            retrieved_chunks = await self.vector_storage.retrieve_similar(
                query_embedding=chunk.embedding,
                top_k=top_k
            )

            # Filter out the original chunk
            related_chunks = [
                c for c in retrieved_chunks
                if c.chunk_id != chunk.chunk_id
            ][:top_k]

            logger.debug(f"Found {len(related_chunks)} related chunks for {chunk.chunk_id}")
            return related_chunks

        except Exception as e:
            logger.error(f"Error finding related content for chunk {chunk.chunk_id}: {str(e)}")
            return []

    async def validate_relevance(
        self,
        query: str,
        chunks: List[RetrievedChunk],
        threshold: float = 0.5
    ) -> List[RetrievedChunk]:
        """
        Validate the relevance of retrieved chunks against the query.

        Args:
            query: Original query for relevance validation
            chunks: List of retrieved chunks to validate
            threshold: Minimum similarity score for relevance

        Returns:
            List of chunks that meet the relevance threshold
        """
        relevant_chunks = [chunk for chunk in chunks if chunk.similarity_score >= threshold]

        logger.info(f"Of {len(chunks)} retrieved chunks, {len(relevant_chunks)} met relevance threshold of {threshold}")
        return relevant_chunks

    async def retrieve_with_context_boost(
        self,
        query: str,
        context_chunks: List[ContentChunk],
        top_k: int = 5
    ) -> List[RetrievedChunk]:
        """
        Retrieve content with boosted relevance for specific context chunks.

        Args:
            query: User's natural language query
            context_chunks: Context chunks to boost relevance for
            top_k: Number of top results to return

        Returns:
            List of relevant content chunks with similarity scores
        """
        # Generate embedding for the query
        query_embedding = self.embedding_gen.generate_embedding(query)

        # For each context chunk, retrieve similar content
        all_retrieved = []
        for ctx_chunk in context_chunks:
            if ctx_chunk.embedding:
                # Retrieve content similar to the context chunk
                ctx_retrieved = await self.vector_storage.retrieve_similar(
                    query_embedding=ctx_chunk.embedding,
                    top_k=top_k // len(context_chunks) + 1  # Distribute among context chunks
                )
                all_retrieved.extend(ctx_retrieved)

        # Remove duplicates while preserving order
        seen_ids = set()
        unique_retrieved = []
        for chunk in all_retrieved:
            if chunk.chunk_id not in seen_ids:
                seen_ids.add(chunk.chunk_id)
                unique_retrieved.append(chunk)

        # Sort by similarity score (descending)
        unique_retrieved.sort(key=lambda x: x.similarity_score, reverse=True)

        # Return top_k results
        return unique_retrieved[:top_k]


# Global instance for easy access
retrieval_service = ContentRetrievalService()


def get_retrieval_service() -> ContentRetrievalService:
    """
    Get the content retrieval service instance.

    Returns:
        ContentRetrievalService instance
    """
    return retrieval_service