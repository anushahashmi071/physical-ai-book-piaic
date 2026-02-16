from typing import List, Optional, Dict, Any
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from models.content import ContentChunk, RetrievedChunk
from config.settings import settings


logger = logging.getLogger(__name__)


class VectorStorageService:
    """
    Service for managing vector storage in Qdrant.
    Handles creating collections, storing chunks, and retrieving similar content.
    """

    def __init__(self):
        # Initialize Qdrant client
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                port=settings.qdrant_port,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                port=settings.qdrant_port,
                timeout=10
            )

        self.collection_name = settings.qdrant_collection_name
        self.vector_size = settings.embedding_dimension

    async def initialize_collection(self) -> bool:
        """
        Initialize the Qdrant collection for storing content chunks.

        Returns:
            True if initialization was successful, False otherwise
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with specified vector size
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")

            # Create payload index for efficient filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source.chapter",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source.section",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            logger.info(f"Initialized payload indices for collection: {self.collection_name}")
            return True

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            return False

    async def store_chunk(self, chunk: ContentChunk) -> bool:
        """
        Store a content chunk in the vector database.

        Args:
            chunk: Content chunk to store

        Returns:
            True if storage was successful, False otherwise
        """
        try:
            if not chunk.embedding:
                logger.error(f"Chunk {chunk.chunk_id} has no embedding to store")
                return False

            # Prepare the payload with chunk information
            payload = {
                "chunk_id": chunk.chunk_id,
                "content": chunk.content,
                "source": {
                    "document_id": chunk.source.document_id,
                    "chapter": chunk.source.chapter,
                    "section": chunk.source.section,
                    "page_reference": chunk.source.page_reference
                },
                "metadata": {
                    "tags": chunk.metadata.tags,
                    "concepts": chunk.metadata.concepts,
                    "difficulty_level": chunk.metadata.difficulty_level
                },
                "created_at": chunk.created_at.isoformat()
            }

            # Prepare the point to insert
            points = [
                PointStruct(
                    id=chunk.chunk_id,
                    vector=chunk.embedding,
                    payload=payload
                )
            ]

            # Insert the point into the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.debug(f"Stored chunk in Qdrant: {chunk.chunk_id}")
            return True

        except Exception as e:
            logger.error(f"Error storing chunk {chunk.chunk_id} in Qdrant: {str(e)}")
            return False

    async def store_chunks(self, chunks: List[ContentChunk]) -> int:
        """
        Store multiple content chunks in the vector database.

        Args:
            chunks: List of content chunks to store

        Returns:
            Number of successfully stored chunks
        """
        successful_stores = 0

        for chunk in chunks:
            if await self.store_chunk(chunk):
                successful_stores += 1

        logger.info(f"Successfully stored {successful_stores}/{len(chunks)} chunks in Qdrant")
        return successful_stores

    async def retrieve_similar(self, query_embedding: List[float], top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Retrieve similar content chunks based on the query embedding.

        Args:
            query_embedding: Embedding vector to search for similar content
            top_k: Number of top results to return
            filters: Optional filters to apply to the search

        Returns:
            List of retrieved content chunks with similarity scores
        """
        try:
            # Build search filters if provided
            search_filter = None
            if filters:
                must_conditions = []

                if "chapter" in filters:
                    must_conditions.append(
                        models.FieldCondition(
                            key="source.chapter",
                            match=models.MatchValue(value=filters["chapter"])
                        )
                    )

                if "section" in filters:
                    must_conditions.append(
                        models.FieldCondition(
                            key="source.section",
                            match=models.MatchValue(value=filters["section"])
                        )
                    )

                if must_conditions:
                    search_filter = models.Filter(must=must_conditions)

            # Perform the search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=search_filter,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )

            # Convert results to RetrievedChunk objects
            retrieved_chunks = []
            for result in search_results:
                payload = result.payload

                # Create ContentSource from payload
                source_data = payload.get("source", {})
                content_source = {
                    "document_id": source_data.get("document_id", ""),
                    "chapter": source_data.get("chapter", ""),
                    "section": source_data.get("section"),
                    "page_reference": source_data.get("page_reference")
                }

                # Create ContentMetadata from payload
                metadata_data = payload.get("metadata", {})
                content_metadata = {
                    "tags": metadata_data.get("tags", []),
                    "concepts": metadata_data.get("concepts", []),
                    "difficulty_level": metadata_data.get("difficulty_level", "beginner")
                }

                # Create ContentChunk with the retrieved data
                chunk = ContentChunk(
                    chunk_id=payload.get("chunk_id", ""),
                    content=payload.get("content", ""),
                    source=content_source,
                    metadata=content_metadata,
                    created_at=payload.get("created_at", "")
                )

                # Create RetrievedChunk with similarity score
                retrieved_chunk = RetrievedChunk(
                    **chunk.dict(),
                    similarity_score=float(result.score)
                )

                retrieved_chunks.append(retrieved_chunk)

            logger.debug(f"Retrieved {len(retrieved_chunks)} similar chunks from Qdrant")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving similar content from Qdrant: {str(e)}")
            return []

    async def delete_chunk(self, chunk_id: str) -> bool:
        """
        Delete a content chunk from the vector database.

        Args:
            chunk_id: ID of the chunk to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[chunk_id]
                )
            )

            logger.debug(f"Deleted chunk from Qdrant: {chunk_id}")
            return True

        except Exception as e:
            logger.error(f"Error deleting chunk {chunk_id} from Qdrant: {str(e)}")
            return False

    async def clear_collection(self) -> bool:
        """
        Clear all content from the vector database collection.

        Returns:
            True if clearing was successful, False otherwise
        """
        try:
            # Delete all points in the collection
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=models.Filter(
                        must=[]
                    )
                )
            )

            logger.info(f"Cleared all content from Qdrant collection: {self.collection_name}")
            return True

        except Exception as e:
            logger.error(f"Error clearing Qdrant collection: {str(e)}")
            return False

    async def get_collection_info(self) -> Optional[Dict[str, Any]]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information or None if error
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors.size,
                "vector_size": collection_info.config.params.vectors.size,
                "distance": collection_info.config.params.vectors.distance,
                "points_count": collection_info.points_count,
                "indexed_vectors_count": collection_info.indexed_vectors_count
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None


# Global instance for easy access
vector_storage_service = VectorStorageService()


def get_vector_storage_service() -> VectorStorageService:
    """
    Get the vector storage service instance.

    Returns:
        VectorStorageService instance
    """
    return vector_storage_service