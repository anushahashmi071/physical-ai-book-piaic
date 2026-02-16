from typing import List, Dict, Any
import logging
from uuid import uuid4
from datetime import datetime
from models.content import Document, IngestionResult, ContentChunk, ContentSource, ContentMetadata
from utils.parsing import get_content_chunker
from utils.embedding import get_embedding_generator
from config.settings import settings


logger = logging.getLogger(__name__)


class ContentIngestionService:
    """
    Service for ingesting textbook content into the RAG system.
    Handles parsing, chunking, embedding, and storing content in the vector database.
    """

    def __init__(self):
        self.chunker = get_content_chunker()
        self.embedding_gen = get_embedding_generator(settings.embedding_model)

    async def ingest_documents(self, documents: List[Document]) -> IngestionResult:
        """
        Ingest a list of documents into the RAG system.

        Args:
            documents: List of documents to ingest

        Returns:
            Result of the ingestion process
        """
        job_id = str(uuid4())
        logger.info(f"Starting ingestion job {job_id} with {len(documents)} documents")

        processed_count = 0
        failed_count = 0

        try:
            for document in documents:
                try:
                    await self._process_document(document)
                    processed_count += 1
                except Exception as e:
                    logger.error(f"Failed to process document {document.id}: {str(e)}")
                    failed_count += 1

            logger.info(f"Ingestion job {job_id} completed: {processed_count} processed, {failed_count} failed")

            return IngestionResult(
                job_id=job_id,
                status="completed" if failed_count == 0 else ("failed" if processed_count == 0 else "partial"),
                processed_documents=processed_count,
                failed_documents=failed_count,
                timestamp=datetime.utcnow()
            )

        except Exception as e:
            logger.error(f"Ingestion job {job_id} failed: {str(e)}")
            return IngestionResult(
                job_id=job_id,
                status="failed",
                processed_documents=processed_count,
                failed_documents=len(documents) - processed_count,
                timestamp=datetime.utcnow()
            )

    async def _process_document(self, document: Document) -> None:
        """
        Process a single document: parse, chunk, embed, and store.

        Args:
            document: Document to process
        """
        logger.debug(f"Processing document: {document.id}")

        # Determine source information from metadata
        source_info = self._extract_source_info(document.metadata)

        # Chunk the content
        chunks = self.chunker.chunk_content(
            document.content,
            strategy="semantic",
            max_chunk_size=settings.embedding_dimension * 2,  # Approximate size based on embedding model
            overlap=64
        )

        logger.debug(f"Document {document.id} chunked into {len(chunks)} parts")

        # Process each chunk
        for i, chunk_text in enumerate(chunks):
            chunk_id = f"{document.id}_chunk_{i}"

            # Generate embedding
            embedding = self.embedding_gen.generate_embedding(chunk_text)

            # Create content chunk
            content_chunk = ContentChunk(
                chunk_id=chunk_id,
                content=chunk_text,
                source=source_info,
                embedding=embedding,
                metadata=self._extract_metadata(document.metadata)
            )

            # Store in vector database
            await self._store_chunk(content_chunk)

        logger.debug(f"Completed processing document: {document.id}")

    def _extract_source_info(self, metadata: Dict[str, Any]) -> ContentSource:
        """
        Extract source information from document metadata.

        Args:
            metadata: Document metadata

        Returns:
            ContentSource object with extracted information
        """
        return ContentSource(
            document_id=metadata.get("document_id", "unknown"),
            chapter=metadata.get("chapter", "unknown"),
            section=metadata.get("section"),
            page_reference=metadata.get("page_reference")
        )

    def _extract_metadata(self, metadata: Dict[str, Any]) -> ContentMetadata:
        """
        Extract content metadata from document metadata.

        Args:
            metadata: Document metadata

        Returns:
            ContentMetadata object with extracted information
        """
        return ContentMetadata(
            tags=metadata.get("tags", []),
            concepts=metadata.get("concepts", []),
            difficulty_level=metadata.get("difficulty_level", "beginner")
        )

    async def _store_chunk(self, chunk: ContentChunk) -> None:
        """
        Store a content chunk in the vector database.

        Args:
            chunk: Content chunk to store
        """
        # In a real implementation, this would store the chunk in Qdrant
        # For now, we'll simulate the storage
        logger.debug(f"Storing chunk: {chunk.chunk_id}")

        # Placeholder for actual database storage
        # await self.vector_db_client.store_chunk(chunk)
        pass

    async def initialize_collection(self) -> bool:
        """
        Initialize the vector database collection for content storage.

        Returns:
            True if initialization was successful, False otherwise
        """
        # In a real implementation, this would create the collection in Qdrant
        # For now, we'll just return True
        logger.info("Initializing vector database collection")
        return True

    async def clear_collection(self) -> bool:
        """
        Clear all content from the vector database collection.

        Returns:
            True if clearing was successful, False otherwise
        """
        # In a real implementation, this would clear the collection in Qdrant
        # For now, we'll just return True
        logger.info("Clearing vector database collection")
        return True


# Global instance for easy access
ingestion_service = ContentIngestionService()


def get_ingestion_service() -> ContentIngestionService:
    """
    Get the content ingestion service instance.

    Returns:
        ContentIngestionService instance
    """
    return ingestion_service