"""
Script to run the content ingestion pipeline on full textbook content.
"""

import json
import asyncio
from typing import List, Dict, Any
from models.content import Document
from services.ingestion import get_ingestion_service
from services.vector_storage import get_vector_storage_service
from config.settings import settings


async def load_processed_content(file_path: str) -> List[Dict[str, Any]]:
    """
    Load processed textbook content from JSON file.

    Args:
        file_path: Path to the processed content file

    Returns:
        List of processed content items
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = json.load(f)
    return content


async def prepare_documents_for_ingestion(processed_content: List[Dict[str, Any]]) -> List[Document]:
    """
    Prepare documents for ingestion from processed content.

    Args:
        processed_content: List of processed content items

    Returns:
        List of Document objects ready for ingestion
    """
    documents = []

    for item in processed_content:
        # Create metadata for the document
        metadata = {
            "document_id": item.get('id', ''),
            "chapter": item.get('chapter', 'unknown'),
            "source_file": item.get('source_file', ''),
            "title": item.get('title', ''),
            "chunk_index": item.get('chunk_index', 0),
            "original_title": item.get('metadata', {}).get('original_title', ''),
            "tags": ["textbook", "physical-ai", "robotics", item.get('chapter', '').lower().replace(' ', '-')],
            "concepts": [],  # This could be extracted from the content
            "difficulty_level": "intermediate"  # Default, could be determined from content
        }

        # Create document object
        document = Document(
            id=item['id'],
            content=item['content'],
            metadata=metadata
        )

        documents.append(document)

    return documents


async def run_ingestion_pipeline():
    """
    Run the full content ingestion pipeline on textbook content.
    """
    print("Starting content ingestion pipeline...")

    # Load processed content
    processed_content_file = "processed_textbook_content.json"
    print(f"Loading processed content from: {processed_content_file}")

    try:
        processed_content = await load_processed_content(processed_content_file)
        print(f"Loaded {len(processed_content)} content items")
    except FileNotFoundError:
        print(f"Error: {processed_content_file} not found. Please run the processing script first.")
        return

    # Prepare documents for ingestion
    print("Preparing documents for ingestion...")
    documents = await prepare_documents_for_ingestion(processed_content)
    print(f"Prepared {len(documents)} documents for ingestion")

    # Initialize ingestion service
    print("Initializing ingestion service...")
    ingestion_service = get_ingestion_service()

    # Initialize vector storage service
    print("Initializing vector storage service...")
    vector_storage = get_vector_storage_service()

    # Initialize the collection
    print("Initializing vector database collection...")
    success = await vector_storage.initialize_collection()
    if not success:
        print("Error: Failed to initialize vector database collection")
        return

    print("Starting ingestion process...")

    # Split documents into batches to manage memory
    batch_size = 10  # Small batch size for testing
    total_processed = 0
    total_failed = 0

    for i in range(0, len(documents), batch_size):
        batch = documents[i:i + batch_size]

        print(f"Processing batch {i//batch_size + 1}/{(len(documents)-1)//batch_size + 1}")

        # Run ingestion for this batch
        result = await ingestion_service.ingest_documents(batch)

        print(f"Batch result - Processed: {result.processed_documents}, Failed: {result.failed_documents}")

        total_processed += result.processed_documents
        total_failed += result.failed_documents

        # Small delay to be gentle with resources
        await asyncio.sleep(0.1)

    print(f"\nIngestion completed!")
    print(f"Total documents processed: {total_processed}")
    print(f"Total documents failed: {total_failed}")
    print(f"Success rate: {total_processed/(total_processed+total_failed)*100:.2f}%")

    # Get collection info
    collection_info = await vector_storage.get_collection_info()
    if collection_info:
        print(f"Collection now contains {collection_info['points_count']} vectors")


async def main():
    """
    Main function to run the ingestion pipeline.
    """
    try:
        await run_ingestion_pipeline()
    except Exception as e:
        print(f"Error running ingestion pipeline: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())