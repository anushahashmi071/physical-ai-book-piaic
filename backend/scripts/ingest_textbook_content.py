#!/usr/bin/env python3
"""
Script to ingest textbook content from the documentation into the RAG system.
"""

import asyncio
import os
from pathlib import Path
import logging
from typing import List, Dict, Any

# Add the backend directory to the path so we can import our modules
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from services.ingestion import get_ingestion_service
from models.content import Document
from utils.parsing import get_content_chunker


async def extract_textbook_content(docs_dir: str) -> List[Document]:
    """
    Extract content from textbook markdown files.

    Args:
        docs_dir: Directory containing textbook markdown files

    Returns:
        List of Document objects ready for ingestion
    """
    documents = []
    content_dir = Path(docs_dir)

    # Find all markdown files in the documentation
    for md_file in content_dir.rglob("*.md"):
        if any(excluded in str(md_file) for excluded in ['/node_modules/', 'README.md', 'CHANGELOG.md']):
            continue

        try:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract chapter/section information from file path
            relative_path = md_file.relative_to(content_dir)
            path_parts = relative_path.parts

            # Determine chapter and section from path
            chapter = ""
            section = ""

            # If it's in a chapter-* format, extract chapter info
            for part in path_parts:
                if part.startswith('chapter-'):
                    chapter = part.replace('-', ' ').title()
                    break
                elif part.startswith(('ros2-', 'digital-twin-', 'perception-', 'vla-')):
                    chapter = part.replace('-', ' ').title()
                    break

            # Section could be derived from the filename if it's not just a generic name
            filename = md_file.stem
            if filename not in ['index', 'intro', 'assessments', 'README']:
                section = filename.replace('-', ' ').title()

            # Create document
            document = Document(
                id=str(md_file),
                content=content,
                metadata={
                    "source_file": str(relative_path),
                    "chapter": chapter or "General",
                    "section": section or "Overview",
                    "file_path": str(md_file)
                }
            )
            documents.append(document)

            print(f"Added document: {relative_path} (Chapter: {chapter}, Section: {section})")

        except Exception as e:
            print(f"Error processing {md_file}: {str(e)}")
            continue

    print(f"Total documents found: {len(documents)}")
    return documents


async def run_ingestion_pipeline():
    """
    Run the full content ingestion pipeline.
    """
    print("Starting textbook content ingestion pipeline...")

    # Get the ingestion service
    ingestion_service = get_ingestion_service()

    # Extract textbook content from docs directory
    # Use absolute path to ensure we can find the docs directory
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    docs_path = os.path.join(current_dir, "..", "..", "..", "frontend", "docs")

    print(f"Reading textbook content from: {docs_path}")
    if os.path.exists(docs_path):
        documents = await extract_textbook_content(docs_path)
    else:
        print(f"ERROR: Docs directory does not exist at {docs_path}")
        documents = []

    if not documents:
        print("No documents found to ingest. Check the path and file extensions.")
        return

    # Process documents through the ingestion pipeline
    print("Starting ingestion process...")

    try:
        # Process the documents
        result = await ingestion_service.ingest_documents(documents)

        print(f"Ingestion completed!")
        print(f"- Documents processed: {result.processed_documents}")
        print(f"- Documents failed: {result.failed_documents}")
        print(f"- Status: {result.status}")
        print(f"- Timestamp: {result.timestamp}")

        if result.failed_documents > 0:
            print(f"Warning: {result.failed_documents} documents failed to process")

        print("\nContent ingestion pipeline completed successfully!")
        print("The RAG system now has access to the textbook content.")

    except Exception as e:
        print(f"Error during ingestion: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Run the ingestion pipeline
    asyncio.run(run_ingestion_pipeline())