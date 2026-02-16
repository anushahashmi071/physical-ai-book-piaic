"""
Script to process and clean textbook content for RAG ingestion.
"""

import json
import re
from typing import List, Dict, Any
from pathlib import Path


def clean_markdown_content(content: str) -> str:
    """
    Clean markdown content by removing syntax elements and normalizing text.

    Args:
        content: Raw markdown content

    Returns:
        Cleaned text content
    """
    # Remove markdown syntax but preserve the text
    # Remove headers but keep the text
    content = re.sub(r'^#+\s*', '', content, flags=re.MULTILINE)

    # Remove bold and italic formatting
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)
    content = re.sub(r'\*(.*?)\*', r'\1', content)
    content = re.sub(r'__(.*?)__', r'\1', content)
    content = re.sub(r'_(.*?)_', r'\1', content)

    # Remove inline code markers
    content = re.sub(r'`(.*?)`', r'\1', content)

    # Remove image and link syntax, keeping the alt text/link text
    content = re.sub(r'!\[([^\]]*)\]\([^)]*\)', r'\1', content)  # Images
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)   # Links

    # Remove horizontal rules
    content = re.sub(r'^-{3,}$', '', content, flags=re.MULTILINE)

    # Normalize whitespace
    content = re.sub(r'\n\s*\n', '\n\n', content)  # Remove excessive blank lines
    content = re.sub(r'[ \t]+', ' ', content)      # Normalize spaces

    # Remove leading/trailing whitespace
    content = content.strip()

    return content


def remove_boilerplate_content(content: str) -> str:
    """
    Remove common boilerplate content from documentation.

    Args:
        content: Content to clean

    Returns:
        Content with boilerplate removed
    """
    # Remove common documentation boilerplate
    patterns_to_remove = [
        r'<!--.*?-->',  # HTML comments
        r'^Created by.*$',  # Author lines
        r'^Last updated.*$',  # Date lines
        r'^Table of contents.*$',  # TOC lines
        r'^Back to.*$',  # Navigation lines
        r'^Next.*$',  # Navigation lines
        r'^Previous.*$',  # Navigation lines
        r'^Copyright.*$',  # Copyright notices
    ]

    for pattern in patterns_to_remove:
        content = re.sub(pattern, '', content, flags=re.MULTILINE | re.IGNORECASE)

    # Remove excessive whitespace again after removal
    content = re.sub(r'\n\s*\n', '\n\n', content, flags=re.MULTILINE)
    content = content.strip()

    return content


def create_content_chunks(content: str, max_chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split content into chunks of specified size with overlap.

    Args:
        content: Content to chunk
        max_chunk_size: Maximum size of each chunk
        overlap: Number of characters to overlap between chunks

    Returns:
        List of content chunks
    """
    if len(content) <= max_chunk_size:
        return [content]

    chunks = []
    start = 0

    while start < len(content):
        end = start + max_chunk_size

        # If we're near the end, include the remainder
        if end >= len(content):
            chunks.append(content[start:].strip())
            break

        # Find a good breaking point (try to break at sentence or paragraph boundary)
        chunk = content[start:end]

        # Look for good breaking points
        break_points = ['. ', '! ', '? ', '\n\n', '; ', ': ']
        best_break = -1

        for bp in break_points:
            last_bp = chunk.rfind(bp)
            if last_bp > best_break:
                best_break = last_bp

        # If we found a good break point, use it
        if best_break > max_chunk_size * 0.5:  # If we found a reasonable break point
            actual_end = start + best_break + len(break_points[0])  # Just use the length of the first break point
            for bp in break_points:
                if bp in chunk and chunk.rfind(bp) == best_break:
                    actual_end = start + best_break + len(bp)
                    break

            chunks.append(content[start:actual_end].strip())
            start = actual_end - overlap  # Move start forward with overlap
        else:
            # If no good break point, just take the max chunk size
            chunks.append(content[start:end].strip())
            start = end - overlap

        # Safety check to prevent infinite loop
        if start <= 0:
            start = 1
        elif start >= len(content):
            break

    # Remove empty chunks
    chunks = [chunk for chunk in chunks if chunk.strip()]

    return chunks


def process_textbook_content(input_file: str, output_file: str):
    """
    Process and clean textbook content for RAG ingestion.

    Args:
        input_file: Input file with extracted content
        output_file: Output file for processed content
    """
    print(f"Processing content from: {input_file}")

    # Load the extracted content
    with open(input_file, 'r', encoding='utf-8') as f:
        extracted_content = json.load(f)

    processed_content = []

    for item in extracted_content:
        # Clean the content
        cleaned_content = clean_markdown_content(item['content'])
        cleaned_content = remove_boilerplate_content(cleaned_content)

        # Only process items that have substantial content
        if len(cleaned_content.strip()) < 50:  # Skip very short items
            continue

        # Create chunks for this content
        chunks = create_content_chunks(cleaned_content)

        # Create processed entries for each chunk
        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) > 10:  # Only include substantial chunks
                processed_item = {
                    'id': f"{item['id']}_chunk_{i}",
                    'title': item['title'],
                    'content': chunk,
                    'source_file': item['source_file'],
                    'chapter': item['chapter'],
                    'metadata': item['metadata'],
                    'chunk_index': i
                }
                processed_content.append(processed_item)

    # Save processed content
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(processed_content, f, indent=2, ensure_ascii=False)

    print(f"Processed {len(extracted_content)} original items into {len(processed_content)} content chunks")
    print(f"Saved processed content to: {output_file}")


def main():
    """
    Main function to process textbook content.
    """
    input_file = "extracted_textbook_content.json"
    output_file = "processed_textbook_content.json"

    # Check if input file exists
    if not Path(input_file).exists():
        print(f"Input file {input_file} not found. Please run the extraction script first.")
        return

    process_textbook_content(input_file, output_file)


if __name__ == "__main__":
    main()