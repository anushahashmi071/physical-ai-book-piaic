import re
from typing import List, Dict, Any, Tuple
import markdown
from bs4 import BeautifulSoup
import logging


logger = logging.getLogger(__name__)


class MarkdownParser:
    """
    Utility class for parsing markdown content and extracting structured information.
    """

    def __init__(self):
        pass

    def extract_text_from_markdown(self, markdown_content: str) -> str:
        """
        Extract plain text from markdown content.

        Args:
            markdown_content: Raw markdown content

        Returns:
            Plain text extracted from markdown
        """
        # Convert markdown to HTML
        html_content = markdown.markdown(markdown_content)

        # Parse HTML and extract text
        soup = BeautifulSoup(html_content, 'html.parser')
        return soup.get_text(separator=' ', strip=True)

    def extract_headers(self, markdown_content: str) -> List[Dict[str, Any]]:
        """
        Extract headers from markdown content.

        Args:
            markdown_content: Raw markdown content

        Returns:
            List of header information with level and text
        """
        headers = []
        lines = markdown_content.split('\n')

        for line_num, line in enumerate(lines):
            # Match markdown headers (# Header, ## Header, etc.)
            header_match = re.match(r'^(#{1,6})\s+(.+)', line.strip())
            if header_match:
                level = len(header_match.group(1))
                text = header_match.group(2).strip()
                headers.append({
                    'level': level,
                    'text': text,
                    'line_number': line_num
                })

        return headers

    def split_by_headers(self, markdown_content: str) -> List[str]:
        """
        Split markdown content by headers to create semantic chunks.

        Args:
            markdown_content: Raw markdown content

        Returns:
            List of content chunks split by headers
        """
        lines = markdown_content.split('\n')
        chunks = []
        current_chunk = []
        current_header = ""

        for line in lines:
            header_match = re.match(r'^(#{1,6})\s+(.+)', line.strip())
            if header_match:
                # Save the previous chunk if it exists
                if current_chunk and any(part.strip() for part in current_chunk):
                    chunks.append(current_header + '\n' + '\n'.join(current_chunk))

                # Start new chunk with this header
                current_header = line.strip()
                current_chunk = []
            else:
                current_chunk.append(line)

        # Add the last chunk
        if current_chunk and any(part.strip() for part in current_chunk):
            chunks.append(current_header + '\n' + '\n'.join(current_chunk))

        return [chunk.strip() for chunk in chunks if chunk.strip()]

    def semantic_chunk(self, text: str, max_chunk_size: int = 512, overlap: int = 64) -> List[str]:
        """
        Split text into semantic chunks that respect sentence boundaries.

        Args:
            text: Input text to chunk
            max_chunk_size: Maximum size of each chunk
            overlap: Number of characters to overlap between chunks

        Returns:
            List of text chunks
        """
        # Split by sentences
        sentences = re.split(r'(?<=[.!?])\s+', text)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            # If adding this sentence would exceed the chunk size
            if len(current_chunk) + len(sentence) > max_chunk_size and current_chunk:
                chunks.append(current_chunk.strip())

                # Create overlapping chunk
                if overlap > 0:
                    # Take the end of the current chunk for overlap
                    overlap_text = current_chunk[-overlap:]
                    current_chunk = overlap_text + " " + sentence
                else:
                    current_chunk = sentence
            else:
                current_chunk += " " + sentence if current_chunk else sentence

        # Add the last chunk
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return [chunk for chunk in chunks if len(chunk.strip()) > 0]

    def parse_markdown_to_chunks(self, markdown_content: str, max_chunk_size: int = 512, overlap: int = 64) -> List[str]:
        """
        Parse markdown content and convert to semantic chunks.

        Args:
            markdown_content: Raw markdown content
            max_chunk_size: Maximum size of each chunk
            overlap: Number of characters to overlap between chunks

        Returns:
            List of semantic content chunks
        """
        # First, try to split by headers to maintain semantic boundaries
        header_based_chunks = self.split_by_headers(markdown_content)

        final_chunks = []
        for chunk in header_based_chunks:
            # For each header-based chunk, further split if it's too large
            if len(chunk) <= max_chunk_size:
                final_chunks.append(chunk)
            else:
                # Further chunk large sections while respecting sentence boundaries
                subchunks = self.semantic_chunk(chunk, max_chunk_size, overlap)
                final_chunks.extend(subchunks)

        return final_chunks


class ContentChunker:
    """
    Utility class for chunking content with various strategies.
    """

    def __init__(self):
        self.parser = MarkdownParser()

    def chunk_content(self, content: str, strategy: str = "semantic", **kwargs) -> List[str]:
        """
        Chunk content using the specified strategy.

        Args:
            content: Content to chunk
            strategy: Chunking strategy ('semantic', 'fixed_size', 'by_headers')
            **kwargs: Additional parameters for the chunking strategy

        Returns:
            List of content chunks
        """
        max_chunk_size = kwargs.get('max_chunk_size', 512)
        overlap = kwargs.get('overlap', 64)

        if strategy == "semantic":
            return self.parser.semantic_chunk(content, max_chunk_size, overlap)
        elif strategy == "by_headers":
            return self.parser.split_by_headers(content)
        elif strategy == "fixed_size":
            # Simple fixed-size chunking
            chunks = []
            for i in range(0, len(content), max_chunk_size - overlap):
                chunk = content[i:i + max_chunk_size]
                if chunk.strip():
                    chunks.append(chunk)
            return chunks
        else:
            raise ValueError(f"Unknown chunking strategy: {strategy}")


# Global instance for easy access
content_chunker = ContentChunker()


def get_content_chunker() -> ContentChunker:
    """
    Get the content chunker instance.

    Returns:
        ContentChunker instance
    """
    return content_chunker