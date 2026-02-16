"""
Script to extract textbook content from existing Docusaurus documentation.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any


def extract_markdown_content(file_path: str) -> Dict[str, Any]:
    """
    Extract content from a markdown file, separating frontmatter from content.

    Args:
        file_path: Path to the markdown file

    Returns:
        Dictionary with file info and content
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract frontmatter if it exists
    frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)', content, re.DOTALL)
    frontmatter = {}
    body = content

    if frontmatter_match:
        frontmatter_text = frontmatter_match.group(1)
        body = frontmatter_match.group(2)

        # Parse simple frontmatter (key: value format)
        for line in frontmatter_text.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                frontmatter[key.strip()] = value.strip().strip('"\'')

    # Extract title from the first heading if not in frontmatter
    title_match = re.search(r'^#\s+(.+)$', body, re.MULTILINE)
    title = frontmatter.get('title', '')
    if not title and title_match:
        title = title_match.group(1)

    # Remove markdown headings to extract sections
    sections = []
    lines = body.split('\n')
    current_section = {'title': title or 'Introduction', 'content': ''}

    for line in lines:
        heading_match = re.match(r'^(#+)\s+(.+)', line)
        if heading_match:
            # Save current section if it has content
            if current_section['content'].strip():
                sections.append(current_section)

            # Start new section
            current_section = {
                'title': heading_match.group(2),
                'content': line + '\n'
            }
        else:
            current_section['content'] += line + '\n'

    # Add the last section
    if current_section['content'].strip():
        sections.append(current_section)

    return {
        'file_path': file_path,
        'title': title,
        'frontmatter': frontmatter,
        'sections': sections,
        'full_content': body
    }


def get_all_markdown_files(docs_dir: str) -> List[str]:
    """
    Recursively get all markdown files in the documentation directory.

    Args:
        docs_dir: Directory to search for markdown files

    Returns:
        List of markdown file paths
    """
    md_files = []
    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith('.md') or file.endswith('.mdx'):
                md_files.append(os.path.join(root, file))
    return md_files


def extract_textbook_content(docs_directory: str) -> List[Dict[str, Any]]:
    """
    Extract content from all markdown files in the documentation directory.

    Args:
        docs_directory: Root directory of the documentation

    Returns:
        List of extracted content from all files
    """
    print(f"Extracting content from: {docs_directory}")

    all_files = get_all_markdown_files(docs_directory)
    print(f"Found {len(all_files)} markdown files")

    extracted_content = []

    for file_path in all_files:
        try:
            content = extract_markdown_content(file_path)
            extracted_content.append(content)
            print(f"Extracted content from: {file_path}")
        except Exception as e:
            print(f"Error extracting content from {file_path}: {str(e)}")

    return extracted_content


def save_extracted_content(extracted_content: List[Dict[str, Any]], output_file: str):
    """
    Save extracted content to a JSON file for further processing.

    Args:
        extracted_content: List of extracted content
        output_file: Output file path
    """
    import json

    # Convert content to a simplified format for RAG processing
    simplified_content = []
    for item in extracted_content:
        for section in item['sections']:
            simplified_content.append({
                'id': f"{item['file_path']}#{section['title']}",
                'title': section['title'],
                'content': section['content'],
                'source_file': item['file_path'],
                'chapter': os.path.basename(os.path.dirname(item['file_path'])),
                'metadata': {
                    'original_title': item['title'],
                    'frontmatter': item['frontmatter']
                }
            })

    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(simplified_content, f, indent=2, ensure_ascii=False)

    print(f"Saved {len(simplified_content)} content sections to {output_file}")


def main():
    """
    Main function to extract textbook content from Docusaurus documentation.
    """
    docs_dir = "../../frontend/docs"  # Relative to the backend/rag-service directory
    output_file = "extracted_textbook_content.json"

    # Check if docs directory exists
    if not os.path.exists(docs_dir):
        print(f"Documentation directory not found: {docs_dir}")
        # Try alternative paths
        alt_paths = ["../frontend/docs", "../../frontend/docs", "../../../frontend/docs"]
        for alt_path in alt_paths:
            if os.path.exists(alt_path):
                docs_dir = alt_path
                print(f"Found documentation directory at: {docs_dir}")
                break
        else:
            print("Could not find documentation directory. Please ensure frontend/docs exists.")
            return

    # Extract content
    extracted_content = extract_textbook_content(docs_dir)

    # Save to file
    save_extracted_content(extracted_content, output_file)

    print(f"Content extraction completed. Processed {len(extracted_content)} files.")


if __name__ == "__main__":
    main()