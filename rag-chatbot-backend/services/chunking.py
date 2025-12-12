"""
Document Chunking Service
Feature: 004-rag-chat

Implements tiktoken-based text chunking with configurable token size and overlap.
Preserves markdown heading context for better semantic retrieval.
"""

import re
from typing import List, Dict, Optional
import tiktoken


def extract_heading(text: str, char_position: int) -> Optional[str]:
    """
    Extract the nearest markdown heading before the given character position.

    Args:
        text: Full document text
        char_position: Character position to search backwards from

    Returns:
        Heading text (without # markers) or None if no heading found

    Example:
        >>> text = "# Chapter 1\\n\\nSome text\\n\\n## Section 1.1\\n\\nMore text"
        >>> extract_heading(text, 50)
        'Section 1.1'
    """
    # Extract text up to the position
    text_before = text[:char_position]

    # Find all markdown headings (# to ######)
    heading_pattern = r'^#{1,6}\s+(.+)$'
    matches = list(re.finditer(heading_pattern, text_before, re.MULTILINE))

    if not matches:
        return None

    # Return the last (nearest) heading
    last_match = matches[-1]
    heading_text = last_match.group(1).strip()

    return heading_text


def chunk_document(
    text: str,
    file_path: str,
    chunk_size: int = 512,
    chunk_overlap: int = 128,
    encoding_name: str = "cl100k_base"
) -> List[Dict[str, any]]:
    """
    Chunk a markdown document into fixed-size token segments with overlap.

    Args:
        text: Document text to chunk
        file_path: Source file path for metadata
        chunk_size: Maximum tokens per chunk (default: 512)
        chunk_overlap: Overlapping tokens between chunks (default: 128)
        encoding_name: tiktoken encoding name (default: cl100k_base for GPT-4)

    Returns:
        List of chunk dictionaries with:
        - text: Chunk content
        - file_path: Source file path
        - chunk_index: 0-based chunk number
        - token_count: Actual token count
        - heading: Nearest markdown heading (or None)
        - char_start: Starting character position
        - char_end: Ending character position

    Example:
        >>> text = "# Introduction\\n\\nROS 2 is a framework..." * 100
        >>> chunks = chunk_document(text, "docs/intro.md")
        >>> len(chunks)  # Number of chunks
        >>> chunks[0]["heading"]  # 'Introduction'
        >>> chunks[0]["token_count"]  # ~512
    """
    # Initialize tiktoken encoder
    try:
        encoding = tiktoken.get_encoding(encoding_name)
    except KeyError:
        raise ValueError(f"Unknown encoding: {encoding_name}")

    # Encode full text
    tokens = encoding.encode(text)
    total_tokens = len(tokens)

    chunks = []
    chunk_index = 0
    start_token = 0

    while start_token < total_tokens:
        # Calculate end token for this chunk
        end_token = min(start_token + chunk_size, total_tokens)

        # Extract chunk tokens and decode to text
        chunk_tokens = tokens[start_token:end_token]
        chunk_text = encoding.decode(chunk_tokens)

        # Find character positions in original text
        # Approximate by finding the decoded chunk text in the original
        char_start = text.find(chunk_text[:50])  # Use first 50 chars for matching
        if char_start == -1:
            char_start = 0  # Fallback if not found
        char_end = char_start + len(chunk_text)

        # Extract nearest heading
        heading = extract_heading(text, char_start) if char_start > 0 else extract_heading(text, 0)

        # Create chunk metadata
        chunk = {
            "text": chunk_text.strip(),
            "file_path": file_path,
            "chunk_index": chunk_index,
            "token_count": len(chunk_tokens),
            "heading": heading,
            "char_start": char_start,
            "char_end": char_end
        }

        chunks.append(chunk)

        # Move to next chunk with overlap
        start_token += chunk_size - chunk_overlap
        chunk_index += 1

    return chunks


def chunk_multiple_documents(
    documents: List[Dict[str, str]],
    chunk_size: int = 512,
    chunk_overlap: int = 128
) -> List[Dict[str, any]]:
    """
    Chunk multiple documents in batch.

    Args:
        documents: List of dicts with 'text' and 'file_path' keys
        chunk_size: Maximum tokens per chunk
        chunk_overlap: Overlapping tokens between chunks

    Returns:
        Flattened list of all chunks from all documents

    Example:
        >>> docs = [
        ...     {"text": "...", "file_path": "docs/ch1.md"},
        ...     {"text": "...", "file_path": "docs/ch2.md"}
        ... ]
        >>> all_chunks = chunk_multiple_documents(docs)
        >>> len(all_chunks)  # Total chunks across all docs
    """
    all_chunks = []

    for doc in documents:
        text = doc["text"]
        file_path = doc["file_path"]

        doc_chunks = chunk_document(
            text=text,
            file_path=file_path,
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap
        )

        all_chunks.extend(doc_chunks)

    return all_chunks


if __name__ == "__main__":
    # Test chunking with sample text
    sample_text = """# Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications.

## Key Features

- Real-time capable
- Multi-platform support
- Security features

## Architecture

ROS 2 uses a distributed architecture with DDS middleware.
"""

    chunks = chunk_document(sample_text, "test.md", chunk_size=50, chunk_overlap=10)

    print(f"Generated {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"\nChunk {i}:")
        print(f"  Heading: {chunk['heading']}")
        print(f"  Tokens: {chunk['token_count']}")
        print(f"  Text: {chunk['text'][:100]}...")
