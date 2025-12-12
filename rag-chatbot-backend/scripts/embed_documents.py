"""
Document Embedding Script
Feature: 004-rag-chat

One-time script to process markdown documents, chunk them, generate embeddings,
and upload to Qdrant vector database.

Usage:
    python scripts/embed_documents.py

Environment Variables Required:
    - OPENAI_API_KEY: OpenAI API key
    - QDRANT_URL: Qdrant Cloud URL
    - QDRANT_API_KEY: Qdrant API key
    - QDRANT_COLLECTION_NAME: Collection name (default: robotics_textbook)
    - CHUNK_SIZE: Tokens per chunk (default: 512)
    - CHUNK_OVERLAP: Overlap tokens (default: 128)
"""

import os
import sys
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from services.chunking import chunk_document
from services.embeddings import embed_chunks
from services.qdrant_client import (
    get_qdrant_client,
    create_collection,
    upsert_chunks,
    get_collection_info
)

# Load environment variables
load_dotenv()


def find_markdown_files(docs_dir: str = "../../docs") -> List[str]:
    """
    Find all markdown files in the docs directory.

    Args:
        docs_dir: Path to docs directory (relative to script location)

    Returns:
        List of absolute file paths
    """
    script_dir = Path(__file__).parent
    docs_path = (script_dir / docs_dir).resolve()

    if not docs_path.exists():
        raise FileNotFoundError(f"Docs directory not found: {docs_path}")

    # Find all .md and .mdx files recursively
    md_files = list(docs_path.glob("**/*.md"))
    mdx_files = list(docs_path.glob("**/*.mdx"))

    all_files = md_files + mdx_files

    if not all_files:
        raise ValueError(f"No markdown files found in {docs_path}")

    print(f"Found {len(all_files)} markdown files in {docs_path}")

    return [str(f) for f in all_files]


def read_file(file_path: str) -> str:
    """
    Read file content with UTF-8 encoding.

    Args:
        file_path: Path to file

    Returns:
        File content as string
    """
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception as e:
        print(f"[WARNING] Failed to read {file_path}: {e}")
        return ""


def generate_url_from_path(file_path: str, docs_dir: str = "../../docs") -> str:
    """
    Generate GitHub Pages URL from file path with baseUrl.

    Args:
        file_path: Absolute file path
        docs_dir: Docs directory (relative to script)

    Returns:
        GitHub Pages URL with baseUrl (e.g., /physical-ai-humanoid-robotics/docs/module-1/chapter-1)

    Example:
        >>> generate_url_from_path("/path/to/docs/module-1/chapter-1.md")
        '/physical-ai-humanoid-robotics/docs/module-1/chapter-1'
    """
    script_dir = Path(__file__).parent
    docs_path = (script_dir / docs_dir).resolve()

    file_path_obj = Path(file_path)
    relative_path = file_path_obj.relative_to(docs_path)

    # Remove .md or .mdx extension
    url_path = str(relative_path).replace(".mdx", "").replace(".md", "").replace("\\", "/")

    # Convert to URL format with baseUrl from docusaurus.config.ts
    url = f"/physical-ai-humanoid-robotics/docs/{url_path}"

    return url


def process_documents(
    chunk_size: int = 512,
    chunk_overlap: int = 128
) -> List[Dict[str, any]]:
    """
    Process all markdown documents: read, chunk, and prepare for embedding.

    Args:
        chunk_size: Tokens per chunk
        chunk_overlap: Overlap tokens

    Returns:
        List of chunks with metadata (without embeddings yet)
    """
    print("\n" + "=" * 60)
    print("STEP 1: Processing Documents")
    print("=" * 60)

    # Find markdown files
    file_paths = find_markdown_files()

    all_chunks = []

    for file_path in file_paths:
        # Read file
        text = read_file(file_path)

        if not text or len(text.strip()) < 100:
            print(f"[SKIP] Skipping {Path(file_path).name} (too short or empty)")
            continue

        # Generate URL
        url = generate_url_from_path(file_path)

        # Chunk document
        chunks = chunk_document(
            text=text,
            file_path=file_path,
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap
        )

        # Add URL to each chunk
        for chunk in chunks:
            chunk["url"] = url
            chunk["chapter_title"] = Path(file_path).stem.replace("-", " ").title()

        all_chunks.extend(chunks)

        print(f"[OK] {Path(file_path).name}: {len(chunks)} chunks")

    print(f"\n[STATS] Total chunks generated: {len(all_chunks)}")

    return all_chunks


def embed_and_upload(
    chunks: List[Dict[str, any]],
    collection_name: str,
    embedding_model: str = "text-embedding-3-small",
    batch_size: int = 20
) -> int:
    """
    Generate embeddings for chunks and upload to Qdrant.

    Args:
        chunks: List of chunks without embeddings
        collection_name: Qdrant collection name
        embedding_model: OpenAI embedding model
        batch_size: Batch size for embedding generation

    Returns:
        Number of chunks successfully uploaded
    """
    print("\n" + "=" * 60)
    print("STEP 2: Generating Embeddings")
    print("=" * 60)

    # Generate embeddings
    chunks_with_embeddings = embed_chunks(
        chunks=chunks,
        model=embedding_model,
        batch_size=batch_size
    )

    print(f"[OK] Generated embeddings for {len(chunks_with_embeddings)} chunks")

    print("\n" + "=" * 60)
    print("STEP 3: Uploading to Qdrant")
    print("=" * 60)

    # Create collection
    client = get_qdrant_client()
    create_collection(collection_name, vector_size=1536, client=client)

    # Upsert chunks
    count = upsert_chunks(
        chunks=chunks_with_embeddings,
        collection_name=collection_name,
        client=client
    )

    # Get collection info
    info = get_collection_info(collection_name, client=client)
    print(f"\n[STATS] Collection '{collection_name}' now has {info['points_count']} points")

    return count


def main():
    """
    Main execution flow.
    """
    print("\n" + "=" * 60)
    print("RAG CHATBOT: Document Embedding Pipeline")
    print("=" * 60)

    # Load configuration
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")
    chunk_size = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "128"))
    embedding_model = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")

    print(f"\nConfiguration:")
    print(f"   Collection: {collection_name}")
    print(f"   Chunk Size: {chunk_size} tokens")
    print(f"   Chunk Overlap: {chunk_overlap} tokens")
    print(f"   Embedding Model: {embedding_model}")

    try:
        # Step 1: Process documents
        chunks = process_documents(chunk_size, chunk_overlap)

        if not chunks:
            print("\n[ERROR] No chunks generated. Exiting.")
            return

        # Step 2 & 3: Embed and upload
        count = embed_and_upload(chunks, collection_name, embedding_model)

        print("\n" + "=" * 60)
        print("[SUCCESS] EMBEDDING PIPELINE COMPLETE")
        print("=" * 60)
        print(f"   Processed: {len(chunks)} chunks")
        print(f"   Uploaded: {count} vectors")
        print(f"   Collection: {collection_name}")
        print("\nYou can now start the FastAPI server and test the RAG chatbot!")

    except Exception as e:
        print(f"\n[ERROR] Pipeline failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
