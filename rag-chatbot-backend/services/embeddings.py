"""
OpenAI Embeddings Service
Feature: 004-rag-chat

Generates text embeddings using OpenAI's text-embedding-3-small model.
Supports both single and batch embedding generation.
"""

import os
from typing import List, Dict, Optional
from openai import OpenAI
import asyncio
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def get_openai_client() -> OpenAI:
    """
    Initialize OpenAI client with API key from environment.

    Returns:
        Configured OpenAI client

    Raises:
        ValueError: If OPENAI_API_KEY not found in environment
    """
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError("OPENAI_API_KEY environment variable not set")

    return OpenAI(api_key=api_key)


def generate_embedding(
    text: str,
    model: str = "text-embedding-3-small",
    client: Optional[OpenAI] = None
) -> List[float]:
    """
    Generate embedding vector for a single text string.

    Args:
        text: Text to embed
        model: OpenAI embedding model name (default: text-embedding-3-small)
        client: Optional pre-initialized OpenAI client

    Returns:
        Embedding vector as list of floats (1536 dimensions for text-embedding-3-small)

    Example:
        >>> embedding = generate_embedding("What is ROS 2?")
        >>> len(embedding)  # 1536
        >>> isinstance(embedding[0], float)  # True

    Raises:
        ValueError: If text is empty
        Exception: If OpenAI API call fails
    """
    if not text or not text.strip():
        raise ValueError("Text cannot be empty")

    # Initialize client if not provided
    if client is None:
        client = get_openai_client()

    try:
        # Call OpenAI Embeddings API
        response = client.embeddings.create(
            model=model,
            input=text.strip()
        )

        # Extract embedding vector
        embedding = response.data[0].embedding

        return embedding

    except Exception as e:
        raise Exception(f"Failed to generate embedding: {str(e)}")


def generate_embeddings_batch(
    texts: List[str],
    model: str = "text-embedding-3-small",
    batch_size: int = 100,
    client: Optional[OpenAI] = None
) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batches.

    OpenAI allows up to 100 texts per API call for embeddings.
    This function automatically batches large lists.

    Args:
        texts: List of texts to embed
        model: OpenAI embedding model name
        batch_size: Maximum texts per API call (default: 100)
        client: Optional pre-initialized OpenAI client

    Returns:
        List of embedding vectors (same order as input texts)

    Example:
        >>> texts = ["What is ROS 2?", "What is Gazebo?", "What is Isaac Sim?"]
        >>> embeddings = generate_embeddings_batch(texts)
        >>> len(embeddings)  # 3
        >>> len(embeddings[0])  # 1536

    Raises:
        ValueError: If texts list is empty
        Exception: If any OpenAI API call fails
    """
    if not texts:
        raise ValueError("Texts list cannot be empty")

    # Initialize client if not provided
    if client is None:
        client = get_openai_client()

    all_embeddings = []

    # Process in batches
    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]

        # Filter out empty strings
        non_empty_batch = [text.strip() for text in batch if text.strip()]

        if not non_empty_batch:
            continue

        # Retry logic for transient errors
        max_retries = 3
        retry_count = 0
        batch_num = i // batch_size + 1

        while retry_count < max_retries:
            try:
                # Call OpenAI Embeddings API for batch
                response = client.embeddings.create(
                    model=model,
                    input=non_empty_batch
                )

                # Extract embeddings in order
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)
                break  # Success, exit retry loop

            except Exception as e:
                retry_count += 1
                if retry_count >= max_retries:
                    print(f"[ERROR] Failed batch {batch_num} after {max_retries} retries: {str(e)}")
                    raise Exception(f"Failed to generate batch embeddings (batch {batch_num}): {str(e)}")
                else:
                    print(f"[RETRY] Batch {batch_num} failed, retrying ({retry_count}/{max_retries})...")
                    import time
                    time.sleep(2 ** retry_count)  # Exponential backoff

    return all_embeddings


async def generate_embeddings_async(
    texts: List[str],
    model: str = "text-embedding-3-small",
    batch_size: int = 100
) -> List[List[float]]:
    """
    Async wrapper for batch embedding generation.

    Useful for FastAPI endpoints that need to embed queries asynchronously.

    Args:
        texts: List of texts to embed
        model: OpenAI embedding model name
        batch_size: Maximum texts per API call

    Returns:
        List of embedding vectors

    Example:
        >>> embeddings = await generate_embeddings_async(["What is ROS 2?"])
        >>> len(embeddings[0])  # 1536
    """
    # Run sync function in executor to avoid blocking event loop
    loop = asyncio.get_event_loop()
    embeddings = await loop.run_in_executor(
        None,
        lambda: generate_embeddings_batch(texts, model, batch_size)
    )

    return embeddings


def embed_chunks(
    chunks: List[Dict[str, any]],
    model: str = "text-embedding-3-small",
    batch_size: int = 100
) -> List[Dict[str, any]]:
    """
    Add embedding vectors to chunk dictionaries.

    Args:
        chunks: List of chunk dicts from chunking.py (must have 'text' key)
        model: OpenAI embedding model name
        batch_size: Maximum texts per API call

    Returns:
        Chunks with added 'embedding' key containing vector

    Example:
        >>> chunks = [{"text": "ROS 2 is...", "file_path": "docs/ch1.md"}]
        >>> chunks_with_embeddings = embed_chunks(chunks)
        >>> 'embedding' in chunks_with_embeddings[0]  # True
        >>> len(chunks_with_embeddings[0]['embedding'])  # 1536
    """
    # Extract texts from chunks
    texts = [chunk["text"] for chunk in chunks]

    # Generate embeddings
    embeddings = generate_embeddings_batch(texts, model, batch_size)

    # Add embeddings to chunks
    for i, chunk in enumerate(chunks):
        chunk["embedding"] = embeddings[i]

    return chunks


if __name__ == "__main__":
    # Test single embedding
    print("Testing single embedding...")
    try:
        embedding = generate_embedding("What is ROS 2?")
        print(f"✅ Generated embedding with {len(embedding)} dimensions")
        print(f"   First 5 values: {embedding[:5]}")
    except Exception as e:
        print(f"❌ Failed: {e}")

    # Test batch embedding
    print("\nTesting batch embedding...")
    try:
        texts = [
            "What is ROS 2?",
            "What is Gazebo?",
            "What is Isaac Sim?"
        ]
        embeddings = generate_embeddings_batch(texts)
        print(f"✅ Generated {len(embeddings)} embeddings")
        for i, emb in enumerate(embeddings):
            print(f"   Text {i + 1}: {len(emb)} dimensions")
    except Exception as e:
        print(f"❌ Failed: {e}")
