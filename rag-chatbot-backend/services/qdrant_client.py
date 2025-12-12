"""
Qdrant Vector Database Client
Feature: 004-rag-chat

Async client for Qdrant operations: collection creation, upserting chunks, and semantic search.
"""

import os
import uuid
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue
)
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def get_qdrant_client() -> QdrantClient:
    """
    Initialize Qdrant client with credentials from environment.

    Returns:
        Configured QdrantClient

    Raises:
        ValueError: If required environment variables not found
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        raise ValueError("QDRANT_URL environment variable not set")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY environment variable not set")

    return QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=300  # Increase timeout to 300 seconds (5 minutes)
    )


def create_collection(
    collection_name: str,
    vector_size: int = 1536,
    distance: Distance = Distance.COSINE,
    client: Optional[QdrantClient] = None
) -> bool:
    """
    Create a Qdrant collection with specified vector configuration.

    Args:
        collection_name: Name of collection to create
        vector_size: Dimensionality of vectors (default: 1536 for text-embedding-3-small)
        distance: Distance metric (default: COSINE)
        client: Optional pre-initialized QdrantClient

    Returns:
        True if collection created successfully or already exists

    Example:
        >>> create_collection("robotics_textbook")
        True

    Raises:
        Exception: If collection creation fails
    """
    if client is None:
        client = get_qdrant_client()

    try:
        # Check if collection already exists
        collections = client.get_collections().collections
        collection_names = [col.name for col in collections]

        if collection_name in collection_names:
            print(f"Collection '{collection_name}' already exists")
            return True

        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=vector_size,
                distance=distance
            )
        )

        print(f"[OK] Created collection '{collection_name}' (vector_size={vector_size}, distance={distance.name})")
        return True

    except Exception as e:
        raise Exception(f"Failed to create collection '{collection_name}': {str(e)}")


def upsert_chunks(
    chunks: List[Dict[str, any]],
    collection_name: str,
    client: Optional[QdrantClient] = None
) -> int:
    """
    Upsert chunks with embeddings to Qdrant collection.

    Args:
        chunks: List of chunk dicts with 'embedding', 'text', 'file_path', etc.
        collection_name: Target collection name
        client: Optional pre-initialized QdrantClient

    Returns:
        Number of points successfully upserted

    Example:
        >>> chunks = [{
        ...     "text": "ROS 2 is...",
        ...     "embedding": [0.1, 0.2, ...],
        ...     "file_path": "docs/ch1.md",
        ...     "heading": "Introduction"
        ... }]
        >>> upsert_chunks(chunks, "robotics_textbook")
        1

    Raises:
        ValueError: If chunks missing required fields
        Exception: If upsert operation fails
    """
    if client is None:
        client = get_qdrant_client()

    if not chunks:
        raise ValueError("Chunks list cannot be empty")

    # Validate chunks have required fields
    for i, chunk in enumerate(chunks):
        if "embedding" not in chunk:
            raise ValueError(f"Chunk {i} missing 'embedding' field")
        if "text" not in chunk:
            raise ValueError(f"Chunk {i} missing 'text' field")

    # Prepare points for upsert
    points = []
    for chunk in chunks:
        point_id = str(uuid.uuid4())  # Generate unique ID

        # Extract payload (all fields except embedding)
        payload = {
            key: value for key, value in chunk.items()
            if key != "embedding"
        }

        point = PointStruct(
            id=point_id,
            vector=chunk["embedding"],
            payload=payload
        )

        points.append(point)

    try:
        # Upsert points in smaller batches to avoid timeouts
        batch_size = 20  # Upload 20 vectors at a time (reduced from 50)
        total_uploaded = 0

        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            client.upsert(
                collection_name=collection_name,
                points=batch,
                wait=True  # Wait for operation to complete before continuing
            )
            total_uploaded += len(batch)
            print(f"[PROGRESS] Uploaded {total_uploaded}/{len(points)} points...")

        print(f"[OK] Upserted {len(points)} points to collection '{collection_name}'")
        return len(points)

    except Exception as e:
        raise Exception(f"Failed to upsert chunks to '{collection_name}': {str(e)}")


def search_qdrant(
    query_embedding: List[float],
    collection_name: str,
    limit: int = 5,
    score_threshold: float = 0.7,
    filter_conditions: Optional[Filter] = None,
    client: Optional[QdrantClient] = None
) -> List[Dict[str, any]]:
    """
    Search Qdrant collection for similar vectors.

    Args:
        query_embedding: Query vector (1536 dimensions)
        collection_name: Collection to search
        limit: Maximum number of results (default: 5)
        score_threshold: Minimum similarity score (default: 0.7)
        filter_conditions: Optional metadata filters
        client: Optional pre-initialized QdrantClient

    Returns:
        List of search results with:
        - id: Point ID
        - score: Similarity score (0-1)
        - payload: Chunk metadata (text, file_path, heading, etc.)

    Example:
        >>> results = search_qdrant(
        ...     query_embedding=[0.1, 0.2, ...],
        ...     collection_name="robotics_textbook",
        ...     limit=5,
        ...     score_threshold=0.7
        ... )
        >>> len(results)  # 0-5 results
        >>> results[0]['score']  # >= 0.7
        >>> results[0]['payload']['text']  # Chunk text

    Raises:
        Exception: If search operation fails
    """
    if client is None:
        client = get_qdrant_client()

    try:
        # Search collection
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=filter_conditions
        )

        # Convert results to dict format
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "score": result.score,
                "payload": result.payload
            })

        return results

    except Exception as e:
        raise Exception(f"Failed to search collection '{collection_name}': {str(e)}")


def get_collection_info(
    collection_name: str,
    client: Optional[QdrantClient] = None
) -> Dict[str, any]:
    """
    Get collection metadata and statistics.

    Args:
        collection_name: Collection name
        client: Optional pre-initialized QdrantClient

    Returns:
        Dict with collection info:
        - name: Collection name
        - vectors_count: Number of vectors
        - points_count: Number of points
        - status: Collection status

    Example:
        >>> info = get_collection_info("robotics_textbook")
        >>> info['points_count']  # 150-200
    """
    if client is None:
        client = get_qdrant_client()

    try:
        collection_info = client.get_collection(collection_name)

        return {
            "name": collection_name,
            "vectors_count": collection_info.vectors_count,
            "points_count": collection_info.points_count,
            "status": collection_info.status
        }

    except Exception as e:
        raise Exception(f"Failed to get collection info for '{collection_name}': {str(e)}")


def delete_collection(
    collection_name: str,
    client: Optional[QdrantClient] = None
) -> bool:
    """
    Delete a Qdrant collection.

    Args:
        collection_name: Name of collection to delete
        client: Optional pre-initialized QdrantClient

    Returns:
        True if deleted successfully

    Example:
        >>> delete_collection("test_collection")
        True
    """
    if client is None:
        client = get_qdrant_client()

    try:
        client.delete_collection(collection_name)
        print(f"[OK] Deleted collection '{collection_name}'")
        return True

    except Exception as e:
        raise Exception(f"Failed to delete collection '{collection_name}': {str(e)}")


if __name__ == "__main__":
    # Test collection creation
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")

    print(f"Testing Qdrant operations with collection '{collection_name}'...")

    try:
        # Test connection and create collection
        client = get_qdrant_client()
        print("[OK] Connected to Qdrant")

        # Create collection
        create_collection(collection_name, client=client)

        # Get collection info
        info = get_collection_info(collection_name, client=client)
        print(f"[OK] Collection info: {info['points_count']} points")

    except Exception as e:
        print(f"[ERROR] Test failed: {e}")
