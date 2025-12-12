"""
Reranking Service
Feature: 005-text-selection

Semantic reranking of retrieved chunks using cosine similarity to selected text.
"""

from scipy.spatial.distance import cosine
from typing import List, Callable, Awaitable


async def rerank_chunks_by_selected_text(
    chunks: List[dict],
    selected_text: str,
    embedding_function: Callable[[str], Awaitable[List[float]]]
) -> List[dict]:
    """
    Rerank chunks using semantic similarity to selected text.

    This function applies two boosting strategies:
    1. Semantic boost (1.5x): For chunks with cosine similarity > 0.5 to selected text
    2. Exact match boost (2.0x): For chunks containing selected text as substring

    Args:
        chunks: List of dicts with 'text', 'embedding', 'score' keys from Qdrant search
        selected_text: User's selected text from textbook (10-500 chars)
        embedding_function: Async function to generate embeddings (same model as retrieval)

    Returns:
        Top 5 chunks sorted by boosted score, with added fields:
        - 'selected_similarity': float (0-1) cosine similarity to selected text
        - 'boosted_score': float, original score * boost factors

    Performance:
        ~10-50ms for 10 chunks (SciPy cosine is O(n) where n = embedding dimensions)

    Example:
        >>> chunks = [
        ...     {'text': 'ROS 2 nodes...', 'embedding': [0.1, 0.2, ...], 'score': 0.85},
        ...     {'text': 'Gazebo simulation...', 'embedding': [0.3, 0.4, ...], 'score': 0.78}
        ... ]
        >>> selected = "ROS 2 nodes are fundamental building blocks"
        >>> reranked = await rerank_chunks_by_selected_text(chunks, selected, embed_fn)
        >>> reranked[0]['boosted_score']  # Higher due to semantic + exact match
        2.55  # 0.85 * 1.5 (semantic) * 2.0 (exact match)
    """
    # Generate embedding for selected text
    selected_embedding = await embedding_function(selected_text)

    # Apply boosting to each chunk
    for chunk in chunks:
        # Calculate cosine similarity (convert distance to similarity: 1 - distance)
        # Cosine distance is in [0, 2], similarity in [0, 1] where 1 = identical
        similarity = 1 - cosine(chunk['embedding'], selected_embedding)
        chunk['selected_similarity'] = similarity

        # Start with original score
        boosted_score = chunk['score']

        # Apply semantic boost (similarity > 0.5 threshold)
        if similarity > 0.5:
            boosted_score *= 1.5  # 50% boost

        # Apply exact match boost (case-insensitive substring)
        if selected_text.lower() in chunk['text'].lower():
            boosted_score *= 2.0  # 100% boost

        chunk['boosted_score'] = boosted_score

    # Sort by boosted score (descending), return top 5
    reranked = sorted(chunks, key=lambda c: c['boosted_score'], reverse=True)
    return reranked[:5]
