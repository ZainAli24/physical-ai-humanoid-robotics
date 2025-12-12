"""
OpenAI Agent Tools
Feature: 004-rag-chat (Updated for 005-text-selection)

Custom tools for the RAG Agent, implemented using @function_tool decorator.
"""

import os
from typing import List, Dict
from agents import function_tool
from dotenv import load_dotenv

# Import services
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from services.embeddings import generate_embedding
from services.qdrant_client import get_qdrant_client, search_qdrant
from services.reranking import rerank_chunks_by_selected_text  # Feature 005

# Load environment variables
load_dotenv()


@function_tool
async def retrieve_context(query: str, selected_text: str | None = None) -> str:
    """
    Retrieve relevant context from the textbook using semantic search.
    If selected_text is provided, rerank chunks by relevance to the selection.

    This tool searches the vector database for passages related to the user's query
    and returns formatted context with source citations. When selected_text is provided,
    chunks are reranked using semantic similarity and exact matching to prioritize
    content related to the user's text selection.

    Args:
        query: The user's question or search query
        selected_text: Optional. User's selected text from textbook (10-500 chars).
                      If provided, chunks are reranked by relevance to this text.

    Returns:
        Formatted context string with:
        - Retrieved text passages
        - Source citations (chapter, section, URL)
        - Similarity scores

    Example (without selected text):
        When user asks "What is ROS 2?", this tool:
        1. Generates embedding for "What is ROS 2?"
        2. Searches Qdrant for top 5 similar chunks (score >= 0.7)
        3. Returns: "Context from Chapter 1: ROS 2 is... [Source: Introduction to ROS 2]"

    Example (with selected text):
        When user selects "A node is a participant in the ROS 2 graph" and asks "Explain this":
        1. Generates embedding for "Explain this"
        2. Searches Qdrant for top 10 similar chunks
        3. Reranks chunks by similarity to selected text + exact matching
        4. Returns top 5 reranked chunks prioritizing node-related content

    Implementation Details:
        - Embedding model: text-embedding-3-small (1536 dimensions)
        - Retrieval limit: 10 chunks if selected_text provided, else 5
        - Similarity threshold: 0.7 (configurable via SIMILARITY_THRESHOLD env var)
        - Reranking: Semantic boost (1.5x if similarity > 0.5), Exact match boost (2.0x)
        - Fallback: If all similarities < 0.5, use top 5 from original search
        - Deduplication: Max 1 chunk per file (prevents repetitive context)
    """
    if not query or not query.strip():
        return "Error: Query cannot be empty."

    try:
        # Load configuration
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")
        retrieval_limit = int(os.getenv("RETRIEVAL_LIMIT", "5"))
        similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))
        embedding_model = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")

        # Adjust retrieval limit if reranking is needed
        # Get 10 chunks instead of 5 to allow better reranking results
        search_limit = 10 if selected_text else retrieval_limit

        # Step 1: Generate query embedding
        query_embedding = generate_embedding(query, model=embedding_model)

        # Step 2: Search Qdrant
        client = get_qdrant_client()
        results = search_qdrant(
            query_embedding=query_embedding,
            collection_name=collection_name,
            limit=search_limit,
            score_threshold=similarity_threshold,
            client=client
        )

        # Step 3: Rerank if selected_text provided
        if selected_text and results:
            # Convert results to format expected by reranking function
            chunks_for_reranking = []
            for result in results:
                chunk = {
                    'text': result['payload'].get('text', ''),
                    'embedding': query_embedding,  # Will be replaced with chunk embedding
                    'score': result['score'],
                    'payload': result['payload']  # Preserve original payload
                }
                # Get chunk embedding from Qdrant result (stored in vector)
                if 'vector' in result:
                    chunk['embedding'] = result['vector']
                chunks_for_reranking.append(chunk)

            # Define async wrapper for embedding function
            async def embed_wrapper(text: str):
                return generate_embedding(text, model=embedding_model)

            # Rerank chunks using semantic similarity to selected text (await directly since we're async now)
            reranked_chunks = await rerank_chunks_by_selected_text(
                chunks=chunks_for_reranking,
                selected_text=selected_text,
                embedding_function=embed_wrapper
            )

            # Check if any chunks have good similarity (> 0.5)
            has_good_matches = any(
                chunk.get('selected_similarity', 0) > 0.5
                for chunk in reranked_chunks
            )

            # Fallback to top 5 from original search if no good matches
            if not has_good_matches:
                results = results[:retrieval_limit]
            else:
                # Use reranked results (already limited to top 5)
                results = [
                    {
                        'payload': chunk['payload'],
                        'score': chunk.get('boosted_score', chunk['score'])
                    }
                    for chunk in reranked_chunks
                ]

        # Step 4: Check if no results
        if not results:
            return (
                "No relevant information found in the textbook for this query. "
                "The question may be off-topic or not covered in the available chapters."
            )

        # Step 5: Deduplicate by file (max 1 chunk per file for diversity)
        seen_files = set()
        unique_results = []

        for result in results:
            file_path = result["payload"].get("file_path", "")
            if file_path not in seen_files:
                unique_results.append(result)
                seen_files.add(file_path)

        # Step 6: Format context with sources
        context_parts = []
        sources = []

        for i, result in enumerate(unique_results, 1):
            payload = result["payload"]
            text = payload.get("text", "")
            chapter_title = payload.get("chapter_title", "Unknown Chapter")
            heading = payload.get("heading", "")
            url = payload.get("url", "")
            score = result["score"]

            # Format context entry
            context_entry = f"[Source {i}] {chapter_title}"
            if heading:
                context_entry += f" - {heading}"
            context_entry += f":\n{text}\n"

            context_parts.append(context_entry)

            # Format source citation
            source_citation = {
                "title": chapter_title,
                "heading": heading or "Main content",
                "url": url,
                "score": round(score, 3),
                "excerpt": text[:150] + "..." if len(text) > 150 else text
            }
            sources.append(source_citation)

        # Step 7: Combine into final context string
        context_text = "\n\n".join(context_parts)

        # Add sources summary
        sources_summary = "\n\n---\n**Sources:**\n"
        for i, source in enumerate(sources, 1):
            sources_summary += f"{i}. {source['title']}"
            if source['heading']:
                sources_summary += f" - {source['heading']}"
            sources_summary += f" (relevance: {source['score']})\n"
            if source['url']:
                sources_summary += f"   URL: {source['url']}\n"

        final_context = context_text + sources_summary

        return final_context

    except Exception as e:
        return f"Error retrieving context: {str(e)}"


if __name__ == "__main__":
    # Test retrieve_context tool
    print("Testing retrieve_context tool...")

    test_queries = [
        "What is ROS 2?",
        "How does Gazebo simulation work?",
        "What is Isaac Sim?"
    ]

    for query in test_queries:
        print(f"\n{'=' * 60}")
        print(f"Query: {query}")
        print(f"{'=' * 60}")

        try:
            context = retrieve_context(query)
            print(context)
        except Exception as e:
            print(f"❌ Error: {e}")
