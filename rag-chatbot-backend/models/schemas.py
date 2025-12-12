"""
Pydantic Models for RAG Chatbot API
Feature: 004-rag-chat (Updated for 005-text-selection)

Request/Response schemas for chat and embed endpoints.
"""

from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field, field_validator


# ============================================================================
# Chat Endpoint Models
# ============================================================================

class ChatRequest(BaseModel):
    """
    Request model for POST /api/chat endpoint.

    Attributes:
        message: User's question (max 500 characters)
        session_id: Optional session ID for conversation history
        stream: Whether to stream response via SSE (default: True)
        selected_text: Optional user-selected text from textbook (Feature 005)
    """
    message: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="User's question to the RAG chatbot"
    )
    session_id: Optional[str] = Field(
        default=None,
        description="Session ID for conversation history (optional)"
    )
    stream: bool = Field(
        default=True,
        description="Enable SSE streaming for response"
    )
    selected_text: Optional[str] = Field(
        default=None,
        min_length=10,
        max_length=500,
        description="User-selected text from textbook for context-aware retrieval (Feature 005)"
    )

    @field_validator("message")
    @classmethod
    def validate_message(cls, v: str) -> str:
        """Validate message is not empty after stripping whitespace."""
        if not v.strip():
            raise ValueError("Message cannot be empty or whitespace only")
        return v.strip()

    @field_validator("selected_text")
    @classmethod
    def validate_selected_text(cls, v: Optional[str]) -> Optional[str]:
        """Validate selected_text length if provided."""
        if v is not None:
            v = v.strip()
            if len(v) < 10:
                raise ValueError("Selected text must be at least 10 characters")
            if len(v) > 500:
                # Truncate to 500 chars with ellipsis
                v = v[:500]
        return v if v else None


class SourceCitation(BaseModel):
    """
    Source citation metadata for retrieved context.

    Attributes:
        title: Chapter or document title
        heading: Section heading (if available)
        url: Link to source chapter
        score: Similarity score (0-1)
        excerpt: Text excerpt from source (max 150 chars)
    """
    title: str = Field(..., description="Chapter or document title")
    heading: Optional[str] = Field(default=None, description="Section heading")
    url: str = Field(..., description="URL to source chapter")
    score: float = Field(..., ge=0.0, le=1.0, description="Similarity score")
    excerpt: str = Field(..., max_length=200, description="Text excerpt")


class ChatResponse(BaseModel):
    """
    Response model for POST /api/chat endpoint (non-streaming).

    Attributes:
        answer: Generated answer from agent
        sources: List of source citations
        session_id: Session ID for this conversation
    """
    answer: str = Field(..., description="Generated answer from RAG agent")
    sources: List[SourceCitation] = Field(
        default_factory=list,
        description="Source citations for the answer"
    )
    session_id: str = Field(..., description="Session ID")


class ChatStreamEvent(BaseModel):
    """
    SSE event model for streaming chat responses.

    Event Types:
        - "content": Streaming answer tokens
        - "sources": Source citations (sent after answer complete)
        - "done": End of stream marker
        - "error": Error message

    Attributes:
        event: Event type
        data: Event payload (string for content/error, list for sources, null for done)
    """
    event: str = Field(..., description="Event type: content, sources, done, error")
    data: Optional[Any] = Field(default=None, description="Event payload")


# ============================================================================
# Embed Endpoint Models (Admin-only)
# ============================================================================

class EmbedRequest(BaseModel):
    """
    Request model for POST /api/embed endpoint (admin-only).

    Attributes:
        admin_api_key: Admin authentication key
        docs_dir: Path to docs directory (optional, defaults to ../docs)
        force: Force re-embedding even if collection exists
    """
    admin_api_key: str = Field(..., description="Admin API key for authentication")
    docs_dir: Optional[str] = Field(
        default="../docs",
        description="Path to docs directory"
    )
    force: bool = Field(
        default=False,
        description="Force re-embedding (deletes existing collection)"
    )


class EmbedResponse(BaseModel):
    """
    Response model for POST /api/embed endpoint.

    Attributes:
        success: Whether embedding succeeded
        message: Status message
        chunks_processed: Number of chunks processed
        chunks_uploaded: Number of chunks uploaded to Qdrant
        collection_name: Target Qdrant collection name
    """
    success: bool = Field(..., description="Whether operation succeeded")
    message: str = Field(..., description="Status message")
    chunks_processed: int = Field(default=0, description="Chunks processed")
    chunks_uploaded: int = Field(default=0, description="Chunks uploaded to Qdrant")
    collection_name: str = Field(..., description="Qdrant collection name")


# ============================================================================
# Error Response Model
# ============================================================================

class ErrorResponse(BaseModel):
    """
    Standard error response model.

    Attributes:
        error: Error type/category
        message: Human-readable error message
        detail: Additional error details (optional)
    """
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Error message")
    detail: Optional[str] = Field(default=None, description="Additional details")


# ============================================================================
# Health Check Model
# ============================================================================

class HealthResponse(BaseModel):
    """
    Health check response model for GET / endpoint.

    Attributes:
        status: Service status (healthy, unhealthy)
        version: API version
        model: Chat model being used
        qdrant_connected: Whether Qdrant is accessible
        collection_points: Number of points in Qdrant collection
    """
    status: str = Field(..., description="Service status")
    version: str = Field(default="1.0.0", description="API version")
    model: str = Field(..., description="Chat model name")
    qdrant_connected: bool = Field(..., description="Qdrant connection status")
    collection_points: Optional[int] = Field(
        default=None,
        description="Number of points in collection"
    )


if __name__ == "__main__":
    # Test model validation
    print("Testing Pydantic models...")

    # Test ChatRequest
    try:
        req = ChatRequest(message="What is ROS 2?")
        print(f"✅ ChatRequest: {req.message}")
    except Exception as e:
        print(f"❌ ChatRequest failed: {e}")

    # Test ChatResponse
    try:
        resp = ChatResponse(
            answer="ROS 2 is a robotics framework...",
            sources=[
                SourceCitation(
                    title="Introduction to ROS 2",
                    heading="Overview",
                    url="/docs/module-1/chapter-1",
                    score=0.92,
                    excerpt="ROS 2 is a set of software libraries..."
                )
            ],
            session_id="test-123"
        )
        print(f"✅ ChatResponse: {len(resp.sources)} sources")
    except Exception as e:
        print(f"❌ ChatResponse failed: {e}")

    # Test validation error
    try:
        invalid_req = ChatRequest(message="")
        print("❌ Validation should have failed for empty message")
    except Exception as e:
        print(f"✅ Validation correctly rejected empty message: {e}")
