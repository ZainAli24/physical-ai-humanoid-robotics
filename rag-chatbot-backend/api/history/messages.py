"""
Chat Messages API Endpoints
Feature: 006-auth-sessions - User Story 3

GET /api/sessions/{session_id}/messages - Get messages for a specific chat session
"""

from fastapi import APIRouter, Depends, HTTPException, status, Path
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
from datetime import datetime

# Import models and services
import sys
from pathlib import Path as FilePath
sys.path.insert(0, str(FilePath(__file__).parent.parent.parent))

from database.connection import get_db
from database.models import User
from middleware.auth_middleware import get_current_user
from services.history_service import HistoryService

# Create router
router = APIRouter(prefix="/api/sessions", tags=["history"])


# Response Models
class MessageResponse(BaseModel):
    """Chat message response"""
    id: str
    role: str  # 'user' or 'assistant'
    content: str
    sources: Optional[List[Dict[str, Any]]] = None
    agent_trace_id: Optional[str] = None
    created_at: datetime

    class Config:
        from_attributes = True


class MessagesListResponse(BaseModel):
    """Response for GET /api/sessions/{session_id}/messages"""
    session_id: str
    messages: List[MessageResponse]
    total: int


@router.get(
    "/{session_id}/messages",
    response_model=MessagesListResponse,
    summary="Get messages for a chat session",
    description="Retrieve all messages for a specific chat session (requires session ownership)"
)
async def get_session_messages(
    session_id: str = Path(..., description="Chat session UUID"),
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Get all messages for a specific chat session.

    **Authentication Required**: Yes (session token in Authorization header)

    **Path Parameters**:
    - `session_id`: UUID of the chat session

    **Returns**:
    - List of messages sorted chronologically (oldest first)
    - Each message includes: id, role, content, sources (JSONB), timestamps

    **Security**:
    - Verifies user owns the session (T051)
    - Returns 403 if session belongs to different user
    - Returns 404 if session doesn't exist

    **Example Request**:
    ```bash
    curl -H "Authorization: Bearer <token>" \
         "http://localhost:8000/api/sessions/<session-id>/messages"
    ```

    **Example Response**:
    ```json
    {
      "session_id": "uuid-here",
      "messages": [
        {
          "id": "msg-uuid-1",
          "role": "user",
          "content": "What is ROS 2?",
          "sources": null,
          "agent_trace_id": null,
          "created_at": "2025-12-09T10:00:00Z"
        },
        {
          "id": "msg-uuid-2",
          "role": "assistant",
          "content": "ROS 2 is...",
          "sources": [
            {
              "title": "Introduction to ROS 2",
              "url": "/docs/module-1/chapter-1",
              "score": 0.92,
              "excerpt": "..."
            }
          ],
          "agent_trace_id": "trace-123",
          "created_at": "2025-12-09T10:00:05Z"
        }
      ],
      "total": 2
    }
    ```
    """
    try:
        # T051: Verify user owns the session
        session = await HistoryService.get_session(db, session_id, user.id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Chat session not found or you don't have permission to access it"
            )

        # Get all messages for the session
        messages = await HistoryService.get_session_messages(db, session_id)

        # Transform to response model
        message_responses = [
            MessageResponse(
                id=msg.id,
                role=msg.role,
                content=msg.content,
                sources=msg.sources,  # Already JSONB list
                agent_trace_id=msg.agent_trace_id,
                created_at=msg.created_at
            )
            for msg in messages
        ]

        return MessagesListResponse(
            session_id=session_id,
            messages=message_responses,
            total=len(message_responses)
        )

    except HTTPException:
        # Re-raise HTTP exceptions (404, 403, etc.)
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to fetch messages: {str(e)}"
        )
