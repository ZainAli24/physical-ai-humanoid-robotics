"""
Chat Sessions API Endpoints
Feature: 006-auth-sessions - User Story 3 & 6

GET /api/sessions - List user's chat sessions with pagination
PATCH /api/sessions/:id - Rename a chat session (T079)
DELETE /api/sessions/:id - Delete a chat session and all messages (T080)
"""

from fastapi import APIRouter, Depends, HTTPException, status, Query, Path
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from pydantic import BaseModel, Field
from datetime import datetime

# Import models and services
import sys
from pathlib import Path as PathLib
sys.path.insert(0, str(PathLib(__file__).parent.parent.parent))

from database.connection import get_db
from database.models import User
from middleware.auth_middleware import get_current_user
from services.history_service import HistoryService

# Create router
router = APIRouter(prefix="/api", tags=["history"])


# Response Models
class SessionSummary(BaseModel):
    """Chat session summary for list view"""
    id: str
    title: str
    created_at: datetime
    updated_at: datetime
    message_count: int

    class Config:
        from_attributes = True


class SessionsListResponse(BaseModel):
    """Response for GET /api/sessions"""
    sessions: List[SessionSummary]
    total: int
    limit: int
    offset: int


class UpdateSessionRequest(BaseModel):
    """Request for PATCH /api/sessions/:id (T079)"""
    title: str = Field(..., min_length=1, max_length=200, description="New session title")


class UpdateSessionResponse(BaseModel):
    """Response for PATCH /api/sessions/:id (T079)"""
    message: str
    session_id: str
    title: str
    updated_at: datetime


class DeleteSessionResponse(BaseModel):
    """Response for DELETE /api/sessions/:id (T080)"""
    message: str
    session_id: str
    deleted_messages: int


@router.get(
    "/sessions",
    response_model=SessionsListResponse,
    summary="List user's chat sessions",
    description="Get paginated list of user's chat sessions with message counts"
)
async def list_sessions(
    limit: int = Query(default=50, ge=1, le=100, description="Number of sessions to return"),
    offset: int = Query(default=0, ge=0, description="Number of sessions to skip"),
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    List user's chat sessions with pagination.

    **Authentication Required**: Yes (session token in Authorization header)

    **Query Parameters**:
    - `limit`: Max sessions to return (1-100, default 50)
    - `offset`: Number of sessions to skip (default 0)

    **Returns**:
    - List of sessions sorted by most recent first
    - Each session includes: id, title, timestamps, message_count

    **Example Request**:
    ```bash
    curl -H "Authorization: Bearer <token>" \
         "http://localhost:8000/api/sessions?limit=20&offset=0"
    ```

    **Example Response**:
    ```json
    {
      "sessions": [
        {
          "id": "uuid-here",
          "title": "What is ROS 2?",
          "created_at": "2025-12-09T10:00:00Z",
          "updated_at": "2025-12-09T10:05:00Z",
          "message_count": 4
        }
      ],
      "total": 1,
      "limit": 50,
      "offset": 0
    }
    ```
    """
    try:
        # Get sessions with message counts (T052)
        sessions = await HistoryService.list_user_sessions(
            db=db,
            user_id=user.id,
            limit=limit,
            offset=offset
        )

        # Transform to response model
        session_summaries = [
            SessionSummary(
                id=session['id'],
                title=session['title'],
                created_at=session['created_at'],
                updated_at=session['updated_at'],
                message_count=session['message_count']
            )
            for session in sessions
        ]

        return SessionsListResponse(
            sessions=session_summaries,
            total=len(session_summaries),  # Note: For true pagination, need separate count query
            limit=limit,
            offset=offset
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to fetch sessions: {str(e)}"
        )


@router.patch(
    "/sessions/{session_id}",
    response_model=UpdateSessionResponse,
    summary="Rename a chat session",
    description="Update the title of a chat session (T079)"
)
async def update_session(
    session_id: str = Path(..., description="Chat session UUID"),
    request: UpdateSessionRequest = ...,
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Rename a chat session.

    **Authentication Required**: Yes (session token in Authorization header)

    **T081: Session Ownership Verification**: Returns 403 if session doesn't belong to user

    **Request Body**:
    ```json
    {
      "title": "New Session Title"
    }
    ```

    **Example Request**:
    ```bash
    curl -X PATCH \
         -H "Authorization: Bearer <token>" \
         -H "Content-Type: application/json" \
         -d '{"title":"New Title"}' \
         http://localhost:8000/api/sessions/<id>
    ```

    **Example Response**:
    ```json
    {
      "message": "Session renamed successfully",
      "session_id": "uuid-here",
      "title": "New Title",
      "updated_at": "2025-12-09T10:30:00Z"
    }
    ```
    """
    try:
        # T081: Verify user owns the session
        session = await HistoryService.get_session(db, session_id, user.id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Session not found or you don't have permission to modify it"
            )

        # Update the title
        updated_session = await HistoryService.update_session_title(
            db=db,
            session_id=session_id,
            title=request.title
        )
        await db.commit()

        return UpdateSessionResponse(
            message="Session renamed successfully",
            session_id=session_id,
            title=request.title,
            updated_at=updated_session.updated_at
        )

    except HTTPException:
        raise
    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to rename session: {str(e)}"
        )


@router.delete(
    "/sessions/{session_id}",
    response_model=DeleteSessionResponse,
    summary="Delete a chat session",
    description="Delete a chat session and all its messages (T080)"
)
async def delete_session(
    session_id: str = Path(..., description="Chat session UUID"),
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Delete a chat session and all its messages.

    **CRITICAL**: This operation is IRREVERSIBLE!

    **Authentication Required**: Yes (session token in Authorization header)

    **T081: Session Ownership Verification**: Returns 403 if session doesn't belong to user

    **What Gets Deleted** (CASCADE):
    - Chat session
    - All messages in the session (with JSONB sources)

    **Example Request**:
    ```bash
    curl -X DELETE \
         -H "Authorization: Bearer <token>" \
         http://localhost:8000/api/sessions/<id>
    ```

    **Example Response**:
    ```json
    {
      "message": "Session deleted successfully",
      "session_id": "uuid-here",
      "deleted_messages": 8
    }
    ```
    """
    try:
        # T081: Verify user owns the session
        session = await HistoryService.get_session(db, session_id, user.id)

        if not session:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Session not found or you don't have permission to delete it"
            )

        # Count messages before deletion (for response)
        from sqlalchemy import select, func
        from database.models import Message

        messages_count_stmt = select(func.count()).select_from(Message).filter_by(session_id=session_id)
        messages_result = await db.execute(messages_count_stmt)
        messages_count = messages_result.scalar() or 0

        # T080: Delete session (CASCADE will delete all messages)
        # Database models have CASCADE delete configured:
        # ChatSession → Messages (CASCADE)
        await db.delete(session)
        await db.commit()

        return DeleteSessionResponse(
            message="Session deleted successfully",
            session_id=session_id,
            deleted_messages=messages_count
        )

    except HTTPException:
        raise
    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete session: {str(e)}"
        )
