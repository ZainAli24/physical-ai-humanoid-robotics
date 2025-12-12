"""
User Data Deletion API Endpoint
Feature: 006-auth-sessions - User Story 5 (GDPR Compliance)

DELETE /api/user/data - Delete all user data (account, sessions, chat history)
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel

# Import models and services
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from database.connection import get_db
from database.models import User
from middleware.auth_middleware import get_current_user

# Create router
router = APIRouter(prefix="/api/user", tags=["user"])


# Response Model
class DeleteDataResponse(BaseModel):
    """Response for DELETE /api/user/data"""
    message: str
    user_id: str
    deleted_sessions: int
    deleted_chat_sessions: int
    deleted_messages: int


@router.delete(
    "/data",
    response_model=DeleteDataResponse,
    summary="Delete all user data (GDPR)",
    description="Permanently delete user account and all associated data (sessions, chat history, messages)"
)
async def delete_user_data(
    user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_db)
):
    """
    Delete all user data for GDPR compliance.

    **CRITICAL**: This operation is IRREVERSIBLE!

    **Authentication Required**: Yes (session token in Authorization header)

    **What Gets Deleted** (CASCADE):
    - User account
    - All authentication sessions
    - All chat sessions
    - All messages (with JSONB sources)

    **Process**:
    1. Count associated data before deletion
    2. Delete user (CASCADE deletes all related data)
    3. Return confirmation with deletion counts

    **Example Request**:
    ```bash
    curl -X DELETE \
         -H "Authorization: Bearer <token>" \
         http://localhost:8000/api/user/data
    ```

    **Example Response**:
    ```json
    {
      "message": "All user data deleted successfully",
      "user_id": "uuid-here",
      "deleted_sessions": 1,
      "deleted_chat_sessions": 5,
      "deleted_messages": 20
    }
    ```

    **After This Request**:
    - User is automatically signed out (session token invalidated)
    - User cannot sign in again (account no longer exists)
    - All chat history permanently deleted
    """
    try:
        user_id = user.id

        # T072: Count data before CASCADE delete (for confirmation response)
        from sqlalchemy import select, func
        from database.models import Session, ChatSession, Message

        # Count sessions
        sessions_count_stmt = select(func.count()).select_from(Session).filter_by(user_id=user_id)
        sessions_result = await db.execute(sessions_count_stmt)
        sessions_count = sessions_result.scalar() or 0

        # Count chat sessions
        chat_sessions_count_stmt = select(func.count()).select_from(ChatSession).filter_by(user_id=user_id)
        chat_sessions_result = await db.execute(chat_sessions_count_stmt)
        chat_sessions_count = chat_sessions_result.scalar() or 0

        # Count messages (across all user's chat sessions)
        messages_count_stmt = (
            select(func.count())
            .select_from(Message)
            .join(ChatSession, Message.session_id == ChatSession.id)
            .filter(ChatSession.user_id == user_id)
        )
        messages_result = await db.execute(messages_count_stmt)
        messages_count = messages_result.scalar() or 0

        # T071: Delete user (CASCADE will delete all related data)
        # Database models have CASCADE delete configured:
        # User → Sessions (CASCADE)
        # User → ChatSessions (CASCADE)
        # ChatSession → Messages (CASCADE)
        await db.delete(user)
        await db.commit()

        return DeleteDataResponse(
            message="All user data deleted successfully",
            user_id=user_id,
            deleted_sessions=sessions_count,
            deleted_chat_sessions=chat_sessions_count,
            deleted_messages=messages_count
        )

    except Exception as e:
        await db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete user data: {str(e)}"
        )
