"""
POST /api/auth/signout endpoint
Invalidate session token (logout)
"""

from fastapi import APIRouter, HTTPException, status, Depends, Request
from pydantic import BaseModel, Field
from sqlalchemy.ext.asyncio import AsyncSession

from database.connection import get_db
from services.auth_service import AuthService
from middleware.auth_middleware import get_session_token

router = APIRouter()


class SignoutResponse(BaseModel):
    """Signout success response"""
    message: str = Field(default="Signed out successfully")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "message": "Signed out successfully"
            }]
        }
    }


@router.post(
    "/signout",
    response_model=SignoutResponse,
    status_code=status.HTTP_200_OK,
    tags=["Authentication"],
    summary="Sign out and invalidate session",
    description="""
    Invalidate the current session token (logout).

    **Authentication Required**: Must provide valid session token in Authorization header.

    **Effect**:
    - Deletes session from database (instant logout)
    - Token can no longer be used for authenticated requests
    - User must sign in again to get new token

    **Headers**:
    ```
    Authorization: Bearer <session_token>
    ```

    **Database-backed sessions** enable instant logout (unlike JWT which requires expiry wait).
    """
)
async def signout(
    request: Request,
    db: AsyncSession = Depends(get_db),
    token: str = Depends(get_session_token)
):
    """
    Invalidate session token.

    Args:
        request: FastAPI request
        db: Database session
        token: Session token from Authorization header

    Returns:
        Success message

    Raises:
        HTTPException 401: Missing or invalid Authorization header
        HTTPException 500: Server error
    """
    # Delete session from database
    deleted = await AuthService.invalidate_session(db, token)

    # Commit transaction
    await db.commit()

    # Note: Even if session doesn't exist, return success (idempotent)
    return SignoutResponse(message="Signed out successfully")
