"""
GET /api/auth/session endpoint
Verify session token and return current user
"""

from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field
from sqlalchemy.ext.asyncio import AsyncSession

from database.connection import get_db
from database.models import User
from middleware.auth_middleware import get_current_user

router = APIRouter()


class SessionResponse(BaseModel):
    """Session verification response with user data"""
    user: dict = Field(..., description="Current authenticated user")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "user": {
                    "id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "student@example.com",
                    "name": "John Doe",
                    "created_at": "2024-12-04T10:00:00Z"
                }
            }]
        }
    }


@router.get(
    "/session",
    response_model=SessionResponse,
    status_code=200,
    tags=["Authentication"],
    summary="Verify session and get current user",
    description="""
    Verify session token is valid and return current user data.

    **Use Case**: Check authentication state on page load/refresh.

    **Authentication Required**: Must provide valid session token.

    **Headers**:
    ```
    Authorization: Bearer <session_token>
    ```

    **Returns**:
    - User object if session is valid and not expired
    - 401 Unauthorized if token is missing, invalid, or expired

    **Frontend Integration**:
    ```typescript
    // On app mount, check if user is still signed in
    const checkSession = async () => {
      const token = localStorage.getItem('sessionToken');
      if (!token) return null;

      const res = await fetch('/api/auth/session', {
        headers: { Authorization: `Bearer ${token}` }
      });

      if (res.ok) {
        const { user } = await res.json();
        return user;
      }
      return null;
    };
    ```
    """
)
async def verify_session(
    db: AsyncSession = Depends(get_db),
    user: User = Depends(get_current_user)
):
    """
    Verify session token and return user.

    Args:
        db: Database session
        user: Current user (from auth middleware)

    Returns:
        User object

    Raises:
        HTTPException 401: Invalid or expired session token
    """
    return SessionResponse(
        user={
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "created_at": user.created_at.isoformat()
        }
    )
