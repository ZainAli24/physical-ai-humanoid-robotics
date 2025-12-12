"""
POST /api/auth/signin endpoint
Authenticate user with email and password
"""

from fastapi import APIRouter, HTTPException, status, Depends, Request
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from database.connection import get_db
from database.models import User
from services.auth_service import AuthService
from middleware.rate_limit import check_rate_limit_middleware, get_rate_limiter

router = APIRouter()


class SigninRequest(BaseModel):
    """Signin request body validation"""
    email: EmailStr = Field(..., description="Registered email address")
    password: str = Field(..., description="Account password")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "email": "student@example.com",
                "password": "securepassword123"
            }]
        }
    }


class SigninResponse(BaseModel):
    """Signin response with user data and session token"""
    user: dict = Field(..., description="User object (id, email, name, created_at)")
    session_token: str = Field(..., description="Session token for Authorization header")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "user": {
                    "id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "student@example.com",
                    "name": "John Doe",
                    "created_at": "2024-12-04T10:00:00Z"
                },
                "session_token": "abc123xyz789..."
            }]
        }
    }


@router.post(
    "/signin",
    response_model=SigninResponse,
    status_code=status.HTTP_200_OK,
    dependencies=[Depends(check_rate_limit_middleware)],
    tags=["Authentication"],
    summary="Sign in to existing account",
    description="""
    Authenticate user with email and password.

    **Rate Limited**: 5 attempts per 15 minutes per IP address (brute-force protection).

    **Returns**:
    - User object with id, email, name, created_at
    - Session token (43 characters, valid for 30 days)

    **Use session token in subsequent requests**:
    ```
    Authorization: Bearer <session_token>
    ```

    **Security Notes**:
    - Failed attempts are tracked per IP address
    - Passwords are compared using bcrypt (constant-time)
    - Sessions are database-backed (can be instantly revoked)
    """
)
async def signin(
    request: Request,
    body: SigninRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Authenticate user and create session.

    Args:
        request: FastAPI request (for rate limiting)
        body: Signin request with email and password
        db: Database session

    Returns:
        User object and session token

    Raises:
        HTTPException 401: Invalid credentials
        HTTPException 429: Too many attempts (rate limit)
        HTTPException 500: Server error
    """
    # Query user by email
    stmt = select(User).filter_by(email=body.email)
    result = await db.execute(stmt)
    user = result.scalar_one_or_none()

    # Verify user exists and password matches
    if not user or not AuthService.verify_password(body.password, user.password_hash):
        # Record failed attempt for rate limiting
        limiter = get_rate_limiter()
        limiter.record_failed_attempt(request)

        # Generic error message (don't reveal if email exists)
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Create new session
    session = await AuthService.create_session(db, user.id)

    # Commit transaction
    await db.commit()

    # Reset rate limit for this IP (successful signin)
    limiter = get_rate_limiter()
    limiter.reset_ip(request)

    return SigninResponse(
        user={
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "created_at": user.created_at.isoformat()
        },
        session_token=session.token
    )
