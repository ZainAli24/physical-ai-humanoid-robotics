"""
POST /api/auth/signup endpoint
Create new user account with email and password
"""

from fastapi import APIRouter, HTTPException, status, Depends
from fastapi import Request
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from database.connection import get_db
from database.models import User
from services.auth_service import AuthService
from middleware.rate_limit import check_rate_limit_middleware, get_rate_limiter

router = APIRouter()


class SignupRequest(BaseModel):
    """Signup request body validation"""
    email: EmailStr = Field(..., description="Valid email address")
    password: str = Field(..., min_length=8, max_length=128, description="Password (8-128 characters)")
    name: str | None = Field(None, max_length=255, description="Optional display name")

    model_config = {
        "json_schema_extra": {
            "examples": [{
                "email": "student@example.com",
                "password": "securepassword123",
                "name": "John Doe"
            }]
        }
    }


class SignupResponse(BaseModel):
    """Signup response with user data and session token"""
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
    "/signup",
    response_model=SignupResponse,
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(check_rate_limit_middleware)],
    tags=["Authentication"],
    summary="Create new user account",
    description="""
    Register a new user with email and password.

    **Rate Limited**: 5 attempts per 15 minutes per IP address.

    **Password Requirements**:
    - Minimum 8 characters
    - Maximum 128 characters
    - Hashed with bcrypt (12 rounds)

    **Returns**:
    - User object with id, email, name, created_at
    - Session token (43 characters, valid for 30 days)

    **Use session token in subsequent requests**:
    ```
    Authorization: Bearer <session_token>
    ```
    """
)
async def signup(
    request: Request,
    body: SignupRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Create new user account.

    Args:
        request: FastAPI request (for rate limiting)
        body: Signup request with email, password, optional name
        db: Database session

    Returns:
        User object and session token

    Raises:
        HTTPException 400: Email already registered
        HTTPException 429: Too many attempts (rate limit)
        HTTPException 500: Server error
    """
    # Check if email already exists
    is_unique = await AuthService.validate_email_unique(db, body.email)

    if not is_unique:
        # Record failed attempt for rate limiting
        limiter = get_rate_limiter()
        limiter.record_failed_attempt(request)

        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )

    # Hash password
    password_hash = AuthService.hash_password(body.password)

    # Create user
    user = User(
        email=body.email,
        password_hash=password_hash,
        name=body.name
    )
    db.add(user)
    await db.flush()  # Get user.id without committing

    # Create session
    session = await AuthService.create_session(db, user.id)

    # Commit transaction
    await db.commit()

    # Reset rate limit for this IP (successful signup)
    limiter = get_rate_limiter()
    limiter.reset_ip(request)

    return SignupResponse(
        user={
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "created_at": user.created_at.isoformat()
        },
        session_token=session.token
    )
