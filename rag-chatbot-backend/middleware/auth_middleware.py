"""
Authentication middleware for FastAPI routes.
Validates session tokens and loads user context.
"""

from fastapi import Request, HTTPException, status, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional

from database.connection import get_db
from database.models import User
from services.auth_service import AuthService


async def get_current_user(
    request: Request,
    db: AsyncSession = Depends(get_db)
) -> User:
    """
    FastAPI dependency to get current authenticated user (REQUIRED).

    Validates Authorization header with Bearer token.
    Raises 401 if token is missing, invalid, or expired.

    Usage:
        @app.get("/protected")
        async def protected_route(user: User = Depends(get_current_user)):
            return {"user_id": user.id, "email": user.email}

    Args:
        request: FastAPI request object
        db: Database session

    Returns:
        User object for authenticated user

    Raises:
        HTTPException: 401 Unauthorized if token is invalid or missing
    """
    # Extract Authorization header
    auth_header = request.headers.get('Authorization')

    if not auth_header:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing",
            headers={"WWW-Authenticate": "Bearer"}
        )

    # Validate Bearer token format
    parts = auth_header.split()
    if len(parts) != 2 or parts[0].lower() != 'bearer':
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid Authorization header format. Expected: Bearer <token>",
            headers={"WWW-Authenticate": "Bearer"}
        )

    token = parts[1]

    # Verify session token and get user
    user = await AuthService.verify_session(db, token)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired session token",
            headers={"WWW-Authenticate": "Bearer"}
        )

    return user


async def get_current_user_optional(
    request: Request,
    db: AsyncSession = Depends(get_db)
) -> Optional[User]:
    """
    FastAPI dependency to get current user (OPTIONAL).

    Returns User if valid token provided, None if no token or invalid.
    Does NOT raise exceptions - useful for routes that support both
    authenticated and anonymous access.

    Usage:
        @app.post("/chat")
        async def chat(
            message: str,
            user: Optional[User] = Depends(get_current_user_optional)
        ):
            if user:
                # Save to chat history
                ...
            # Process message (works for both auth and anon)

    Args:
        request: FastAPI request object
        db: Database session

    Returns:
        User object if authenticated, None otherwise
    """
    # Extract Authorization header
    auth_header = request.headers.get('Authorization')

    if not auth_header:
        return None

    # Validate Bearer token format
    parts = auth_header.split()
    if len(parts) != 2 or parts[0].lower() != 'bearer':
        return None

    token = parts[1]

    # Verify session token and get user (returns None if invalid)
    return await AuthService.verify_session(db, token)


def get_session_token(request: Request) -> str:
    """
    Extract session token from Authorization header.

    Useful for signout endpoint where we need the token itself
    (not the user).

    Args:
        request: FastAPI request object

    Returns:
        Session token string

    Raises:
        HTTPException: 401 if header missing or malformed
    """
    auth_header = request.headers.get('Authorization')

    if not auth_header:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing",
            headers={"WWW-Authenticate": "Bearer"}
        )

    parts = auth_header.split()
    if len(parts) != 2 or parts[0].lower() != 'bearer':
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid Authorization header format",
            headers={"WWW-Authenticate": "Bearer"}
        )

    return parts[1]
