"""
Authentication service for password hashing, session token generation, and verification.
"""

import bcrypt
import secrets
from datetime import datetime, timedelta, timezone
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional

from database.models import User, Session


class AuthService:
    """Service for authentication operations."""

    # Password hashing configuration
    BCRYPT_ROUNDS = 12  # Cost factor (2^12 = 4096 iterations)

    # Session token configuration
    TOKEN_BYTES = 32  # 32 bytes = 43 characters in base64
    SESSION_EXPIRY_DAYS = 30

    @staticmethod
    def hash_password(password: str) -> str:
        """
        Hash password using bcrypt with 12 rounds.

        Args:
            password: Plain text password

        Returns:
            Hashed password (60+ characters)

        Example:
            >>> hashed = AuthService.hash_password('my_secure_password')
            >>> len(hashed) >= 60
            True
        """
        salt = bcrypt.gensalt(rounds=AuthService.BCRYPT_ROUNDS)
        hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
        return hashed.decode('utf-8')

    @staticmethod
    def verify_password(password: str, password_hash: str) -> bool:
        """
        Verify password against bcrypt hash.

        Args:
            password: Plain text password to verify
            password_hash: Stored bcrypt hash

        Returns:
            True if password matches, False otherwise

        Example:
            >>> hashed = AuthService.hash_password('test123')
            >>> AuthService.verify_password('test123', hashed)
            True
            >>> AuthService.verify_password('wrong', hashed)
            False
        """
        try:
            return bcrypt.checkpw(
                password.encode('utf-8'),
                password_hash.encode('utf-8')
            )
        except (ValueError, AttributeError):
            return False

    @staticmethod
    def generate_session_token() -> str:
        """
        Generate cryptographically secure session token.

        Returns:
            URL-safe base64 token (43 characters)

        Example:
            >>> token = AuthService.generate_session_token()
            >>> len(token)
            43
        """
        return secrets.token_urlsafe(AuthService.TOKEN_BYTES)

    @staticmethod
    def get_expiry_timestamp(days: int = SESSION_EXPIRY_DAYS) -> datetime:
        """
        Get session expiry timestamp (UTC).

        Args:
            days: Number of days until expiry (default 30)

        Returns:
            Timezone-aware datetime in UTC

        Example:
            >>> expiry = AuthService.get_expiry_timestamp()
            >>> expiry > datetime.now(timezone.utc)
            True
        """
        return datetime.now(timezone.utc) + timedelta(days=days)

    @staticmethod
    async def verify_session(db: AsyncSession, token: str) -> Optional[User]:
        """
        Verify session token and return associated user.

        Args:
            db: Database session
            token: Session token to verify

        Returns:
            User object if session is valid and not expired, None otherwise

        Example:
            >>> user = await AuthService.verify_session(db, 'abc123...')
            >>> user.email if user else None
            'student@example.com'
        """
        # Query for session with valid token and not expired
        stmt = select(Session).filter_by(token=token).filter(
            Session.expires_at > datetime.now(timezone.utc)
        )
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if not session:
            return None

        # Load associated user
        user_stmt = select(User).filter_by(id=session.user_id)
        user_result = await db.execute(user_stmt)
        return user_result.scalar_one_or_none()

    @staticmethod
    async def create_session(db: AsyncSession, user_id: str) -> Session:
        """
        Create new authentication session for user.

        Args:
            db: Database session
            user_id: User UUID

        Returns:
            Created Session object with token and expiry

        Example:
            >>> session = await AuthService.create_session(db, user.id)
            >>> len(session.token)
            43
        """
        session = Session(
            user_id=user_id,
            token=AuthService.generate_session_token(),
            expires_at=AuthService.get_expiry_timestamp()
        )
        db.add(session)
        await db.flush()  # Get session.id without committing
        return session

    @staticmethod
    async def invalidate_session(db: AsyncSession, token: str) -> bool:
        """
        Invalidate (delete) session by token.

        Args:
            db: Database session
            token: Session token to invalidate

        Returns:
            True if session was deleted, False if not found

        Example:
            >>> await AuthService.invalidate_session(db, 'abc123...')
            True
        """
        stmt = select(Session).filter_by(token=token)
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if session:
            await db.delete(session)
            await db.flush()
            return True
        return False

    @staticmethod
    async def validate_email_unique(db: AsyncSession, email: str) -> bool:
        """
        Check if email is not already registered.

        Args:
            db: Database session
            email: Email to check

        Returns:
            True if email is available, False if already exists

        Example:
            >>> await AuthService.validate_email_unique(db, 'new@example.com')
            True
        """
        stmt = select(User).filter_by(email=email)
        result = await db.execute(stmt)
        user = result.scalar_one_or_none()
        return user is None
