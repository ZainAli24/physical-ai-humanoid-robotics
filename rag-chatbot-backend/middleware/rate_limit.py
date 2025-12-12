"""
In-memory rate limiting middleware for authentication endpoints.
Prevents brute-force attacks on signup/signin.
"""

from fastapi import Request, HTTPException, status
from collections import defaultdict
from datetime import datetime, timedelta, timezone
from typing import Dict, Tuple
import os


class RateLimiter:
    """
    Simple in-memory rate limiter using sliding window.

    Tracks failed authentication attempts by IP address.
    """

    def __init__(
        self,
        max_attempts: int = 5,
        window_minutes: int = 15,
        enabled: bool = True
    ):
        """
        Initialize rate limiter.

        Args:
            max_attempts: Maximum failed attempts per window
            window_minutes: Time window in minutes
            enabled: Whether rate limiting is active
        """
        self.max_attempts = max_attempts
        self.window_minutes = window_minutes
        self.enabled = enabled

        # Track failed attempts: {ip_address: [(timestamp1, timestamp2, ...)]}
        self._attempts: Dict[str, list[datetime]] = defaultdict(list)

    def _get_client_ip(self, request: Request) -> str:
        """
        Extract client IP address from request.

        Checks X-Forwarded-For header first (for proxies),
        then falls back to client.host.

        Args:
            request: FastAPI request object

        Returns:
            Client IP address
        """
        # Check X-Forwarded-For header (set by proxies/load balancers)
        forwarded = request.headers.get('X-Forwarded-For')
        if forwarded:
            # Get first IP in chain (original client)
            return forwarded.split(',')[0].strip()

        # Fallback to direct connection IP
        return request.client.host if request.client else 'unknown'

    def _clean_old_attempts(self, ip: str) -> None:
        """
        Remove attempts outside the current time window.

        Args:
            ip: IP address to clean
        """
        if ip not in self._attempts:
            return

        cutoff = datetime.now(timezone.utc) - timedelta(minutes=self.window_minutes)
        self._attempts[ip] = [
            timestamp for timestamp in self._attempts[ip]
            if timestamp > cutoff
        ]

        # Remove IP if no recent attempts
        if not self._attempts[ip]:
            del self._attempts[ip]

    def record_failed_attempt(self, request: Request) -> None:
        """
        Record failed authentication attempt for IP.

        Args:
            request: FastAPI request object
        """
        if not self.enabled:
            return

        ip = self._get_client_ip(request)
        self._attempts[ip].append(datetime.now(timezone.utc))

    def check_rate_limit(self, request: Request) -> Tuple[bool, int]:
        """
        Check if IP has exceeded rate limit.

        Args:
            request: FastAPI request object

        Returns:
            Tuple of (is_allowed, remaining_attempts)

        Example:
            >>> allowed, remaining = limiter.check_rate_limit(request)
            >>> if not allowed:
            ...     raise HTTPException(429, "Too many attempts")
        """
        if not self.enabled:
            return True, self.max_attempts

        ip = self._get_client_ip(request)

        # Clean old attempts before checking
        self._clean_old_attempts(ip)

        # Count recent attempts
        recent_attempts = len(self._attempts[ip])

        # Check if over limit
        is_allowed = recent_attempts < self.max_attempts
        remaining = max(0, self.max_attempts - recent_attempts)

        return is_allowed, remaining

    def reset_ip(self, request: Request) -> None:
        """
        Reset rate limit for IP (call after successful auth).

        Args:
            request: FastAPI request object
        """
        if not self.enabled:
            return

        ip = self._get_client_ip(request)
        if ip in self._attempts:
            del self._attempts[ip]

    def clear_all(self) -> None:
        """Clear all rate limit data (for testing)."""
        self._attempts.clear()


# Global rate limiter instance (initialized from environment variables)
_rate_limiter: RateLimiter = None


def get_rate_limiter() -> RateLimiter:
    """
    Get or create global rate limiter instance.

    Reads configuration from environment variables:
    - RATE_LIMIT_ENABLED: "true" or "false" (default: true)
    - RATE_LIMIT_MAX_ATTEMPTS: integer (default: 5)
    - RATE_LIMIT_WINDOW_MINUTES: integer (default: 15)

    Returns:
        RateLimiter instance
    """
    global _rate_limiter

    if _rate_limiter is None:
        enabled = os.getenv('RATE_LIMIT_ENABLED', 'true').lower() == 'true'
        max_attempts = int(os.getenv('RATE_LIMIT_MAX_ATTEMPTS', '5'))
        window_minutes = int(os.getenv('RATE_LIMIT_WINDOW_MINUTES', '15'))

        _rate_limiter = RateLimiter(
            max_attempts=max_attempts,
            window_minutes=window_minutes,
            enabled=enabled
        )

    return _rate_limiter


async def check_rate_limit_middleware(request: Request) -> None:
    """
    FastAPI dependency for rate limit checking.

    Raises HTTPException 429 if rate limit exceeded.

    Usage:
        @app.post("/auth/signin", dependencies=[Depends(check_rate_limit_middleware)])
        async def signin(...):
            ...

    Args:
        request: FastAPI request object

    Raises:
        HTTPException: 429 Too Many Requests if limit exceeded
    """
    limiter = get_rate_limiter()
    is_allowed, remaining = limiter.check_rate_limit(request)

    if not is_allowed:
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail=f"Too many failed attempts. Try again in {limiter.window_minutes} minutes.",
            headers={"Retry-After": str(limiter.window_minutes * 60)}
        )
