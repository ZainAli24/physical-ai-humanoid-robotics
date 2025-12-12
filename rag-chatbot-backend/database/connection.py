"""
Database connection pool and session management.
"""

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.pool import NullPool
import os
from typing import AsyncGenerator

# Get DATABASE_URL from environment
DATABASE_URL = os.getenv('DATABASE_URL')

if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable is required")

# Convert postgres:// to postgresql+asyncpg:// for async support
if DATABASE_URL.startswith('postgres://'):
    DATABASE_URL = DATABASE_URL.replace('postgres://', 'postgresql+asyncpg://', 1)
elif DATABASE_URL.startswith('postgresql://'):
    DATABASE_URL = DATABASE_URL.replace('postgresql://', 'postgresql+asyncpg://', 1)

# Remove sslmode query parameter (asyncpg doesn't support it, uses ssl= parameter instead)
# Neon uses sslmode=require which we'll handle with connect_args
import re
DATABASE_URL = re.sub(r'[?&]sslmode=[^&]*', '', DATABASE_URL)
DATABASE_URL = re.sub(r'[?&]channel_binding=[^&]*', '', DATABASE_URL)

# Create async engine with connection pool
engine = create_async_engine(
    DATABASE_URL,
    echo=False,  # Set to True for SQL query logging during development
    pool_size=10,  # Max connections in pool
    max_overflow=20,  # Max additional connections beyond pool_size
    pool_pre_ping=True,  # Verify connections before using (handles stale connections)
    pool_recycle=3600,  # Recycle connections after 1 hour (Neon serverless timeout)
    poolclass=NullPool if 'TESTING' in os.environ else None,  # Disable pooling for tests
    connect_args={
        "ssl": "require",  # Enable SSL for Neon (asyncpg format)
        "server_settings": {
            "application_name": "rag_chatbot_backend"
        }
    }
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,  # Prevent lazy loading after commit
    autoflush=False,  # Explicit control over when to flush
    autocommit=False,  # Always use transactions
)

async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """
    FastAPI dependency for database sessions.

    Usage:
        @app.get("/users")
        async def get_users(db: AsyncSession = Depends(get_db)):
            result = await db.execute(select(User))
            return result.scalars().all()
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()

async def close_db():
    """Close database engine (call on application shutdown)."""
    await engine.dispose()
