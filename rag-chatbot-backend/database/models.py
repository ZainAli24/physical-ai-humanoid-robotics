"""
SQLAlchemy models for authentication and chat history.
"""

from sqlalchemy import Column, String, Text, ForeignKey, CheckConstraint, Index
from sqlalchemy.dialects.postgresql import UUID, TIMESTAMP, JSONB
from sqlalchemy.orm import relationship, declarative_base
from sqlalchemy.sql import func
from sqlalchemy.types import TypeDecorator
import uuid
import json

Base = declarative_base()

class NullableJSONB(TypeDecorator):
    """
    Custom JSONB type that treats Python None as SQL NULL.
    By default, asyncpg converts None to JSON 'null' for JSONB columns,
    but we want SQL NULL for user messages (no sources).
    """
    impl = JSONB
    cache_ok = True

    def process_bind_param(self, value, dialect):
        """Convert Python value to database value."""
        if value is None:
            return None  # SQL NULL, not JSON null
        return value  # Let JSONB handle the JSON encoding

    def process_result_value(self, value, dialect):
        """Convert database value to Python value."""
        return value  # Already decoded by asyncpg

def generate_uuid():
    """Generate UUID v4 as string for primary keys."""
    return str(uuid.uuid4())

class User(Base):
    """User account for authentication."""
    __tablename__ = 'users'

    id = Column(UUID(as_uuid=False), primary_key=True, default=generate_uuid)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    name = Column(String(255), nullable=True)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now(), onupdate=func.now())

    # Relationships (CASCADE delete)
    sessions = relationship('Session', back_populates='user', cascade='all, delete-orphan')
    chat_sessions = relationship('ChatSession', back_populates='user', cascade='all, delete-orphan')

    __table_args__ = (
        CheckConstraint("email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Za-z]{2,}'", name='valid_email'),
        CheckConstraint("LENGTH(password_hash) >= 60", name='valid_password_hash'),
    )

class Session(Base):
    """Authentication session (database-backed tokens)."""
    __tablename__ = 'sessions'

    id = Column(UUID(as_uuid=False), primary_key=True, default=generate_uuid)
    user_id = Column(UUID(as_uuid=False), ForeignKey('users.id', ondelete='CASCADE'), nullable=False, index=True)
    token = Column(String(255), unique=True, nullable=False, index=True)
    expires_at = Column(TIMESTAMP(timezone=True), nullable=False, index=True)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())

    user = relationship('User', back_populates='sessions')

    __table_args__ = (
        CheckConstraint('expires_at > created_at', name='valid_expiry'),
        CheckConstraint("LENGTH(token) = 43", name='valid_token_length'),
    )

    @staticmethod
    def generate_token() -> str:
        """Generate cryptographically secure session token (32 bytes = 43 chars base64)."""
        import secrets
        return secrets.token_urlsafe(32)

class ChatSession(Base):
    """Chat conversation thread."""
    __tablename__ = 'chat_sessions'

    id = Column(UUID(as_uuid=False), primary_key=True, default=generate_uuid)
    user_id = Column(UUID(as_uuid=False), ForeignKey('users.id', ondelete='CASCADE'), nullable=False, index=True)
    title = Column(String(100), nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now(), onupdate=func.now())

    user = relationship('User', back_populates='chat_sessions')
    messages = relationship('Message', back_populates='chat_session', cascade='all, delete-orphan', order_by='Message.created_at')

    __table_args__ = (
        CheckConstraint("LENGTH(title) > 0 AND LENGTH(title) <= 100", name='valid_title_length'),
        CheckConstraint('updated_at >= created_at', name='valid_updated_at'),
        Index('idx_chat_sessions_updated_at_desc', updated_at.desc()),
    )

    @staticmethod
    def generate_title(first_message: str, max_length: int = 60) -> str:
        """Generate session title from first user message. Truncates at word boundary."""
        if len(first_message) <= max_length:
            return first_message
        truncated = first_message[:max_length]
        last_space = truncated.rfind(' ')
        if last_space > int(max_length * 0.7):
            return truncated[:last_space] + '...'
        return truncated + '...'

class Message(Base):
    """Individual chat message (user question or assistant response)."""
    __tablename__ = 'messages'

    id = Column(UUID(as_uuid=False), primary_key=True, default=generate_uuid)
    session_id = Column(UUID(as_uuid=False), ForeignKey('chat_sessions.id', ondelete='CASCADE'), nullable=False, index=True)
    role = Column(String(20), nullable=False, index=True)
    content = Column(Text, nullable=False)
    sources = Column(NullableJSONB, nullable=True)  # null for user messages, array for assistant
    agent_trace_id = Column(String(255), nullable=True)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now(), index=True)

    chat_session = relationship('ChatSession', back_populates='messages')

    __table_args__ = (
        CheckConstraint("role IN ('user', 'assistant')", name='valid_role'),
        CheckConstraint("LENGTH(content) > 0", name='non_empty_content'),
        CheckConstraint("role = 'assistant' OR sources IS NULL", name='sources_only_for_assistant'),
    )
