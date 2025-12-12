"""
Chat history service for managing chat sessions and messages.
"""

from sqlalchemy import select, func
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional, List, Dict, Any
from datetime import datetime, timezone

from database.models import ChatSession, Message


class HistoryService:
    """Service for chat history operations."""

    @staticmethod
    def generate_title(first_message: str, max_length: int = 60) -> str:
        """
        Generate session title from first user message.
        Truncates at word boundary if longer than max_length.

        Args:
            first_message: User's first message
            max_length: Maximum title length (default 60)

        Returns:
            Generated title

        Example:
            >>> HistoryService.generate_title("What is ROS 2?")
            "What is ROS 2?"
            >>> HistoryService.generate_title("This is a very long message that exceeds the maximum length and needs truncation")
            "This is a very long message that exceeds the maximum..."
        """
        if len(first_message) <= max_length:
            return first_message

        # Truncate at max_length
        truncated = first_message[:max_length]

        # Find last space within 70% of max_length to avoid cutting mid-word
        last_space = truncated.rfind(' ')
        threshold = int(max_length * 0.7)

        if last_space > threshold:
            return truncated[:last_space] + '...'

        return truncated + '...'

    @staticmethod
    async def create_session(
        db: AsyncSession,
        user_id: str,
        title: Optional[str] = None
    ) -> ChatSession:
        """
        Create new chat session for user.

        T088: Enforces 100-session limit - deletes oldest session if limit exceeded.

        Args:
            db: Database session
            user_id: User UUID
            title: Optional session title (default "New Chat")

        Returns:
            Created ChatSession

        Example:
            >>> session = await HistoryService.create_session(db, user.id)
            >>> session.title
            "New Chat"
        """
        # T088: Check session count and enforce 100-session limit
        count_stmt = select(func.count()).select_from(ChatSession).filter_by(user_id=user_id)
        count_result = await db.execute(count_stmt)
        session_count = count_result.scalar() or 0

        # If user has 100 or more sessions, delete the oldest one
        if session_count >= 100:
            # Find oldest session (by created_at)
            oldest_stmt = (
                select(ChatSession)
                .filter_by(user_id=user_id)
                .order_by(ChatSession.created_at.asc())
                .limit(1)
            )
            oldest_result = await db.execute(oldest_stmt)
            oldest_session = oldest_result.scalar_one_or_none()

            if oldest_session:
                # Delete oldest session (CASCADE will delete messages)
                await db.delete(oldest_session)
                await db.flush()  # Ensure delete completes before creating new session

        # Create new session
        session = ChatSession(
            user_id=user_id,
            title=title or "New Chat"
        )
        db.add(session)
        await db.flush()  # Get session.id without committing
        return session

    @staticmethod
    async def save_message(
        db: AsyncSession,
        session_id: str,
        role: str,
        content: str,
        sources: Optional[List[Dict[str, Any]]] = None,
        agent_trace_id: Optional[str] = None
    ) -> Message:
        """
        Save message to chat session.

        Args:
            db: Database session
            session_id: Chat session UUID
            role: "user" or "assistant"
            content: Message text
            sources: Optional RAG sources (for assistant messages)
            agent_trace_id: Optional trace ID for debugging

        Returns:
            Created Message

        Example:
            >>> message = await HistoryService.save_message(
            ...     db, session.id, "user", "What is ROS 2?"
            ... )
        """
        # Debug logging
        import logging
        logger = logging.getLogger(__name__)
        logger.info(f"[DEBUG] save_message called with role={role}, sources={sources}, type(sources)={type(sources)}")

        message = Message(
            session_id=session_id,
            role=role,
            content=content,
            sources=sources,
            agent_trace_id=agent_trace_id
        )

        # Debug: Check what Message object has
        logger.info(f"[DEBUG] Message object created: sources={message.sources}, type={type(message.sources)}")

        db.add(message)
        await db.flush()
        return message

    @staticmethod
    async def update_session_title(
        db: AsyncSession,
        session_id: str,
        title: str
    ) -> Optional[ChatSession]:
        """
        Update session title (for rename or auto-generation).

        Args:
            db: Database session
            session_id: Chat session UUID
            title: New title

        Returns:
            Updated ChatSession or None if not found
        """
        stmt = select(ChatSession).filter_by(id=session_id)
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if session:
            session.title = title
            await db.flush()

        return session

    @staticmethod
    async def get_session(
        db: AsyncSession,
        session_id: str,
        user_id: str
    ) -> Optional[ChatSession]:
        """
        Get chat session if it belongs to user.

        Args:
            db: Database session
            session_id: Chat session UUID
            user_id: User UUID (for ownership verification)

        Returns:
            ChatSession or None if not found/not owned
        """
        stmt = select(ChatSession).filter_by(
            id=session_id,
            user_id=user_id
        )
        result = await db.execute(stmt)
        return result.scalar_one_or_none()

    @staticmethod
    async def get_user_sessions(
        db: AsyncSession,
        user_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> List[ChatSession]:
        """
        Get user's chat sessions (most recent first).

        Args:
            db: Database session
            user_id: User UUID
            limit: Maximum sessions to return (default 50)
            offset: Number of sessions to skip (default 0)

        Returns:
            List of ChatSession objects
        """
        stmt = (
            select(ChatSession)
            .filter_by(user_id=user_id)
            .order_by(ChatSession.updated_at.desc())
            .limit(limit)
            .offset(offset)
        )
        result = await db.execute(stmt)
        return list(result.scalars().all())

    @staticmethod
    async def get_session_messages(
        db: AsyncSession,
        session_id: str
    ) -> List[Message]:
        """
        Get all messages for session (chronological order).

        Args:
            db: Database session
            session_id: Chat session UUID

        Returns:
            List of Message objects
        """
        stmt = (
            select(Message)
            .filter_by(session_id=session_id)
            .order_by(Message.created_at.asc())
        )
        result = await db.execute(stmt)
        return list(result.scalars().all())

    @staticmethod
    async def list_user_sessions(
        db: AsyncSession,
        user_id: str,
        limit: int = 50,
        offset: int = 0
    ) -> List[Dict[str, Any]]:
        """
        Get user's chat sessions with message counts (T052).
        Returns dict with session data + message_count.

        Args:
            db: Database session
            user_id: User UUID
            limit: Max sessions to return (default 50)
            offset: Number of sessions to skip (default 0)

        Returns:
            List of dicts with keys: id, title, created_at, updated_at, message_count
        """
        # Query sessions with message counts using LEFT JOIN
        stmt = (
            select(
                ChatSession.id,
                ChatSession.title,
                ChatSession.created_at,
                ChatSession.updated_at,
                func.count(Message.id).label('message_count')
            )
            .outerjoin(Message, ChatSession.id == Message.session_id)
            .filter(ChatSession.user_id == user_id)
            .group_by(ChatSession.id)
            .order_by(ChatSession.updated_at.desc())
            .limit(limit)
            .offset(offset)
        )

        result = await db.execute(stmt)
        rows = result.all()

        # Convert to list of dicts
        return [
            {
                'id': row.id,
                'title': row.title,
                'created_at': row.created_at,
                'updated_at': row.updated_at,
                'message_count': row.message_count
            }
            for row in rows
        ]

    @staticmethod
    async def delete_session(
        db: AsyncSession,
        session_id: str,
        user_id: str
    ) -> bool:
        """
        Delete chat session if owned by user.

        Args:
            db: Database session
            session_id: Chat session UUID
            user_id: User UUID (for ownership verification)

        Returns:
            True if deleted, False if not found/not owned
        """
        session = await HistoryService.get_session(db, session_id, user_id)

        if session:
            await db.delete(session)
            await db.flush()
            return True

        return False
