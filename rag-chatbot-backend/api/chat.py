"""
Chat API Endpoint
Feature: 004-rag-chat

POST /api/chat endpoint for RAG chatbot with SSE streaming support.
"""

import os
import json
import uuid
import re
import sys
import logging
from typing import AsyncIterator, Optional
from fastapi import APIRouter, HTTPException, status, Depends
from fastapi.responses import StreamingResponse
from sqlalchemy.ext.asyncio import AsyncSession
from agents import Runner, InputGuardrailTripwireTriggered
from dotenv import load_dotenv

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import models and services
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from models.schemas import ChatRequest, ChatResponse, SourceCitation, ErrorResponse
from custom_agents.rag_agent import create_rag_agent
# Note: retrieve_context is a FunctionTool and cannot be called directly
from database.connection import get_db
from database.models import User
from middleware.auth_middleware import get_current_user_optional
from services.history_service import HistoryService

# Load environment variables
load_dotenv()

# Create router
router = APIRouter(prefix="/api", tags=["chat"])


def extract_sources_from_context(context: str) -> list[SourceCitation]:
    """
    Extract source citations from retrieve_context tool output.

    Args:
        context: Context string from retrieve_context tool

    Returns:
        List of SourceCitation objects

    Example context format:
        [Source 1] Introduction to ROS 2:
        ROS 2 is a framework...

        ---
        **Sources:**
        1. Introduction to ROS 2 (relevance: 0.92)
           URL: /docs/module-1/chapter-1
    """
    sources = []

    # Find Sources section
    sources_section_match = re.search(r'\*\*Sources:\*\*\n(.+)', context, re.DOTALL)
    if not sources_section_match:
        return sources

    sources_text = sources_section_match.group(1)

    # Parse each source entry
    # Pattern: 1. Title - Heading (relevance: 0.92)\n   URL: /path
    source_pattern = r'(\d+)\.\s+(.+?)\s+(?:-\s+(.+?))?\s*\(relevance:\s+([\d.]+)\)\s+URL:\s+(.+)'

    for match in re.finditer(source_pattern, sources_text):
        index, title, heading, score, url = match.groups()

        # Extract excerpt from context (use first 150 chars of source text)
        excerpt_pattern = rf'\[Source {index}\].+?:\n(.+?)(?:\n\n|\Z)'
        excerpt_match = re.search(excerpt_pattern, context, re.DOTALL)
        excerpt = excerpt_match.group(1).strip()[:150] + "..." if excerpt_match else ""

        source = SourceCitation(
            title=title.strip(),
            heading=heading.strip() if heading else None,
            url=url.strip(),
            score=float(score),
            excerpt=excerpt
        )
        sources.append(source)

    return sources


async def stream_chat_response(
    message: str,
    session_id: str,
    selected_text: str | None = None,
    user: Optional[User] = None,
    db: Optional[AsyncSession] = None
) -> AsyncIterator[str]:
    """
    Stream chat response using Server-Sent Events (SSE).

    Yields SSE events in format:
        event: content
        data: {"event": "content", "data": "token"}

        event: sources
        data: {"event": "sources", "data": [...]}

        event: done
        data: {"event": "done", "data": null}

    Args:
        message: User's question
        session_id: Session ID for conversation (chat session UUID for authenticated users)
        selected_text: Optional user-selected text from textbook (Feature 005)
        user: Optional authenticated user
        db: Optional database session (required if user is authenticated)

    Yields:
        SSE-formatted event strings
    """
    try:
        # Auto-create chat session if authenticated and no session_id provided (T040)
        chat_session_id = session_id
        if user and db:
            if not session_id or session_id == "null":
                # Create new chat session with temporary title
                chat_session = await HistoryService.create_session(db, user.id, "New Chat")
                chat_session_id = chat_session.id
                await db.commit()
            else:
                # Verify session exists and belongs to user
                chat_session = await HistoryService.get_session(db, session_id, user.id)
                if not chat_session:
                    raise HTTPException(
                        status_code=status.HTTP_404_NOT_FOUND,
                        detail="Chat session not found"
                    )
                chat_session_id = session_id

            # Save user message before processing (T042)
            await HistoryService.save_message(
                db=db,
                session_id=chat_session_id,
                role="user",
                content=message,
                sources=None  # Explicitly None for user messages
            )
            await db.commit()

        # Create agent
        logger.info("=" * 60)
        logger.info("CREATING RAG AGENT")
        logger.info("=" * 60)
        agent = create_rag_agent()
        logger.info("[OK] Agent created successfully")
        sys.stdout.flush()

        # Prepare agent input with selected text if provided (Feature 005 - Better Approach)
        agent_input = message
        if selected_text:
            logger.info(f"[DEBUG] Selected text received: {selected_text[:200]}...")
            # Include selected text directly in agent input for context
            agent_input = f"""User's question: {message}

[SELECTED TEXT FROM TEXTBOOK]
{selected_text}

Please answer the user's question based on this selected text and relevant context from the textbook."""
            logger.info(f"[DEBUG] Agent input with selected text included directly")
        else:
            logger.info(f"[DEBUG] No selected text - standard query: {message[:100]}...")

        logger.info(f"Starting agent execution...")
        sys.stdout.flush()

        # Run agent
        logger.info("Calling Runner.run()...")
        sys.stdout.flush()

        result = await Runner.run(
            starting_agent=agent,
            input=agent_input
        )

        logger.info(f"[OK] Agent execution completed. Final output length: {len(result.final_output) if result.final_output else 0}")
        sys.stdout.flush()

        # [DEBUG] Log all tool calls made by the agent
        logger.info(f"[DEBUG] Number of items in result.new_items: {len(result.new_items)}")
        for i, item in enumerate(result.new_items):
            if hasattr(item, 'tool_name'):
                logger.info(f"[DEBUG] Tool call {i+1}: {item.tool_name}")
                if hasattr(item, 'input'):
                    logger.info(f"[DEBUG] Tool input {i+1}: {item.input}")

        # Get final output
        final_output = result.final_output or "No response generated."
        logger.info(f"Final output to stream: {final_output[:100]}...")
        sys.stdout.flush()

        # Stream answer token by token (simulate streaming for now)
        # In production, you'd use Agent's streaming capabilities
        words = final_output.split()
        logger.info(f"Streaming {len(words)} words...")
        sys.stdout.flush()

        for i, word in enumerate(words):
            token = word if i == 0 else f" {word}"
            event_data = json.dumps({"event": "content", "data": token})
            yield f"event: content\ndata: {event_data}\n\n"

        logger.info(f"[OK] Finished streaming content")
        sys.stdout.flush()

        # Extract sources from agent tool calls
        logger.info("Extracting sources from result.new_items...")
        sys.stdout.flush()

        sources = []
        for item in result.new_items:
            if hasattr(item, 'tool_name') and item.tool_name == 'retrieve_context':
                if hasattr(item, 'output'):
                    sources = extract_sources_from_context(item.output)
                    break

        logger.info(f"Extracted {len(sources)} sources")
        sys.stdout.flush()

        # Send sources event
        if sources:
            sources_data = [source.model_dump() for source in sources]
            event_data = json.dumps({"event": "sources", "data": sources_data})
            yield f"event: sources\ndata: {event_data}\n\n"
            logger.info("[OK] Sent sources event")
            sys.stdout.flush()

        # Save assistant message after processing (T042)
        if user and db:
            # Convert sources to JSONB format
            sources_jsonb = [source.model_dump() for source in sources] if sources else None

            await HistoryService.save_message(
                db=db,
                session_id=chat_session_id,
                role="assistant",
                content=final_output,
                sources=sources_jsonb
            )

            # Generate title from first user message (T041)
            # Check if this is the first exchange (2 messages: user + assistant)
            messages = await HistoryService.get_session_messages(db, chat_session_id)
            if len(messages) == 2:  # First exchange complete
                title = HistoryService.generate_title(message)
                await HistoryService.update_session_title(db, chat_session_id, title)

            await db.commit()
            # T043: updated_at is automatically updated by SQLAlchemy onupdate

        # Send done event with session_id (T045, T046)
        logger.info(f"Sending done event with session_id: {chat_session_id}")
        sys.stdout.flush()

        event_data = json.dumps({"event": "done", "data": {"session_id": chat_session_id}})
        yield f"event: done\ndata: {event_data}\n\n"

        logger.info("[OK] SSE streaming complete!")
        sys.stdout.flush()

    except InputGuardrailTripwireTriggered as e:
        # Guardrail triggered - question is off-topic
        # Return the helpful refusal message
        refusal_message = "I can only answer questions about topics covered in the Physical AI & Humanoid Robotics textbook, such as ROS 2, Gazebo, NVIDIA Isaac Sim, and VLA models. Please ask a question related to these topics."

        event_data = json.dumps({"event": "content", "data": refusal_message})
        yield f"event: content\ndata: {event_data}\n\n"

        # Send done event with session_id (even for guardrail errors)
        event_data = json.dumps({"event": "done", "data": {"session_id": chat_session_id}})
        yield f"event: done\ndata: {event_data}\n\n"

    except Exception as e:
        # Send error event
        error_data = json.dumps({"event": "error", "data": str(e)})
        yield f"event: error\ndata: {error_data}\n\n"


@router.post(
    "/chat",
    response_model=ChatResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Chat with RAG assistant",
    description="Send a question to the RAG chatbot and receive an answer with source citations. Supports SSE streaming."
)
async def chat(
    request: ChatRequest,
    user: Optional[User] = Depends(get_current_user_optional),
    db: AsyncSession = Depends(get_db)
):
    """
    Chat endpoint with SSE streaming support.

    Supports both authenticated and anonymous users:
    - Authenticated: Saves messages to database with chat history
    - Anonymous: Returns responses without persistence

    **Streaming Mode** (stream=true):
    - Returns Server-Sent Events (SSE) with incremental response
    - Events: "content" (tokens), "sources" (citations), "done" (end), "error"

    **Non-Streaming Mode** (stream=false):
    - Returns complete ChatResponse JSON

    **Example Request**:
    ```json
    {
        "message": "What is ROS 2?",
        "session_id": "user-123",
        "stream": true
    }
    ```

    **Example SSE Response**:
    ```
    event: content
    data: {"event": "content", "data": "ROS"}

    event: content
    data: {"event": "content", "data": " 2"}

    event: sources
    data: {"event": "sources", "data": [...]}

    event: done
    data: {"event": "done", "data": null}
    ```
    """
    # Validate message
    if not request.message.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Message cannot be empty"
        )

    if len(request.message) > 500:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Message exceeds 500 character limit"
        )

    # Generate session ID for anonymous users only
    # Authenticated users: session_id stays None to trigger auto-create logic (T040)
    if not user:
        session_id = request.session_id or str(uuid.uuid4())
    else:
        session_id = request.session_id

    # Return streaming or non-streaming response
    if request.stream:
        return StreamingResponse(
            stream_chat_response(request.message, session_id, request.selected_text, user, db),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no"  # Disable proxy buffering
            }
        )
    else:
        # Non-streaming mode
        try:
            # Auto-create chat session if authenticated and no session_id provided (T040)
            chat_session_id = session_id
            if user and db:
                if not session_id or session_id == "null":
                    # Create new chat session with temporary title
                    chat_session = await HistoryService.create_session(db, user.id, "New Chat")
                    chat_session_id = chat_session.id
                    await db.commit()
                else:
                    # Verify session exists and belongs to user
                    chat_session = await HistoryService.get_session(db, session_id, user.id)
                    if not chat_session:
                        raise HTTPException(
                            status_code=status.HTTP_404_NOT_FOUND,
                            detail="Chat session not found"
                        )
                    chat_session_id = session_id

                # Save user message before processing (T042)
                await HistoryService.save_message(
                    db=db,
                    session_id=chat_session_id,
                    role="user",
                    content=request.message,
                    sources=None  # Explicitly None for user messages
                )
                await db.commit()

            logger.info("=" * 60)
            logger.info("NON-STREAMING MODE: Creating RAG agent")
            logger.info("=" * 60)
            agent = create_rag_agent()
            logger.info("[OK] Agent created successfully")
            sys.stdout.flush()

            # Prepare agent input with selected text if provided (Feature 005 - Better Approach)
            agent_input = request.message
            if request.selected_text:
                logger.info(f"[DEBUG] Selected text received: {request.selected_text[:200]}...")
                # Include selected text directly in agent input for context
                agent_input = f"""User's question: {request.message}

[SELECTED TEXT FROM TEXTBOOK]
{request.selected_text}

Please answer the user's question based on this selected text and relevant context from the textbook."""
                logger.info(f"[DEBUG] Agent input with selected text included directly")
            else:
                logger.info(f"[DEBUG] No selected text - standard query: {request.message[:100]}...")

            logger.info(f"Starting agent execution...")
            logger.info("Calling Runner.run()...")
            sys.stdout.flush()

            result = await Runner.run(
                starting_agent=agent,
                input=agent_input
            )

            logger.info(f"[OK] Agent execution completed. Final output length: {len(result.final_output) if result.final_output else 0}")
            sys.stdout.flush()

            final_output = result.final_output or "No response generated."

            # Extract sources from agent tool calls
            sources = []
            for item in result.new_items:
                if hasattr(item, 'tool_name') and item.tool_name == 'retrieve_context':
                    if hasattr(item, 'output'):
                        sources = extract_sources_from_context(item.output)
                        break
            logger.info(f"Extracted {len(sources)} sources")
            sys.stdout.flush()

            # Save assistant message after processing (T042)
            if user and db:
                # Convert sources to JSONB format
                sources_jsonb = [source.model_dump() for source in sources] if sources else None

                await HistoryService.save_message(
                    db=db,
                    session_id=chat_session_id,
                    role="assistant",
                    content=final_output,
                    sources=sources_jsonb
                )

                # Generate title from first user message (T041)
                # Check if this is the first exchange (2 messages: user + assistant)
                messages = await HistoryService.get_session_messages(db, chat_session_id)
                if len(messages) == 2:  # First exchange complete
                    title = HistoryService.generate_title(request.message)
                    await HistoryService.update_session_title(db, chat_session_id, title)

                await db.commit()
                # T043: updated_at is automatically updated by SQLAlchemy onupdate

            return ChatResponse(
                answer=final_output,
                sources=sources,
                session_id=chat_session_id
            )

        except InputGuardrailTripwireTriggered as e:
            # Guardrail triggered - return refusal message
            refusal_message = "I can only answer questions about topics covered in the Physical AI & Humanoid Robotics textbook, such as ROS 2, Gazebo, NVIDIA Isaac Sim, and VLA models. Please ask a question related to these topics."

            return ChatResponse(
                answer=refusal_message,
                sources=[],
                session_id=session_id
            )

        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Failed to generate response: {str(e)}"
            )


if __name__ == "__main__":
    # Test source extraction
    test_context = """[Source 1] Introduction to ROS 2 - Overview:
ROS 2 is a set of software libraries and tools for building robot applications.

---
**Sources:**
1. Introduction to ROS 2 - Overview (relevance: 0.92)
   URL: /docs/module-1/chapter-1
"""

    sources = extract_sources_from_context(test_context)
    print(f"✅ Extracted {len(sources)} sources:")
    for source in sources:
        print(f"   - {source.title}: {source.score}")
