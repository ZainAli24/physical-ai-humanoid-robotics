"""
RAG Chatbot Backend - FastAPI Application
Feature: 004-rag-chat

Main entry point for the FastAPI backend server.
"""

import os
from fastapi import FastAPI, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv

# Import routers
from api.chat import router as chat_router
from api.embed import router as embed_router
from api.auth.signup import router as signup_router
from api.auth.signin import router as signin_router
from api.auth.signout import router as signout_router
from api.auth.session import router as session_router
from api.history.sessions import router as sessions_router  # T053
from api.history.messages import router as messages_router  # T053
from api.user.delete import router as delete_user_router  # T074

# Import models and services
from models.schemas import HealthResponse
from services.qdrant_client import get_qdrant_client, get_collection_info

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="RAG-powered chatbot for Physical AI & Humanoid Robotics textbook using OpenAI Agents SDK with Gemini 2.5 Flash",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Configure CORS
cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"]
)

# Include routers
app.include_router(chat_router)
app.include_router(embed_router)

# Authentication routers (Feature 006)
app.include_router(signup_router, prefix="/api/auth", tags=["Authentication"])
app.include_router(signin_router, prefix="/api/auth", tags=["Authentication"])
app.include_router(signout_router, prefix="/api/auth", tags=["Authentication"])
app.include_router(session_router, prefix="/api/auth", tags=["Authentication"])

# History routers (Feature 006 - User Story 3: T053)
app.include_router(sessions_router, tags=["History"])
app.include_router(messages_router, tags=["History"])

# User data deletion router (Feature 006 - User Story 5: T074)
app.include_router(delete_user_router, tags=["User"])


# ============================================================================
# Health Check Endpoint
# ============================================================================

@app.get(
    "/",
    response_model=HealthResponse,
    summary="Health check",
    description="Check API health status and Qdrant connection"
)
async def health_check():
    """
    Health check endpoint.

    Returns API status, version, model info, and Qdrant connection status.

    **Example Response**:
    ```json
    {
        "status": "healthy",
        "version": "1.0.0",
        "model": "gemini-2.5-flash",
        "qdrant_connected": true,
        "collection_points": 187
    }
    ```
    """
    # Get configuration
    model = os.getenv("CHAT_MODEL", "gemini-2.5-flash")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")

    # Check Qdrant connection
    qdrant_connected = False
    collection_points = None

    try:
        client = get_qdrant_client()
        info = get_collection_info(collection_name, client=client)
        qdrant_connected = True
        collection_points = info.get("points_count", 0)
    except Exception as e:
        print(f"⚠️  Qdrant health check failed: {e}")

    # Determine overall status
    service_status = "healthy" if qdrant_connected else "degraded"

    return HealthResponse(
        status=service_status,
        version="1.0.0",
        model=model,
        qdrant_connected=qdrant_connected,
        collection_points=collection_points
    )


# ============================================================================
# Exception Handlers
# ============================================================================

@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """
    Global exception handler for unhandled errors.

    Returns sanitized error response without exposing sensitive details.
    """
    print(f"❌ Unhandled exception: {exc}")

    return JSONResponse(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        content={
            "error": "InternalServerError",
            "message": "An unexpected error occurred. Please try again later.",
            "detail": None  # Don't expose internal error details
        }
    )


# ============================================================================
# Startup and Shutdown Events
# ============================================================================

@app.on_event("startup")
async def startup_event():
    """
    Run on application startup.

    Validates required environment variables and connections.
    """
    print("\n" + "=" * 60)
    print("RAG CHATBOT BACKEND STARTING")
    print("=" * 60)

    # Check required environment variables
    required_vars = [
        "GEMINI_API_KEY",
        "OPENAI_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "QDRANT_COLLECTION_NAME"
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"\nWARNING: Missing environment variables:")
        for var in missing_vars:
            print(f"   - {var}")
        print("\nThe server will start but may not function correctly.")
        print("Please configure missing variables in .env file.\n")
    else:
        print("All required environment variables configured")

    # Display configuration
    print(f"\nConfiguration:")
    print(f"   Chat Model: {os.getenv('CHAT_MODEL', 'gemini-2.5-flash')}")
    print(f"   Embedding Model: {os.getenv('EMBEDDING_MODEL', 'text-embedding-3-small')}")
    print(f"   Qdrant Collection: {os.getenv('QDRANT_COLLECTION_NAME', 'robotics_textbook')}")
    print(f"   Port: {os.getenv('PORT', '8000')}")
    print(f"   CORS Origins: {os.getenv('CORS_ORIGINS', 'http://localhost:3000')}")

    # Test Qdrant connection
    print(f"\nTesting Qdrant connection...")
    try:
        client = get_qdrant_client()
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")
        info = get_collection_info(collection_name, client=client)
        print(f"Connected to Qdrant")
        print(f"   Collection: {collection_name}")
        print(f"   Points: {info.get('points_count', 0)}")

        if info.get('points_count', 0) == 0:
            print(f"\nWARNING: Collection is empty!")
            print(f"   Run embedding script: python scripts/embed_documents.py")
            print(f"   Or use POST /api/embed endpoint (admin only)")

    except Exception as e:
        print(f"Qdrant connection failed: {e}")
        print(f"   Server will start but RAG functionality will not work.")

    print("\n" + "=" * 60)
    print("SERVER READY")
    print("=" * 60)
    print(f"   API Docs: http://localhost:{os.getenv('PORT', '8000')}/docs")
    print(f"   Health Check: http://localhost:{os.getenv('PORT', '8000')}/")
    print("=" * 60 + "\n")


@app.on_event("shutdown")
async def shutdown_event():
    """
    Run on application shutdown.

    Cleanup resources if needed.
    """
    print("\n" + "=" * 60)
    print("RAG CHATBOT BACKEND SHUTTING DOWN")
    print("=" * 60 + "\n")


# ============================================================================
# Run with Uvicorn (for development)
# ============================================================================

if __name__ == "__main__":
    import uvicorn

    port = int(os.getenv("PORT", "8000"))
    host = os.getenv("HOST", "0.0.0.0")

    uvicorn.run(
        "main:app",
        host=host,
        port=port,
        reload=True,  # Auto-reload on code changes
        log_level="info"
    )
