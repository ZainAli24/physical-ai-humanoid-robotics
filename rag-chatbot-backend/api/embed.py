"""
Embed API Endpoint (Admin-only)
Feature: 004-rag-chat

POST /api/embed endpoint for triggering document embedding pipeline.
Requires admin authentication.
"""

import os
import subprocess
import sys
from pathlib import Path
from fastapi import APIRouter, HTTPException, status, Header
from dotenv import load_dotenv

# Import models and services
sys.path.insert(0, str(Path(__file__).parent.parent))

from models.schemas import EmbedRequest, EmbedResponse, ErrorResponse
from services.qdrant_client import get_qdrant_client, get_collection_info, delete_collection

# Load environment variables
load_dotenv()

# Create router
router = APIRouter(prefix="/api", tags=["admin"])


def verify_admin_key(admin_api_key: str) -> bool:
    """
    Verify admin API key from environment.

    Args:
        admin_api_key: API key to verify

    Returns:
        True if valid, False otherwise
    """
    expected_key = os.getenv("ADMIN_API_KEY")

    if not expected_key:
        raise ValueError("ADMIN_API_KEY not configured in environment")

    return admin_api_key == expected_key


@router.post(
    "/embed",
    response_model=EmbedResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Unauthorized - invalid admin key"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Trigger document embedding (Admin only)",
    description="Run the embedding pipeline to process documents and upload to Qdrant. Requires admin authentication."
)
async def embed_documents(
    request: EmbedRequest,
    authorization: str = Header(None, description="Admin API key in Authorization header")
):
    """
    Trigger embedding pipeline (admin-only endpoint).

    **Authentication**:
    - Requires `Authorization: Bearer <ADMIN_API_KEY>` header
    - OR `admin_api_key` in request body

    **Process**:
    1. Validates admin credentials
    2. Optionally deletes existing collection (if force=true)
    3. Runs `scripts/embed_documents.py`
    4. Returns embedding statistics

    **Example Request**:
    ```bash
    curl -X POST http://localhost:8000/api/embed \
      -H "Content-Type: application/json" \
      -H "Authorization: Bearer your_admin_key" \
      -d '{
        "docs_dir": "../docs",
        "force": false
      }'
    ```

    **Example Response**:
    ```json
    {
        "success": true,
        "message": "Embedding pipeline completed successfully",
        "chunks_processed": 187,
        "chunks_uploaded": 187,
        "collection_name": "robotics_textbook"
    }
    ```
    """
    # Verify admin authentication
    # Check Authorization header first, then request body
    admin_key = None

    if authorization and authorization.startswith("Bearer "):
        admin_key = authorization.replace("Bearer ", "")
    elif request.admin_api_key:
        admin_key = request.admin_api_key

    if not admin_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Admin API key required (Authorization header or request body)"
        )

    try:
        if not verify_admin_key(admin_key):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid admin API key"
            )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=str(e)
        )

    # Get collection name
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "robotics_textbook")

    try:
        # Handle force re-embedding
        if request.force:
            try:
                client = get_qdrant_client()
                delete_collection(collection_name, client=client)
                print(f"✅ Deleted existing collection '{collection_name}'")
            except Exception as e:
                print(f"⚠️  Warning: Could not delete collection: {e}")

        # Run embedding script
        script_path = Path(__file__).parent.parent / "scripts" / "embed_documents.py"

        if not script_path.exists():
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Embedding script not found: {script_path}"
            )

        # Execute embedding script
        print(f"🚀 Running embedding script: {script_path}")

        result = subprocess.run(
            [sys.executable, str(script_path)],
            capture_output=True,
            text=True,
            cwd=str(script_path.parent.parent),  # Run from backend root
            timeout=600  # 10 minute timeout
        )

        # Check if script succeeded
        if result.returncode != 0:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Embedding script failed: {result.stderr}"
            )

        # Get collection info
        try:
            client = get_qdrant_client()
            info = get_collection_info(collection_name, client=client)
            chunks_uploaded = info.get("points_count", 0)
        except Exception as e:
            chunks_uploaded = 0
            print(f"⚠️  Could not get collection info: {e}")

        # Parse output for chunk count (look for "Processed: X chunks" in output)
        chunks_processed = chunks_uploaded  # Default to uploaded count

        if "Processed:" in result.stdout:
            import re
            match = re.search(r'Processed:\s+(\d+)\s+chunks', result.stdout)
            if match:
                chunks_processed = int(match.group(1))

        return EmbedResponse(
            success=True,
            message="Embedding pipeline completed successfully",
            chunks_processed=chunks_processed,
            chunks_uploaded=chunks_uploaded,
            collection_name=collection_name
        )

    except subprocess.TimeoutExpired:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Embedding script timed out (>10 minutes)"
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Embedding pipeline failed: {str(e)}"
        )


if __name__ == "__main__":
    # Test admin key verification
    print("Testing admin key verification...")

    try:
        # This should fail if ADMIN_API_KEY not set
        test_key = "test_key_123"
        is_valid = verify_admin_key(test_key)
        print(f"   Key '{test_key}' valid: {is_valid}")
    except ValueError as e:
        print(f"   ✅ Correctly detected missing ADMIN_API_KEY: {e}")
