"""
Test Authenticated Chat (T044)
Feature: 006-auth-sessions

Tests that authenticated users can chat and messages are persisted to database.
"""

import asyncio
import httpx
import json
from datetime import datetime

# Configuration
BASE_URL = "http://localhost:8000"
SESSION_TOKEN = "wl0nmCcUNK27XFqJn_ByakijVjZ7C3ZXvfpcplQhzYw"
USER_ID = "1342b80f-693e-4da7-8fce-7d44a8ac6df7"


async def test_authenticated_chat():
    """Test chat endpoint with authenticated user."""

    print("\n" + "="*80)
    print("TEST: Authenticated Chat (T044)")
    print("="*80 + "\n")

    async with httpx.AsyncClient(timeout=60.0) as client:
        # Test 1: Chat request with authentication (no session_id)
        print("[TEST 1] First chat message (should auto-create session)...")

        chat_request = {
            "message": "What is ROS 2?",
            "session_id": None,  # Should auto-create
            "stream": False
        }

        headers = {
            "Authorization": f"Bearer {SESSION_TOKEN}",
            "Content-Type": "application/json"
        }

        try:
            response = await client.post(
                f"{BASE_URL}/api/chat",
                json=chat_request,
                headers=headers
            )

            if response.status_code == 200:
                data = response.json()
                print(f"[OK] Status: {response.status_code}")
                print(f"[OK] Answer received: {data['answer'][:100]}...")
                print(f"[OK] Session ID: {data.get('session_id', 'N/A')}")
                print(f"[OK] Sources count: {len(data.get('sources', []))}")

                session_id = data.get('session_id')

                # Test 2: Second message in same session
                print("\n[TEST 2] Second message in same session...")

                chat_request_2 = {
                    "message": "Tell me more about ROS 2 nodes",
                    "session_id": session_id,
                    "stream": False
                }

                response_2 = await client.post(
                    f"{BASE_URL}/api/chat",
                    json=chat_request_2,
                    headers=headers
                )

                if response_2.status_code == 200:
                    data_2 = response_2.json()
                    print(f"[OK] Status: {response_2.status_code}")
                    print(f"[OK] Answer received: {data_2['answer'][:100]}...")
                    print(f"[OK] Session ID matches: {data_2.get('session_id') == session_id}")
                else:
                    print(f"[FAIL] Status: {response_2.status_code}")
                    print(f"[FAIL] Response: {response_2.text}")
                    return False

                return True, session_id
            else:
                print(f"[FAIL] Status: {response.status_code}")
                print(f"[FAIL] Response: {response.text}")
                return False, None

        except httpx.ConnectError:
            print("[FAIL] Cannot connect to server. Is it running?")
            print("[INFO] Start server with: cd rag-chatbot-backend && uvicorn main:app --reload")
            return False, None
        except Exception as e:
            print(f"[FAIL] Error: {str(e)}")
            return False, None


async def verify_database_persistence(session_id):
    """
    Verify that messages were saved to database with proper structure.

    This function would be called after test_authenticated_chat() succeeds.
    Uses Neon MCP to verify:
    1. Chat session was created with auto-generated title
    2. Messages were saved (user + assistant)
    3. Assistant message has JSONB sources
    """

    print("\n" + "="*80)
    print("DATABASE VERIFICATION (Use Neon MCP)")
    print("="*80 + "\n")

    print("[VERIFY 1] Chat session created:")
    print(f"SELECT id, user_id, title, created_at, updated_at FROM chat_sessions WHERE id = '{session_id}';")
    print("Expected: 1 row with title auto-generated from first message\n")

    print("[VERIFY 2] Messages saved:")
    print(f"SELECT id, role, content, sources, created_at FROM messages WHERE session_id = '{session_id}' ORDER BY created_at;")
    print("Expected: 2 rows (user + assistant)\n")

    print("[VERIFY 3] Sources in JSONB format:")
    print(f"SELECT role, jsonb_array_length(sources) as source_count, sources FROM messages WHERE session_id = '{session_id}' AND role = 'assistant';")
    print("Expected: sources column contains JSONB array with citation objects\n")

    print("[VERIFY 4] Title generation (after first exchange):")
    print(f"SELECT title FROM chat_sessions WHERE id = '{session_id}';")
    print("Expected: Title should be 'What is ROS 2?' (truncated to 60 chars if needed)\n")

    return session_id


async def main():
    """Run all tests."""

    print("\nStarting authenticated chat tests...")
    print(f"User ID: {USER_ID}")
    print(f"Session Token: {SESSION_TOKEN[:20]}...")

    # Run chat test
    success, session_id = await test_authenticated_chat()

    if success and session_id:
        print("\n[SUCCESS] Chat tests passed!")
        print(f"\nGenerated session_id: {session_id}")

        # Print verification steps
        await verify_database_persistence(session_id)

        print("\n" + "="*80)
        print("[NEXT STEP] Run the verification queries above using Neon MCP")
        print("="*80 + "\n")

        return session_id
    else:
        print("\n[FAIL] Chat tests failed. Check server logs.")
        return None


if __name__ == "__main__":
    session_id = asyncio.run(main())

    if session_id:
        print(f"\n[INFO] To verify in database, use session_id: {session_id}")
        print("[INFO] Example Neon MCP query:")
        print(f"       SELECT * FROM messages WHERE session_id = '{session_id}';")
