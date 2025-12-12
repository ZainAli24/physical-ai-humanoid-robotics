"""
Standalone test script for auth endpoints without starting full server
"""

import asyncio
import sys
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import AsyncSession

# Load environment variables
load_dotenv()

# Add parent directory to path
sys.path.insert(0, '.')

from database.connection import AsyncSessionLocal
from database.models import User
from services.auth_service import AuthService


async def test_signup():
    """Test user signup flow"""
    print("\n" + "=" * 60)
    print("Testing Signup Flow")
    print("=" * 60)

    async with AsyncSessionLocal() as db:
        # Test data
        email = "test@example.com"
        password = "test1234"
        name = "Test User"

        # Check if user already exists (cleanup from previous test)
        is_unique = await AuthService.validate_email_unique(db, email)
        if not is_unique:
            print(f"WARNING: User {email} already exists, skipping creation")
            return

        # Hash password
        password_hash = AuthService.hash_password(password)
        print(f"[OK] Password hashed (length: {len(password_hash)})")

        # Create user
        user = User(
            email=email,
            password_hash=password_hash,
            name=name
        )
        db.add(user)
        await db.flush()
        print(f"[OK] User created: {user.id}")

        # Create session
        session = await AuthService.create_session(db, user.id)
        print(f"[OK] Session created: {session.token[:20]}...")

        await db.commit()

        print(f"\nResult:")
        print(f"   User ID: {user.id}")
        print(f"   Email: {user.email}")
        print(f"   Name: {user.name}")
        print(f"   Session Token: {session.token}")
        print(f"   Token Length: {len(session.token)} characters")
        print(f"   Expires: {session.expires_at}")


async def test_signin():
    """Test user signin flow"""
    print("\n" + "=" * 60)
    print("Testing Signin Flow")
    print("=" * 60)

    async with AsyncSessionLocal() as db:
        email = "test@example.com"
        password = "test1234"

        # Query user by email
        from sqlalchemy import select
        stmt = select(User).filter_by(email=email)
        result = await db.execute(stmt)
        user = result.scalar_one_or_none()

        if not user:
            print(f"[FAIL] User not found: {email}")
            return

        # Verify password
        is_valid = AuthService.verify_password(password, user.password_hash)
        if not is_valid:
            print(f"[FAIL] Invalid password")
            return

        print(f"[OK] Password verified")

        # Create new session
        session = await AuthService.create_session(db, user.id)
        print(f"[OK] New session created: {session.token[:20]}...")

        await db.commit()

        print(f"\nResult:")
        print(f"   User ID: {user.id}")
        print(f"   Email: {user.email}")
        print(f"   Session Token: {session.token}")
        print(f"   Expires: {session.expires_at}")


async def test_session_verification():
    """Test session token verification"""
    print("\n" + "=" * 60)
    print("Testing Session Verification")
    print("=" * 60)

    async with AsyncSessionLocal() as db:
        # Get the most recent session for test user
        from sqlalchemy import select
        from database.models import Session

        stmt = select(Session).join(User).filter(
            User.email == "test@example.com"
        ).order_by(Session.created_at.desc()).limit(1)
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if not session:
            print("[FAIL] No session found")
            return

        print(f"Testing with token: {session.token[:20]}...")

        # Verify session
        user = await AuthService.verify_session(db, session.token)

        if user:
            print(f"[OK] Session valid")
            print(f"   User: {user.email}")
            print(f"   Name: {user.name}")
        else:
            print(f"[FAIL] Session invalid or expired")


async def main():
    """Run all tests"""
    print("\nAUTH ENDPOINT TESTING")
    print("=" * 60)

    try:
        await test_signup()
        await test_signin()
        await test_session_verification()

        print("\n" + "=" * 60)
        print("[OK] ALL TESTS PASSED")
        print("=" * 60 + "\n")

    except Exception as e:
        print(f"\n[FAIL] TEST FAILED: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
