"""
Test Guardrails - Off-topic Refusal Tests
Feature: 004-rag-chat / Phase 4 (User Story 2)

Tests the off-topic guardrail to ensure it correctly refuses off-topic questions
and allows on-topic and borderline questions through.
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from agents import Runner, InputGuardrailTripwireTriggered
from custom_agents.rag_agent import create_rag_agent


# Test cases
OFF_TOPIC_QUESTIONS = [
    "What is Python?",  # General programming
    "How do I cook pasta?",  # Cooking
    "Explain quantum physics",  # Unrelated science
    "What is JavaScript?",  # General programming
    "Tell me about the history of AI",  # Too broad
    "How do I bake a cake?",  # Cooking
    "What are the best programming languages?",  # General programming
]

ON_TOPIC_QUESTIONS = [
    "What is ROS 2?",  # Core topic
    "How do I create a ROS 2 node?",  # Core topic
    "What is Gazebo?",  # Core topic
    "Explain inverse kinematics",  # Core topic
    "What is NVIDIA Isaac Sim?",  # Core topic
]

BORDERLINE_QUESTIONS = [
    "What is machine learning?",  # Should check retrieval (VLA-related)
    "What is Python used for in robotics?",  # Python in robotics context
    "How does computer vision work?",  # Computer vision for robotics
    "What is C++?",  # General programming (should refuse)
]


async def test_off_topic_refusal():
    """Test T032: Off-topic questions should be refused."""
    print("=" * 60)
    print("TEST T032: Off-Topic Refusal")
    print("=" * 60)

    agent = create_rag_agent()
    passed = 0
    failed = 0

    for question in OFF_TOPIC_QUESTIONS:
        print(f"\n📝 Testing: \"{question}\"")
        try:
            result = await Runner.run(
                starting_agent=agent,
                input=question
            )

            # If we get here, guardrail did NOT trip (FAIL)
            print(f"   ❌ FAIL: Guardrail should have tripped but didn't")
            print(f"   Response: {result.final_output[:100]}...")
            failed += 1

        except InputGuardrailTripwireTriggered:
            # Guardrail tripped (PASS)
            print(f"   ✅ PASS: Guardrail tripped correctly")
            passed += 1

        except Exception as e:
            print(f"   ❌ ERROR: {type(e).__name__}: {str(e)[:100]}")
            failed += 1

    print(f"\n{'-' * 60}")
    print(f"Off-Topic Tests: {passed} passed, {failed} failed")
    print(f"{'-' * 60}\n")

    return passed, failed


async def test_on_topic_allowed():
    """Test that on-topic questions are allowed through."""
    print("=" * 60)
    print("TEST: On-Topic Questions Allowed")
    print("=" * 60)

    agent = create_rag_agent()
    passed = 0
    failed = 0

    for question in ON_TOPIC_QUESTIONS:
        print(f"\n📝 Testing: \"{question}\"")
        try:
            result = await Runner.run(
                starting_agent=agent,
                input=question
            )

            # If we get here, guardrail did NOT trip (PASS)
            print(f"   ✅ PASS: Question allowed through")
            print(f"   Response: {result.final_output[:100]}...")
            passed += 1

        except InputGuardrailTripwireTriggered:
            # Guardrail tripped (FAIL - on-topic should NOT trip)
            print(f"   ❌ FAIL: Guardrail should NOT have tripped for on-topic question")
            failed += 1

        except Exception as e:
            print(f"   ❌ ERROR: {type(e).__name__}: {str(e)[:100]}")
            failed += 1

    print(f"\n{'-' * 60}")
    print(f"On-Topic Tests: {passed} passed, {failed} failed")
    print(f"{'-' * 60}\n")

    return passed, failed


async def test_borderline_queries():
    """Test T033: Borderline queries should be handled contextually."""
    print("=" * 60)
    print("TEST T033: Borderline Queries")
    print("=" * 60)

    agent = create_rag_agent()
    results = []

    for question in BORDERLINE_QUESTIONS:
        print(f"\n📝 Testing: \"{question}\"")
        try:
            result = await Runner.run(
                starting_agent=agent,
                input=question
            )

            # Guardrail did NOT trip - question allowed through
            print(f"   ℹ️  Allowed through (will check retrieval)")
            print(f"   Response: {result.final_output[:150]}...")

            # Check if retrieve_context was called
            has_context = any(
                hasattr(item, 'tool_name') and item.tool_name == 'retrieve_context'
                for item in result.new_items
            )

            if has_context:
                print(f"   ✅ retrieve_context was called")
            else:
                print(f"   ⚠️  retrieve_context was NOT called")

            results.append({
                "question": question,
                "status": "allowed",
                "has_context": has_context
            })

        except InputGuardrailTripwireTriggered:
            # Guardrail tripped
            print(f"   ℹ️  Guardrail tripped (refused)")
            results.append({
                "question": question,
                "status": "refused",
                "has_context": False
            })

        except Exception as e:
            print(f"   ❌ ERROR: {type(e).__name__}: {str(e)[:100]}")
            results.append({
                "question": question,
                "status": "error",
                "has_context": False
            })

    print(f"\n{'-' * 60}")
    print(f"Borderline Query Results:")
    for r in results:
        status_icon = "✅" if r["status"] == "allowed" else "❌" if r["status"] == "refused" else "⚠️"
        print(f"  {status_icon} {r['question']}: {r['status']}")
    print(f"{'-' * 60}\n")

    return results


async def main():
    """Run all guardrail tests."""
    print("\n" + "=" * 60)
    print("GUARDRAILS TEST SUITE - Phase 4 (User Story 2)")
    print("=" * 60 + "\n")

    # Test 1: Off-topic refusal (T032)
    off_topic_passed, off_topic_failed = await test_off_topic_refusal()

    # Test 2: On-topic allowed
    on_topic_passed, on_topic_failed = await test_on_topic_allowed()

    # Test 3: Borderline queries (T033)
    borderline_results = await test_borderline_queries()

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Off-Topic Refusal (T032): {off_topic_passed} passed, {off_topic_failed} failed")
    print(f"On-Topic Allowed: {on_topic_passed} passed, {on_topic_failed} failed")
    print(f"Borderline Queries (T033): {len(borderline_results)} tested")

    total_passed = off_topic_passed + on_topic_passed
    total_failed = off_topic_failed + on_topic_failed

    print(f"\nTotal: {total_passed} passed, {total_failed} failed")

    if total_failed == 0:
        print("\n✅ ALL TESTS PASSED!")
    else:
        print(f"\n⚠️  {total_failed} TEST(S) FAILED")

    print("=" * 60 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
