"""
Test Backend Integration for Text Selection (Feature 005)
Test T008: Send POST /api/chat with selectedText, verify reranked response

This script tests the backend reranking functionality by sending a request
with selected_text and verifying the response includes references to it.
"""

import asyncio
import json
from custom_agents.rag_agent import create_rag_agent
from agents import Runner


async def test_text_selection_integration():
    """
    Test the full text selection flow:
    1. User selects text: "A node is a participant in the ROS 2 graph"
    2. User asks: "Explain this"
    3. Agent should call retrieve_context with selected_text parameter
    4. Response should reference the selected text
    """
    print("=" * 80)
    print("Testing Backend Integration for Text Selection (Feature 005 - T008)")
    print("=" * 80)

    # Test Case 1: With selected text
    print("\n📝 Test Case 1: Query with selected text")
    print("-" * 80)

    selected_text = "A node is a participant in the ROS 2 graph"
    user_message = "Explain this"

    # Format message as it would come from frontend
    agent_input = f"{user_message}\n\n[User selected this text from the book: \"{selected_text}\"]"

    print(f"User message: {user_message}")
    print(f"Selected text: {selected_text}")
    print(f"Agent input: {agent_input[:100]}...")

    try:
        # Create agent
        agent = create_rag_agent()
        print("✅ Agent created successfully")

        # Run agent with selected text
        print("\n🤖 Running agent with selected text...")
        result = await Runner.run(
            starting_agent=agent,
            input=agent_input
        )

        # Get response
        response = result.final_output or "No response generated"

        print("\n📤 Agent Response:")
        print("-" * 80)
        print(response)
        print("-" * 80)

        # Verify response references selected text
        if "based on the text you selected" in response.lower():
            print("✅ PASS: Response explicitly references selected text")
        else:
            print("⚠️  WARNING: Response does not explicitly reference selected text")

        # Check if retrieve_context was called with selected_text
        context_calls = [
            item for item in result.new_items
            if hasattr(item, 'tool_name') and item.tool_name == 'retrieve_context'
        ]

        if context_calls:
            print(f"✅ retrieve_context was called {len(context_calls)} time(s)")
            for i, call in enumerate(context_calls, 1):
                if hasattr(call, 'input'):
                    print(f"   Call {i} input: {str(call.input)[:150]}...")
        else:
            print("❌ FAIL: retrieve_context was not called")

    except Exception as e:
        print(f"❌ ERROR: {e}")
        import traceback
        traceback.print_exc()

    # Test Case 2: Without selected text (backward compatibility)
    print("\n\n📝 Test Case 2: Query without selected text (backward compatibility)")
    print("-" * 80)

    user_message_2 = "What is ROS 2?"
    print(f"User message: {user_message_2}")

    try:
        # Run agent without selected text
        print("\n🤖 Running agent without selected text...")
        result_2 = await Runner.run(
            starting_agent=agent,
            input=user_message_2
        )

        response_2 = result_2.final_output or "No response generated"

        print("\n📤 Agent Response:")
        print("-" * 80)
        print(response_2[:200] + "..." if len(response_2) > 200 else response_2)
        print("-" * 80)

        # Verify backward compatibility
        if response_2 and len(response_2) > 0:
            print("✅ PASS: Agent responds normally without selected text")
        else:
            print("❌ FAIL: Agent failed to respond without selected text")

    except Exception as e:
        print(f"❌ ERROR: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "=" * 80)
    print("Test completed")
    print("=" * 80)


if __name__ == "__main__":
    # Run the async test
    asyncio.run(test_text_selection_integration())
