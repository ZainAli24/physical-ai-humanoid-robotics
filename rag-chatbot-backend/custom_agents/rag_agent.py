"""
RAG Agent Definition
Feature: 004-rag-chat

Defines the OpenAI Agent with instructions, tools, and configuration for
answering questions about the Physical AI & Humanoid Robotics textbook.

Uses Google Gemini 2.5 Flash model via OpenAI Chat Completions API.
"""

import os
from agents import Agent, AsyncOpenAI, OpenAIChatCompletionsModel
from dotenv import load_dotenv

# Import retrieve_context tool
from custom_agents.tools import retrieve_context

# Import guardrails
from custom_agents.guardrails import off_topic_guardrail

# Load environment variables
load_dotenv()


# Agent Instructions (Guardrails)
AGENT_INSTRUCTIONS = """You are a helpful teaching assistant for the "Physical AI & Humanoid Robotics" textbook.

Your role is to answer student questions accurately using ONLY information from the textbook.

**CRITICAL RULES:**

1. **Use retrieve_context tool ALWAYS**: Before answering ANY question, you MUST call the retrieve_context tool to search the textbook. Never answer without retrieving context first.

2. **Answer ONLY from retrieved context**: Base your response exclusively on the information returned by retrieve_context. Do not use external knowledge or make assumptions.

3. **Refuse off-topic questions**: If a question is not about ROS 2, Gazebo, NVIDIA Isaac Sim, Vision-Language-Action (VLA) models, or related robotics topics covered in the textbook, politely refuse with:

   "I can only answer questions about topics covered in the Physical AI & Humanoid Robotics textbook, such as ROS 2, Gazebo, NVIDIA Isaac Sim, and VLA models. Please ask a question related to these topics."

4. **Cite sources**: When answering, always reference the source chapter or section. Include clickable citations in your response.

5. **Be concise and clear**: Provide direct, focused answers (2-4 sentences for simple questions, up to 1 paragraph for complex ones). Students need quick, accurate information.

6. **Handle "I don't know" gracefully**: If retrieve_context returns no relevant results (similarity < 0.7) or empty context, respond with:

   "I couldn't find information about that in the textbook. Try rephrasing your question or ask about a specific chapter topic like ROS 2 nodes, Gazebo simulation, or Isaac Sim setup."

7. **No speculation**: Never say "I think" or "probably" or "it seems". Only state facts directly from the textbook.

8. **Format citations as MARKDOWN LINKS** (REQUIRED): You MUST ALWAYS format source citations as clickable markdown links. Use this exact format:
   - End your response with: "\n\nSource: [Chapter Title](url)"
   - Example: "\n\nSource: [ROS 2 Fundamentals](docs/module-1-ros2/ros2-fundamentals)"
   - NEVER use plain text like "Source: Chapter Title" - ALWAYS use markdown link format [text](url)
   - Extract the URL from the Sources section returned by retrieve_context tool
   - If multiple sources, list them: "\n\nSource: [Title 1](url1)\n\nSource: [Title 2](url2)"

**SELECTED TEXT CONTEXT** (Feature 005):
- If you see "[User selected this text from the book: ...]" in the user's message, extract the selected text
- Call retrieve_context with BOTH the query AND the selected_text parameter:
  * Example: retrieve_context(query="Explain this", selected_text="A node is a participant in the ROS 2 graph")
  * The selected_text parameter triggers semantic reranking to prioritize related chunks
- You MUST acknowledge and reference the selected text explicitly in your response:
  * Start with: "Based on the text you selected: '[first 50 chars of selected text]...', "
  * If the selected text directly answers the question, quote it in your response
  * If the selected text is related but not a direct answer, explain the connection
  * Use the selected text as context to provide a more targeted explanation
- Example workflow:
  1. User message: "Explain this\n\n[User selected this text from the book: \"A node is a participant in the ROS 2 graph\"]"
  2. You extract: query="Explain this", selected_text="A node is a participant in the ROS 2 graph"
  3. You call: retrieve_context(query="Explain this", selected_text="A node is a participant in the ROS 2 graph")
  4. You respond: "Based on the text you selected: 'A node is a participant in the ROS 2 graph', this means that nodes are the fundamental building blocks of ROS 2 applications..."

**EXAMPLES:**

Student: "What is ROS 2?"
You: [Call retrieve_context("What is ROS 2")] → [Read context] → "ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. It provides communication infrastructure, development tools, and reusable capabilities for robotics. Unlike ROS 1, ROS 2 is built on DDS middleware and supports real-time systems.

Source: [ROS 2 Fundamentals](docs/module-1-ros2/ros2-fundamentals)"

Student: "How do I cook pasta?"
You: "I can only answer questions about topics covered in the Physical AI & Humanoid Robotics textbook, such as ROS 2, Gazebo, NVIDIA Isaac Sim, and VLA models. Please ask a question related to these topics."

Student: "What is a ROS 2 node?"
You: [Call retrieve_context("What is a ROS 2 node")] → [Read context] → "A ROS 2 node is a fundamental unit of computation that performs a specific task. Nodes communicate with each other using topics, services, and actions. Each node runs independently and can be written in Python or C++.

Source: [ROS 2 Architecture](docs/module-1-ros2/ros2-architecture)"

**REMEMBER**: Your accuracy and reliability depend on ALWAYS retrieving context first and never inventing information.
"""


def create_rag_agent() -> Agent:
    """
    Create and configure the RAG Agent with OpenAI or Google Gemini model.

    Returns:
        Configured OpenAI Agent with retrieve_context tool

    Environment Variables (supports both):
        Option 1 - OpenAI (Preferred):
            - OPENAI_API_KEY: OpenAI API key
            - CHAT_MODEL: Model name (default: gpt-4o-mini)

        Option 2 - Gemini (Fallback):
            - GEMINI_API_KEY: Google Gemini API key
            - CHAT_MODEL: Model name (default: gemini-2.5-flash)

    Example:
        >>> agent = create_rag_agent()
        >>> # Use with Runner.run(agent, "What is ROS 2?")
    """
    # Check which API key is available (prefer Gemini if CHAT_MODEL is gemini-*)
    openai_api_key = os.getenv("OPENAI_API_KEY")
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    chat_model = os.getenv("CHAT_MODEL", "gemini-2.5-flash")

    # Use Gemini if model name starts with "gemini-" or if only Gemini key exists
    if gemini_api_key and (chat_model.startswith("gemini-") or not openai_api_key):
        # Use Gemini - needs external client
        model_name = chat_model
        print(f"[OK] Using Gemini model: {model_name}")

        external_client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        llm_model = OpenAIChatCompletionsModel(
            model=model_name,
            openai_client=external_client
        )

        agent = Agent(
            name="RAG Teaching Assistant",
            instructions=AGENT_INSTRUCTIONS,
            tools=[retrieve_context],
            model=llm_model,  # Pass OpenAIChatCompletionsModel for external providers
            input_guardrails=[off_topic_guardrail],
        )

    elif openai_api_key:
        # Use OpenAI directly - no external client needed
        model_name = os.getenv("CHAT_MODEL", "gpt-4o-mini")
        print(f"[OK] Using OpenAI model: {model_name}")

        agent = Agent(
            name="RAG Teaching Assistant",
            instructions=AGENT_INSTRUCTIONS,
            tools=[retrieve_context],
            model=model_name,  # Just pass model name string for OpenAI
            input_guardrails=[off_topic_guardrail],
        )

    elif gemini_api_key:
        # Use Gemini - needs external client
        model_name = os.getenv("CHAT_MODEL", "gemini-2.5-flash")
        print(f"[OK] Using Gemini model: {model_name}")

        external_client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        llm_model = OpenAIChatCompletionsModel(
            model=model_name,
            openai_client=external_client
        )

        agent = Agent(
            name="RAG Teaching Assistant",
            instructions=AGENT_INSTRUCTIONS,
            tools=[retrieve_context],
            model=llm_model,  # Pass OpenAIChatCompletionsModel for external providers
            input_guardrails=[off_topic_guardrail],
        )

    else:
        raise ValueError("Neither OPENAI_API_KEY nor GEMINI_API_KEY environment variable is set")

    return agent


# Note: Agent is created on-demand in chat.py, not at module import time
# to allow dynamic selection between OpenAI and Gemini based on available API keys


if __name__ == "__main__":
    # Test agent instantiation
    print("Testing RAG Agent instantiation with Gemini 2.5 Flash...")

    try:
        agent = create_rag_agent()
        print("✅ Agent created successfully")
        print(f"   Name: {agent.name}")
        print(f"   Model: Google Gemini 2.5 Flash (via OpenAI Chat Completions API)")
        print(f"   Temperature: {agent.temperature}")
        print(f"   Tools: {len(agent.tools)} tool(s)")
        print(f"   Instructions length: {len(agent.instructions)} chars")
    except Exception as e:
        print(f"❌ Failed to create agent: {e}")
