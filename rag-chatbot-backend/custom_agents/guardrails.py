"""
Guardrails for RAG Agent
Feature: 004-rag-chat

Implements input and output guardrails for the RAG teaching assistant.
"""

from pydantic import BaseModel
from agents import (
    Agent,
    GuardrailFunctionOutput,
    RunContextWrapper,
    Runner,
    TResponseInputItem,
    input_guardrail,
)


class TopicCheckOutput(BaseModel):
    """Output schema for topic relevance check."""
    is_off_topic: bool
    reasoning: str
    suggested_response: str | None = None


# Guardrail Agent - Checks if question is off-topic
topic_guardrail_agent = Agent(
    name="Topic Relevance Checker",
    instructions="""You are a topic relevance checker for the Physical AI & Humanoid Robotics textbook.

**ALLOWED TOPICS (In-Scope):**
- ROS 2 (Robot Operating System 2): nodes, topics, services, actions, launch files, packages
- Gazebo Simulation: world files, models, plugins, sensors, physics
- Unity Robotics: Unity ML-Agents, robotics simulation
- NVIDIA Isaac Sim: Omniverse, physics simulation, robot training
- Vision-Language-Action (VLA) models: robotics AI, embodied AI
- Control Systems: PID control, feedback loops, controllers
- Kinematics: forward kinematics, inverse kinematics, motion planning
- Sensors: LiDAR, cameras, IMU, odometry
- Computer Vision (for robotics): object detection, image processing
- Machine Learning (ONLY if related to robotics/VLA): reinforcement learning for robots
- Physical AI: embodied intelligence, robot perception and action

**OFF-TOPIC (Out of Scope):**
- General programming languages (Python basics, C++ basics, JavaScript) unrelated to robotics
- Web development, mobile apps, game development (unless Unity for robotics)
- Cooking, recipes, general life advice
- Science topics unrelated to robotics (quantum physics, chemistry, biology, history)
- General machine learning unrelated to robotics
- Current events, politics, entertainment

**YOUR TASK:**
Determine if the user's question is OFF-TOPIC. Return:
- `is_off_topic: true` if the question is clearly off-topic
- `is_off_topic: false` if the question is on-topic or borderline (let the main agent decide)
- `reasoning`: Brief explanation of your decision
- `suggested_response`: If off-topic, suggest what the user should ask instead

**EXAMPLES:**

User: "What is Python?"
Response: {
  "is_off_topic": true,
  "reasoning": "Question asks about Python programming in general, not robotics-specific Python usage",
  "suggested_response": "Ask about ROS 2 Python nodes, robot control with Python, or VLA models"
}

User: "How do I cook pasta?"
Response: {
  "is_off_topic": true,
  "reasoning": "Question is about cooking, completely unrelated to robotics",
  "suggested_response": "Ask about robotics topics like ROS 2, Gazebo, or Isaac Sim"
}

User: "Explain quantum physics"
Response: {
  "is_off_topic": true,
  "reasoning": "Quantum physics is not covered in the robotics textbook",
  "suggested_response": "Ask about robotics topics like sensors, kinematics, or control systems"
}

User: "What is ROS 2?"
Response: {
  "is_off_topic": false,
  "reasoning": "ROS 2 is a core topic in the textbook",
  "suggested_response": null
}

User: "What is machine learning?"
Response: {
  "is_off_topic": false,
  "reasoning": "Machine learning is borderline - could relate to VLA models. Let main agent check retrieval",
  "suggested_response": null
}

User: "How do I create a ROS 2 node in Python?"
Response: {
  "is_off_topic": false,
  "reasoning": "ROS 2 nodes are a core topic, Python is used in robotics context",
  "suggested_response": null
}
""",
    output_type=TopicCheckOutput,
    # Note: temperature parameter removed - not supported in openai-agents 0.6.2
    # Configure temperature via agent.run() calls if needed
)


@input_guardrail
async def off_topic_guardrail(
    ctx: RunContextWrapper[None],
    agent: Agent,
    input: str | list[TResponseInputItem],
) -> GuardrailFunctionOutput:
    """
    Input guardrail that checks if the user's question is off-topic.

    If the question is clearly off-topic (e.g., "How do I cook pasta?"),
    the guardrail trips and prevents the main agent from running.

    Borderline questions (e.g., "What is machine learning?") are allowed
    through so the main agent can check retrieval results.

    Args:
        ctx: Run context wrapper
        agent: The main agent
        input: User's input (question)

    Returns:
        GuardrailFunctionOutput with tripwire_triggered=True if off-topic
    """
    # Run the guardrail agent to check topic relevance
    result = await Runner.run(topic_guardrail_agent, input, context=ctx.context)

    check_output: TopicCheckOutput = result.final_output

    # If off-topic, prepare a helpful response
    output_info = None
    if check_output.is_off_topic:
        output_info = {
            "message": "I can only answer questions about topics covered in the Physical AI & Humanoid Robotics textbook, such as ROS 2, Gazebo, NVIDIA Isaac Sim, and VLA models. Please ask a question related to these topics.",
            "reasoning": check_output.reasoning,
            "suggestion": check_output.suggested_response,
        }

    return GuardrailFunctionOutput(
        output_info=output_info,
        tripwire_triggered=check_output.is_off_topic,
    )


# Export guardrail
__all__ = ["off_topic_guardrail", "topic_guardrail_agent", "TopicCheckOutput"]
