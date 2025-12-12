# RAG Chatbot Backend

**Feature**: 004-rag-chat
**Stack**: Python + FastAPI + OpenAI Agents SDK + Google Gemini + Qdrant
**Deployment**: Render Free Tier
**Status**: Phase 4 Complete (Guardrails Implemented) ✅

## Overview

This is the backend for the RAG chatbot embedded in the Physical AI & Humanoid Robotics textbook. It uses:
- **Google Gemini 2.5 Flash** (via OpenAI Chat Completions API) for chat responses
- **OpenAI Agents SDK** with input guardrails for off-topic detection
- **OpenAI text-embedding-3-small** for embeddings
- **Qdrant** for vector search

## Tech Stack

- **FastAPI 0.115.0** - Async Python web framework
- **OpenAI Agents SDK 0.1.0** - Agent orchestration with guardrails
- **Google Gemini 2.5 Flash** - Chat completions model (via OpenAI API)
- **OpenAI API 1.54.0** - Embeddings (text-embedding-3-small)
- **Qdrant Client 1.12.0** - Vector database for semantic search
- **tiktoken 0.8.0** - Token counting for chunking (512 tokens, 128 overlap)
- **Uvicorn** - ASGI server

## Project Structure

```
rag-chatbot-backend/
├── main.py                     # FastAPI app entry point
├── requirements.txt            # Python dependencies
├── render.yaml                 # Render deployment config
├── .env.example                # Environment variables template
├── agents/
│   ├── __init__.py
│   ├── rag_agent.py           # OpenAI Agent with instructions + tools
│   ├── tools.py               # Custom tools (@function_tool)
│   └── guardrails.py          # Input guardrails (off-topic detection)
├── services/
│   ├── __init__.py
│   ├── qdrant_client.py       # Qdrant operations (async)
│   ├── embeddings.py          # OpenAI embeddings
│   └── chunking.py            # Document chunking
├── api/
│   ├── __init__.py
│   ├── chat.py                # POST /api/chat (Agent runner)
│   └── embed.py               # POST /api/embed (admin only)
├── models/
│   ├── __init__.py
│   └── schemas.py             # Pydantic models
└── scripts/
    └── embed_documents.py     # One-time embedding script
```

## Setup Instructions

### 1. Install Dependencies

```bash
cd rag-chatbot-backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

```bash
cp .env.example .env
# Edit .env with your API keys
```

Required variables:
- `GEMINI_API_KEY` - Google Gemini API key (for chat model)
- `CHATCOMPLETION_BASE_URL` - Gemini base URL (default: https://generativelanguage.googleapis.com/v1beta/openai/)
- `CHAT_MODEL` - Model name (default: gemini-2.5-flash)
- `OPENAI_API_KEY` - OpenAI API key (for embeddings)
- `QDRANT_URL` - Qdrant cluster URL
- `QDRANT_API_KEY` - Qdrant API key

### 3. Embed Documents (One-Time Setup)

```bash
python scripts/embed_documents.py
```

This script:
- Reads markdown files from `../docs/` (Docusaurus content)
- Chunks them into 512-token chunks (128 overlap)
- Generates embeddings with OpenAI text-embedding-3-small
- Stores vectors in Qdrant Cloud

### 4. Run Development Server

```bash
uvicorn main:app --reload --port 8000
```

API will be available at `http://localhost:8000`

## API Endpoints

### POST /api/chat

**Request**:
```json
{
  "message": "What is ROS 2?",
  "sessionId": "uuid-v4",
  "selectedText": null
}
```

**Response** (Server-Sent Events):
```json
{
  "type": "content",
  "content": "ROS 2 is...",
  "done": false
}

{
  "type": "sources",
  "sources": [
    {
      "title": "Chapter 1: Introduction to ROS 2",
      "url": "/docs/module-1/chapter-1",
      "score": 0.89,
      "excerpt": "..."
    }
  ]
}

{
  "type": "done",
  "done": true
}
```

### POST /api/embed (Admin Only)

Trigger document embedding (requires `ADMIN_API_KEY` header).

## OpenAI Agent Architecture

The chatbot uses OpenAI Agents SDK with a custom `retrieve_context` tool:

```python
from agents import Agent, function_tool, Runner

@function_tool
async def retrieve_context(query: str) -> dict:
    """Search Qdrant for relevant chunks"""
    # ... implementation
    return {"context": "...", "sources": [...]}

rag_agent = Agent(
    name="RoboticsAssistant",
    instructions="You are an expert...",
    tools=[retrieve_context],
    model="gpt-4o"
)

# In endpoint:
result = await Runner.run(rag_agent, user_message)
```

## Deployment (Render Free Tier)

### Prerequisites
- Render account
- Qdrant Cloud Free Tier cluster
- OpenAI API key

### Steps

1. Push code to GitHub repository
2. Create new Web Service on Render
3. Connect GitHub repo: `rag-chatbot-backend/`
4. Configure environment variables in Render dashboard
5. Deploy (Render auto-detects Python and runs `pip install`)

**render.yaml** configuration is included for automated deployment.

### Cold Start Handling

Render Free Tier spins down after 15 minutes of inactivity. First request after spin-down takes ~30 seconds. Consider:
- Adding loading state in frontend
- Using Render paid tier for production

## Development Notes

- **Separate Dependencies**: This backend has its own `requirements.txt`, separate from the Docusaurus book dependencies
- **CORS**: Configure `CORS_ORIGINS` in `.env` to match your frontend URL
- **Rate Limiting**: OpenAI API has rate limits; consider implementing queue for production
- **Embeddings Cost**: Initial embedding is one-time cost (~$0.10 for 1M tokens)

## Testing

```bash
# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "sessionId": "test-123"}'
```

## License

Same as parent project.
