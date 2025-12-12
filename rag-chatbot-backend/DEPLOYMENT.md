# Deployment Guide - RAG Chatbot Backend

**Feature**: 004-rag-chat
**Platform**: Render Free Tier
**Created**: 2025-12-04

---

## Prerequisites

Before deploying, ensure you have:

1. **Render Account**: Sign up at [render.com](https://render.com)
2. **GitHub Repository**: Backend code pushed to GitHub
3. **API Keys**:
   - Google Gemini API Key (for chat model)
   - OpenAI API Key (for embeddings)
   - Qdrant Cloud API Key + URL
4. **Qdrant Collection**: `robotics_textbook` collection with embeddings already uploaded

---

## Step 1: Prepare Backend for Deployment

### 1.1 Verify Dependencies

Ensure `requirements.txt` contains all dependencies:

```bash
cd rag-chatbot-backend
pip install -r requirements.txt
```

### 1.2 Test Locally

Run the backend locally to verify it works:

```bash
# Set environment variables
export GEMINI_API_KEY="your-gemini-key"
export OPENAI_API_KEY="your-openai-key"
export QDRANT_URL="your-qdrant-url"
export QDRANT_API_KEY="your-qdrant-key"

# Run server
uvicorn main:app --reload --port 8000
```

Test the `/api/chat` endpoint:

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "stream": false}'
```

---

## Step 2: Push to GitHub

### 2.1 Create .gitignore

Ensure `.gitignore` excludes sensitive files:

```
.env
__pycache__/
*.pyc
venv/
.venv/
*.egg-info/
dist/
build/
.pytest_cache/
```

### 2.2 Commit and Push

```bash
cd rag-chatbot-backend
git add .
git commit -m "feat: RAG chatbot backend with Gemini and guardrails"
git push origin main
```

---

## Step 3: Deploy to Render

### 3.1 Create New Web Service

1. Go to [Render Dashboard](https://dashboard.render.com/)
2. Click **"New +"** → **"Web Service"**
3. Connect your GitHub repository
4. Configure:
   - **Name**: `rag-chatbot-backend`
   - **Root Directory**: `rag-chatbot-backend/` (if backend is in subdirectory)
   - **Runtime**: Python
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free

### 3.2 Set Environment Variables

In Render dashboard, go to **Environment** tab and add:

| Key | Value | Secret? |
|-----|-------|---------|
| `PYTHON_VERSION` | `3.11.0` | No |
| `GEMINI_API_KEY` | `<your-gemini-key>` | Yes |
| `CHATCOMPLETION_BASE_URL` | `https://generativelanguage.googleapis.com/v1beta/openai/` | No |
| `CHAT_MODEL` | `gemini-2.5-flash` | No |
| `OPENAI_API_KEY` | `<your-openai-key>` | Yes |
| `EMBEDDING_MODEL` | `text-embedding-3-small` | No |
| `QDRANT_URL` | `<your-qdrant-url>` | Yes |
| `QDRANT_API_KEY` | `<your-qdrant-key>` | Yes |
| `QDRANT_COLLECTION_NAME` | `robotics_textbook` | No |
| `CHUNK_SIZE` | `512` | No |
| `CHUNK_OVERLAP` | `128` | No |
| `RETRIEVAL_LIMIT` | `5` | No |
| `SIMILARITY_THRESHOLD` | `0.7` | No |
| `CORS_ORIGINS` | `https://yourusername.github.io` | No |
| `ADMIN_API_KEY` | `<random-secret-key>` | Yes |

**Note**: Replace `yourusername.github.io` with your actual GitHub Pages URL.

### 3.3 Deploy

Click **"Create Web Service"**. Render will:
1. Clone your repository
2. Install dependencies (~2-3 minutes)
3. Start the server

---

## Step 4: Verify Deployment

### 4.1 Check Logs

In Render dashboard, go to **Logs** tab and verify:

```
✅ Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:10000
```

### 4.2 Test Health Check

Get your Render URL (e.g., `https://rag-chatbot-backend.onrender.com`) and test:

```bash
curl https://rag-chatbot-backend.onrender.com/
```

Expected response:
```json
{
  "message": "RAG Chatbot API",
  "version": "1.0.0",
  "status": "running"
}
```

### 4.3 Test Chat Endpoint

```bash
curl -X POST https://rag-chatbot-backend.onrender.com/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "stream": false}'
```

Expected: JSON response with answer and sources.

---

## Step 5: Update Frontend

### 5.1 Update API URL

Edit `src/hooks/useChat.ts`:

```typescript
const API_BASE_URL =
  process.env.NODE_ENV === 'production'
    ? 'https://rag-chatbot-backend.onrender.com'  // Replace with your Render URL
    : 'http://localhost:8000';
```

### 5.2 Build and Deploy Frontend

```bash
# Build Docusaurus
npm run build

# Deploy to GitHub Pages
npm run deploy
```

---

## Step 6: Test End-to-End

1. Open your GitHub Pages site: `https://yourusername.github.io/physical_ai_humanoid_robotics`
2. Navigate to any doc page
3. Click the floating chat button (bottom-right)
4. Ask: **"What is ROS 2?"**
5. Verify:
   - ✅ Response appears in < 3 seconds
   - ✅ Source citations display
   - ✅ Clicking source navigates to chapter

---

## Troubleshooting

### Issue: CORS Errors

**Symptom**: Browser console shows CORS errors

**Solution**: Update `CORS_ORIGINS` in Render environment variables to match your GitHub Pages URL exactly.

### Issue: Cold Starts (~30s)

**Symptom**: First request after 15+ minutes takes 30+ seconds

**Solution**: This is normal for Render Free Tier. Consider:
- Adding "Warming up..." indicator in frontend
- Upgrading to Render Starter Plan ($7/month)

### Issue: 429 Rate Limit Errors

**Symptom**: OpenAI API returns 429 errors

**Solution**: Check your OpenAI API usage and upgrade plan if needed.

### Issue: Guardrail Agent Failing

**Symptom**: All questions return "error"

**Solution**: Check Render logs for Gemini API errors. Verify `GEMINI_API_KEY` is set correctly.

---

## Cost Estimate

**Monthly Cost** (assuming 100 queries/day):

| Service | Cost |
|---------|------|
| Render Free Tier | $0 (750 hours/month) |
| Qdrant Cloud Free | $0 (1GB storage) |
| OpenAI Embeddings | ~$0.50 (text-embedding-3-small) |
| Google Gemini API | ~$1.00 (gemini-2.5-flash) |
| **Total** | **~$1.50/month** |

---

## Monitoring

### Check Render Metrics

1. Go to Render Dashboard → Your Service
2. Check **Metrics** tab:
   - Response time (p95 should be < 3s)
   - Error rate (should be < 5%)
   - CPU/Memory usage

### Check Logs

```bash
# View recent logs
render logs -f rag-chatbot-backend
```

---

## Updating Deployment

### Push Updates

```bash
git add .
git commit -m "fix: improve guardrails"
git push origin main
```

Render will automatically redeploy when you push to `main` branch.

---

## Rollback

If deployment fails, use Render's **"Rollback"** button to revert to previous version.

---

**Status**: Deployment guide complete ✅
**Next**: Deploy to Render and test production
