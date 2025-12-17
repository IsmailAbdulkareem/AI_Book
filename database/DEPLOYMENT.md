# RAG Chatbot API Deployment Guide

This guide explains how to deploy the RAG Chatbot API backend.

## Prerequisites

Before deploying, you need:

1. **Qdrant Cloud Account** (free tier available)
   - Sign up at https://cloud.qdrant.io/
   - Create a cluster and get your URL and API key

2. **Cohere API Key** (free tier available)
   - Sign up at https://dashboard.cohere.com/
   - Get your API key from the dashboard

3. **OpenAI API Key**
   - Sign up at https://platform.openai.com/
   - Create an API key

---

## Step 1: Set Up Qdrant Cloud

1. Go to https://cloud.qdrant.io/
2. Create a free cluster
3. Copy your cluster URL (e.g., `https://xxx-xxx.cloud.qdrant.io:6333`)
4. Create an API key and copy it

---

## Step 2: Ingest Your Documentation

**IMPORTANT**: Before deployment, you MUST populate your Qdrant database with embeddings.

```bash
cd database

# Create .env file with your credentials
cp .env.example .env
# Edit .env with your actual keys:
# COHERE_API_KEY=your-key
# QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
# QDRANT_API_KEY=your-key
# OPENAI_API_KEY=your-key

# Install dependencies
pip install -r requirements.txt

# Run the embedding pipeline
python main.py
```

This will crawl your documentation site and store embeddings in Qdrant Cloud.

**Verify embeddings exist:**
```bash
python -c "from retrieval import RetrievalPipeline; p = RetrievalPipeline(); print('Vectors exist:', p.validate_collection())"
```

---

## Step 3: Deploy to Render (Recommended)

### 3.1 Create Render Account
- Go to https://render.com/
- Sign up with GitHub (recommended for easy repo access)

### 3.2 Create New Web Service

1. Click **"New +"** → **"Web Service"**
2. Connect your GitHub account if not already connected
3. Select your **AI_Book** repository
4. Configure the service:

| Setting | Value |
|---------|-------|
| **Name** | `rag-chatbot-api` |
| **Region** | Singapore (or closest to you) |
| **Branch** | `main` |
| **Root Directory** | `database` |
| **Runtime** | `Python 3` |
| **Build Command** | `pip install -r requirements.txt` |
| **Start Command** | `uvicorn app:app --host 0.0.0.0 --port $PORT` |
| **Instance Type** | Free |

### 3.3 Add Environment Variables

Click **"Advanced"** or go to **Environment** tab and add:

| Key | Value |
|-----|-------|
| `COHERE_API_KEY` | Your Cohere API key |
| `QDRANT_URL` | `https://xxx.cloud.qdrant.io:6333` |
| `QDRANT_API_KEY` | Your Qdrant API key |
| `OPENAI_API_KEY` | Your OpenAI API key |
| `PYTHON_VERSION` | `3.11.0` |

### 3.4 Deploy

- Click **"Create Web Service"**
- Wait for build (2-5 minutes)
- You'll get a URL like: `https://rag-chatbot-api.onrender.com`

### 3.5 Test Your Deployment

```bash
# Health check
curl https://rag-chatbot-api.onrender.com/health

# Test a question
curl -X POST https://rag-chatbot-api.onrender.com/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

---

## Step 4: Update Frontend Configuration

Edit `docusaurus.config.js` to point to your Render URL:

```javascript
plugins: [
  [
    './src/plugins/docusaurus-plugin-chatbot',
    {
      apiUrl: 'https://rag-chatbot-api.onrender.com',  // Your Render URL
    },
  ],
],
```

Then commit and push:
```bash
git add docusaurus.config.js
git commit -m "Update chatbot API URL to Render deployment"
git push origin main
```

GitHub Actions will rebuild and deploy your site.

---

## RAG Guarantees Checklist

Ensure these are maintained:

- [ ] **Backend owns all prompting** - Frontend sends only `{ question, context?, top_k? }`
- [ ] **Frontend renders verbatim** - No modification of backend responses
- [ ] **Citations from backend** - Frontend only parses `[N]` markers, URLs come from backend
- [ ] **Full context sent** - Text selection sends complete text, truncation is UI-only
- [ ] **No fallback answers** - Frontend shows error if backend fails, never generates own answers

---

## Render Free Tier Notes

| Limit | Value |
|-------|-------|
| **Cold Starts** | Service sleeps after 15 min idle, ~30-60s wake time |
| **Build Minutes** | 750/month free |
| **Bandwidth** | 100 GB/month free |
| **RAM** | 512 MB |

**Tip**: The first request after idle will be slow. Consider upgrading to Starter ($7/month) for always-on.

---

## Troubleshooting

### Build Fails
- Check `requirements.txt` has all dependencies
- Verify `Root Directory` is set to `database`

### Health Check Fails
- Ensure `/health` endpoint returns 200
- Check environment variables are set
- Look at Render logs for errors

### "Vector database unavailable"
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant cluster is running
- Ensure you ran `python main.py` to populate embeddings

### "No results found"
- Run embedding pipeline: `python main.py`
- Verify your site is accessible at the URL in `main.py`

### CORS Errors
- API has CORS enabled for all origins (`*`)
- If issues persist, check browser console for specific error

### Chatbot Shows Error
- Check Render logs for backend errors
- Verify OpenAI API key has credits
- Test API directly with curl

---

## Environment Variables Reference

| Variable | Required | Description |
|----------|----------|-------------|
| `COHERE_API_KEY` | Yes | Cohere API key for embeddings |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `OPENAI_API_KEY` | Yes | OpenAI API key for generation |
| `PORT` | Auto | Set by Render automatically |
| `PYTHON_VERSION` | Recommended | `3.11.0` |

---

## Local Development

```bash
cd database

# Install dependencies
pip install -r requirements.txt

# Start server
uvicorn app:app --reload --port 8000

# Test
curl http://localhost:8000/health
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```
