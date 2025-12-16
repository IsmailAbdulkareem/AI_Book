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

## Step 1: Set Up Qdrant Cloud

1. Go to https://cloud.qdrant.io/
2. Create a free cluster
3. Copy your cluster URL (e.g., `https://xxx-xxx.cloud.qdrant.io:6333`)
4. Create an API key and copy it

## Step 2: Ingest Your Documentation

Before deployment, you need to populate your Qdrant database with embeddings:

```bash
cd database

# Create .env file with your credentials
cp .env.example .env
# Edit .env with your actual keys

# Install dependencies
pip install -r requirements.txt

# Run the embedding pipeline
python main.py
```

This will crawl your documentation site and store embeddings in Qdrant Cloud.

## Step 3: Deploy to Railway (Recommended)

Railway offers free deployment with generous limits.

### Option A: Deploy via GitHub

1. Go to https://railway.app/
2. Sign in with GitHub
3. Click "New Project" → "Deploy from GitHub repo"
4. Select your repository
5. Set the root directory to `database`
6. Add environment variables:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `OPENAI_API_KEY`
7. Railway will auto-deploy

### Option B: Deploy via CLI

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Create project
railway init

# Set environment variables
railway variables set COHERE_API_KEY=your-key
railway variables set QDRANT_URL=your-qdrant-url
railway variables set QDRANT_API_KEY=your-qdrant-key
railway variables set OPENAI_API_KEY=your-openai-key

# Deploy
railway up
```

## Step 4: Update Frontend Configuration

Once deployed, update `docusaurus.config.js` with your Railway URL:

```javascript
plugins: [
  [
    './src/plugins/docusaurus-plugin-chatbot',
    {
      apiUrl: 'https://your-app.railway.app',  // Your Railway URL
    },
  ],
],
```

Or set it via environment variable in your build:

```bash
RAG_CHATBOT_API_URL=https://your-app.railway.app npm run build
```

## Alternative: Deploy to Render

1. Go to https://render.com/
2. Create a new Web Service
3. Connect your GitHub repo
4. Set:
   - Root Directory: `database`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn api:app --host 0.0.0.0 --port $PORT`
5. Add environment variables
6. Deploy

## Alternative: Deploy to Fly.io

```bash
# Install flyctl
curl -L https://fly.io/install.sh | sh

# Login
fly auth login

# Create app (from database directory)
cd database
fly launch

# Set secrets
fly secrets set COHERE_API_KEY=xxx QDRANT_URL=xxx QDRANT_API_KEY=xxx OPENAI_API_KEY=xxx

# Deploy
fly deploy
```

## Local Development

For local testing:

```bash
cd database

# Install dependencies
pip install -r requirements.txt
# Or with uv: uv sync

# Start server
uvicorn api:app --reload --port 8000

# Test the API
curl http://localhost:8000/health
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

## Environment Variables Reference

| Variable | Required | Description |
|----------|----------|-------------|
| `COHERE_API_KEY` | Yes | Cohere API key for embeddings |
| `QDRANT_URL` | Yes | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Yes | Qdrant Cloud API key |
| `OPENAI_API_KEY` | Yes | OpenAI API key for generation |
| `PORT` | Auto | Port for the server (set by platform) |

## Troubleshooting

### "Vector database unavailable"
- Check your `QDRANT_URL` and `QDRANT_API_KEY`
- Ensure your Qdrant cluster is running
- Make sure you've run the embedding pipeline

### "No results found"
- Run the embedding pipeline: `python main.py`
- Check that your documentation site is accessible

### "Generation failed"
- Verify your `OPENAI_API_KEY` is valid
- Check your OpenAI account has credits

### CORS errors
- The API has CORS enabled for all origins
- For production, update `allow_origins` in `api.py`
