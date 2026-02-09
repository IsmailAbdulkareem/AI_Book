# Hugging Face Deployment Guide

## Backend Deployment to Hugging Face Spaces

### Prerequisites

1. Hugging Face account: https://huggingface.co/join
2. Valid Qdrant Cloud cluster (see QDRANT-SETUP.md)
3. OpenAI API key
4. Cohere API key
5. PostgreSQL database (Neon, Supabase, or similar)

### Step 1: Create a New Space

1. Go to https://huggingface.co/spaces
2. Click "Create new Space"
3. Configure:
   - **Name**: `physical-ai-chatbot-api`
   - **License**: MIT
   - **SDK**: Docker
   - **Hardware**: CPU Basic (free tier)
4. Click "Create Space"

### Step 2: Prepare Backend Files

```bash
cd backend

# Copy Dockerfile for HF
cp Dockerfile.hf Dockerfile

# Copy README for HF
cp README.hf.md README.md
```

### Step 3: Set Environment Variables

In your Hugging Face Space settings, add these secrets:

```
OPENAI_API_KEY=sk-proj-...
COHERE_API_KEY=...
QDRANT_URL=https://xxxxx.cloud.qdrant.io:6333
QDRANT_API_KEY=...
QDRANT_COLLECTION=AI-book
DATABASE_URL=postgresql://...
OPENAI_MODEL=gpt-4
COHERE_MODEL=embed-english-v3.0
```

### Step 4: Deploy

**Option A: Using Git (Recommended)**

```bash
# Initialize git in backend folder
cd backend
git init
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/physical-ai-chatbot-api

# Add files
git add .
git commit -m "Initial deployment"

# Push to Hugging Face
git push hf main
```

**Option B: Using Hugging Face Web UI**

1. Go to your Space's "Files" tab
2. Upload all files from `backend/` folder
3. Space will automatically build and deploy

### Step 5: Test Deployment

Once deployed (takes 2-3 minutes):

```bash
# Test health endpoint
curl https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/health

# Test chatbot
curl -X POST https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?", "session_id": "test-123"}'
```

### Step 6: Update Frontend

Update `frontend/docusaurus.config.js`:

```javascript
plugins: [
  [
    './src/plugins/docusaurus-plugin-chatbot',
    {
      apiUrl: 'https://YOUR_USERNAME-physical-ai-chatbot-api.hf.space',
    },
  ],
],
```

## Auth Server Deployment (Optional)

If you want to re-enable authentication:

1. Create another Space for auth-server
2. Follow similar steps
3. Update frontend auth-client.ts with new URL

## Troubleshooting

### Build Fails

- Check Dockerfile syntax
- Verify all dependencies in requirements.txt
- Check Space logs in "Logs" tab

### Runtime Errors

- Verify all environment variables are set
- Check Qdrant connection
- Verify OpenAI API key is valid
- Check database connection string

### CORS Issues

Backend already has CORS configured for all origins. If issues persist:
- Check browser console for specific errors
- Verify frontend is using correct API URL

## Cost Considerations

**Free Tier Limits:**
- Hugging Face: CPU Basic (free, may sleep after inactivity)
- Qdrant Cloud: 1GB free tier
- Neon PostgreSQL: 0.5GB free tier
- OpenAI: Pay per use
- Cohere: Free tier available

**Upgrade Options:**
- HF Spaces: $0.60/hour for persistent CPU
- Qdrant: $25/month for 2GB
- Neon: $19/month for 10GB

## Next Steps

1. Monitor Space logs for errors
2. Test all chatbot features
3. Set up custom domain (optional)
4. Enable Space analytics
5. Add rate limiting if needed
