# Qdrant Cloud Setup Guide

## Quick Setup (5 minutes)

### Step 1: Create Free Qdrant Cloud Account

1. Go to: https://cloud.qdrant.io/
2. Sign up with GitHub or email
3. Click "Create Cluster"

### Step 2: Cluster Configuration

- **Name**: `ai-book-rag`
- **Region**: Choose closest to you (e.g., `us-east-1` or `europe-west3`)
- **Plan**: Free tier (1GB storage, sufficient for testing)
- Click "Create"

### Step 3: Get Credentials

After cluster is created (takes ~2 minutes):

1. Click on your cluster name
2. Copy the **Cluster URL** (looks like: `https://xxxxx-xxxxx.region.gcp.cloud.qdrant.io:6333`)
3. Go to "API Keys" tab
4. Copy the **API Key**

### Step 4: Update Backend .env

Edit `backend/.env` and replace:

```env
QDRANT_URL=<your-cluster-url>
QDRANT_API_KEY=<your-api-key>
QDRANT_COLLECTION=AI-book
```

### Step 5: Populate Vector Database

Run the ingestion pipeline to load your documents:

```bash
cd backend
python main.py
```

This will:
- Scrape documentation from your Docusaurus site
- Generate embeddings using Cohere
- Store vectors in Qdrant

### Step 6: Restart Backend

```bash
cd backend
python app.py
```

Now test: http://localhost:8000/health

## Alternative: Use Existing Cluster

If you have an existing Qdrant cluster, just update the credentials in `backend/.env`.

## Troubleshooting

**404 Error**: Cluster URL is wrong or cluster was deleted
**401 Error**: API key is invalid
**Connection timeout**: Check firewall/network settings
