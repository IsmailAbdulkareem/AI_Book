# Quickstart: Embedding Pipeline Setup

**Feature**: 001-embedding-pipeline
**Date**: 2025-12-15

## Prerequisites

- Python 3.11 or higher
- UV package manager ([install guide](https://docs.astral.sh/uv/getting-started/installation/))
- Cohere API key ([get one here](https://dashboard.cohere.com/api-keys))
- Qdrant Cloud account ([sign up here](https://cloud.qdrant.io/))

## Setup Steps

### 1. Install UV (if not already installed)

```bash
# Windows (PowerShell)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. Navigate to Backend Folder

```bash
cd database
```

### 3. Initialize UV Project (first time only)

```bash
uv init
```

### 4. Install Dependencies

```bash
uv add cohere qdrant-client httpx beautifulsoup4 lxml python-dotenv
```

### 5. Configure Environment Variables

Create a `.env` file in the `database/` folder:

```bash
# database/.env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster-id.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
```

**Getting Your API Keys**:

1. **Cohere**: Go to https://dashboard.cohere.com/api-keys and create a trial key
2. **Qdrant Cloud**:
   - Create account at https://cloud.qdrant.io/
   - Create a new cluster (free tier available)
   - Copy the cluster URL and API key from the dashboard

### 6. Run the Pipeline

```bash
uv run python main.py
```

## Expected Output

```
Found 45 URLs
Processing: https://ismailabdulkareem.github.io/AI_Book/docs/intro
Processing: https://ismailabdulkareem.github.io/AI_Book/docs/module-1/chapter-1
...
Created 523 chunks
Generating embeddings (batch 1/6)...
Generating embeddings (batch 2/6)...
...
Generated 523 embeddings
Upserting to Qdrant (batch 1/6)...
...
Pipeline complete! 523 points saved to collection 'AI-Book'
```

## Verification

### Check Qdrant Dashboard

1. Go to your Qdrant Cloud dashboard
2. Select your cluster
3. Navigate to "Collections"
4. Verify "AI-Book" collection exists
5. Check point count matches expected (~500-1000)

### Test Search Query

```python
# Quick test in Python REPL
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere

load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Embed a test query
query = "What is ROS 2?"
query_embedding = co.embed(
    model="embed-english-v3.0",
    input_type="search_query",
    texts=[query]
).embeddings[0]

# Search
results = qdrant.search(
    collection_name="AI-Book",
    query_vector=query_embedding,
    limit=3
)

for result in results:
    print(f"Score: {result.score:.3f}")
    print(f"Title: {result.payload['title']}")
    print(f"Text: {result.payload['text'][:200]}...")
    print("---")
```

## Troubleshooting

### "Rate limit exceeded" error

**Cause**: Cohere trial key has 5 calls/minute limit

**Solution**: The script includes a 12-second delay between batches. If you still hit limits, increase the delay:

```python
time.sleep(15)  # Increase from 12 to 15 seconds
```

### "Connection refused" to Qdrant

**Cause**: Invalid Qdrant URL or network issue

**Solution**:
1. Verify QDRANT_URL includes port (`:6333`)
2. Check Qdrant Cloud dashboard for correct URL
3. Ensure your IP is not blocked

### "Empty content" warnings

**Cause**: Some pages may have no text content (images only, redirects)

**Solution**: These are expected and skipped automatically. Check the final count to ensure most pages were processed.

### "lxml not found" error

**Cause**: lxml installation failed (common on Windows)

**Solution**:
```bash
uv add lxml --reinstall
# Or fall back to html.parser
# Change in code: BeautifulSoup(html, 'html.parser')
```

## Re-running the Pipeline

To update embeddings after content changes:

```bash
# Full re-ingestion (replaces all data)
uv run python main.py

# The script will recreate the collection by default
```

## Next Steps

After successful ingestion:

1. Integrate with RAG chatbot backend
2. Set up scheduled re-ingestion (e.g., daily cron job)
3. Add incremental update support (check lastmod dates)
