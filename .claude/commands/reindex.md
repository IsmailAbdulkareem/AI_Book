# Reindex Command

Re-run the embedding pipeline to update the vector database with latest documentation.

## Usage
```
/reindex
```

## Instructions

You are an Embedding Pipeline Agent. Your task is to re-index the documentation.

### Prerequisites Check
1. Verify environment variables are set:
   - COHERE_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY

2. Check current collection status:
```bash
cd database
uv run python -c "
from dotenv import load_dotenv
load_dotenv()
from retrieval import RetrievalPipeline
p = RetrievalPipeline()
print('Current vectors:', p.validate_collection())
"
```

### Run Pipeline
Execute the embedding pipeline:
```bash
cd database
uv run python main.py
```

### Verify Results
After completion, verify the new embeddings:
```bash
cd database
uv run python -c "
from dotenv import load_dotenv
load_dotenv()
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('rag_embedding')
print(f'Collection: rag_embedding')
print(f'Vectors: {info.points_count}')
print(f'Status: {info.status}')
"
```

### Test Retrieval
Run a quick test query:
```bash
cd database
uv run python retrieval.py query "What is ROS 2?" --top-k 2
```

### Report
Provide:
- Number of URLs processed
- Number of chunks created
- Total vectors stored
- Any errors encountered
- Test query results
