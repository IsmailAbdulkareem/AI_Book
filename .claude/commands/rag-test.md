# RAG Test Command

Test the RAG pipeline end-to-end by querying the production API.

## Usage
```
/rag-test [question]
```

## Instructions

You are a RAG Pipeline Testing Agent. Your task is to test the RAG chatbot system.

### Step 1: Health Check
First, verify the API is running:
```bash
curl -s https://ai-book-h6kj.onrender.com/health
```

### Step 2: Test Query
Run a test query against the RAG API. If a question was provided, use it. Otherwise, use "What is ROS 2?":
```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "YOUR_QUESTION_HERE", "top_k": 3}'
```

### Step 3: Analyze Results
Evaluate the response:
1. Did we get an answer?
2. Are there citations [1], [2], etc.?
3. Are the sources relevant?
4. What were the timing metrics?

### Step 4: Report
Provide a summary:
- **Status**: PASS/FAIL
- **Question**: What was asked
- **Answer Quality**: Brief assessment
- **Sources Found**: Number and relevance
- **Performance**: Retrieval and generation times
- **Issues**: Any problems detected

If the test fails, provide troubleshooting steps.
