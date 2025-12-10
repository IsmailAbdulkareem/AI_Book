# RAG Chatbot for Physical AI & Humanoid Robotics Book

This repository hosts the technical book "Physical AI & Humanoid Robotics: Bridging Digital AI with Physical Bodies," built using Docusaurus with an integrated RAG (Retrieval-Augmented Generation) chatbot that allows readers to ask questions about the book content and get AI-powered answers with citations.

## Project Structure

- `docs/`: Markdown files for book chapters and modules.
- `src/`: React components and custom CSS for Docusaurus theming, plus backend services for the RAG chatbot.
- `static/`: Static assets like images and favicons.
- `docusaurus.config.js`: Docusaurus site configuration.
- `sidebars.js`: Configuration for the documentation sidebar navigation.
- `specs/`: Specification files for the book content and project planning.
- `history/`: Prompt history records (PHRs) and architectural decision records (ADRs).
- `src/main.py` - Main FastAPI application
- `src/ai/` - AI clients (Cohere, OpenAI)
- `src/vector_store/` - Qdrant vector store client
- `src/services/` - Core services (RAG, content retrieval, etc.)
- `src/api/` - API endpoints
- `src/database/` - Database models and repositories
- `src/frontend/` - Frontend components (React chat widget)
- `src/docusaurus-plugin/` - Docusaurus plugin for chat widget integration

## Local Development

To set up and run the project locally, follow these steps:

### Prerequisites

- Node.js (v14 or higher)
- npm (v6 or higher)
- Python 3.8+
- pip

### Installation

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-org/physical-ai-robotics-book.git
    cd physical-ai-robotics-book
    ```

2.  **Install Python dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Install Node dependencies**:
    ```bash
    npm install
    ```

4.  **Set up environment variables**:
    Create a `.env` file based on `.env.example`:
    ```bash
    cp .env.example .env
    # Edit .env with your API keys and service URLs
    ```

### Running the Development Servers

You need to run both the backend and frontend servers:

1. **Start the backend server**:
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

2. **In a separate terminal, start the frontend (Docusaurus)**:
   ```bash
   npm start
   ```

This will open the site in your browser at `http://localhost:3000`, with the RAG chatbot widget available on all pages.

### Building the Site

To build a static version of the site for deployment:

```bash
npm run build
```

The built static files will be located in the `build/` directory.

## API Endpoints

- `GET /health` - Health check
- `POST /api/chat` - General Q&A chat endpoint
- `POST /api/chat/selected` - Q&A with selected text context
- `GET /api/sessions/{sessionId}` - Get chat history

## Environment Variables

Required environment variables:
- `QDRANT_URL` - QDRANT Cloud URL
- `QDRANT_API_KEY` - QDRANT API Key
- `QDRANT_COLLECTION` - QDRANT Collection Name
- `COHERE_API_KEY` - Cohere API Key
- `COHERE_EMBED_MODEL` - Cohere Embedding Model (default: embed-english-v3.0)
- `OPENAI_API_KEY` - OpenAI API Key
- `OPENAI_MODEL` - OpenAI Model (default: gpt-4o)
- `NEON_DATABASE_URL` - Neon Postgres Database URL

## Frontend Integration

The chatbot is integrated into the Docusaurus documentation site via a plugin that adds a floating chat widget to all pages. The widget allows users to:

1. Ask general questions about the book content
2. Select text on the page and ask questions about the selected text
3. View sources for the AI responses with confidence scores
4. Maintain conversation context through sessions
5. See loading states and error handling in the UI
6. Access the chat history within the same session

## Testing

To test the API functionality:

```bash
python tests/test_api.py
```

## Contributing

See `CONTRIBUTING.md` for guidelines on how to contribute to this project.
