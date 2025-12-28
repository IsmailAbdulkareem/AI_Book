# Physical AI & Humanoid Robotics Book

This repository hosts the technical book "Physical AI & Humanoid Robotics: Bridging Digital AI with Physical Bodies," built using Docusaurus with an integrated RAG (Retrieval-Augmented Generation) chatbot that allows readers to ask questions about the book content and get AI-powered answers with citations.

## Project Structure

This is a monorepo organized into frontend, backend, and shared components:

```
hackathon-01-physical-ai-robotics/
├── frontend/              # Docusaurus documentation website
│   ├── docs/             # Markdown book chapters
│   ├── src/              # React components, pages, plugins
│   ├── static/           # Images, favicons, assets
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   └── package.json
│
├── backend/              # Python FastAPI RAG service
│   ├── app.py           # Main FastAPI application
│   ├── agent.py         # AI agent logic
│   ├── chatbot.py       # Chatbot service
│   ├── main.py          # Entry point
│   ├── models.py        # Data models
│   ├── pyproject.toml   # Python dependencies
│   └── requirements.txt
│
├── auth-server/         # Authentication service
│
├── specs/               # Feature specifications
├── history/             # PHRs and ADRs
├── .specify/            # SpecKit templates
└── .github/             # CI/CD workflows
```

### Frontend (Docusaurus)
Located in `frontend/`:
- `docs/` - Markdown files for book chapters and modules
- `src/` - React components and custom CSS for Docusaurus theming
- `src/components/` - React components including chat widget
- `src/css/` - Custom styling
- `static/` - Static assets like images and favicons
- `docusaurus.config.js` - Docusaurus site configuration
- `sidebars.js` - Configuration for the documentation sidebar navigation

### Backend (RAG Chatbot)
Located in `backend/`:
- `app.py` - Main FastAPI application
- `agent.py` - AI agent logic and orchestration
- `chatbot.py` - Chatbot service with Cohere/OpenAI integration
- `models.py` - Database models and repositories
- `main.py` - Application entry point

### Project Management
- `specs/` - Specification files for the book content and project planning
- `history/` - Prompt history records (PHRs) and architectural decision records (ADRs)
- `.specify/` - SpecKit Plus templates and scripts for spec-driven development

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
    git clone https://github.com/IsmailAbdulkareem/AI_Book.git
    cd hackathon-01-physical-ai-robotics
    ```

2.  **Install frontend dependencies**:
    ```bash
    cd frontend
    npm install
    ```

3.  **Install backend dependencies**:
    ```bash
    cd ../backend
    pip install -r requirements.txt
    # Or using uv:
    uv sync
    ```

4.  **Set up environment variables**:
    Create `.env` files for both frontend and backend based on `.env.example`:
    ```bash
    # Root level
    cp .env.example .env

    # Backend
    cp backend/.env.example backend/.env
    # Edit with your API keys (Cohere, OpenAI, Qdrant, Neon DB)
    ```

### Running the Development Servers

You need to run both the backend and frontend servers:

1. **Start the backend server** (from project root):
   ```bash
   cd backend
   uv run python main.py
   # Or: uvicorn main:app --reload --port 8000
   ```

2. **In a separate terminal, start the frontend** (from project root):
   ```bash
   cd frontend
   npm start
   ```

This will open the site in your browser at `http://localhost:3000`, with the RAG chatbot widget available on all pages.

### Building the Frontend for Production

To build a static version of the site for deployment:

```bash
cd frontend
npm run build
```

The built static files will be located in the `frontend/build/` directory.

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
