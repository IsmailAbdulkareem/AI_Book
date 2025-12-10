---
id: 1
title: "Integrated RAG Chatbot for Physical AI Book Spec"
stage: spec
date: "2025-12-10"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "rag-chatbot-book"
branch: "002-rag-chatbot-book"
user: "user"
command: "/sp.specify"
labels:
  - "specification"
  - "rag"
  - "chatbot"
  - "ai"
links:
  spec: "specs/002-rag-chatbot-book/spec.md"
  ticket: null
  adr: null
  pr: null
files: |
  - specs/002-rag-chatbot-book/spec.md
  - specs/002-rag-chatbot-book/checklists/requirements.md
tests: |
  - spec generation
  - quality validation
prompt: |
  /sp.specify Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

  Target audience:
  - Readers of the Physical AI & Humanoid Robotics Docusaurus book who want an interactive Q&A assistant.
  - Instructors and course designers who want a reproducible “chat with the book” pattern.
  - Developers maintaining the book who need a clear, documented RAG pipeline (ingestion + retrieval + chat UI).

  Focus:
  - **Integrated RAG Chatbot Development**: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published Docusaurus book.
  - The chatbot MUST:
    - Use **Qdrant Cloud Free Tier** as the vector store for the book’s content.
    - Use **Cohere** for embeddings of both documents and user queries.
    - Use **OpenAI Agents / ChatKit SDKs** for answer generation and chat UI integration.
    - Use **FastAPI** as the backend HTTP API.
    - Use **Neon Serverless Postgres** as the relational database for chat logs, analytics, and (optionally) user selections.
    - Use **python-dotenv** and a `.env` file for all credentials and configuration (no secrets in code).
    - Be able to answer:
      - General questions about the book’s content.
      - Questions **based only on text selected by the user** on a page.

  ---

  ## High-Level Architecture

  Comeved context.
     - Frontend may optionally use **ChatKit** (JS) for chat UI, streaming, and conversation state management.

  6. **Relational DB – Neon Serverless Postgres**
     - Stores:
       - Chat message logs (question, answer, timestamps).
       - Optional user IDs / session IDs.
       - Optional selected-text snippets and metadata (page URL, selection range).

  7. **Docusaurus frontend integration**
     - A chat widget in the book (React component):
       - Sends general questions to `/api/chat`.
       - Sends selected text + question to `/api/chat/selected`.
     - Optionally uses ChatKit on the frontend for better chat UX.

  ---

  ## Environment & Packages (Python side)

  Assumptions:
  - Python 3.10+ or 3.11+.
  - You have already created a virtual environment and installed the following packages manually.

  Required Python packages (backend + ingestion):

  - Web framework:
    - `fastapi`
    - `uvicorn[standard]` (ASGI server)

  - Vector DB:
    - `qdrant-client`

  - Embeddings:
    - `cohere`

  - LLM / Agents:
    - `openai` (official OpenAI Python SDK)

  - Config & parsing:
    - `python-dotenv` (to load `.env`)
    - `python-frontmatter` (for Markdown frontmatter)
    - `pyyaml` (frontmatter parsing support)

  - Database (Neon Postgres):
    - A Postgres driver such as:
      - `psycopg[binary]` (recommended) **or**
      - `psycopg2-binary`
    - OPTIONAL: `sqlalchemy` for ORM, if desired.

  > NOTE: This spec assumes all these packages are already installed in your virtualenv via `pip install ...`. No package may be assumed globally.

  ---

  ## .env and Secrets Management

  All credentials and environment configuration MUST be stored in a `.env` file (which is `.gitignore`d) and loaded via `python-dotenv`.

  ### Required environment variables

  Example `.env` structure (values are placeholders; real keys must NOT appear in code or repo):

  ```env
  # Qdrant
  QDRANT_URL=https://<your-qdrant-endpoint>.cloud.qdrant.io
  QDRANT_API_KEY=your_qdrant_api_key_here
  QDRANT_COLLECTION=physical_ai_book

  # Cohere
  COHERE_API_KEY=your_cohere_api_key_here
  COHERE_EMBED_MODEL=embed-english-v3.0  # or chosen model

  # OpenAI
  OPENAI_API_KEY=your_openai_api_key_here
  OPENAI_MODEL=gpt-4o-mini  # or chosen GPT model
  OPENAI_ASSISTANT_ID=optional_assistant_id_if_used

  # Neon Postgres
  NEON_DATABASE_URL=postgresql://user:password@host/dbname  # from Neon dashboard
  ```
response: |
  Created specification for Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book with user stories, functional requirements, and success criteria. Created branch 002-rag-chatbot-book and validated against quality checklist.
outcome: |
  Specification successfully created and validated. Ready for planning phase.
evaluation: |
  The specification meets all quality criteria and addresses the user's requirements for an integrated RAG chatbot. All mandatory sections are completed with clear user scenarios, testable requirements, and measurable success criteria.
---