# Feature Specification: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

**Feature Branch**: `002-rag-chatbot-book`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

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
```"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Ask General Questions About Book Content (Priority: P1)

As a reader of the Physical AI & Humanoid Robotics book, I want to ask general questions about the book content so that I can get immediate answers without having to search through pages.

**Why this priority**: This is the core functionality that provides immediate value to readers by enabling quick access to book information through natural language queries.

**Independent Test**: Can be fully tested by asking various questions about the book content and verifying that relevant answers are provided based on the book's content.

**Acceptance Scenarios**:

1. **Given** I am viewing the book with the RAG chatbot widget, **When** I type a general question about the book content, **Then** I receive an accurate answer based on the book's content.
2. **Given** I have asked a question that is covered in the book, **When** I submit the question to the chatbot, **Then** the response should include relevant citations or references to the specific parts of the book that support the answer.

---

### User Story 2 - Ask Questions About Selected Text (Priority: P2)

As a reader studying the book, I want to select specific text on a page and ask questions about that text so that I can get more detailed explanations or clarifications.

**Why this priority**: This advanced feature allows for more focused interactions with specific content, enhancing the learning experience for students and instructors.

**Independent Test**: Can be tested by selecting text on a book page, asking a question about it, and receiving an answer based only on the selected text.

**Acceptance Scenarios**:

1. **Given** I have selected text on a book page, **When** I ask a question about the selected text, **Then** the chatbot should provide an answer based only on that selected text and not broader book content.
2. **Given** I have selected a paragraph from the book, **When** I ask "What does this mean?" about the selection, **Then** the response should explain the meaning of the selected text specifically.

---

### User Story 3 - Access Chat History and Analytics (Priority: P3)

As an instructor or course designer, I want to access chat logs and analytics so that I can understand how students are interacting with the book content.

**Why this priority**: This provides valuable insights for educators and course designers to improve content and understand learning patterns.

**Independent Test**: Can be tested by reviewing stored chat logs and analytics data to verify they contain the necessary information for educational analysis.

**Acceptance Scenarios**:

1. **Given** students have interacted with the RAG chatbot, **When** an instructor accesses the analytics dashboard, **Then** they should see aggregated data about common questions and content areas of interest.
2. **Given** a user has had multiple chat sessions, **When** they review their chat history, **Then** they should be able to see their previous questions and answers.

---

### Edge Cases

- What happens when a user asks a question that is not covered in the book content?
- How does the system handle very long text selections for context-based questions?
- How does the system respond when the vector store is temporarily unavailable?
- What happens when a user asks a question that requires information from multiple sections of the book?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST provide a chat interface embedded within the Docusaurus book pages for readers to ask questions about book content
- **FR-002**: System MUST use Qdrant Cloud as the vector store to store and retrieve book content embeddings
- **FR-003**: System MUST use Cohere for generating embeddings for both book content and user queries
- **FR-004**: System MUST use OpenAI for generating answers based on retrieved context and user questions
- **FR-005**: System MUST store chat logs and analytics in Neon Serverless Postgres database
- **FR-006**: System MUST allow users to ask questions about selected text on a page and respond based only on that selection
- **FR-007**: System MUST provide general Q&A capability about the book content when no text is selected
- **FR-008**: System MUST use environment variables loaded from a `.env` file for all configuration and credentials
- **FR-009**: System MUST have a FastAPI backend that exposes endpoints for chat interactions
- **FR-010**: System MUST preserve user session context during a conversation to maintain coherence

### Key Entities *(include if feature involves data)*

- **ChatMessage**: Represents a single message in a conversation, including question, answer, timestamps, and metadata
- **UserSession**: Represents a user's chat session with conversation history and context
- **BookContent**: Represents the book's content that has been processed and stored in the vector database for retrieval
- **TextSelection**: Represents a user's selected text on a page when asking context-specific questions
- **AnalyticsRecord**: Represents aggregated data about user interactions for educational insights

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can ask questions about book content and receive relevant answers within 3 seconds of submitting their query
- **SC-002**: The system successfully answers at least 80% of questions based on the book's content with acceptable relevance
- **SC-003**: Users can ask questions about selected text and receive answers that are specifically based on that selection 95% of the time
- **SC-004**: The chat interface is available and responsive 99% of the time during normal usage hours
- **SC-005**: At least 70% of users who try the chatbot feature continue to use it for subsequent questions about the book
- **SC-006**: The system can handle 100 concurrent users asking questions without significant performance degradation