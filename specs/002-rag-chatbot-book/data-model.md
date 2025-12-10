# Data Model: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Book

**Feature**: 002-rag-chatbot-book
**Created**: 2025-12-10
**Status**: Draft

## Entity Definitions

### ChatMessage

Represents a single message in a conversation between user and system.

**Fields**:
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key to UserSession)
- `role`: String (enum: "user", "assistant")
- `content`: Text (the actual message content)
- `timestamp`: DateTime (when message was created)
- `metadata`: JSON (additional context like page_url, selected_text)
- `source_chunks`: Array of UUIDs (references to BookContent chunks used in response)

**Validation**:
- `role` must be one of allowed values
- `content` length between 1 and 10000 characters
- `timestamp` defaults to current time

### UserSession

Represents a user's chat session with conversation history and context.

**Fields**:
- `id`: UUID (Primary Key)
- `user_id`: UUID (nullable, for registered users)
- `session_token`: String (for anonymous users)
- `created_at`: DateTime
- `updated_at`: DateTime
- `is_active`: Boolean (default: true)
- `metadata`: JSON (user preferences, current page, etc.)

**Validation**:
- Either `user_id` or `session_token` must be present
- `session_token` is unique for active sessions

### BookContent

Represents the book's content that has been processed and stored in the vector database for retrieval.

**Fields**:
- `id`: UUID (Primary Key)
- `section_id`: String (identifier for the section/chapter)
- `title`: String (title of the section)
- `content`: Text (the actual content)
- `page_url`: String (URL of the page in the book)
- `content_type`: String (enum: "chapter", "section", "subsection", "figure", "table")
- `embedding_vector`: Array of Float (Cohere embedding vector)
- `metadata`: JSON (additional information like tags, difficulty level)

**Validation**:
- `embedding_vector` must have exactly 1024 elements
- `content_type` must be one of allowed values
- `page_url` must be a valid URL

### TextSelection

Represents a user's selected text on a page when asking context-specific questions.

**Fields**:
- `id`: UUID (Primary Key)
- `session_id`: UUID (Foreign Key to UserSession)
- `selected_text`: Text (the actual selected text)
- `page_url`: String (URL of the page where text was selected)
- `selection_bounds`: JSON (start/end positions, coordinates)
- `timestamp`: DateTime
- `query_id`: UUID (Foreign Key to ChatMessage that used this selection)

**Validation**:
- `selected_text` length between 10 and 5000 characters
- `page_url` must be a valid URL

### AnalyticsRecord

Represents aggregated data about user interactions for educational insights.

**Fields**:
- `id`: UUID (Primary Key)
- `record_type`: String (enum: "chat_session", "query", "engagement", "feedback")
- `user_id`: UUID (nullable)
- `session_id`: UUID (nullable)
- `action`: String (the specific action taken)
- `target_id`: UUID (ID of the target entity, e.g., ChatMessage ID)
- `metadata`: JSON (additional context like timing, success metrics)
- `timestamp`: DateTime

**Validation**:
- `record_type` must be one of allowed values
- Either `user_id` or `session_id` must be present

## Relationships

- `UserSession` 1-to-many `ChatMessage`
- `UserSession` 1-to-many `TextSelection`
- `ChatMessage` many-to-many `BookContent` (via source_chunks)
- `TextSelection` 1-to-1 `ChatMessage` (via query_id)
- Various entities to `AnalyticsRecord` (for tracking interactions)

## State Transitions

### UserSession States
- `active`: Session is currently in use
- `paused`: Session inactive but preserved
- `completed`: Session finished and archived

### ChatMessage States
- `pending`: Message sent, waiting for response
- `processing`: System is generating response
- `completed`: Response generated and returned
- `error`: Error occurred during processing

## Indexing Strategy

### Database Indexes
- `UserSession.session_token` (for quick session lookup)
- `ChatMessage.session_id` and `timestamp` (for chronological message retrieval)
- `BookContent.section_id` (for content organization)
- `AnalyticsRecord.record_type` and `timestamp` (for analytics queries)

### Vector Store Indexes
- `BookContent.embedding_vector` (for semantic search)
- `BookContent.page_url` (for content retrieval by location)

## API Contract Schema

### Chat Endpoints

```
POST /api/chat
Request:
{
  "message": "What is physical AI?",
  "session_id": "uuid",
  "selected_text": "optional selected text from page",
  "page_url": "optional URL of current page"
}

Response:
{
  "response": "Detailed answer to the question",
  "session_id": "uuid",
  "sources": [
    {
      "section_id": "chapter-3-section-2",
      "title": "Introduction to Physical AI",
      "page_url": "/docs/chapter-3/section-2",
      "relevance_score": 0.85
    }
  ],
  "timestamp": "2025-12-10T10:30:00Z"
}
```

### Content Management Endpoints

```
GET /api/content/search
Request:
{
  "query": "search terms",
  "filters": {
    "content_type": ["chapter", "section"],
    "tags": ["advanced", "intro"]
  },
  "limit": 10
}

Response:
{
  "results": [
    {
      "id": "uuid",
      "title": "Section Title",
      "content_preview": "First 200 characters...",
      "page_url": "/docs/path",
      "relevance_score": 0.92
    }
  ]
}
```

## Data Validation Rules

1. All user-generated content must be sanitized to prevent XSS
2. Embedding vectors must be normalized to unit length
3. Session tokens must expire after 24 hours of inactivity
4. Chat messages older than 30 days may be archived
5. All external URLs must be validated before storage

## Migration Considerations

- Existing chat logs should be preserved during schema updates
- Content embeddings may need regeneration when embedding models update
- User sessions should maintain continuity across deployments
- Analytics data should maintain historical consistency