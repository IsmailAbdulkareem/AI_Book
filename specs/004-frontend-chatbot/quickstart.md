# Quickstart: Frontend RAG Chatbot

**Feature Branch**: `004-frontend-chatbot`
**Prerequisites**: Spec 3 (RAG Agent API) must be running

---

## Quick Start (2 minutes)

### 1. Ensure Backend is Running

```bash
# Terminal 1: Start RAG Agent API (from database/ directory)
cd database
uv run uvicorn agent:app --host 127.0.0.1 --port 8000

# Verify it's running
curl http://localhost:8000/health
# Should return: {"status": "healthy", ...}
```

### 2. Start Docusaurus Development Server

```bash
# Terminal 2: Start frontend (from project root)
npm start
# Opens http://localhost:3000/AI_Book/
```

### 3. Use the Chatbot

1. **Open Chatbot**: Click the chat icon in the bottom-right corner
2. **Ask a Question**: Type "What is ROS 2?" and press Enter
3. **View Sources**: Click on source citations to navigate to book sections

---

## Feature Walkthrough

### Floating Chatbot (US1)

The chatbot icon appears on every page in the bottom-right corner.

**Try it**:
```
1. Navigate to any page in the book
2. Click the chat icon (💬)
3. Panel opens without page reload
4. Type: "What is URDF?"
5. Press Enter or click Send
6. View answer with [1], [2] citations
7. Click a citation to navigate to source
8. Press Escape or click X to close
```

### Text Selection "Ask AI" (US2)

Select text anywhere in the book content to ask about it.

**Try it**:
```
1. Go to any documentation page
2. Highlight/select some text (e.g., a definition)
3. "Ask AI" button appears near selection
4. Click "Ask AI"
5. Chatbot opens with selected text as context
6. Add your question or press Enter to ask about the selection
```

### Error Handling (US3)

The chatbot gracefully handles errors and edge cases.

**Test scenarios**:
```bash
# Test 1: Backend unavailable
# Stop the backend server and try to ask a question
# Expected: "Service temporarily unavailable" message with retry button

# Test 2: Empty chat welcome state
# Open chatbot without any messages
# Expected: Welcome message with example questions

# Test 3: Network timeout
# Configure slow network in browser DevTools
# Expected: Timeout message after 30 seconds
```

---

## Configuration

### API Base URL

The chatbot connects to the RAG Agent API. Configure the URL:

**Development** (default):
```
http://localhost:8000
```

**Custom URL** (via environment):
```bash
# In .env or environment
RAG_API_URL=https://your-api.example.com
```

### Docusaurus Configuration

The chatbot is registered in `docusaurus.config.js`:

```javascript
// Already configured - no changes needed
themeConfig: {
  // Chatbot auto-enabled via src/theme/Root.tsx
}
```

---

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| Enter | Submit question |
| Shift+Enter | New line in input |
| Escape | Close chat panel |
| Tab | Navigate between elements |

---

## Mobile Usage

The chatbot is fully responsive:

- **Small screens (< 768px)**: Chat panel opens full-width
- **Touch devices**: Tap icon to open, swipe down to close
- **Text selection**: Long-press to select, "Ask AI" appears above selection

---

## Troubleshooting

### Chatbot icon doesn't appear

1. Check browser console for JavaScript errors
2. Ensure JavaScript is enabled
3. Verify `src/theme/Root.tsx` exists and is valid

### "Service unavailable" error

1. Check backend is running: `curl http://localhost:8000/health`
2. Verify CORS is configured (check browser console for CORS errors)
3. Ensure correct API URL is configured

### Text selection "Ask AI" doesn't appear

1. Selection must be within book content area (not nav/header)
2. Selection must contain at least 1 character
3. Check browser console for errors

### Slow responses

1. Check backend health: `curl http://localhost:8000/health`
2. First request may be slower (model loading)
3. Typical response time: 5-15 seconds

---

## Example Questions

Try these questions to test the chatbot:

```
"What is ROS 2 and why is it important for robotics?"
"How do I create a URDF file for a robot?"
"What are the differences between Gazebo and Isaac Sim?"
"Explain the concept of physical AI"
"What hardware do I need for humanoid robotics?"
```

---

## Development Tips

### Hot Reload

Both frontend and backend support hot reload:
- Frontend: Changes to React components auto-refresh
- Backend: Use `--reload` flag with uvicorn

### Debug Mode

Enable verbose logging in browser console:
```javascript
localStorage.setItem('chatbot_debug', 'true');
// Refresh page to see detailed logs
```

### Testing Changes

```bash
# Run Docusaurus build to catch errors
npm run build

# Check for TypeScript errors
npm run typecheck

# Run linter
npm run lint
```
