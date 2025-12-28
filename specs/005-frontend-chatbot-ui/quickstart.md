# Quickstart: Frontend RAG Chatbot UI

**Feature**: 005-frontend-chatbot-ui
**Date**: 2025-12-17
**Prerequisites**: Node.js 18+, npm, Backend API running (Spec 3)

---

## Overview

This guide covers setting up the chatbot UI plugin for the Docusaurus book website.

---

## Prerequisites Check

```bash
# Check Node.js version (18+ required)
node --version

# Check npm version
npm --version

# Verify Docusaurus site builds
cd /path/to/ai-native
npm run build

# Verify backend is running (from Spec 3)
curl http://localhost:8000/health
```

---

## Quick Setup (5 minutes)

### 1. Enable the Plugin

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config

  plugins: [
    [
      './src/plugins/docusaurus-plugin-chatbot',
      {
        apiUrl: process.env.RAG_CHATBOT_API_URL || 'http://localhost:8000',
        enabled: true,
        position: 'bottom-right',
      }
    ]
  ],
};
```

### 2. Start Development Server

```bash
# Start the Docusaurus dev server
npm run start

# In another terminal, ensure backend is running
cd database
python agent.py
```

### 3. Verify Installation

1. Open http://localhost:3000
2. Look for the chat icon in the bottom-right corner
3. Click the icon to open the chat panel
4. Ask: "What is ROS 2?"
5. Verify you receive an answer with source citations

---

## Plugin Structure

```
src/
└── plugins/
    └── docusaurus-plugin-chatbot/
        ├── index.js              # Plugin entry point
        ├── chatbot-client.js     # Client module registration
        └── components/
            ├── Chatbot/
            │   ├── index.jsx      # Main component
            │   ├── ChatPanel.jsx  # Chat UI panel
            │   ├── ChatIcon.jsx   # Floating icon
            │   ├── Message.jsx    # Message rendering
            │   └── styles.css     # Component styles
            └── TextSelection/
                ├── index.jsx      # Selection detector
                └── AskAIButton.jsx # Floating button
```

---

## Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `apiUrl` | string | `'http://localhost:8000'` | Backend API URL |
| `enabled` | boolean | `true` | Enable/disable the chatbot |
| `position` | string | `'bottom-right'` | Icon position (`'bottom-right'` or `'bottom-left'`) |
| `welcomeMessage` | string | (built-in) | Custom welcome message |
| `exampleQuestions` | string[] | (built-in) | Custom example questions |

### Example with All Options

```javascript
{
  apiUrl: 'https://api.example.com',
  enabled: process.env.NODE_ENV === 'production',
  position: 'bottom-right',
  welcomeMessage: 'Ask me anything about Physical AI!',
  exampleQuestions: [
    'What is ROS 2?',
    'How does Isaac Sim work?',
    'Explain VLA models'
  ]
}
```

---

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `RAG_CHATBOT_API_URL` | Backend API URL | `http://localhost:8000` |
| `RAG_CHATBOT_ENABLED` | Enable/disable at runtime | `true` |

### .env Example

```bash
# .env.local (for development)
RAG_CHATBOT_API_URL=http://localhost:8000
RAG_CHATBOT_ENABLED=true

# .env.production
RAG_CHATBOT_API_URL=https://your-api-domain.com
RAG_CHATBOT_ENABLED=true
```

---

## Testing the Integration

### Manual Testing Checklist

- [ ] Chat icon appears on all pages
- [ ] Click icon opens panel without page reload
- [ ] Submit question shows loading state
- [ ] Answer renders with `[N]` as clickable links
- [ ] Sources section shows below answer
- [ ] Close button/Escape key closes panel
- [ ] Text selection in content shows "Ask AI" button
- [ ] "Ask AI" opens chat with selected text as context
- [ ] Error states display user-friendly messages
- [ ] Mobile responsive (test at 320px width)

### Automated Testing (Optional)

```bash
# Run component tests
npm run test -- --testPathPattern=chatbot

# Run E2E tests (if configured)
npm run test:e2e -- --grep "chatbot"
```

---

## Troubleshooting

### Chat icon doesn't appear

1. Check plugin is enabled in `docusaurus.config.js`
2. Clear Docusaurus cache: `npm run clear`
3. Check browser console for errors
4. Verify JavaScript is enabled

### API errors

1. Check backend is running: `curl http://localhost:8000/health`
2. Verify CORS allows your origin
3. Check network tab for actual error response
4. Verify `apiUrl` configuration matches backend

### Text selection "Ask AI" doesn't appear

1. Ensure selection is within main content area
2. Check browser console for errors
3. Verify selection event listeners are attached

### Build failures

1. Run `npm run clear` before building
2. Check for syntax errors in plugin files
3. Ensure all imports use correct paths

---

## Development Workflow

### Making Changes

```bash
# Start dev server with hot reload
npm run start

# Plugin changes may require restart
# Ctrl+C, then npm run start

# Clear cache if changes don't appear
npm run clear && npm run start
```

### Testing Production Build

```bash
# Build static site
npm run build

# Serve locally
npm run serve

# Test at http://localhost:3000
```

---

## Deployment Notes

### GitHub Pages

The chatbot works with GitHub Pages static hosting. Ensure:
1. Backend API is deployed and accessible
2. `apiUrl` points to production backend
3. CORS allows `https://your-username.github.io`

### Environment-Specific Config

```javascript
// docusaurus.config.js
const isProd = process.env.NODE_ENV === 'production';

plugins: [
  [
    './src/plugins/docusaurus-plugin-chatbot',
    {
      apiUrl: isProd
        ? 'https://your-api.com'
        : 'http://localhost:8000',
      enabled: true,
    }
  ]
],
```

---

## Next Steps

After setup:
1. Run `/sp.tasks` to generate implementation tasks
2. Implement the plugin components
3. Test all user stories from the spec
4. Deploy to staging environment
5. Verify production deployment
