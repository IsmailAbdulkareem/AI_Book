# Docusaurus Plugin Development Skill

A reusable skill for developing and maintaining Docusaurus plugins in this project.

## Purpose
This skill provides patterns and guidelines for creating Docusaurus plugins, specifically the chatbot plugin architecture.

## Plugin Structure
```
src/plugins/docusaurus-plugin-chatbot/
├── index.js              # Plugin entry point
├── chatbot-client.js     # Client module (deprecated)
├── types.d.ts            # TypeScript definitions
├── theme/
│   └── Root.js           # Theme wrapper component
└── components/
    ├── Chatbot/
    │   ├── index.jsx     # Main wrapper
    │   ├── ChatIcon.jsx  # Floating button
    │   ├── ChatPanel.jsx # Chat interface
    │   ├── Message.jsx   # Message bubbles
    │   ├── SourceList.jsx # Citation display
    │   ├── styles.module.css
    │   └── hooks/
    │       ├── useApiClient.js
    │       └── useChatSession.js
    └── TextSelection/
        ├── index.jsx
        ├── AskAIButton.jsx
        └── useTextSelection.js
```

## Plugin Registration Pattern
```javascript
// index.js
const path = require('path');

module.exports = function pluginChatbot(context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',

    // Provide theme components
    getThemePath() {
      return path.resolve(__dirname, './theme');
    },

    // Share data with client
    contentLoaded({ actions }) {
      actions.setGlobalData({ apiUrl: options.apiUrl });
    },
  };
};
```

## Theme Root Wrapper Pattern
```jsx
// theme/Root.js
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={null}>
        {() => <YourComponent />}
      </BrowserOnly>
    </>
  );
}
```

## Key Patterns

### 1. BrowserOnly Wrapper
Always wrap browser-dependent code:
```jsx
import BrowserOnly from '@docusaurus/BrowserOnly';

<BrowserOnly fallback={null}>
  {() => <ClientOnlyComponent />}
</BrowserOnly>
```

### 2. CSS Modules
Use `.module.css` for scoped styles:
```jsx
import styles from './styles.module.css';
<div className={styles.chatPanel}>
```

### 3. Global Data Access
```jsx
import { usePluginData } from '@docusaurus/useGlobalData';
const data = usePluginData('docusaurus-plugin-chatbot');
```

### 4. Lazy Loading
```jsx
const Component = React.lazy(() => import('./Component'));
<React.Suspense fallback={null}>
  <Component />
</React.Suspense>
```

## Configuration
```javascript
// docusaurus.config.js
plugins: [
  [
    './src/plugins/docusaurus-plugin-chatbot',
    {
      apiUrl: 'https://ai-book-h6kj.onrender.com',
    },
  ],
],
```

## Testing
```bash
# Build test
npm run build

# Development server
npm run start

# Check for SSR issues
npm run build && npm run serve
```

## Common Issues

### SSR Errors
- Use BrowserOnly wrapper
- Check for `window` or `document` access
- Use dynamic imports with React.lazy

### Style Conflicts
- Use CSS Modules
- Avoid global selectors
- Use high z-index for overlays (9999+)

### Plugin Not Loading
- Check plugin path in config
- Verify getThemePath returns correct path
- Check for JavaScript errors in console
