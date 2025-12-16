import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Lazy load the chatbot components to avoid SSR issues
const ChatbotLazy = React.lazy(() => import('../components/Chatbot'));
const TextSelectionLazy = React.lazy(() => import('../components/TextSelection'));

function ChatbotWrapper() {
  return (
    <React.Suspense fallback={null}>
      <ChatbotLazy />
      <TextSelectionLazy />
    </React.Suspense>
  );
}

// Root theme component - wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <BrowserOnly fallback={null}>
        {() => <ChatbotWrapper />}
      </BrowserOnly>
    </>
  );
}
