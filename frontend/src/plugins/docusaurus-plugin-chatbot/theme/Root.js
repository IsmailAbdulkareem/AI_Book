import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { AuthProvider } from '../components/Auth';

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
// AuthProvider wraps entire app so navbar can access auth context
export default function Root({ children }) {
  return (
    <BrowserOnly fallback={<>{children}</>}>
      {() => (
        <AuthProvider>
          {children}
          <ChatbotWrapper />
        </AuthProvider>
      )}
    </BrowserOnly>
  );
}
