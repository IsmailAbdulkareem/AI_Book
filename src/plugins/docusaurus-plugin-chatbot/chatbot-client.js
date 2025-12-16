/**
 * Client Module: Chatbot Injection
 *
 * This module runs on the client side and injects the chatbot component
 * into the Docusaurus root. Uses React's createRoot for rendering.
 */
import React from 'react';
import { createRoot } from 'react-dom/client';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Chatbot from './components/Chatbot';
import TextSelection from './components/TextSelection';

// Inject chatbot on page load
if (typeof window !== 'undefined') {
  // Wait for DOM to be ready
  const injectChatbot = () => {
    // Check if already injected
    if (document.getElementById('chatbot-root')) {
      return;
    }

    // Create container element
    const chatbotContainer = document.createElement('div');
    chatbotContainer.id = 'chatbot-root';
    document.body.appendChild(chatbotContainer);

    // Render chatbot with BrowserOnly wrapper
    const root = createRoot(chatbotContainer);
    root.render(
      <BrowserOnly fallback={null}>
        {() => (
          <>
            <Chatbot />
            <TextSelection />
          </>
        )}
      </BrowserOnly>
    );
  };

  // Inject on DOMContentLoaded or immediately if already loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', injectChatbot);
  } else {
    injectChatbot();
  }
}

export default {};
