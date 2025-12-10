// Chat Widget Injector for Docusaurus
// This file is loaded by Docusaurus to inject the React chat widget

import React from 'react';
import ReactDOM from 'react-dom';
import ChatWidget from '@site/src/frontend/chat-widget.jsx';
import { initTextSelection } from '@site/src/frontend/text-selection.js';
import { apiClient, sessionManager } from '@site/src/frontend/api-client.js';

// Create a wrapper component that handles initialization
class ChatWidgetWrapper extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      config: {
        apiUrl: window.RAG_CHATBOT_CONFIG?.apiUrl || 'http://localhost:8000',
        enabled: window.RAG_CHATBOT_CONFIG?.enabled !== false,
        position: window.RAG_CHATBOT_CONFIG?.position || 'bottom-right',
        theme: window.RAG_CHATBOT_CONFIG?.theme || 'default'
      },
      isInitialized: false
    };
  }

  componentDidMount() {
    // Initialize the API client with the configured URL
    apiClient.setBaseUrl(this.state.config.apiUrl);

    // Initialize text selection detection
    initTextSelection((selectedText, range) => {
      // This callback is triggered when text is selected
      // The ChatWidget component will handle this internally
    });

    this.setState({ isInitialized: true });
  }

  render() {
    if (!this.state.config.enabled || !this.state.isInitialized) {
      return null;
    }

    return (
      <ChatWidget apiUrl={this.state.config.apiUrl} />
    );
  }
}

// Function to render the chat widget
function renderChatWidget() {
  const container = document.getElementById('rag-chatbot-container');
  if (container) {
    ReactDOM.render(<ChatWidgetWrapper />, container);
  }
}

// Wait for the DOM to be ready before rendering
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', renderChatWidget);
} else {
  renderChatWidget();
}

// Also render after React hydration in case of SSR
if (window.Docusaurus) {
  window.Docusaurus.onRouteUpdate = function() {
    // Re-render the widget after route changes
    setTimeout(renderChatWidget, 100);
  };
}

export default ChatWidgetWrapper;