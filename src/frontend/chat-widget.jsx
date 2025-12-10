import React, { useState, useEffect, useRef } from 'react';
import './chat-widget.css';

const ChatWidget = ({ apiUrl = 'http://localhost:8000' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Initialize session
  useEffect(() => {
    // Create a new session if one doesn't exist
    if (!sessionId) {
      const newSessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
      setSessionId(newSessionId);
    }
  }, [sessionId]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText && selectedText.length > 0 && selectedText.length < 5000) {
        setSelectedText(selectedText);
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      let response;
      if (selectedText) {
        // Use selected text endpoint
        response = await fetch(`${apiUrl}/api/chat/selected`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: inputValue,
            selected_text: selectedText,
            session_id: sessionId
          })
        });
      } else {
        // Use general chat endpoint
        response = await fetch(`${apiUrl}/api/chat`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: inputValue,
            session_id: sessionId
          })
        });
      }

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        timestamp: data.timestamp,
        sources: data.sources || [],
        confidence: data.confidence_score
      };

      setMessages(prev => [...prev, botMessage]);

      // Clear selected text after using it
      if (selectedText) {
        setSelectedText(null);
        window.getSelection().removeAllRanges();
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date().toISOString(),
        isError: true
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-title">AI Assistant</div>
            <div className="chat-controls">
              <button
                className="chat-clear-btn"
                onClick={clearChat}
                title="Clear chat"
              >
                ✕
              </button>
              <button
                className="chat-close-btn"
                onClick={toggleChat}
                title="Close chat"
              >
                −
              </button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome">
                <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book.</p>
                <p>You can ask me questions about the book content, or select text on the page and ask questions about it.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chat-message ${message.sender}-message`}
                >
                  <div className="message-content">
                    <p>{message.text}</p>
                    {message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        <details>
                          <summary>Sources ({message.sources.length})</summary>
                          <ul>
                            {message.sources.map((source, index) => (
                              <li key={index}>
                                <a
                                  href={source.url}
                                  target="_blank"
                                  rel="noopener noreferrer"
                                >
                                  {source.title || `Source ${index + 1}`}
                                </a>
                                <small>Confidence: {Math.round(source.confidence * 100)}%</small>
                              </li>
                            ))}
                          </ul>
                        </details>
                      </div>
                    )}
                    {message.confidence && message.confidence < 0.5 && (
                      <small className="low-confidence">Low confidence response</small>
                    )}
                    {message.isError && (
                      <small className="error-message">Error occurred</small>
                    )}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chat-message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            {selectedText && (
              <div className="selected-text-preview">
                <small>Selected text: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</small>
              </div>
            )}
            <div className="chat-input-container">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about the book content..."
                className="chat-input"
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="chat-send-btn"
              >
                {isLoading ? 'Sending...' : 'Send'}
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button className="chat-toggle-btn" onClick={toggleChat}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H17L14.25 20.75C14.0285 21.0382 13.7298 21.2467 13.3897 21.3481C13.0496 21.4495 12.6855 21.438 12.3437 21.3155C12.0019 21.193 11.7013 20.9659 11.4822 20.6631C11.2632 20.3603 11.1368 20.0001 11.119 19.629L10.062 15H5C4.46957 15 3.96086 14.7893 3.58579 14.4142C3.21071 14.0391 3 13.5304 3 13V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </button>
      )}
    </div>
  );
};

export default ChatWidget;