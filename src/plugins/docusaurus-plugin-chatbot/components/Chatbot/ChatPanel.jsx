/**
 * ChatPanel Component
 *
 * Main chat interface containing message list, input area, and controls.
 * Handles user input, API calls, and displays conversation history.
 */
import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';
import Message from './Message';
import { useApiClient } from './hooks/useApiClient';
import { useChatSession } from './hooks/useChatSession';

const EXAMPLE_QUESTIONS = [
  'What is ROS 2?',
  'How do I create a robot model with URDF?',
  'What is the difference between Gazebo and Isaac Sim?',
];

/**
 * Loading indicator with animated dots
 */
function LoadingIndicator() {
  return (
    <div className={styles.loadingIndicator} aria-live="polite">
      <div className={styles.loadingDots}>
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
        <span className={styles.loadingDot}></span>
      </div>
      <span>Thinking...</span>
    </div>
  );
}

/**
 * Welcome screen shown when chat is empty
 */
function WelcomeScreen({ onQuestionClick }) {
  return (
    <div className={styles.welcomeContainer}>
      <h3 className={styles.welcomeTitle}>Ask me anything about the book!</h3>
      <p className={styles.welcomeText}>
        I can help you understand Physical AI, ROS 2, robotics simulation, and more.
      </p>
      <div className={styles.exampleQuestions}>
        {EXAMPLE_QUESTIONS.map((question, index) => (
          <button
            key={index}
            className={styles.exampleQuestion}
            onClick={() => onQuestionClick(question)}
            type="button"
          >
            {question}
          </button>
        ))}
      </div>
    </div>
  );
}

/**
 * Context preview for text selection
 */
function ContextPreview({ context, onClear }) {
  if (!context) return null;

  const truncated = context.length > 150 ? context.slice(0, 150) + '...' : context;

  return (
    <div className={styles.contextPreview}>
      <div className={styles.contextLabel}>
        Selected text context:
        <button
          onClick={onClear}
          style={{
            marginLeft: '8px',
            background: 'none',
            border: 'none',
            cursor: 'pointer',
            color: 'inherit',
            textDecoration: 'underline',
          }}
          type="button"
        >
          Clear
        </button>
      </div>
      <div className={styles.contextText}>{truncated}</div>
    </div>
  );
}

export function ChatPanel({ onClose, selectedContext, onClearContext }) {
  const [inputValue, setInputValue] = useState('');
  const [lastFailedQuestion, setLastFailedQuestion] = useState(null);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const { messages, addMessage, isLoaded } = useChatSession();
  const { askQuestion, isLoading, error, clearError } = useApiClient();

  // Scroll to bottom when messages change
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Focus input on mount
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  // Handle keyboard shortcuts for panel (Escape only)
  const handlePanelKeyDown = useCallback(
    (e) => {
      if (e.key === 'Escape') {
        onClose();
      }
    },
    [onClose]
  );

  // Submit question to API
  const handleSubmit = useCallback(async () => {
    const question = inputValue.trim();
    if (!question || isLoading) return;

    // Clear input and error state
    setInputValue('');
    clearError();
    setLastFailedQuestion(null);

    // Add user message
    addMessage({
      role: 'user',
      content: question,
    });

    try {
      // Call API with optional context
      const response = await askQuestion(question, selectedContext);

      // Add assistant message with sources
      addMessage({
        role: 'assistant',
        content: response.answer,
        sources: response.sources,
        timing: {
          retrieval_time_ms: response.retrieval_time_ms,
          generation_time_ms: response.generation_time_ms,
        },
      });

      // Clear context after successful submission
      if (selectedContext && onClearContext) {
        onClearContext();
      }
    } catch (err) {
      // Store failed question for retry
      setLastFailedQuestion(question);

      // Add error message
      addMessage({
        role: 'assistant',
        content: '',
        error: error || 'Failed to get response. Please try again.',
      });
    }
  }, [inputValue, isLoading, askQuestion, addMessage, selectedContext, onClearContext, clearError, error]);

  // Handle keyboard shortcuts for input (Enter to submit)
  // Must be defined after handleSubmit
  const handleInputKeyDown = useCallback(
    (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        handleSubmit();
      }
    },
    [handleSubmit]
  );

  // Handle example question click
  const handleExampleClick = useCallback((question) => {
    setInputValue(question);
    // Submit after a brief delay to show the input
    setTimeout(() => {
      inputRef.current?.focus();
    }, 100);
  }, []);

  // Handle retry
  const handleRetry = useCallback(() => {
    if (lastFailedQuestion) {
      setInputValue(lastFailedQuestion);
      setLastFailedQuestion(null);
      clearError();
    }
  }, [lastFailedQuestion, clearError]);

  const isEmpty = messages.length === 0 && !isLoading;
  const canSubmit = inputValue.trim().length > 0 && !isLoading;

  return (
    <div
      className={styles.chatPanel}
      role="dialog"
      aria-label="Chat with AI assistant"
      onKeyDown={handlePanelKeyDown}
    >
      {/* Header */}
      <div className={styles.panelHeader}>
        <h2 className={styles.panelTitle}>Ask about the book</h2>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chat panel"
          type="button"
        >
          <span aria-hidden="true">×</span>
        </button>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer} role="log" aria-live="polite">
        {isEmpty && !selectedContext ? (
          <WelcomeScreen onQuestionClick={handleExampleClick} />
        ) : (
          <>
            {messages.map((msg, index) => (
              <Message key={msg.id || index} message={msg} />
            ))}
          </>
        )}

        {isLoading && <LoadingIndicator />}

        {/* Error with retry */}
        {error && lastFailedQuestion && (
          <div className={styles.errorContainer}>
            <div className={styles.errorText}>{error}</div>
            <button className={styles.retryButton} onClick={handleRetry} type="button">
              Retry
            </button>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input Area */}
      <div className={styles.inputArea}>
        <ContextPreview context={selectedContext} onClear={onClearContext} />
        <div className={styles.inputWrapper}>
          <textarea
            ref={inputRef}
            className={styles.textInput}
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={handleInputKeyDown}
            placeholder="Ask a question..."
            aria-label="Type your question"
            rows={1}
            disabled={isLoading}
          />
          <button
            className={styles.submitButton}
            onClick={handleSubmit}
            disabled={!canSubmit}
            aria-label="Send message"
            type="button"
          >
            Send
          </button>
        </div>
      </div>
    </div>
  );
}

export default ChatPanel;
