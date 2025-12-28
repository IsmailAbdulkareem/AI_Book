/**
 * useChatSession Hook
 *
 * Manages chat conversation history with sessionStorage persistence.
 * Data is automatically cleared when the browser tab closes.
 */
import { useState, useEffect, useCallback } from 'react';

const STORAGE_KEY = 'rag-chatbot-session';

/**
 * Generates a unique message ID
 */
function generateMessageId() {
  return `msg-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

/**
 * Safely reads from sessionStorage
 */
function readFromStorage() {
  try {
    const stored = sessionStorage.getItem(STORAGE_KEY);
    if (stored) {
      const parsed = JSON.parse(stored);
      // Validate structure
      if (parsed && Array.isArray(parsed.messages)) {
        return parsed;
      }
    }
  } catch (err) {
    console.warn('Failed to read chat session from storage:', err);
  }
  return null;
}

/**
 * Safely writes to sessionStorage
 */
function writeToStorage(session) {
  try {
    sessionStorage.setItem(STORAGE_KEY, JSON.stringify(session));
  } catch (err) {
    console.warn('Failed to save chat session to storage:', err);
  }
}

/**
 * Creates a new empty session
 */
function createEmptySession() {
  const now = Date.now();
  return {
    messages: [],
    createdAt: now,
    lastUpdatedAt: now,
  };
}

export function useChatSession() {
  const [session, setSession] = useState(createEmptySession);
  const [isLoaded, setIsLoaded] = useState(false);

  // Load session from storage on mount (client-side only)
  useEffect(() => {
    const stored = readFromStorage();
    if (stored) {
      setSession(stored);
    }
    setIsLoaded(true);
  }, []);

  // Persist session changes to storage
  useEffect(() => {
    if (isLoaded) {
      writeToStorage(session);
    }
  }, [session, isLoaded]);

  /**
   * Add a new message to the session
   *
   * @param {Object} messageData - Message data without id and timestamp
   * @param {string} messageData.role - 'user' or 'assistant'
   * @param {string} messageData.content - Message content
   * @param {Array} [messageData.sources] - Sources for assistant messages
   * @param {string} [messageData.error] - Error message if request failed
   */
  const addMessage = useCallback((messageData) => {
    const newMessage = {
      ...messageData,
      id: generateMessageId(),
      timestamp: Date.now(),
    };

    setSession((prev) => ({
      ...prev,
      messages: [...prev.messages, newMessage],
      lastUpdatedAt: Date.now(),
    }));

    return newMessage;
  }, []);

  /**
   * Update the last assistant message (for streaming or retry)
   *
   * @param {Object} updates - Fields to update
   */
  const updateLastAssistantMessage = useCallback((updates) => {
    setSession((prev) => {
      const messages = [...prev.messages];
      // Find last assistant message
      for (let i = messages.length - 1; i >= 0; i--) {
        if (messages[i].role === 'assistant') {
          messages[i] = { ...messages[i], ...updates };
          break;
        }
      }
      return {
        ...prev,
        messages,
        lastUpdatedAt: Date.now(),
      };
    });
  }, []);

  /**
   * Clear all messages and reset session
   */
  const clearSession = useCallback(() => {
    const newSession = createEmptySession();
    setSession(newSession);
    try {
      sessionStorage.removeItem(STORAGE_KEY);
    } catch (err) {
      console.warn('Failed to clear session storage:', err);
    }
  }, []);

  /**
   * Get message count
   */
  const messageCount = session.messages.length;

  return {
    messages: session.messages,
    addMessage,
    updateLastAssistantMessage,
    clearSession,
    isLoaded,
    messageCount,
  };
}

export default useChatSession;
