/**
 * useApiClient Hook
 *
 * Handles communication with the RAG backend API.
 * Uses native fetch with AbortController for timeout handling.
 *
 * Spec 006: Extended to include user context (user_id + user_profile) in requests.
 * The user profile is used by the backend for personalization.
 */
import { useState, useCallback } from 'react';
import { usePluginData } from '@docusaurus/useGlobalData';
import { useAuth } from '../../Auth';

const DEFAULT_TIMEOUT = 30000; // 30 seconds

/**
 * Maps backend error codes to user-friendly messages
 */
const ERROR_MESSAGES = {
  400: 'Invalid request. Please try rephrasing your question.',
  401: 'Authentication required. Please sign in.',
  404: 'Service not found. Please try again later.',
  422: 'Unable to process your question. Please try rephrasing.',
  429: 'Too many requests. Please wait a moment and try again.',
  500: 'Server error. Please try again later.',
  503: 'Service temporarily unavailable. Please try again later.',
  timeout: 'Request timed out. Please try again.',
  network: 'Unable to connect to the server. Please check your connection.',
  default: 'An unexpected error occurred. Please try again.',
};

/**
 * Gets user-friendly error message from error response
 */
function getErrorMessage(error, statusCode) {
  if (error?.name === 'AbortError') {
    return ERROR_MESSAGES.timeout;
  }
  if (error?.message?.includes('Failed to fetch') || error?.message?.includes('NetworkError')) {
    return ERROR_MESSAGES.network;
  }
  return ERROR_MESSAGES[statusCode] || ERROR_MESSAGES.default;
}

export function useApiClient() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Get auth context for user_id and user_profile (Spec 006)
  const { user } = useAuth();

  // Get API URL from plugin global data
  let apiUrl = 'http://localhost:8000';
  try {
    const pluginData = usePluginData('docusaurus-plugin-chatbot');
    if (pluginData?.apiUrl) {
      apiUrl = pluginData.apiUrl;
    }
  } catch {
    // Use default if plugin data not available
  }

  /**
   * Build user profile payload from session user
   *
   * Spec 006: Parse JSON-encoded arrays and map camelCase to snake_case
   *
   * @param {Object} user - User object from Better Auth session
   * @returns {Object} User profile payload for /ask API
   */
  const buildUserProfile = useCallback((sessionUser) => {
    if (!sessionUser) return null;

    // Parse technologies from JSON string (Better Auth stores arrays as strings)
    let technologies = [];
    try {
      if (sessionUser.technologies) {
        technologies = JSON.parse(sessionUser.technologies);
      }
    } catch {
      technologies = [];
    }

    return {
      programming_level: sessionUser.programmingLevel || 'beginner',
      technologies: technologies,
      hardware_access: sessionUser.hardwareAccess || 'none',
    };
  }, []);

  /**
   * Ask a question to the RAG backend
   *
   * Spec 006: Includes user_id and user_profile for personalization
   *
   * @param {string} question - The user's question
   * @param {string} [context] - Optional selected text context
   * @param {number} [topK=5] - Number of sources to retrieve
   * @returns {Promise<Object>} The chat response
   */
  const askQuestion = useCallback(
    async (question, context, topK = 5) => {
      setIsLoading(true);
      setError(null);

      // Build user context from auth session (Spec 006)
      const userProfile = buildUserProfile(user);
      const userId = user?.id;

      // Validate user context is available
      if (!userId || !userProfile) {
        const errorMessage = ERROR_MESSAGES[401];
        setError(errorMessage);
        throw new Error(errorMessage);
      }

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), DEFAULT_TIMEOUT);

      try {
        const response = await fetch(`${apiUrl}/ask`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question,
            ...(context && { context }),
            top_k: topK,
            // Spec 006: Include user context
            user_id: userId,
            user_profile: userProfile,
          }),
          signal: controller.signal,
        });

        clearTimeout(timeoutId);

        if (!response.ok) {
          const errorData = await response.json().catch(() => ({}));
          const errorMessage = getErrorMessage(null, response.status);
          setError(errorMessage);
          throw new Error(errorData.detail || errorMessage);
        }

        const data = await response.json();
        return data;
      } catch (err) {
        clearTimeout(timeoutId);
        const errorMessage = getErrorMessage(err, null);
        setError(errorMessage);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    [apiUrl, user, buildUserProfile]
  );

  /**
   * Check if the backend API is available
   *
   * @returns {Promise<boolean>} True if healthy, false otherwise
   */
  const checkHealth = useCallback(async () => {
    try {
      const response = await fetch(`${apiUrl}/health`, {
        method: 'GET',
      });
      return response.ok;
    } catch {
      return false;
    }
  }, [apiUrl]);

  /**
   * Clear the current error state
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    askQuestion,
    checkHealth,
    isLoading,
    error,
    clearError,
  };
}

export default useApiClient;
