// API Client for frontend-backend communication
// Handles all API calls to the RAG chatbot backend

class ApiClient {
  constructor(baseUrl = 'http://localhost:3000') {
    this.baseUrl = baseUrl;
  }

  // Set a new base URL
  setBaseUrl(baseUrl) {
    this.baseUrl = baseUrl;
  }

  // Make an API request with error handling
  async request(endpoint, options = {}) {
    const url = `${this.baseUrl}${endpoint}`;

    const defaultOptions = {
      headers: {
        'Content-Type': 'application/json',
      },
    };

    const requestOptions = {
      ...defaultOptions,
      ...options,
      headers: {
        ...defaultOptions.headers,
        ...options.headers,
      },
    };

    try {
      const response = await fetch(url, requestOptions);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error(`API request failed: ${endpoint}`, error);
      throw error;
    }
  }

  // Send a general chat message
  async sendChatMessage(query, sessionId) {
    return this.request('/api/chat', {
      method: 'POST',
      body: JSON.stringify({
        query,
        session_id: sessionId
      })
    });
  }

  // Send a message with selected text context
  async sendSelectedTextMessage(query, selectedText, sessionId) {
    return this.request('/api/chat/selected', {
      method: 'POST',
      body: JSON.stringify({
        query,
        selected_text: selectedText,
        session_id: sessionId
      })
    });
  }

  // Get chat history for a session
  async getChatHistory(sessionId) {
    return this.request(`/api/sessions/${sessionId}`);
  }

  // Get health status of the API
  async getHealthStatus() {
    return this.request('/health');
  }

  // Get performance metrics
  async getPerformanceMetrics() {
    return this.request('/api/performance');
  }

  // Send analytics event
  async sendAnalyticsEvent(eventData) {
    return this.request('/api/analytics', {
      method: 'POST',
      body: JSON.stringify(eventData)
    });
  }

  // Upload content for ingestion (if needed)
  async uploadContent(contentData) {
    return this.request('/api/content/upload', {
      method: 'POST',
      body: JSON.stringify(contentData)
    });
  }

  // Validate API connectivity
  async validateConnection() {
    try {
      const health = await this.getHealthStatus();
      return {
        connected: true,
        status: health.status,
        timestamp: health.timestamp
      };
    } catch (error) {
      return {
        connected: false,
        error: error.message
      };
    }
  }
}

// Session management utilities
class SessionManager {
  constructor() {
    this.currentSessionId = this.getStoredSessionId();
  }

  // Generate a new session ID
  generateSessionId() {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  // Get stored session ID from localStorage or create new one
  getStoredSessionId() {
    const stored = localStorage.getItem('chatbot_session_id');
    if (stored) {
      return stored;
    }

    const newSessionId = this.generateSessionId();
    localStorage.setItem('chatbot_session_id', newSessionId);
    return newSessionId;
  }

  // Get the current session ID
  getCurrentSessionId() {
    if (!this.currentSessionId) {
      this.currentSessionId = this.getStoredSessionId();
    }
    return this.currentSessionId;
  }

  // Reset the current session
  resetSession() {
    const oldSessionId = this.currentSessionId;
    this.currentSessionId = this.generateSessionId();
    localStorage.setItem('chatbot_session_id', this.currentSessionId);

    return {
      oldSessionId,
      newSessionId: this.currentSessionId
    };
  }

  // Clear session data
  clearSession() {
    localStorage.removeItem('chatbot_session_id');
    this.currentSessionId = null;
  }
}

// Export the API client and session manager
export { ApiClient, SessionManager };

// Create a default instance for easy use
export const apiClient = new ApiClient();
export const sessionManager = new SessionManager();

// Utility function to initialize API client with proper configuration
export const initApiClient = (baseUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000') => {
  const client = new ApiClient(baseUrl);
  const sessionMgr = new SessionManager();

  return { client, sessionMgr };
};