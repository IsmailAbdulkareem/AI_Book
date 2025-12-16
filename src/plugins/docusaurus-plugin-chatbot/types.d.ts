/**
 * TypeScript Type Definitions for RAG Chatbot Plugin
 */

// API Response Types
export interface Source {
  url: string;
  content: string;
}

export interface ChatResponse {
  question: string;
  answer: string;
  sources: Source[];
  retrieval_time_ms: number;
  generation_time_ms: number;
}

export interface ChatRequest {
  question: string;
  context?: string;
  top_k?: number;
}

export interface ErrorResponse {
  detail: string;
  status_code?: number;
}

// Chat Session Types
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  timestamp: number;
  error?: string;
}

export interface ChatSession {
  messages: ChatMessage[];
  createdAt: number;
  lastUpdatedAt: number;
}

// Text Selection Types
export interface SelectionPosition {
  x: number;
  y: number;
}

export interface TextSelection {
  text: string;
  truncatedText: string;
  position: SelectionPosition;
  isWithinContentArea: boolean;
}

// UI State Types
export type ChatPanelState = 'closed' | 'open' | 'loading' | 'error';

export interface ChatUIState {
  panelState: ChatPanelState;
  inputValue: string;
  selectedContext?: string;
}

// Plugin Configuration
export interface PluginConfig {
  apiUrl: string;
}

// Hook Return Types
export interface UseApiClientReturn {
  askQuestion: (question: string, context?: string, topK?: number) => Promise<ChatResponse>;
  checkHealth: () => Promise<boolean>;
  isLoading: boolean;
  error: string | null;
}

export interface UseChatSessionReturn {
  messages: ChatMessage[];
  addMessage: (message: Omit<ChatMessage, 'id' | 'timestamp'>) => void;
  clearSession: () => void;
  isLoaded: boolean;
}

export interface UseTextSelectionReturn {
  selection: TextSelection | null;
  clearSelection: () => void;
}
