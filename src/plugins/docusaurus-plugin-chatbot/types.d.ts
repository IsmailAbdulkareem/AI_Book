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

// ============================================================================
// Authentication Types (Better Auth)
// ============================================================================

// Programming level enum
export type ProgrammingLevel = "beginner" | "intermediate" | "advanced";

// Hardware access level enum
export type HardwareAccess = "none" | "simulator_only" | "real_robots";

// Technology options
export type TechnologyOption = "Python" | "JS" | "ROS2" | "AI/ML" | "Web" | "Other";

// Device options
export type DeviceOption = "Jetson" | "Raspberry Pi" | "Arduino" | "GPU" | "Other";

// User profile stored in Better Auth additionalFields
export interface UserProfile {
  programmingLevel: ProgrammingLevel;
  technologies: string; // JSON string array: '["Python", "ROS2"]'
  aiRoboticsExperience: boolean;
  hardwareAccess: HardwareAccess;
  devicesOwned?: string; // JSON string array (optional)
}

// User from Better Auth session
export interface AuthUser {
  id: string;
  name: string;
  email: string;
  emailVerified: boolean;
  image?: string;
  createdAt: Date;
  updatedAt: Date;
  // Profile fields from additionalFields
  programmingLevel: ProgrammingLevel;
  technologies: string; // JSON string
  aiRoboticsExperience: boolean;
  hardwareAccess: HardwareAccess;
  devicesOwned?: string; // JSON string
}

// Auth session from Better Auth
export interface AuthSession {
  user: AuthUser;
  session: {
    id: string;
    userId: string;
    expiresAt: Date;
  };
}

// Auth context state
export interface AuthContextType {
  session: AuthSession | null;
  user: AuthUser | null;
  isPending: boolean;
  error: Error | null;
  isAuthenticated: boolean;
  isProfileComplete: boolean;
}

// User profile for /ask API (decoded from session)
export interface UserProfilePayload {
  programming_level: ProgrammingLevel;
  technologies: string[]; // Decoded array
  hardware_access: HardwareAccess;
}

// Extended ChatRequest with user context
export interface AuthenticatedChatRequest extends ChatRequest {
  user_id: string;
  user_profile: UserProfilePayload;
}
