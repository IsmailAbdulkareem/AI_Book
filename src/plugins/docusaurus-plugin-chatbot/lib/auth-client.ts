import { createAuthClient } from "better-auth/react";

/**
 * Better Auth Client Configuration
 *
 * This client connects to the Better Auth server for authentication.
 * It provides hooks and methods for:
 * - useSession() - Get current session state
 * - signIn.email() - Sign in with email/password
 * - signUp.email() - Sign up with email/password + profile
 * - signOut() - End session
 */

// Auth server URL - defaults to local dev, override in production
const AUTH_SERVER_URL =
  typeof window !== "undefined"
    ? // Browser: check for production domain
      window.location.hostname === "ismailabdulkareem.github.io"
      ? "https://ai-book-auth.onrender.com" // Production auth server
      : "http://localhost:3001" // Local development
    : "http://localhost:3001"; // SSR fallback

export const authClient = createAuthClient({
  baseURL: AUTH_SERVER_URL,
});

// Export convenience methods
export const { useSession, signIn, signUp, signOut } = authClient;

// Export the client for direct access if needed
export default authClient;
