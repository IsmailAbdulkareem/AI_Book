import React, { createContext, useContext, ReactNode } from "react";
import { useSession } from "../../lib/auth-client";

/**
 * Auth Context Type
 *
 * Provides authentication state to child components.
 */
interface AuthContextType {
  session: any;
  user: any;
  isPending: boolean;
  error: Error | null;
  isAuthenticated: boolean;
  isProfileComplete: boolean;
}

const AuthContext = createContext<AuthContextType | null>(null);

/**
 * Check if user profile is complete
 *
 * A profile is complete when all required fields are filled:
 * - programmingLevel
 * - technologies (non-empty array)
 * - aiRoboticsExperience (boolean)
 * - hardwareAccess
 */
export function isProfileComplete(user: any): boolean {
  if (!user) return false;

  try {
    // Check required string fields exist and are non-empty
    if (!user.programmingLevel) return false;
    if (!user.hardwareAccess) return false;

    // Check technologies is a valid JSON array with at least one item
    if (!user.technologies) return false;
    const techs = JSON.parse(user.technologies);
    if (!Array.isArray(techs) || techs.length === 0) return false;

    // Check aiRoboticsExperience is explicitly set (boolean)
    if (typeof user.aiRoboticsExperience !== "boolean") return false;

    return true;
  } catch {
    return false;
  }
}

/**
 * AuthProvider Component
 *
 * Wraps the application to provide authentication context.
 * Uses Better Auth's useSession hook to manage session state.
 */
export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: sessionData, isPending, error } = useSession();

  const session = sessionData || null;
  const user = session?.user || null;
  const isAuthenticated = !!user;
  const profileComplete = isProfileComplete(user);

  const value: AuthContextType = {
    session,
    user,
    isPending,
    error: error || null,
    isAuthenticated,
    isProfileComplete: profileComplete,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * useAuth Hook
 *
 * Access authentication context from any component.
 * Must be used within AuthProvider.
 */
export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error("useAuth must be used within AuthProvider");
  }
  return context;
}

export default AuthProvider;
