import { betterAuth } from "better-auth";
import { Pool } from "pg";
import "dotenv/config";

/**
 * Better Auth Configuration
 *
 * This configuration:
 * - Uses PostgreSQL adapter (shared database with FastAPI backend)
 * - Enables email/password authentication
 * - Stores user profile in additionalFields (NOT separate table)
 * - Configures 7-day cookie-based sessions
 */
export const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
    ssl: {
      rejectUnauthorized: false, // Required for Neon PostgreSQL
    },
  }),

  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24,     // Update session daily
    cookieCache: {
      enabled: true,
      maxAge: 60 * 5, // 5 minutes cache
    },
  },

  // User profile fields stored in user table via additionalFields
  // Arrays are stored as JSON strings (Better Auth limitation)
  user: {
    additionalFields: {
      programmingLevel: {
        type: "string",
        required: true,
        input: true, // Allows input during signup
      },
      technologies: {
        type: "string", // JSON array as string: '["Python", "ROS2"]'
        required: true,
        input: true,
      },
      aiRoboticsExperience: {
        type: "boolean",
        required: true,
        input: true,
      },
      hardwareAccess: {
        type: "string", // "none" | "simulator_only" | "real_robots"
        required: true,
        input: true,
      },
      devicesOwned: {
        type: "string", // JSON array as string, optional
        required: false,
        input: true,
      },
    },
  },

  // Trusted origins for CORS
  trustedOrigins: [
    process.env.FRONTEND_URL || "http://localhost:3000",
    "https://ismailabdulkareem.github.io",
    "http://localhost:3000",
  ],

  // Cross-domain cookie configuration
  // Required because frontend (github.io) and backend (render.com) are different domains
  advanced: {
    defaultCookieAttributes: {
      sameSite: "none",    // Allow cross-origin cookies
      secure: true,        // Required when sameSite is "none"
      partitioned: true,   // Required by modern browsers for third-party cookies
    },
  },

  // Base path for auth endpoints
  basePath: "/api/auth",
});

// Export auth type for client
export type Auth = typeof auth;
