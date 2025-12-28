import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";
import "dotenv/config";

const app = express();
const port = process.env.PORT || 3001;

// ============================================================================
// CORS Configuration
// ============================================================================
// CRITICAL: credentials: true is REQUIRED for cookie-based auth
app.use(
  cors({
    origin: [
      "https://ismailabdulkareem.github.io",
      "http://localhost:3000",
      "http://127.0.0.1:3000",
    ],
    credentials: true, // REQUIRED for session cookies
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Parse JSON bodies
app.use(express.json());

// ============================================================================
// Better Auth Handler
// ============================================================================
// All auth endpoints are handled by Better Auth under /api/auth/*
app.all("/api/auth/*", toNodeHandler(auth));

// ============================================================================
// Health Check Endpoint
// ============================================================================
app.get("/health", (_req, res) => {
  res.json({
    status: "healthy",
    service: "ai-book-auth-server",
    timestamp: new Date().toISOString(),
  });
});

// ============================================================================
// Root Endpoint
// ============================================================================
app.get("/", (_req, res) => {
  res.json({
    name: "AI Book Auth Server",
    version: "1.0.0",
    endpoints: {
      health: "/health",
      auth: "/api/auth/*",
    },
  });
});

// ============================================================================
// Start Server
// ============================================================================
app.listen(port, () => {
  console.log(`🔐 Auth server running on http://localhost:${port}`);
  console.log(`   Health check: http://localhost:${port}/health`);
  console.log(`   Auth endpoints: http://localhost:${port}/api/auth/*`);
});
