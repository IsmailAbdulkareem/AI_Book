import React, { useState } from "react";
import { signIn, signUp } from "../../lib/auth-client";
import ProfileForm from "./ProfileForm";
import styles from "./styles.module.css";

/**
 * AuthModal Props
 */
interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess?: () => void;
}

type AuthMode = "signin" | "signup" | "profile";

/**
 * AuthModal Component
 *
 * Modal-based authentication UI with:
 * - Sign In form (email/password)
 * - Sign Up form (email/password/name)
 * - Profile form (shown BEFORE account creation during signup)
 *
 * Signup flow (atomic):
 * 1. User enters email/password/name
 * 2. User fills profile form
 * 3. Account created with profile data in single request
 */
export function AuthModal({ isOpen, onClose, onSuccess }: AuthModalProps) {
  // Auth state
  const [mode, setMode] = useState<AuthMode>("signin");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  // Sign in/up fields
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [name, setName] = useState("");
  const [rememberMe, setRememberMe] = useState(true);

  // Profile fields
  const [programmingLevel, setProgrammingLevel] = useState("");
  const [technologies, setTechnologies] = useState<string[]>([]);
  const [aiRoboticsExperience, setAiRoboticsExperience] = useState<boolean>(false);
  const [hardwareAccess, setHardwareAccess] = useState("");
  const [devicesOwned, setDevicesOwned] = useState<string[]>([]);
  const [image, setImage] = useState<string | null>(null);

  // Reset form state
  const resetForm = () => {
    setEmail("");
    setPassword("");
    setName("");
    setProgrammingLevel("");
    setTechnologies([]);
    setAiRoboticsExperience(false);
    setHardwareAccess("");
    setDevicesOwned([]);
    setImage(null);
    setError("");
    setLoading(false);
  };

  // Handle modal close
  const handleClose = () => {
    resetForm();
    setMode("signin");
    onClose();
  };

  // Validate sign in form
  const validateSignIn = (): boolean => {
    if (!email.trim()) {
      setError("Email is required");
      return false;
    }
    if (!password) {
      setError("Password is required");
      return false;
    }
    return true;
  };

  // Validate sign up form (step 1: credentials)
  const validateSignUpCredentials = (): boolean => {
    if (!email.trim()) {
      setError("Email is required");
      return false;
    }
    if (!name.trim()) {
      setError("Name is required");
      return false;
    }
    if (!password || password.length < 8) {
      setError("Password must be at least 8 characters");
      return false;
    }
    return true;
  };

  // Validate profile form (step 2: profile)
  const validateProfile = (): boolean => {
    if (!programmingLevel) {
      setError("Please select your programming level");
      return false;
    }
    if (technologies.length === 0) {
      setError("Please select at least one technology");
      return false;
    }
    if (!hardwareAccess) {
      setError("Please select your hardware access level");
      return false;
    }
    return true;
  };

  // Handle Sign In
  const handleSignIn = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (!validateSignIn()) return;

    setLoading(true);
    try {
      const result = await signIn.email({
        email: email.trim(),
        password,
        rememberMe,
      });

      if (result.error) {
        // Don't reveal which field was incorrect
        setError("Invalid email or password");
      } else {
        handleClose();
        onSuccess?.();
      }
    } catch (err) {
      setError("An error occurred. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  // Handle Sign Up - Step 1: Move to profile form
  const handleSignUpContinue = (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (!validateSignUpCredentials()) return;

    // Move to profile form (step 2)
    setMode("profile");
  };

  // Handle Sign Up - Step 2: Create account with profile (atomic)
  const handleSignUpComplete = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (!validateProfile()) return;

    setLoading(true);
    try {
      // ATOMIC SIGNUP: Create account + profile in single request
      const result = await signUp.email({
        email: email.trim(),
        password,
        name: name.trim(),
        // Profile image (optional)
        image: image || undefined,
        // Profile fields (stored as additionalFields in Better Auth)
        programmingLevel,
        technologies: JSON.stringify(technologies), // Encode array as JSON string
        aiRoboticsExperience,
        hardwareAccess,
        devicesOwned: devicesOwned.length > 0 ? JSON.stringify(devicesOwned) : undefined,
      });

      if (result.error) {
        setError(result.error.message || "Failed to create account");
        // Go back to signup form if error
        setMode("signup");
      } else {
        // Success: user is automatically logged in (session cookie set)
        handleClose();
        onSuccess?.();
      }
    } catch (err) {
      setError("An error occurred. Please try again.");
      setMode("signup");
    } finally {
      setLoading(false);
    }
  };

  // Handle back button (profile -> signup)
  const handleBack = () => {
    setMode("signup");
    setError("");
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={handleClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        {/* Close button */}
        <button className={styles.closeButton} onClick={handleClose} aria-label="Close">
          &times;
        </button>

        {/* Header */}
        <div className={styles.header}>
          <h2 className={styles.title}>
            {mode === "signin" && "Welcome Back"}
            {mode === "signup" && "Create Account"}
            {mode === "profile" && "Complete Your Profile"}
          </h2>
          <p className={styles.subtitle}>
            {mode === "signin" && "Sign in to access the AI assistant"}
            {mode === "signup" && "Join to get personalized assistance"}
            {mode === "profile" && "This helps us tailor responses to your level"}
          </p>
        </div>

        {/* Error message */}
        {error && <div className={styles.error}>{error}</div>}

        {/* Sign In Form */}
        {mode === "signin" && (
          <form onSubmit={handleSignIn} className={styles.form}>
            <div className={styles.formGroup}>
              <label className={styles.label}>Email</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={styles.input}
                placeholder="you@example.com"
                required
                autoFocus
              />
            </div>

            <div className={styles.formGroup}>
              <label className={styles.label}>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className={styles.input}
                placeholder="Enter your password"
                required
              />
            </div>

            <div className={styles.checkboxRow}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={rememberMe}
                  onChange={(e) => setRememberMe(e.target.checked)}
                  className={styles.checkbox}
                />
                Remember me for 7 days
              </label>
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? "Signing in..." : "Sign In"}
            </button>

            <p className={styles.switchMode}>
              Don't have an account?{" "}
              <button
                type="button"
                className={styles.linkButton}
                onClick={() => {
                  setMode("signup");
                  setError("");
                }}
              >
                Sign Up
              </button>
            </p>
          </form>
        )}

        {/* Sign Up Form (Step 1: Credentials) */}
        {mode === "signup" && (
          <form onSubmit={handleSignUpContinue} className={styles.form}>
            <div className={styles.formGroup}>
              <label className={styles.label}>Name</label>
              <input
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                className={styles.input}
                placeholder="Your name"
                required
                autoFocus
              />
            </div>

            <div className={styles.formGroup}>
              <label className={styles.label}>Email</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={styles.input}
                placeholder="you@example.com"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label className={styles.label}>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className={styles.input}
                placeholder="At least 8 characters"
                minLength={8}
                required
              />
            </div>

            <button type="submit" className={styles.submitButton}>
              Continue
            </button>

            <p className={styles.switchMode}>
              Already have an account?{" "}
              <button
                type="button"
                className={styles.linkButton}
                onClick={() => {
                  setMode("signin");
                  setError("");
                }}
              >
                Sign In
              </button>
            </p>
          </form>
        )}

        {/* Profile Form (Step 2) */}
        {mode === "profile" && (
          <form onSubmit={handleSignUpComplete} className={styles.form}>
            <ProfileForm
              programmingLevel={programmingLevel}
              setProgrammingLevel={setProgrammingLevel}
              technologies={technologies}
              setTechnologies={setTechnologies}
              aiRoboticsExperience={aiRoboticsExperience}
              setAiRoboticsExperience={setAiRoboticsExperience}
              hardwareAccess={hardwareAccess}
              setHardwareAccess={setHardwareAccess}
              devicesOwned={devicesOwned}
              setDevicesOwned={setDevicesOwned}
              image={image}
              setImage={setImage}
            />

            <div className={styles.buttonRow}>
              <button
                type="button"
                className={styles.backButton}
                onClick={handleBack}
              >
                Back
              </button>
              <button
                type="submit"
                className={styles.submitButton}
                disabled={loading}
              >
                {loading ? "Creating account..." : "Create Account"}
              </button>
            </div>
          </form>
        )}
      </div>
    </div>
  );
}

export default AuthModal;
