import React, { useState } from "react";
import { authClient } from "../../lib/auth-client";
import { useAuth } from "./AuthProvider";
import ProfileForm from "./ProfileForm";
import styles from "./styles.module.css";

/**
 * ProfileCompletionModal
 *
 * Modal shown to OAuth users who need to complete their profile.
 * Uses Better Auth's updateUser to save profile fields.
 */
export function ProfileCompletionModal() {
  const { user, showProfileCompletion, setShowProfileCompletion } = useAuth();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  // Profile fields
  const [programmingLevel, setProgrammingLevel] = useState("");
  const [technologies, setTechnologies] = useState<string[]>([]);
  const [aiRoboticsExperience, setAiRoboticsExperience] = useState<boolean>(false);
  const [hardwareAccess, setHardwareAccess] = useState("");
  const [devicesOwned, setDevicesOwned] = useState<string[]>([]);

  if (!showProfileCompletion) return null;

  // Validate profile
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

  // Handle profile update
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (!validateProfile()) return;

    setLoading(true);
    try {
      // Update user profile using Better Auth
      await authClient.updateUser({
        programmingLevel,
        technologies: JSON.stringify(technologies),
        aiRoboticsExperience,
        hardwareAccess,
        devicesOwned: devicesOwned.length > 0 ? JSON.stringify(devicesOwned) : undefined,
      });

      // Close modal and reload to refresh session
      setShowProfileCompletion(false);
      window.location.reload();
    } catch (err) {
      setError("Failed to update profile. Please try again.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        {/* Header */}
        <div className={styles.header}>
          <h2 className={styles.title}>Complete Your Profile</h2>
          <p className={styles.subtitle}>
            Welcome, {user?.name || user?.email}! Please complete your profile to get
            personalized assistance.
          </p>
        </div>

        {/* Error message */}
        {error && <div className={styles.error}>{error}</div>}

        {/* Profile Form */}
        <form onSubmit={handleSubmit} className={styles.form}>
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
          />

          <button
            type="submit"
            className={styles.submitButton}
            disabled={loading}
          >
            {loading ? "Saving..." : "Complete Profile"}
          </button>
        </form>
      </div>
    </div>
  );
}

export default ProfileCompletionModal;
