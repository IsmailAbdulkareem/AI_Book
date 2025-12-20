import React from "react";
import ImageUpload from "./ImageUpload";
import styles from "./styles.module.css";

/**
 * Profile Form Props
 */
interface ProfileFormProps {
  // Profile state
  programmingLevel: string;
  setProgrammingLevel: (value: string) => void;
  technologies: string[];
  setTechnologies: (value: string[]) => void;
  aiRoboticsExperience: boolean;
  setAiRoboticsExperience: (value: boolean) => void;
  hardwareAccess: string;
  setHardwareAccess: (value: string) => void;
  devicesOwned: string[];
  setDevicesOwned: (value: string[]) => void;
  // Optional image upload
  image?: string | null;
  setImage?: (url: string | null) => void;
}

// Technology options
const TECHNOLOGY_OPTIONS = [
  { value: "Python", label: "Python" },
  { value: "JS", label: "JavaScript/TypeScript" },
  { value: "ROS2", label: "ROS2" },
  { value: "AI/ML", label: "AI/ML" },
  { value: "Web", label: "Web Development" },
  { value: "Other", label: "Other" },
];

// Device options
const DEVICE_OPTIONS = [
  { value: "Jetson", label: "NVIDIA Jetson" },
  { value: "Raspberry Pi", label: "Raspberry Pi" },
  { value: "Arduino", label: "Arduino" },
  { value: "GPU", label: "GPU (Desktop)" },
  { value: "Other", label: "Other" },
];

/**
 * ProfileForm Component
 *
 * Collects user profile information during signup:
 * - Programming level (required)
 * - Technologies (required, multi-select)
 * - AI/Robotics experience (required)
 * - Hardware access (required)
 * - Devices owned (optional, multi-select)
 */
export function ProfileForm({
  programmingLevel,
  setProgrammingLevel,
  technologies,
  setTechnologies,
  aiRoboticsExperience,
  setAiRoboticsExperience,
  hardwareAccess,
  setHardwareAccess,
  devicesOwned,
  setDevicesOwned,
  image,
  setImage,
}: ProfileFormProps) {
  // Handle technology checkbox toggle
  const handleTechnologyChange = (value: string) => {
    if (technologies.includes(value)) {
      setTechnologies(technologies.filter((t) => t !== value));
    } else {
      setTechnologies([...technologies, value]);
    }
  };

  // Handle device checkbox toggle
  const handleDeviceChange = (value: string) => {
    if (devicesOwned.includes(value)) {
      setDevicesOwned(devicesOwned.filter((d) => d !== value));
    } else {
      setDevicesOwned([...devicesOwned, value]);
    }
  };

  return (
    <div className={styles.profileForm}>
      <h3 className={styles.profileTitle}>Tell us about yourself</h3>
      <p className={styles.profileSubtitle}>
        This helps us personalize your experience
      </p>

      {/* Profile Photo (optional) */}
      {setImage && (
        <div className={styles.formGroup}>
          <label className={styles.label}>
            Profile Photo <span className={styles.optional}>(optional)</span>
          </label>
          <ImageUpload currentImage={image} onImageChange={setImage} />
        </div>
      )}

      {/* Programming Level (required) */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Programming Experience <span className={styles.required}>*</span>
        </label>
        <select
          value={programmingLevel}
          onChange={(e) => setProgrammingLevel(e.target.value)}
          className={styles.select}
          required
        >
          <option value="">Select your level...</option>
          <option value="beginner">Beginner - New to programming</option>
          <option value="intermediate">Intermediate - Comfortable with basics</option>
          <option value="advanced">Advanced - Experienced developer</option>
        </select>
      </div>

      {/* Technologies (required, multi-select) */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Technologies You Know <span className={styles.required}>*</span>
        </label>
        <div className={styles.checkboxGroup}>
          {TECHNOLOGY_OPTIONS.map((option) => (
            <label key={option.value} className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={technologies.includes(option.value)}
                onChange={() => handleTechnologyChange(option.value)}
                className={styles.checkbox}
              />
              {option.label}
            </label>
          ))}
        </div>
        {technologies.length === 0 && (
          <span className={styles.hint}>Select at least one technology</span>
        )}
      </div>

      {/* AI/Robotics Experience (required) */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          AI/Robotics Experience <span className={styles.required}>*</span>
        </label>
        <div className={styles.radioGroup}>
          <label className={styles.radioLabel}>
            <input
              type="radio"
              name="aiExperience"
              checked={aiRoboticsExperience === true}
              onChange={() => setAiRoboticsExperience(true)}
              className={styles.radio}
            />
            Yes, I have experience with AI or Robotics
          </label>
          <label className={styles.radioLabel}>
            <input
              type="radio"
              name="aiExperience"
              checked={aiRoboticsExperience === false}
              onChange={() => setAiRoboticsExperience(false)}
              className={styles.radio}
            />
            No, I'm new to AI and Robotics
          </label>
        </div>
      </div>

      {/* Hardware Access (required) */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Hardware Access <span className={styles.required}>*</span>
        </label>
        <select
          value={hardwareAccess}
          onChange={(e) => setHardwareAccess(e.target.value)}
          className={styles.select}
          required
        >
          <option value="">Select your hardware access...</option>
          <option value="none">None - Learning concepts only</option>
          <option value="simulator_only">Simulator Only - Using simulation tools</option>
          <option value="real_robots">Real Robots - Access to physical hardware</option>
        </select>
      </div>

      {/* Devices Owned (optional, multi-select) */}
      <div className={styles.formGroup}>
        <label className={styles.label}>
          Devices You Own <span className={styles.optional}>(optional)</span>
        </label>
        <div className={styles.checkboxGroup}>
          {DEVICE_OPTIONS.map((option) => (
            <label key={option.value} className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={devicesOwned.includes(option.value)}
                onChange={() => handleDeviceChange(option.value)}
                className={styles.checkbox}
              />
              {option.label}
            </label>
          ))}
        </div>
      </div>
    </div>
  );
}

export default ProfileForm;
