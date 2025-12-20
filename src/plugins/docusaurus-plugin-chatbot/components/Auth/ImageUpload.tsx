import React, { useState, useRef, ChangeEvent } from "react";
import { uploadToCloudinary, fileToDataUrl } from "../../lib/cloudinary";
import styles from "./ImageUpload.module.css";

interface ImageUploadProps {
  currentImage?: string | null;
  onImageChange: (url: string | null) => void;
}

/**
 * ImageUpload Component
 *
 * Allows users to upload a profile picture.
 * Uploads to Cloudinary and returns URL, or falls back to base64.
 */
export function ImageUpload({ currentImage, onImageChange }: ImageUploadProps) {
  const [preview, setPreview] = useState<string | null>(currentImage || null);
  const [uploading, setUploading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const handleFileSelect = async (e: ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    setError(null);

    // Validate file type
    if (!file.type.startsWith("image/")) {
      setError("Please select an image file");
      return;
    }

    // Validate file size (5MB)
    if (file.size > 5 * 1024 * 1024) {
      setError("Image must be less than 5MB");
      return;
    }

    // Show preview immediately
    try {
      const dataUrl = await fileToDataUrl(file);
      setPreview(dataUrl);
    } catch {
      // Continue even if preview fails
    }

    // Upload to Cloudinary
    setUploading(true);
    try {
      const url = await uploadToCloudinary(file);
      onImageChange(url);
      setPreview(url);
    } catch (err) {
      // Fallback to base64 if Cloudinary fails
      try {
        const dataUrl = await fileToDataUrl(file);
        onImageChange(dataUrl);
        setPreview(dataUrl);
      } catch {
        setError("Failed to process image");
        setPreview(null);
        onImageChange(null);
      }
    } finally {
      setUploading(false);
    }
  };

  const handleRemove = () => {
    setPreview(null);
    onImageChange(null);
    setError(null);
    if (fileInputRef.current) {
      fileInputRef.current.value = "";
    }
  };

  return (
    <div className={styles.container}>
      <div className={styles.previewContainer}>
        {preview ? (
          <img src={preview} alt="Profile preview" className={styles.preview} />
        ) : (
          <div className={styles.placeholder}>
            <svg
              className={styles.placeholderIcon}
              viewBox="0 0 24 24"
              fill="currentColor"
            >
              <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
            </svg>
          </div>
        )}
        {uploading && (
          <div className={styles.uploadingOverlay}>
            <div className={styles.spinner} />
          </div>
        )}
      </div>

      <div className={styles.actions}>
        <button
          type="button"
          className={styles.uploadButton}
          onClick={() => fileInputRef.current?.click()}
          disabled={uploading}
        >
          {preview ? "Change Photo" : "Add Photo"}
        </button>
        {preview && !uploading && (
          <button
            type="button"
            className={styles.removeButton}
            onClick={handleRemove}
          >
            Remove
          </button>
        )}
      </div>

      <input
        ref={fileInputRef}
        type="file"
        accept="image/*"
        onChange={handleFileSelect}
        className={styles.hiddenInput}
      />

      {error && <div className={styles.error}>{error}</div>}
    </div>
  );
}

export default ImageUpload;
