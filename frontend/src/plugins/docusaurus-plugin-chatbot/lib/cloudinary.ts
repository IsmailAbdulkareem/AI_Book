/**
 * Cloudinary Image Upload Utility
 *
 * Uploads images to Cloudinary using unsigned upload preset.
 * Returns the secure URL for storing in the user profile.
 *
 * Setup Instructions:
 * 1. Create free account at https://cloudinary.com
 * 2. Go to Settings > Upload > Add upload preset
 * 3. Create unsigned preset named "ai-book-avatars"
 * 4. Update CLOUDINARY_CLOUD_NAME below with your cloud name
 */

// Cloudinary configuration - update with your cloud name
const CLOUDINARY_CLOUD_NAME = "demo"; // TODO: Replace with actual cloud name
const CLOUDINARY_UPLOAD_PRESET = "ai-book-avatars";

/**
 * Upload an image file to Cloudinary
 *
 * @param file - The image file to upload
 * @returns Promise<string> - The secure URL of the uploaded image
 * @throws Error if upload fails
 */
export async function uploadToCloudinary(file: File): Promise<string> {
  // Validate file
  if (!file.type.startsWith("image/")) {
    throw new Error("Please select an image file");
  }

  // 5MB size limit
  const MAX_SIZE = 5 * 1024 * 1024;
  if (file.size > MAX_SIZE) {
    throw new Error("Image must be less than 5MB");
  }

  // Prepare form data for Cloudinary upload
  const formData = new FormData();
  formData.append("file", file);
  formData.append("upload_preset", CLOUDINARY_UPLOAD_PRESET);
  formData.append("folder", "ai-book-avatars");

  // Upload to Cloudinary
  const response = await fetch(
    `https://api.cloudinary.com/v1_1/${CLOUDINARY_CLOUD_NAME}/image/upload`,
    {
      method: "POST",
      body: formData,
    }
  );

  if (!response.ok) {
    const error = await response.json().catch(() => ({}));
    throw new Error(error.error?.message || "Failed to upload image");
  }

  const data = await response.json();
  return data.secure_url;
}

/**
 * Convert file to base64 data URL (fallback if Cloudinary not configured)
 *
 * @param file - The image file to convert
 * @returns Promise<string> - Base64 data URL
 */
export function fileToDataUrl(file: File): Promise<string> {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result as string);
    reader.onerror = reject;
    reader.readAsDataURL(file);
  });
}
