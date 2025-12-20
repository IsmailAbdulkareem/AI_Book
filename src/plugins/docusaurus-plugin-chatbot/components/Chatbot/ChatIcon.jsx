/**
 * ChatIcon Component
 *
 * Floating action button that toggles the chat panel.
 * Shows chat icon when closed, X icon when open.
 */
import React from 'react';
import styles from './styles.module.css';

/**
 * Chat bubble SVG icon
 */
function ChatBubbleIcon() {
  return (
    <svg
      className={styles.chatIconSvg}
      viewBox="0 0 24 24"
      xmlns="http://www.w3.org/2000/svg"
      aria-hidden="true"
    >
      <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
    </svg>
  );
}

/**
 * Close (X) icon
 */
function CloseIcon() {
  return <span className={styles.chatIconClose} aria-hidden="true">×</span>;
}

export function ChatIcon({ isOpen, onClick, isLoading = false }) {
  return (
    <button
      className={`${styles.chatIcon} ${isLoading ? styles.chatIconLoading : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      aria-expanded={isOpen}
      type="button"
      disabled={isLoading}
    >
      {isOpen ? <CloseIcon /> : <ChatBubbleIcon />}
    </button>
  );
}

export default ChatIcon;
