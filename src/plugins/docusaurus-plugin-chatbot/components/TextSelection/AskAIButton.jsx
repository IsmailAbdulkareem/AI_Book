/**
 * AskAIButton Component
 *
 * Floating button that appears near text selection.
 * Clicking sends selected text to the chatbot as context.
 */
import React from 'react';

const buttonStyles = {
  position: 'fixed',
  padding: '8px 12px',
  background: 'var(--ifm-color-primary, #3578e5)',
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  cursor: 'pointer',
  fontSize: '13px',
  fontWeight: '500',
  boxShadow: '0 2px 8px rgba(0, 0, 0, 0.15)',
  zIndex: 1001,
  display: 'flex',
  alignItems: 'center',
  gap: '6px',
  transition: 'transform 0.15s ease, box-shadow 0.15s ease',
  whiteSpace: 'nowrap',
};

const hoverStyles = {
  transform: 'scale(1.02)',
  boxShadow: '0 4px 12px rgba(0, 0, 0, 0.2)',
};

/**
 * Small sparkle icon for the button
 */
function SparkleIcon() {
  return (
    <svg
      width="14"
      height="14"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M12 0L14.59 9.41L24 12L14.59 14.59L12 24L9.41 14.59L0 12L9.41 9.41L12 0Z" />
    </svg>
  );
}

export function AskAIButton({ position, onClick }) {
  const [isHovered, setIsHovered] = React.useState(false);

  // Calculate position ensuring button stays within viewport
  const style = {
    ...buttonStyles,
    ...(isHovered ? hoverStyles : {}),
    left: Math.max(10, Math.min(position.x - 50, window.innerWidth - 120)),
    top: Math.max(10, position.y - 40),
  };

  return (
    <button
      data-ask-ai-button
      style={style}
      onClick={onClick}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      aria-label="Ask AI about selected text"
      type="button"
    >
      <SparkleIcon />
      Ask AI
    </button>
  );
}

export default AskAIButton;
