/**
 * AskAIButton Component
 *
 * Floating button that appears near text selection.
 * Clicking/tapping sends selected text to the chatbot as context.
 * Supports both desktop and mobile touch devices.
 */
import React from 'react';

const buttonStyles = {
  position: 'fixed',
  padding: '10px 16px',
  background: '#25c2a0',
  color: 'white',
  border: 'none',
  borderRadius: '20px',
  cursor: 'pointer',
  fontSize: '14px',
  fontWeight: '600',
  boxShadow: '0 4px 12px rgba(0, 0, 0, 0.25)',
  zIndex: 10001,
  display: 'flex',
  alignItems: 'center',
  gap: '6px',
  transition: 'transform 0.15s ease, box-shadow 0.15s ease',
  whiteSpace: 'nowrap',
  // Touch-friendly tap target
  minHeight: '44px',
  minWidth: '44px',
  // Prevent text selection on the button
  userSelect: 'none',
  WebkitUserSelect: 'none',
  // Prevent touch callout on iOS
  WebkitTouchCallout: 'none',
};

const hoverStyles = {
  transform: 'scale(1.05)',
  boxShadow: '0 6px 16px rgba(0, 0, 0, 0.3)',
};

/**
 * Small sparkle icon for the button
 */
function SparkleIcon() {
  return (
    <svg
      width="16"
      height="16"
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

  // Handle both click and touch
  const handleInteraction = (e) => {
    e.preventDefault();
    e.stopPropagation();
    onClick();
  };

  // Calculate position ensuring button stays within viewport
  // Account for button size (~100px width, ~44px height)
  const buttonWidth = 100;
  const buttonHeight = 44;
  const padding = 10;

  const style = {
    ...buttonStyles,
    ...(isHovered ? hoverStyles : {}),
    left: Math.max(padding, Math.min(position.x - buttonWidth / 2, window.innerWidth - buttonWidth - padding)),
    top: Math.max(padding, Math.min(position.y, window.innerHeight - buttonHeight - padding)),
  };

  return (
    <button
      data-ask-ai-button
      style={style}
      onClick={handleInteraction}
      onTouchEnd={handleInteraction}
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
