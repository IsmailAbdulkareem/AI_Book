/**
 * Chatbot Component
 *
 * Main wrapper component that manages the chatbot state and renders
 * the floating icon and chat panel. Handles text selection context.
 */
import React, { useState, useCallback, useEffect } from 'react';
import ChatIcon from './ChatIcon';
import ChatPanel from './ChatPanel';

export function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedContext, setSelectedContext] = useState(null);

  // Toggle chat panel
  const handleToggle = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  // Close chat panel
  const handleClose = useCallback(() => {
    setIsOpen(false);
  }, []);

  // Set context from text selection (called by TextSelection component)
  const handleSetContext = useCallback((text) => {
    setSelectedContext(text);
    setIsOpen(true); // Open panel when context is set
  }, []);

  // Clear context
  const handleClearContext = useCallback(() => {
    setSelectedContext(null);
  }, []);

  // Expose setContext globally for TextSelection component
  useEffect(() => {
    window.__chatbotSetContext = handleSetContext;
    return () => {
      delete window.__chatbotSetContext;
    };
  }, [handleSetContext]);

  // Handle Escape key to close panel
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'Escape' && isOpen) {
        handleClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen, handleClose]);

  return (
    <>
      <ChatIcon isOpen={isOpen} onClick={handleToggle} />
      {isOpen && (
        <ChatPanel
          onClose={handleClose}
          selectedContext={selectedContext}
          onClearContext={handleClearContext}
        />
      )}
    </>
  );
}

export default Chatbot;
