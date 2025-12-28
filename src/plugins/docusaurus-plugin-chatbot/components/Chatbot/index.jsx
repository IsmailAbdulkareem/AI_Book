/**
 * Chatbot Component
 *
 * Main wrapper component that manages the chatbot state and renders
 * the floating icon and chat panel. Handles text selection context.
 *
 * Spec 006: Authentication Gate
 * - Chatbot is gated behind authentication
 * - If user is not authenticated → show AuthModal instead of opening chatbot
 * - If user is authenticated but profile incomplete → show AuthModal
 * - Only open chatbot when authenticated AND profile complete
 */
import React, { useState, useCallback, useEffect } from 'react';
import ChatIcon from './ChatIcon';
import ChatPanel from './ChatPanel';
import { useAuth, AuthModal } from '../Auth';

export function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedContext, setSelectedContext] = useState(null);
  const [showAuthModal, setShowAuthModal] = useState(false);

  // Get auth state from context
  const { isAuthenticated, isProfileComplete, isPending } = useAuth();

  // Handle chatbot icon click with auth gate
  const handleToggle = useCallback(() => {
    // Don't do anything while checking auth status
    if (isPending) return;

    // Auth gate: If not authenticated OR profile incomplete → show auth modal
    if (!isAuthenticated || !isProfileComplete) {
      setShowAuthModal(true);
      return;
    }

    // User is authenticated with complete profile → toggle chatbot
    setIsOpen((prev) => !prev);
  }, [isAuthenticated, isProfileComplete, isPending]);

  // Close chat panel
  const handleClose = useCallback(() => {
    setIsOpen(false);
  }, []);

  // Close auth modal
  const handleCloseAuthModal = useCallback(() => {
    setShowAuthModal(false);
  }, []);

  // Handle successful auth - open chatbot if profile is complete
  const handleAuthSuccess = useCallback(() => {
    setShowAuthModal(false);
    // Re-check auth state after successful auth
    // The useAuth hook will automatically update
  }, []);

  // Set context from text selection (called by TextSelection component)
  const handleSetContext = useCallback((text) => {
    // Don't do anything while checking auth status
    if (isPending) return;

    // Auth gate for text selection too
    if (!isAuthenticated || !isProfileComplete) {
      setShowAuthModal(true);
      return;
    }

    setSelectedContext(text);
    setIsOpen(true); // Open panel when context is set
  }, [isAuthenticated, isProfileComplete, isPending]);

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

  // Handle Escape key to close panel or modal
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.key === 'Escape') {
        if (showAuthModal) {
          setShowAuthModal(false);
        } else if (isOpen) {
          handleClose();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isOpen, showAuthModal, handleClose]);

  return (
    <>
      {/* Chat Icon - always visible */}
      <ChatIcon
        isOpen={isOpen}
        onClick={handleToggle}
        isLoading={isPending}
      />

      {/* Chat Panel - only shown when authenticated + profile complete + panel open */}
      {isOpen && isAuthenticated && isProfileComplete && (
        <ChatPanel
          onClose={handleClose}
          selectedContext={selectedContext}
          onClearContext={handleClearContext}
        />
      )}

      {/* Auth Modal - shown when auth gate blocks access */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={handleCloseAuthModal}
        onSuccess={handleAuthSuccess}
      />
    </>
  );
}

export default Chatbot;
