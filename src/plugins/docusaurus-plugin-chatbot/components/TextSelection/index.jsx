/**
 * TextSelection Component
 *
 * Wrapper that monitors text selection and shows "Ask AI" button.
 * Integrates with the main Chatbot component via global function.
 */
import React, { useCallback } from 'react';
import { useTextSelection } from './useTextSelection';
import AskAIButton from './AskAIButton';

export function TextSelection() {
  const { selection, clearSelection } = useTextSelection();

  const handleAskAI = useCallback(() => {
    if (!selection) return;

    // Send full text to chatbot (not truncated)
    if (window.__chatbotSetContext) {
      window.__chatbotSetContext(selection.text);
    }

    // Clear the text selection
    window.getSelection()?.removeAllRanges();
    clearSelection();
  }, [selection, clearSelection]);

  // Don't render if no selection
  if (!selection) {
    return null;
  }

  return (
    <AskAIButton
      position={selection.position}
      onClick={handleAskAI}
    />
  );
}

export default TextSelection;
