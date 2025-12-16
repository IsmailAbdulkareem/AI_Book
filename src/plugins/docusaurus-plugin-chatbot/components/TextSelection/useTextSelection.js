/**
 * useTextSelection Hook
 *
 * Monitors text selection events and returns selection data
 * when selection is within the content area.
 */
import { useState, useEffect, useCallback } from 'react';

// Selectors for valid content areas
const CONTENT_SELECTORS = [
  '.theme-doc-markdown',
  'article',
  '.markdown',
  '[class*="docItemContainer"]',
];

/**
 * Check if an element is within a valid content area
 */
function isWithinContentArea(element) {
  if (!element) return false;

  // Check if element or any ancestor matches content selectors
  for (const selector of CONTENT_SELECTORS) {
    if (element.closest(selector)) {
      return true;
    }
  }
  return false;
}

/**
 * Check if selection is within excluded areas (nav, header, footer, sidebar)
 */
function isWithinExcludedArea(element) {
  if (!element) return false;

  const excludedSelectors = [
    'nav',
    'header',
    'footer',
    '.navbar',
    '.sidebar',
    '.menu',
    '.table-of-contents',
    '.pagination-nav',
    '[class*="chatbot"]',
  ];

  for (const selector of excludedSelectors) {
    if (element.closest(selector)) {
      return true;
    }
  }
  return false;
}

/**
 * Truncate text for display (preserves full text for API)
 */
function truncateForDisplay(text, maxLength = 1000) {
  if (!text || text.length <= maxLength) {
    return text;
  }
  return text.slice(0, maxLength) + '...';
}

export function useTextSelection() {
  const [selection, setSelection] = useState(null);

  const clearSelection = useCallback(() => {
    setSelection(null);
  }, []);

  useEffect(() => {
    const handleSelectionChange = () => {
      const windowSelection = window.getSelection();

      // Check if there's a valid selection
      if (!windowSelection || windowSelection.isCollapsed || !windowSelection.toString().trim()) {
        // Don't clear immediately - let mouseup handle it
        return;
      }
    };

    const handleMouseUp = (e) => {
      // Small delay to allow selection to complete
      setTimeout(() => {
        const windowSelection = window.getSelection();

        // Check if there's a valid selection
        if (!windowSelection || windowSelection.isCollapsed) {
          setSelection(null);
          return;
        }

        const selectedText = windowSelection.toString().trim();
        if (!selectedText || selectedText.length < 3) {
          setSelection(null);
          return;
        }

        // Get the anchor node (where selection started)
        const anchorNode = windowSelection.anchorNode;
        const anchorElement = anchorNode?.nodeType === Node.TEXT_NODE
          ? anchorNode.parentElement
          : anchorNode;

        // Check if selection is within valid content area
        if (!isWithinContentArea(anchorElement)) {
          setSelection(null);
          return;
        }

        // Check if selection is within excluded areas
        if (isWithinExcludedArea(anchorElement)) {
          setSelection(null);
          return;
        }

        // Get selection range for positioning
        const range = windowSelection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Calculate position for the button
        const position = {
          x: rect.left + rect.width / 2,
          y: rect.top - 10, // Above the selection
        };

        setSelection({
          text: selectedText,
          truncatedText: truncateForDisplay(selectedText),
          position,
          isWithinContentArea: true,
        });
      }, 10);
    };

    const handleMouseDown = (e) => {
      // Clear selection if clicking outside of it
      if (selection) {
        const target = e.target;
        // Don't clear if clicking on the Ask AI button
        if (target.closest('[data-ask-ai-button]')) {
          return;
        }
        setSelection(null);
      }
    };

    const handleScroll = () => {
      // Clear selection on scroll as position would be stale
      if (selection) {
        setSelection(null);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);
    window.addEventListener('scroll', handleScroll, true);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
      window.removeEventListener('scroll', handleScroll, true);
    };
  }, [selection]);

  return {
    selection,
    clearSelection,
  };
}

export default useTextSelection;
