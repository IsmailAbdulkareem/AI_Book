/**
 * useTextSelection Hook
 *
 * Monitors text selection events and returns selection data
 * when selection is within the content area.
 * Supports both desktop (mouse) and mobile (touch) devices.
 */
import { useState, useEffect, useCallback, useRef } from 'react';

// Selectors for valid content areas
const CONTENT_SELECTORS = [
  '.theme-doc-markdown',
  'article',
  '.markdown',
  '[class*="docItemContainer"]',
  '[class*="docMainContainer"]',
  'main',
  // Headings with hash links
  'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
  '[class*="heading"]',
];

/**
 * Check if device is touch-enabled
 */
function isTouchDevice() {
  return 'ontouchstart' in window || navigator.maxTouchPoints > 0;
}

/**
 * Check if an element is within a valid content area
 */
function isWithinContentArea(element) {
  if (!element) return false;

  // Check if element matches directly (for headings, etc.)
  for (const selector of CONTENT_SELECTORS) {
    try {
      if (element.matches && element.matches(selector)) {
        return true;
      }
    } catch (e) {
      // Invalid selector, skip
    }
  }

  // Check if any ancestor matches content selectors
  for (const selector of CONTENT_SELECTORS) {
    try {
      if (element.closest && element.closest(selector)) {
        return true;
      }
    } catch (e) {
      // Invalid selector, skip
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
  const selectionCheckInterval = useRef(null);

  const clearSelection = useCallback(() => {
    setSelection(null);
  }, []);

  /**
   * Process the current selection and update state
   */
  const processSelection = useCallback(() => {
    const windowSelection = window.getSelection();

    // Check if there's a valid selection
    if (!windowSelection || windowSelection.isCollapsed) {
      return false;
    }

    const selectedText = windowSelection.toString().trim();
    if (!selectedText || selectedText.length < 3) {
      return false;
    }

    // Get the anchor node (where selection started)
    const anchorNode = windowSelection.anchorNode;
    const anchorElement = anchorNode?.nodeType === Node.TEXT_NODE
      ? anchorNode.parentElement
      : anchorNode;

    // Check if selection is within valid content area
    if (!isWithinContentArea(anchorElement)) {
      return false;
    }

    // Check if selection is within excluded areas
    if (isWithinExcludedArea(anchorElement)) {
      return false;
    }

    // Get selection range for positioning
    const range = windowSelection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Calculate position for the button
    // On mobile, position below the selection to avoid OS selection handles
    const isMobile = isTouchDevice();
    const position = {
      x: rect.left + rect.width / 2,
      y: isMobile ? rect.bottom + 10 : rect.top - 10,
    };

    setSelection({
      text: selectedText,
      truncatedText: truncateForDisplay(selectedText),
      position,
      isWithinContentArea: true,
    });

    return true;
  }, []);

  useEffect(() => {
    /**
     * Handle selection change - used for mobile devices
     * On mobile, this fires when user finishes selecting text
     */
    const handleSelectionChange = () => {
      // Use a small delay to let the selection stabilize
      setTimeout(() => {
        const windowSelection = window.getSelection();

        // If there's no selection or it's collapsed, clear our state
        if (!windowSelection || windowSelection.isCollapsed || !windowSelection.toString().trim()) {
          // Don't immediately clear on mobile - user might still be selecting
          if (!isTouchDevice()) {
            // On desktop, we rely on mouseup
          }
          return;
        }

        // On mobile, process selection on selectionchange
        if (isTouchDevice()) {
          processSelection();
        }
      }, 100);
    };

    /**
     * Handle mouse up - primary handler for desktop
     */
    const handleMouseUp = (e) => {
      // Skip on touch devices - use selectionchange instead
      if (isTouchDevice()) {
        return;
      }

      // Small delay to allow selection to complete
      setTimeout(() => {
        if (!processSelection()) {
          setSelection(null);
        }
      }, 10);
    };

    /**
     * Handle touch end - for mobile devices
     */
    const handleTouchEnd = (e) => {
      // Longer delay for mobile to allow native selection UI to appear
      setTimeout(() => {
        processSelection();
      }, 300);
    };

    /**
     * Handle mouse/touch down - clear selection when clicking elsewhere
     */
    const handlePointerDown = (e) => {
      if (selection) {
        const target = e.target;
        // Don't clear if clicking on the Ask AI button
        if (target.closest('[data-ask-ai-button]')) {
          return;
        }
        setSelection(null);
      }
    };

    /**
     * Handle scroll - clear selection as position would be stale
     */
    const handleScroll = () => {
      if (selection) {
        setSelection(null);
      }
    };

    // Start interval to check for selection on mobile
    // This helps catch selections that don't trigger events properly
    if (isTouchDevice()) {
      selectionCheckInterval.current = setInterval(() => {
        const windowSelection = window.getSelection();
        if (windowSelection && !windowSelection.isCollapsed && windowSelection.toString().trim().length >= 3) {
          processSelection();
        }
      }, 500);
    }

    // Add event listeners
    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleTouchEnd);
    document.addEventListener('mousedown', handlePointerDown);
    document.addEventListener('touchstart', handlePointerDown);
    window.addEventListener('scroll', handleScroll, true);

    return () => {
      // Clean up
      if (selectionCheckInterval.current) {
        clearInterval(selectionCheckInterval.current);
      }
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleTouchEnd);
      document.removeEventListener('mousedown', handlePointerDown);
      document.removeEventListener('touchstart', handlePointerDown);
      window.removeEventListener('scroll', handleScroll, true);
    };
  }, [selection, processSelection]);

  return {
    selection,
    clearSelection,
  };
}

export default useTextSelection;
