// Text Selection Detection Utility
// Handles text selection detection and provides utilities for the chat widget

class TextSelectionDetector {
  constructor() {
    this.selectedText = null;
    this.selectionRange = null;
    this.onSelectionChange = null;
  }

  // Initialize the text selection detection
  init(onSelectionChangeCallback = null) {
    this.onSelectionChange = onSelectionChangeCallback;

    // Add event listeners for text selection
    document.addEventListener('mouseup', this.handleSelection.bind(this));
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        this.clearSelection();
      }
    });

    // Also listen for selectionchange event for better compatibility
    document.addEventListener('selectionchange', this.handleSelection.bind(this));
  }

  // Handle the text selection event
  handleSelection() {
    const selection = window.getSelection();

    if (selection.rangeCount > 0) {
      const range = selection.getRangeAt(0);
      const selectedText = selection.toString().trim();

      // Only process if there's actual selected text (more than 1 character)
      if (selectedText.length > 1 && selectedText.length < 5000) {
        this.selectedText = selectedText;
        this.selectionRange = range;

        // Highlight the selection visually (optional)
        this.highlightSelection(range);

        // Notify the callback if provided
        if (this.onSelectionChange) {
          this.onSelectionChange(selectedText, range);
        }
      } else {
        this.clearSelection();
      }
    } else {
      this.clearSelection();
    }
  }

  // Highlight the selected text (visual feedback)
  highlightSelection(range) {
    // Remove any existing highlights first
    this.removeHighlight();

    // Create a highlight element
    const highlight = document.createElement('span');
    highlight.style.backgroundColor = 'rgba(255, 255, 0, 0.3)';
    highlight.style.borderRadius = '2px';
    highlight.className = 'chatbot-selection-highlight';

    try {
      // Surround the selected content with the highlight
      range.surroundContents(highlight);
    } catch (e) {
      // If surrounding fails (e.g., partial element selection),
      // manually extract and reinsert content
      const content = range.extractContents();
      highlight.appendChild(content);
      range.insertNode(highlight);
    }

    // Store reference to highlight for later removal
    this.highlightElement = highlight;
  }

  // Remove the highlight
  removeHighlight() {
    if (this.highlightElement) {
      const parent = this.highlightElement.parentNode;
      const content = this.highlightElement.childNodes;

      // Move children out of the highlight element
      for (let i = content.length - 1; i >= 0; i--) {
        parent.insertBefore(content[i], this.highlightElement);
      }

      // Remove the highlight element
      parent.removeChild(this.highlightElement);
      this.highlightElement = null;
    }
  }

  // Clear the current selection
  clearSelection() {
    this.selectedText = null;
    this.selectionRange = null;

    // Remove highlight if exists
    this.removeHighlight();

    // Clear the browser selection
    window.getSelection().removeAllRanges();

    // Notify callback if provided
    if (this.onSelectionChange) {
      this.onSelectionChange(null, null);
    }
  }

  // Get the currently selected text
  getSelectedText() {
    return this.selectedText;
  }

  // Get the selection range
  getSelectionRange() {
    return this.selectionRange;
  }

  // Check if text is currently selected
  hasSelection() {
    return !!this.selectedText;
  }

  // Validate selected text length
  validateSelection(text) {
    if (!text) return false;
    return text.length > 1 && text.length < 5000;
  }

  // Get selection context (surrounding text)
  getSelectionContext(range, contextLength = 50) {
    if (!range) return { before: '', after: '' };

    try {
      // Get text before selection
      const rangeBefore = document.createRange();
      rangeBefore.setStart(document.body, 0);
      rangeBefore.setEnd(range.startContainer, range.startOffset);
      const beforeText = rangeBefore.toString().slice(-contextLength);

      // Get text after selection
      const rangeAfter = document.createRange();
      rangeAfter.setStart(range.endContainer, range.endOffset);
      rangeAfter.setEnd(document.body, document.body.textContent.length);
      const afterText = rangeAfter.toString().substring(0, contextLength);

      return {
        before: beforeText,
        after: afterText
      };
    } catch (e) {
      console.warn('Could not get selection context:', e);
      return { before: '', after: '' };
    }
  }

  // Destroy the text selection detector and remove event listeners
  destroy() {
    document.removeEventListener('mouseup', this.handleSelection.bind(this));
    document.removeEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        this.clearSelection();
      }
    });
    document.removeEventListener('selectionchange', this.handleSelection.bind(this));

    this.clearSelection();
  }
}

// Utility functions for text processing
const TextSelectionUtils = {
  // Clean selected text by removing extra whitespace and line breaks
  cleanText: (text) => {
    return text
      .replace(/\s+/g, ' ') // Replace multiple whitespace with single space
      .replace(/\n/g, ' ')   // Replace newlines with spaces
      .trim();
  },

  // Extract key terms from selected text
  extractKeyTerms: (text) => {
    // Simple approach: extract capitalized words and words after certain keywords
    const words = text.split(/\s+/);
    const keyTerms = [];

    for (let i = 0; i < words.length; i++) {
      const word = words[i].replace(/[^\w]/g, ''); // Remove punctuation

      // Add capitalized words (potential proper nouns)
      if (word.length > 2 && word[0] === word[0].toUpperCase() && !this.isCommonWord(word.toLowerCase())) {
        keyTerms.push(word);
      }

      // Add words that follow common keywords
      const prevWord = i > 0 ? words[i-1].toLowerCase() : '';
      if (['the', 'a', 'an', 'this', 'that', 'these', 'those'].includes(prevWord) && word.length > 3) {
        keyTerms.push(word);
      }
    }

    return [...new Set(keyTerms)]; // Remove duplicates
  },

  // Check if a word is a common stop word
  isCommonWord: (word) => {
    const commonWords = [
      'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
      'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'have',
      'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could', 'should',
      'this', 'that', 'these', 'those', 'i', 'you', 'he', 'she', 'it', 'we', 'they'
    ];
    return commonWords.includes(word.toLowerCase());
  },

  // Summarize long text selections
  summarizeText: (text, maxLength = 200) => {
    if (text.length <= maxLength) return text;

    // Split into sentences and take the first few that fit within the limit
    const sentences = text.match(/[^\.!?]+[\.!?]+/g) || [text];
    let summary = '';

    for (const sentence of sentences) {
      if ((summary + sentence).length > maxLength - 3) {
        summary += sentence.substring(0, maxLength - summary.length - 3) + '...';
        break;
      }
      summary += sentence;
    }

    return summary || text.substring(0, maxLength) + '...';
  }
};

// Export the classes and utilities
export { TextSelectionDetector, TextSelectionUtils };

// Also provide a simple function to initialize text selection detection globally
export const initTextSelection = (onSelectionChange) => {
  const detector = new TextSelectionDetector();
  detector.init(onSelectionChange);
  return detector;
};