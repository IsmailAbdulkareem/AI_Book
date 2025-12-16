/**
 * Message Component
 *
 * Renders a single chat message with citation parsing.
 * Citations [N] are converted to clickable links using the sources array.
 */
import React from 'react';
import styles from './styles.module.css';

/**
 * Parse message content and replace [N] markers with clickable links
 *
 * @param {string} content - The message content
 * @param {Array} sources - Array of source objects with url and content
 * @returns {Array} Array of React elements
 */
function parseContentWithCitations(content, sources = []) {
  if (!content || !sources.length) {
    return [content];
  }

  // Regex to match citation markers like [1], [2], etc.
  const citationRegex = /\[(\d+)\]/g;
  const parts = [];
  let lastIndex = 0;
  let match;

  while ((match = citationRegex.exec(content)) !== null) {
    // Add text before the citation
    if (match.index > lastIndex) {
      parts.push(content.slice(lastIndex, match.index));
    }

    const citationNumber = parseInt(match[1], 10);
    const sourceIndex = citationNumber - 1; // Citations are 1-indexed
    const source = sources[sourceIndex];

    if (source && source.url) {
      // Render as clickable link
      parts.push(
        <a
          key={`citation-${match.index}`}
          href={source.url}
          className={styles.citation}
          target="_blank"
          rel="noopener noreferrer"
          title={`Source ${citationNumber}: ${source.content?.slice(0, 100) || source.url}`}
        >
          [{citationNumber}]
        </a>
      );
    } else {
      // Keep as plain text if source not found
      parts.push(match[0]);
    }

    lastIndex = match.index + match[0].length;
  }

  // Add remaining text after last citation
  if (lastIndex < content.length) {
    parts.push(content.slice(lastIndex));
  }

  return parts.length > 0 ? parts : [content];
}

export function Message({ message }) {
  const { role, content, sources, error } = message;
  const isUser = role === 'user';
  const isError = !!error;

  const messageClasses = [
    styles.message,
    isUser ? styles.messageUser : styles.messageAssistant,
    isError && styles.messageError,
  ]
    .filter(Boolean)
    .join(' ');

  // Parse citations for assistant messages
  const renderedContent = isUser
    ? content
    : parseContentWithCitations(content, sources);

  return (
    <div className={messageClasses} role="article" aria-label={`${role} message`}>
      {isError ? (
        <span>{error}</span>
      ) : (
        <span>{renderedContent}</span>
      )}
    </div>
  );
}

export default Message;
