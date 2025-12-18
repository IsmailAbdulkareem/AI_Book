/**
 * Message Component
 *
 * Renders a single chat message with markdown support and citations.
 * Supports code blocks, lists, bold, italic, and inline code.
 */
import React, { useState } from 'react';
import styles from './styles.module.css';

/**
 * Simple markdown parser for chat messages
 * Handles: code blocks, inline code, bold, italic, lists, line breaks
 */
function parseMarkdown(text) {
  if (!text) return null;

  const elements = [];
  let key = 0;

  // Split by code blocks first (```...```)
  const codeBlockRegex = /```(\w*)\n?([\s\S]*?)```/g;
  let lastIndex = 0;
  let match;

  const textWithCodeBlocks = [];

  while ((match = codeBlockRegex.exec(text)) !== null) {
    // Add text before code block
    if (match.index > lastIndex) {
      textWithCodeBlocks.push({
        type: 'text',
        content: text.slice(lastIndex, match.index)
      });
    }

    // Add code block
    textWithCodeBlocks.push({
      type: 'codeblock',
      language: match[1] || '',
      content: match[2].trim()
    });

    lastIndex = match.index + match[0].length;
  }

  // Add remaining text
  if (lastIndex < text.length) {
    textWithCodeBlocks.push({
      type: 'text',
      content: text.slice(lastIndex)
    });
  }

  // Process each segment
  textWithCodeBlocks.forEach((segment, segmentIndex) => {
    if (segment.type === 'codeblock') {
      elements.push(
        <pre key={key++} className={styles.codeBlock}>
          <code className={segment.language ? styles[`language-${segment.language}`] : ''}>
            {segment.content}
          </code>
        </pre>
      );
    } else {
      // Process inline markdown for text segments
      const lines = segment.content.split('\n');
      let inList = false;
      let listItems = [];

      lines.forEach((line, lineIndex) => {
        const trimmedLine = line.trim();

        // Check for list items
        const listMatch = trimmedLine.match(/^(\d+\.|[-*])\s+(.+)$/);

        if (listMatch) {
          if (!inList) {
            inList = true;
            listItems = [];
          }
          listItems.push(parseInlineMarkdown(listMatch[2], key++));
        } else {
          // End list if we were in one
          if (inList && listItems.length > 0) {
            elements.push(
              <ul key={key++} className={styles.markdownList}>
                {listItems.map((item, i) => (
                  <li key={i}>{item}</li>
                ))}
              </ul>
            );
            inList = false;
            listItems = [];
          }

          // Skip empty lines but add spacing
          if (trimmedLine === '') {
            if (lineIndex > 0 && lineIndex < lines.length - 1) {
              elements.push(<br key={key++} />);
            }
          } else {
            // Check for headers
            const headerMatch = trimmedLine.match(/^(#{1,6})\s+(.+)$/);
            if (headerMatch) {
              const level = headerMatch[1].length;
              const HeaderTag = `h${Math.min(level + 2, 6)}`; // Offset for chat context
              elements.push(
                <HeaderTag key={key++} className={styles.markdownHeader}>
                  {parseInlineMarkdown(headerMatch[2], key++)}
                </HeaderTag>
              );
            } else {
              elements.push(
                <p key={key++} className={styles.markdownParagraph}>
                  {parseInlineMarkdown(trimmedLine, key++)}
                </p>
              );
            }
          }
        }
      });

      // Handle any remaining list items
      if (inList && listItems.length > 0) {
        elements.push(
          <ul key={key++} className={styles.markdownList}>
            {listItems.map((item, i) => (
              <li key={i}>{item}</li>
            ))}
          </ul>
        );
      }
    }
  });

  return elements;
}

/**
 * Parse inline markdown: bold, italic, inline code, links
 */
function parseInlineMarkdown(text, baseKey = 0) {
  if (!text) return text;

  const elements = [];
  let remaining = text;
  let key = baseKey;

  // Combined regex for inline patterns
  const inlineRegex = /(`[^`]+`|\*\*[^*]+\*\*|\*[^*]+\*|\[[^\]]+\]\([^)]+\)|\[(\d+)\])/g;
  let lastIndex = 0;
  let match;

  while ((match = inlineRegex.exec(text)) !== null) {
    // Add text before match
    if (match.index > lastIndex) {
      elements.push(text.slice(lastIndex, match.index));
    }

    const matched = match[0];

    if (matched.startsWith('`') && matched.endsWith('`')) {
      // Inline code
      elements.push(
        <code key={key++} className={styles.inlineCode}>
          {matched.slice(1, -1)}
        </code>
      );
    } else if (matched.startsWith('**') && matched.endsWith('**')) {
      // Bold
      elements.push(
        <strong key={key++}>{matched.slice(2, -2)}</strong>
      );
    } else if (matched.startsWith('*') && matched.endsWith('*')) {
      // Italic
      elements.push(
        <em key={key++}>{matched.slice(1, -1)}</em>
      );
    } else if (matched.startsWith('[') && matched.includes('](')) {
      // Link [text](url)
      const linkMatch = matched.match(/\[([^\]]+)\]\(([^)]+)\)/);
      if (linkMatch) {
        elements.push(
          <a
            key={key++}
            href={linkMatch[2]}
            target="_blank"
            rel="noopener noreferrer"
            className={styles.markdownLink}
          >
            {linkMatch[1]}
          </a>
        );
      }
    } else if (/^\[\d+\]$/.test(matched)) {
      // Citation marker [N] - keep as is, will be styled
      elements.push(
        <span key={key++} className={styles.citationMarker}>
          {matched}
        </span>
      );
    }

    lastIndex = match.index + matched.length;
  }

  // Add remaining text
  if (lastIndex < text.length) {
    elements.push(text.slice(lastIndex));
  }

  return elements.length > 0 ? elements : text;
}

/**
 * References section with collapsible toggle
 */
function References({ sources, timing }) {
  const [isOpen, setIsOpen] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.referencesContainer}>
      <button
        className={styles.referencesToggle}
        onClick={() => setIsOpen(!isOpen)}
        type="button"
        aria-expanded={isOpen}
      >
        <span className={styles.referencesIcon}>{isOpen ? '▼' : '▶'}</span>
        References ({sources.length})
      </button>

      {isOpen && (
        <div className={styles.referencesContent}>
          {sources.map((source, index) => (
            <div key={index} className={styles.referenceItem}>
              <span className={styles.referenceNumber}>[{index + 1}]</span>
              <a
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.referenceLink}
              >
                {getUrlTitle(source.url)}
              </a>
            </div>
          ))}
          {timing && (
            <div className={styles.referenceTiming}>
              Retrieved in {Math.round(timing.retrieval_time_ms)}ms | Generated in {Math.round(timing.generation_time_ms)}ms
            </div>
          )}
        </div>
      )}
    </div>
  );
}

/**
 * Extract a readable title from URL
 */
function getUrlTitle(url) {
  try {
    const urlObj = new URL(url);
    const path = urlObj.pathname;
    const segments = path.split('/').filter(Boolean);
    if (segments.length > 0) {
      const lastSegment = segments[segments.length - 1];
      return lastSegment
        .replace(/\.(html|md|mdx)$/i, '')
        .replace(/[-_]/g, ' ')
        .replace(/\b\w/g, (c) => c.toUpperCase());
    }
    return urlObj.hostname;
  } catch {
    return url;
  }
}

export function Message({ message }) {
  const { role, content, sources, timing, error } = message;
  const isUser = role === 'user';
  const isError = !!error;

  const messageClasses = [
    styles.message,
    isUser ? styles.messageUser : styles.messageAssistant,
    isError && styles.messageError,
  ]
    .filter(Boolean)
    .join(' ');

  return (
    <div className={messageClasses} role="article" aria-label={`${role} message`}>
      {isError ? (
        <span>{error}</span>
      ) : isUser ? (
        <span>{content}</span>
      ) : (
        <div className={styles.markdownContent}>
          {parseMarkdown(content)}
          <References sources={sources} timing={timing} />
        </div>
      )}
    </div>
  );
}

export default Message;
