/**
 * SourceList Component
 *
 * Displays the list of sources below an assistant message.
 * Shows source URL and content preview.
 */
import React from 'react';
import styles from './styles.module.css';

/**
 * Truncate content to a maximum length
 */
function truncateContent(content, maxLength = 100) {
  if (!content || content.length <= maxLength) {
    return content;
  }
  return content.slice(0, maxLength).trim() + '...';
}

/**
 * Extract a readable title from URL
 */
function getUrlTitle(url) {
  try {
    const urlObj = new URL(url);
    const path = urlObj.pathname;
    // Get the last meaningful segment
    const segments = path.split('/').filter(Boolean);
    if (segments.length > 0) {
      const lastSegment = segments[segments.length - 1];
      // Remove file extension and convert dashes/underscores to spaces
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

export function SourceList({ sources, timing }) {
  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.sourceList}>
      <div className={styles.sourceListTitle}>Sources</div>
      {sources.map((source, index) => (
        <div key={index} className={styles.sourceItem}>
          <span className={styles.sourceNumber}>[{index + 1}]</span>
          <div>
            <a
              href={source.url}
              className={styles.sourceLink}
              target="_blank"
              rel="noopener noreferrer"
            >
              {getUrlTitle(source.url)}
            </a>
            {source.content && (
              <div className={styles.sourceContent}>
                {truncateContent(source.content, 120)}
              </div>
            )}
          </div>
        </div>
      ))}
      {timing && (
        <div className={styles.sourceContent} style={{ marginTop: '8px', fontSize: '11px' }}>
          Retrieved in {timing.retrieval_time_ms}ms | Generated in {timing.generation_time_ms}ms
        </div>
      )}
    </div>
  );
}

export default SourceList;
