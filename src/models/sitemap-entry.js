/**
 * SitemapEntry model
 * Represents a single entry in the sitemap
 */

class SitemapEntry {
  /**
   * @param {string} url - The full URL path for the page
   * @param {string} lastmod - Date of last modification (ISO 8601 format)
   * @param {string} changefreq - How frequently the page changes (always, hourly, daily, weekly, monthly, yearly, never)
   * @param {number} priority - Priority of this URL relative to other URLs (0.0 to 1.0)
   * @param {string} title - Human-readable title for HTML sitemap
   * @param {number} level - Hierarchy level (0 for modules, 1+ for chapters/subsections)
   */
  constructor(url, lastmod, changefreq, priority, title, level = 0) {
    this.url = url;
    this.lastmod = lastmod;
    this.changefreq = changefreq;
    this.priority = priority;
    this.title = title;
    this.level = level;

    // Validate inputs
    this.validate();
  }

  /**
   * Validates the sitemap entry properties
   */
  validate() {
    if (!this.url || typeof this.url !== 'string') {
      throw new Error('URL is required and must be a string');
    }

    if (!this.lastmod || typeof this.lastmod !== 'string') {
      throw new Error('Last modification date is required and must be a string');
    }

    const validChangeFreq = ['always', 'hourly', 'daily', 'weekly', 'monthly', 'yearly', 'never'];
    if (!validChangeFreq.includes(this.changefreq)) {
      throw new Error(`Change frequency must be one of: ${validChangeFreq.join(', ')}`);
    }

    if (typeof this.priority !== 'number' || this.priority < 0.0 || this.priority > 1.0) {
      throw new Error('Priority must be a number between 0.0 and 1.0');
    }

    if (typeof this.title !== 'string') {
      throw new Error('Title is required and must be a string');
    }

    if (typeof this.level !== 'number' || this.level < 0) {
      throw new Error('Level must be a non-negative number');
    }
  }

  /**
   * Converts the entry to XML format for sitemap
   * @returns {string} XML representation of the sitemap entry
   */
  toXml() {
    return `
    <url>
      <loc>${this.url}</loc>
      <lastmod>${this.lastmod}</lastmod>
      <changefreq>${this.changefreq}</changefreq>
      <priority>${this.priority}</priority>
    </url>`;
  }

  /**
   * Converts the entry to JSON format for HTML sitemap
   * @returns {Object} JSON representation of the sitemap entry
   */
  toJson() {
    return {
      url: this.url,
      lastmod: this.lastmod,
      changefreq: this.changefreq,
      priority: this.priority,
      title: this.title,
      level: this.level
    };
  }
}

module.exports = SitemapEntry;