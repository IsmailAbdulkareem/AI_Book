/**
 * Module model
 * Represents a book module that contains chapters
 */

class Module {
  /**
   * @param {string} id - Unique identifier for the module
   * @param {string} title - Display title of the module
   * @param {string} path - Relative path from site root
   * @param {string} lastmod - Date of last modification (ISO 8601 format)
   * @param {Array<Chapter>} chapters - List of chapters in this module
   */
  constructor(id, title, path, lastmod, chapters = []) {
    this.id = id;
    this.title = title;
    this.path = path;
    this.lastmod = lastmod;
    this.chapters = chapters;

    // Validate inputs
    this.validate();
  }

  /**
   * Validates the module properties
   */
  validate() {
    if (!this.id || typeof this.id !== 'string') {
      throw new Error('ID is required and must be a string');
    }

    if (!this.title || typeof this.title !== 'string') {
      throw new Error('Title is required and must be a string');
    }

    if (!this.path || typeof this.path !== 'string') {
      throw new Error('Path is required and must be a string');
    }

    if (!this.lastmod || typeof this.lastmod !== 'string') {
      throw new Error('Last modification date is required and must be a string');
    }

    if (!Array.isArray(this.chapters)) {
      throw new Error('Chapters must be an array');
    }
  }

  /**
   * Adds a chapter to this module
   * @param {Chapter} chapter - The chapter to add
   */
  addChapter(chapter) {
    this.chapters.push(chapter);
    this.updateLastMod();
  }

  /**
   * Removes a chapter from this module by ID
   * @param {string} chapterId - The ID of the chapter to remove
   */
  removeChapter(chapterId) {
    this.chapters = this.chapters.filter(chapter => chapter.id !== chapterId);
    this.updateLastMod();
  }

  /**
   * Updates the last modification date to current date
   */
  updateLastMod() {
    this.lastmod = new Date().toISOString().split('T')[0];
  }

  /**
   * Converts the module to JSON format
   * @returns {Object} JSON representation of the module
   */
  toJson() {
    return {
      id: this.id,
      title: this.title,
      path: this.path,
      lastmod: this.lastmod,
      chapters: this.chapters.map(chapter => chapter.toJson())
    };
  }

  /**
   * Creates a SitemapEntry for this module
   * @param {number} priority - Priority for this module in sitemap
   * @param {string} changefreq - Change frequency for this module in sitemap
   * @returns {SitemapEntry} Sitemap entry for this module
   */
  toSitemapEntry(priority = 0.9, changefreq = 'weekly') {
    const SitemapEntry = require('./sitemap-entry');
    return new SitemapEntry(
      this.path,
      this.lastmod,
      changefreq,
      priority,
      this.title,
      0 // Level 0 for modules
    );
  }
}

module.exports = Module;