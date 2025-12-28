# Research: Book Sitemap Generation

## Overview
This research document addresses the technical approach for implementing sitemap generation for the AI book website, focusing on how to generate XML and human-readable sitemaps that include all modules and chapters.

## Decision: Sitemap Generation Approach
**Approach**: Implement sitemap generation using Docusaurus plugins and custom build scripts

**Rationale**:
- Docusaurus provides built-in SEO capabilities
- Can leverage existing Docusaurus site metadata
- Integrates seamlessly with the build process
- Supports both XML sitemap for search engines and HTML sitemap for users

## Alternatives Considered

### 1. Third-party sitemap generation libraries (e.g., sitemap.js)
- **Pros**: Mature libraries with good XML sitemap support
- **Cons**: Would require custom integration with Docusaurus content structure
- **Verdict**: Not chosen due to integration complexity

### 2. Server-side generation during build process
- **Pros**: Automatic updates with content changes
- **Cons**: Requires build-time processing
- **Verdict**: Chosen approach - fits well with static site generation model

### 3. Client-side generation with JavaScript
- **Pros**: Dynamic updates possible
- **Cons**: Search engines prefer server-generated sitemaps
- **Verdict**: Not chosen due to SEO implications

## Technical Implementation Details

### XML Sitemap
- Generated as `sitemap.xml` at the site root
- Follows standard sitemap protocol (https://www.sitemaps.org/)
- Includes URLs, lastmod, changefreq, and priority elements
- Limited to 50,000 URLs per sitemap (can be split if needed)

### HTML Sitemap
- Generated as a human-readable page at `/sitemap`
- Hierarchical display of modules and chapters
- Links to all book content organized by structure
- Accessible and user-friendly design

### Content Discovery
- Leverage Docusaurus sidebar configuration to identify modules and chapters
- Extract content metadata from MDX frontmatter
- Support for excluding private/draft content
- Automatic detection of new content during build

## Dependencies & Tools
- **@docusaurus/plugin-sitemap**: For basic sitemap generation
- **Custom Node.js scripts**: For enhanced hierarchical sitemap generation
- **sitemap**: npm package if additional control needed

## Performance Considerations
- Sitemap generation should complete within 2 seconds
- Support for sites with 1000+ pages
- Efficient content traversal algorithms
- Caching of content metadata where possible

## Edge Cases Addressed
- Modules with no chapters: Will be included as top-level entries
- Private/unpublished content: Will be excluded from sitemap
- Large sitemaps: Will implement sitemap index files if exceeding 50,000 URLs