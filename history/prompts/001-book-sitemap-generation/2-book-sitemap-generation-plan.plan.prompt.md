---
id: 2
title: book-sitemap-generation-plan
stage: plan
date: 2025-12-09
surface: agent
model: Claude Sonnet 4.5
feature: book-sitemap-generation
branch: 001-book-sitemap-generation
user: ismai
command: /sp.plan
labels: [planning, sitemap, ai-book, docusaurus]
links:
  spec: specs/001-book-sitemap-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-book-sitemap-generation/plan.md
 - specs/001-book-sitemap-generation/research.md
 - specs/001-book-sitemap-generation/data-model.md
 - specs/001-book-sitemap-generation/quickstart.md
 - specs/001-book-sitemap-generation/contracts/sitemap-api.yaml
tests:
 - null
---

## Prompt

/sp.plan

## Response snapshot

# Implementation Plan: Book Sitemap Generation

**Branch**: `001-book-sitemap-generation` | **Date**: 2025-12-09 | **Spec**: [specs/001-book-sitemap-generation/spec.md](../001-book-sitemap-generation/spec.md)
**Input**: Feature specification from `/specs/001-book-sitemap-generation/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Generate XML and human-readable sitemaps for the AI book website that include all modules and chapters in a hierarchical structure. The implementation will focus on creating sitemap generation functionality that updates automatically when new content is added, with proper metadata for SEO purposes.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js environment for build process)
**Primary Dependencies**: Docusaurus framework (as per constitution), sitemap generation libraries (e.g., sitemap.js)
**Storage**: N/A (generated sitemap files stored in build output)
**Testing**: Jest for unit testing, integration tests for sitemap generation
**Target Platform**: Web (Docusaurus static site generation)
**Project Type**: Web (Docusaurus-based documentation site)
**Performance Goals**: Sitemap generation under 2 seconds, support for 1000+ pages
**Constraints**: <200ms sitemap load time for users, SEO compliance with search engine standards
**Scale/Scope**: Support for 50+ modules, 200+ chapters with potential for growth

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Spec-first authoring**: Plan aligns with feature specification in spec.md
2. **Technical accuracy**: Will use established sitemap generation libraries and standards
3. **Clarity for developers**: Implementation will include clear documentation and examples
4. **Reproducibility**: Sitemap generation will be part of the build process
5. **Consistency**: Sitemap structure will follow Docusaurus conventions
6. **Content & structure**: Sitemap will be compatible with Docusaurus Markdown/MDX structure
7. **Technical standards**: Will follow XML sitemap protocol standards
8. **Delivery stack**: Implementation will work with Docusaurus v2 and GitHub Pages deployment

## Project Structure

### Documentation (this feature)

```text
specs/001-book-sitemap-generation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── pages/               # Docusaurus pages directory
├── components/          # React components for sitemap display
├── utils/               # Utility functions for sitemap generation
└── services/            # Sitemap generation service

static/
└── sitemap.xml          # Generated XML sitemap

docs/
├── modules/             # Book modules content
└── chapters/            # Book chapters content
```

**Structure Decision**: Single Docusaurus project with sitemap generation as part of the build process. The sitemap will be generated during the build phase and deployed as part of the static site.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|

## Research: Book Sitemap Generation

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

## Data Model: Book Sitemap Generation

## Overview
This document defines the data structures needed for generating sitemaps for the AI book website that includes modules and chapters.

## Core Entities

### SitemapEntry
Represents a single entry in the sitemap

- **url** (string): The full URL path for the page
- **lastmod** (string, ISO 8601): Date of last modification
- **changefreq** (enum): How frequently the page is likely to change (always, hourly, daily, weekly, monthly, yearly, never)
- **priority** (float): Priority of this URL relative to other URLs on the site (0.0 to 1.0)
- **title** (string): Human-readable title for HTML sitemap
- **level** (integer): Hierarchy level (0 for modules, 1+ for chapters/subsections)

### Module
Represents a book module that contains chapters

- **id** (string): Unique identifier for the module
- **title** (string): Display title of the module
- **path** (string): Relative path from site root
- **lastmod** (string, ISO 8601): Date of last modification
- **children** (array of Chapter): List of chapters in this module

### Chapter
Represents a chapter within a module

- **id** (string): Unique identifier for the chapter
- **title** (string): Display title of the chapter
- **path** (string): Relative path from site root
- **lastmod** (string, ISO 8601): Date of last modification
- **parentModuleId** (string): Reference to the parent module

### SitemapHierarchy
Represents the hierarchical structure for the sitemap

- **modules** (array of Module): All modules in the book
- **chapters** (array of Chapter): All chapters in the book
- **entries** (array of SitemapEntry): Flat list of entries for XML sitemap

## Relationships
- Module contains 0 or more Chapters (one-to-many)
- Chapter belongs to exactly 1 Module (many-to-one)
- SitemapHierarchy contains all Modules and Chapters (one-to-many)

## Validation Rules
- Each URL in the sitemap must be unique
- Module paths must start with the modules directory path
- Chapter paths must be within a module's path scope
- Priority values must be between 0.0 and 1.0
- All required fields must be present for each entity
- URL format must be valid and absolute for the site

## State Transitions
- Content items (modules/chapters) transition from "draft" to "published" status
- Only "published" items should appear in the sitemap
- Lastmod dates update when content is modified

## Quickstart: Book Sitemap Generation

## Overview
This guide provides a quick overview of how to implement and use the sitemap generation feature for the AI book website.

## Prerequisites
- Node.js 16+ installed
- Docusaurus project set up
- Basic knowledge of Docusaurus configuration

## Installation
1. Add sitemap plugin to your Docusaurus configuration
2. Install required dependencies
3. Configure the sitemap generation settings

## Basic Setup

### 1. Install Dependencies
```bash
npm install --save-dev @docusaurus/plugin-sitemap
```

### 2. Configure Docusaurus
Add the sitemap plugin to your `docusaurus.config.js`:

```javascript
module.exports = {
  plugins: [
    [
      '@docusaurus/plugin-sitemap',
      {
        cacheTime: 600 * 1000, // 600 seconds
        changefreq: 'weekly',
        priority: 0.5,
      },
    ],
  ],
};
```

### 3. Generate Sitemap
Run the build command to generate the sitemap:
```bash
npm run build
```

The sitemap will be generated at `/sitemap.xml` in the build output.

## Advanced Configuration

### Custom Sitemap Generation
For more control over sitemap content, create a custom script:

```javascript
// scripts/generate-sitemap.js
const fs = require('fs');
const path = require('path');

// Custom logic to generate hierarchical sitemap based on book structure
function generateSitemap() {
  // Implementation details here
}

module.exports = { generateSitemap };
```

### HTML Sitemap Page
Create an HTML sitemap page at `src/pages/sitemap.js`:

```javascript
import React from 'react';
import Layout from '@theme/Layout';

function Sitemap() {
  return (
    <Layout title="Sitemap" description="AI Book Website Sitemap">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Website Sitemap</h1>
            {/* Sitemap content will be rendered here */}
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default Sitemap;
```

## Running the Implementation
1. Update your package.json to include a sitemap generation script
2. Run `npm run generate-sitemap` to create both XML and HTML sitemaps
3. Verify the sitemap is accessible at `/sitemap.xml` and `/sitemap`

## Testing
- Verify all modules and chapters appear in the sitemap
- Check that the sitemap follows XML sitemap protocol
- Test sitemap loading performance
- Validate sitemap with search engine tools

## Outcome

- ✅ Impact: Created comprehensive implementation plan for book sitemap generation
- 🧪 Tests: null
- 📁 Files: specs/001-book-sitemap-generation/plan.md, specs/001-book-sitemap-generation/research.md, specs/001-book-sitemap-generation/data-model.md, specs/001-book-sitemap-generation/quickstart.md, specs/001-book-sitemap-generation/contracts/sitemap-api.yaml
- 🔁 Next prompts: /sp.tasks to create implementation tasks
- 🧠 Reflection: Plan includes all necessary components for sitemap generation with Docusaurus, including technical context, data models, contracts, and quickstart guide

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null