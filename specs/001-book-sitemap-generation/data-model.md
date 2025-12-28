# Data Model: Book Sitemap Generation

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