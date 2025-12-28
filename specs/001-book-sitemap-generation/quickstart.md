# Quickstart: Book Sitemap Generation

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