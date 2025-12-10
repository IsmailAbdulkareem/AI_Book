Sitemap files for the site
=================================

What I generated
- `static/sitemap.xml` — standard XML sitemap suitable for search engines.
- `static/sitemap.json` — machine-friendly JSON list of URLs and metadata.

How these URLs were discovered
- I read the Docusaurus debug output generated under `.docusaurus/docusaurus-plugin-debug/default/p/docusaurus-debug-content-0d5.json` which lists every doc's `permalink` and the site pages (including `/`). The sitemap entries mirror those permalinks and are combined with the `url` + `baseUrl` from `docusaurus.config.js` (currently `https://IsmailAbdulkareem.github.io` + `/AI_Book/`).

Regeneration recommendation
- The authoritative sitemap is generated at runtime by Docusaurus when using the `@docusaurus/plugin-sitemap` plugin during a production `npm run build` — consider enabling that plugin and configuring it in `docusaurus.config.js` for automatic sitemaps.

Manual regeneration (quick)
1. Start the dev server and rebuild the site (recommended):

```powershell
npm install
npm run build
```

2. If you want a fresh sitemap based on the current local site state, run this (Node must be available):

```powershell
node -e "const fs=require('fs');const debug=fs.readFileSync('.docusaurus/docusaurus-plugin-debug/default/p/docusaurus-debug-content-0d5.json','utf8');console.log('Found '+(debug.length)+' bytes of debug output');"
```

Notes
- If you later change `docusaurus.config.js` `url` or `baseUrl`, update the `baseUrl` value in `static/sitemap.json` and regenerate `static/sitemap.xml` accordingly.
- For GitHub Pages deploys ensure `baseUrl` matches the repository name (example: `/AI_Book/`).

If you want, I can:
- Enable and configure the Docusaurus sitemap plugin and commit that change (recommended).
- Generate sitemaps including lastmod values extracted from git history (requires running git commands).
