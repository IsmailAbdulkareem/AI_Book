# Add Documentation Command

Add new documentation pages to the book and update the vector database.

## Usage
```
/add-docs [topic]
```

## Instructions

You are a Documentation Management Agent. Your task is to add new documentation and ensure it's searchable.

### Step 1: Create Documentation

1. Identify the appropriate location in `docs/` directory
2. Create the markdown file following Docusaurus conventions:

```markdown
---
sidebar_position: X
title: "Your Title"
description: "Brief description for SEO"
---

# Your Title

Content here...
```

### Step 2: Update Sidebar (if needed)

Check `sidebars.js` to ensure new doc is included in navigation.

### Step 3: Local Preview

```bash
npm run start
```

Navigate to the new page and verify:
- Content renders correctly
- Sidebar navigation works
- Links are valid

### Step 4: Build Test

```bash
npm run build
```

Ensure no build errors.

### Step 5: Commit Changes

```bash
git add -A
git commit -m "docs: Add [topic] documentation"
git push origin main
```

### Step 6: Update Vector Database

After deployment, re-index to include new content:

```bash
cd database
uv run python main.py
```

### Step 7: Verify Searchability

Test that the new content is retrievable:

```bash
curl -X POST "https://ai-book-h6kj.onrender.com/ask" \
  -H "Content-Type: application/json" \
  -d '{"question": "YOUR_NEW_TOPIC_QUESTION", "top_k": 3}'
```

### Step 8: Report
- New file path created
- Sidebar position
- Build status
- Vector count before/after reindex
- Test query results
