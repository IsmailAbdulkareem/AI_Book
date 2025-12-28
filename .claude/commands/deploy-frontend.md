# Deploy Frontend Command

Deploy the Docusaurus documentation site to GitHub Pages.

## Usage
```
/deploy-frontend
```

## Instructions

You are a Frontend Deployment Agent. Your task is to build and deploy the documentation site.

### Step 1: Pre-deployment Checks

1. Verify code is committed:
```bash
git status
```

2. Run local build test:
```bash
npm run build
```

### Step 2: Push to GitHub
```bash
git add -A
git commit -m "Deploy: Update documentation site"
git push origin main
```

### Step 3: Trigger GitHub Pages Deploy

GitHub Actions will automatically deploy on push to main. Monitor at:
- Actions: https://github.com/IsmailAbdulkareem/AI_Book/actions

### Step 4: Verify Deployment

1. Wait for GitHub Actions to complete (usually 2-5 minutes)

2. Check live site:
```bash
curl -s -o /dev/null -w "%{http_code}" https://ismailabdulkareem.github.io/AI_Book/
```

3. Verify chatbot loads:
- Open https://ismailabdulkareem.github.io/AI_Book/
- Check browser console for errors
- Verify chat icon appears in bottom-right corner

### Step 5: Report
- Commit SHA deployed
- Build status (success/fail)
- GitHub Actions run URL
- Live site verification
- Chatbot functionality check
- Any issues detected
