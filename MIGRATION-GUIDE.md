# Migration Guide

This document explains the reorganization from the old structure to the new monorepo structure.

## What Changed

### Old Structure (with spaces in path)
```
Hackathon I Physical AI & Humanoid Robotics Textbook/
├── docs/
├── src/
├── static/
├── database/
├── auth-server/
├── specs/
├── history/
├── docusaurus.config.js
├── package.json
└── pyproject.toml
```

### New Structure (clean paths, organized monorepo)
```
hackathon-01-physical-ai-robotics/
├── frontend/              # All Docusaurus files
│   ├── docs/
│   ├── src/
│   ├── static/
│   ├── docusaurus.config.js
│   └── package.json
├── backend/              # All Python/FastAPI files
│   ├── app.py
│   ├── agent.py
│   ├── chatbot.py
│   ├── main.py
│   ├── models.py
│   ├── pyproject.toml
│   └── requirements.txt
├── auth-server/          # Authentication service
├── specs/                # Specifications
└── history/              # PHRs and ADRs
```

## Benefits of New Structure

1. **No Path Issues**: Removed spaces from directory name - `npm start` now works without special handling
2. **Clear Separation**: Frontend and backend are clearly separated
3. **Independent Deploys**: Each service can be deployed independently
4. **Better DevEx**: Easier to navigate, understand, and maintain
5. **Standard Practice**: Follows industry-standard monorepo patterns

## Breaking Changes

### Working Directory
- **Old**: `D:\Projects\spec-driven-development-hacathon\Hackathon I Physical AI & Humanoid Robotics Textbook`
- **New**: `D:\Projects\spec-driven-development-hacathon\hackathon-01-physical-ai-robotics`

### npm Commands
```bash
# Old (required special handling)
npm start  # Would fail due to spaces in path

# New (works perfectly)
cd frontend
npm start
```

### Python Commands
```bash
# Old
cd database
uv run python main.py

# New
cd backend
uv run python main.py
```

### Import Paths
No changes needed - all relative imports remain the same within each service.

## Migration Steps (Already Completed)

1. ✅ Created new directory: `hackathon-01-physical-ai-robotics/`
2. ✅ Moved Docusaurus files to `frontend/`
3. ✅ Merged `database/` into `backend/`
4. ✅ Moved shared folders (specs, history, auth-server) to root
5. ✅ Updated README with new structure
6. ✅ Tested frontend - npm start works!

## Quick Start

### For Frontend Development
```bash
cd hackathon-01-physical-ai-robotics/frontend
npm install  # First time only
npm start
```

### For Backend Development
```bash
cd hackathon-01-physical-ai-robotics/backend
uv sync  # First time only
uv run python main.py
```

### Start Both Services
```powershell
# From project root
.\dev-start.ps1
```

## IDE Setup

### VSCode
Update your workspace folder to:
```
D:\Projects\spec-driven-development-hacathon\hackathon-01-physical-ai-robotics
```

### Terminal Aliases (Optional)
Add to your PowerShell profile:
```powershell
Set-Alias -Name ai-book -Value "D:\Projects\spec-driven-development-hacathon\hackathon-01-physical-ai-robotics"
```

## Git Considerations

The old directory will need to be manually deleted once all processes (IDE, terminals, etc.) release their locks on it. Then:

```bash
cd hackathon-01-physical-ai-robotics
git add -A
git commit -m "refactor: reorganize into frontend/backend monorepo structure"
```

## Troubleshooting

### Old Directory Still Exists
**Cause**: Windows has file locks from VSCode, Terminal, or Explorer.

**Solution**:
1. Close VSCode
2. Close all PowerShell/CMD windows
3. Close Windows Explorer windows showing that path
4. Delete via PowerShell:
   ```powershell
   Remove-Item 'D:\Projects\spec-driven-development-hacathon\Hackathon I Physical AI & Humanoid Robotics Textbook' -Recurse -Force
   ```

### npm start Still Fails
**Cause**: Running from wrong directory.

**Solution**: Always run from `frontend/` directory:
```bash
cd frontend
npm start
```

### Python Dependencies Not Found
**Cause**: Virtual environment is in the old location.

**Solution**: Recreate virtual environment:
```bash
cd backend
uv sync
```

## Questions?

See the main [README.md](./README.md) for full documentation.
