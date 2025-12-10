<!-- .github/copilot-instructions.md: Guidance for AI coding agents working on this repo -->
# Copilot / Agent Instructions — Physical AI & Humanoid Robotics

Purpose: Quickly orient an AI coding agent to the repo's architecture, workflows, and agent-specific conventions so it can be immediately productive.

**Big Picture**
- **Project type:** A Docusaurus-based technical book (site) combining `docs/` markdown content and small React UI in `src/`.
- **Primary flow:** content and specs authored under `docs/` and `specs/` → site preview via `npm start` → static build via `npm run build` → deploy with `npm run deploy` (project-specific deploy hooks may be required).

**Key files & locations**
- `docusaurus.config.js`: site config and theme settings (edit for global site changes).
- `sidebars.js`: controls docs navigation order — update when adding chapters or modules.
- `docs/`: canonical content — add chapters as markdown files with frontmatter (`id`, `title`).
- `src/`: React components and CSS for small UI tweaks (e.g., `src/components/HomepageFeatures.js`).
- `specs/` and `.specify/`: spec-driven templates, plans, and automation scripts used by agent toolchains.
- `history/prompts/`: Prompt History Records (PHRs) — agent-created record of user interactions and actions.

**Build / dev commands (use these exactly in order)**
- Install dependencies: `npm install`
- Run site locally: `npm start` (Docusaurus dev server, default port 3000)
- Build static site: `npm run build` (outputs to `build/`)
- Optional deploy: `npm run deploy` (project may need GH pages config or CI)

**Project conventions agents must follow**
- Small, focused diffs only: do not refactor unrelated files.
- Every user-facing change must be accompanied by a PHR in `history/prompts/` using the repo templates (see `.specify/templates/phr-template.prompt.md`).
- ADRs: detect and suggest Architectural Decision Records but DO NOT auto-create them — ask the user and run `/sp.adr <title>` only after consent (see `CLAUDE.md`).
- Use `.specify` scripts and templates when available (e.g., `.specify/scripts/powershell/update-agent-context.ps1` references agent files and templates).

**Editing docs or chapters — minimal checklist**
- Add new markdown to `docs/<module>/chapter-...md` with YAML frontmatter (`title`, `id` recommended).
- If the order or grouping must change, edit `sidebars.js` to update the sidebar.
- Test locally with `npm start` and confirm nav/frontmatter shows correctly.

**When changing UI code in `src/`**
- Keep components small and run `npm start` to validate visual/JS behavior.
- Follow existing patterns: functional React components, `HomepageFeatures.js` example for layout patterns.

**PHRs and templates (explicit)**
- PHRs must be written to `history/prompts/` and follow the repo template located at `.specify/templates/phr-template.prompt.md` (or `templates/phr-template.prompt.md`).
- PHR filename pattern: `history/prompts/<feature-or-general>/<ID>-<slug>.<stage>.prompt.md` (see `CLAUDE.md` for exact fields).

**Agent execution etiquette**
- Confirm one-sentence goal and acceptance criteria before making changes.
- List constraints and non-goals briefly.
- Propose minimal diffs and ask for approval on any architectural trade‑offs.
- Do not hardcode secrets or external tokens; reference `.env` or docs.

If anything in these instructions is unclear or you want additional detail (e.g., CI workflows, deployment configuration, or the PHR template fields), tell me which section to expand and I will iterate.
