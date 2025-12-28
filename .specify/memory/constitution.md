<!-- Sync Impact Report -->
<!--
Version change: 1.0.0 → 1.1.0
Modified principles: None
Added sections: RAG System Governance
Removed sections: None
Templates requiring updates: None
Follow-up TODOs: None
-->

# AI/Spec-Driven Technical Book with Docusaurus & GitHub Pages Constitution

## Core Principles

### Spec-first authoring
The project is planned and guided by Spec-Kit Plus specs (`/sp.constitution`, `/sp.outline`, `/sp.specify`, `/sp.plan`, `/sp.tasks`), not by ad-hoc prompts or undocumented decisions.

### Technical accuracy
All technical content (tools, commands, code, workflows) is checked against primary sources (official documentation, standards, or canonical references).

### Clarity for developers
Writing targets a technical audience (junior–mid software developers) and favors concrete examples, clear explanations, and minimal ambiguity.

### Reproducibility
Readers and contributors can reproduce all examples, code, and setup steps from the public GitHub repository.

### Consistency
Terminology, formatting, structure, and code style are consistent across all chapters and system components.

### Transparent AI usage
The use of AI tools (Spec-Kit Plus, Claude Code, etc.) is acknowledged; limitations or uncertainties are not hidden.

---

## Key Standards

### Content & Structure
- All chapters are written in Markdown/MDX and compatible with Docusaurus.
- Each chapter includes, when applicable:
  - Frontmatter (title, description, sidebar label)
  - Short abstract / overview
  - Prerequisites
  - Learning objectives
  - Main content (concepts + step-by-step guidance)
  - Code examples or demonstrations
  - Summary / key takeaways
  - Optional exercises or further reading
- Chapter and section hierarchy follows `/sp.outline`.

### Technical Standards
- All code samples are syntactically correct and tested where practical.
- Version numbers are included when behavior depends on them.
- Non-trivial claims are traceable to authoritative sources.

### Writing Style
- Target Flesch-Kincaid grade level: 10–12
- Prefer active voice and precise language
- Define new terms on first use
- Maintain a consistent glossary

### Citations & External Material
- External ideas and examples are credited via links or references
- Direct quotations are clearly marked
- Avoid verbatim copying unless necessary and attributed

### Tooling & Workflow
- Spec-Kit Plus spec files are the single source of truth
- Structural or scope changes must be reflected in specs first
- AI output follows a loop of:
  - Draft → Review → Verify → Revise
- AI output is never accepted blindly

---

## RAG System Governance (Retrieval-Augmented Generation)

### Scope of the RAG System

The project includes a Retrieval-Augmented Generation (RAG) system tightly coupled to the book’s published content.  
This system is governed by the same spec-first discipline as the book.

### RAG Spec Lifecycle

The RAG system is developed in clearly defined phases:

- **Spec 1** — Website content ingestion, embedding generation, vector storage
- **Spec 2** — Retrieval, querying, and pipeline validation
- **Spec 3** — Agent integration using OpenAI Agents / ChatKit SDK
- **Spec 4** — Frontend integration and user-facing interaction

Each spec MUST:
- Include `/sp.specify`, `/sp.plan`, and `/sp.tasks`
- Be independently testable
- Clearly declare out-of-scope functionality

### Determinism & Reproducibility

- Ingestion pipelines MUST be idempotent
- Re-running pipelines MUST NOT create duplicate or corrupted data
- Vector collections MUST be reproducible from source URLs

### Data & Storage Standards

- Persistent artifacts (schemas, configs, snapshots) MUST reside under a dedicated `database/` directory
- Vector metadata schemas MUST be documented
- Secrets are never committed; environment variables are mandatory

### Retrieval Guarantees

- Retrieved content MUST be source-grounded
- Metadata (source URL, page title, section, chunk index) MUST always be preserved
- Retrieval correctness MUST be validated before any LLM or agent integration

### Separation of Concerns

- Ingestion ≠ Retrieval ≠ Generation ≠ UI
- Each layer evolves independently but is governed by specs

---

## Constraints & Success Criteria

### Constraints

#### Delivery Stack
- Docusaurus v2
- Build succeeds via `npm run build`
- Deployed to GitHub Pages with documented steps

#### Repository Requirements
- Contains:
  - All Markdown/MDX content
  - Docusaurus config
  - Build & deployment scripts
  - Local setup instructions
- New contributors can build and run the project from docs alone

#### Scope & Length
- Defined in `/sp.outline`
- Word-count targets respected within reason

#### Quality Gates
- No placeholder or TODO sections in published output
- No broken links or failing code snippets
- Linting and link checks have no critical errors

#### Licensing & IP
- Clear license in `LICENSE`
- Third-party material is attributed or replaced

### Success Criteria

#### Functional Delivery
- Site builds and is live on GitHub Pages
- Navigation reflects `/sp.outline`
- Code highlighting and theming work correctly

#### Content Quality
- Spot-checks reveal no major inaccuracies
- Code examples run as documented
- Readers can complete end-to-end examples

#### Alignment with Specs
- Final output aligns with `/sp.outline`
- Deviations are documented and intentional
- `/sp.tasks` reflects maintenance and extension work

#### Review & Usability
- At least one technical peer review completed
- At least one target reader confirms they can:
  - Set up locally
  - Complete learning objectives

---

## Governance

- This constitution supersedes all other project practices
- Amendments require documentation and a migration plan
- All pull requests must comply with this constitution
- Complexity must always be justified
- Project-specific runtime guidance lives in supporting docs

---

**Version**: 1.1.0  
**Ratified**: 2025-12-04  
**Last Amended**: 2025-12-13
