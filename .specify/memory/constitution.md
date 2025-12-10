<!-- Sync Impact Report -->
<!--
Version change: 0.0.0 → 1.0.0
Modified principles: None (initial creation)
Added sections: Core Principles, Key Standards, Constraints & Success Criteria, Governance
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
Follow-up TODOs: None
-->
# AI/Spec-Driven Technical Book with Docusaurus & GitHub Pages Constitution

## Core Principles

### Spec-first authoring
The book is planned and guided by Spec-Kit Plus specs (/sp.constitution, /sp.outline, /sp.tasks), not by ad-hoc prompts.

### Technical accuracy
All technical content (tools, commands, code, workflows) is checked against primary sources (official documentation, standards, or canonical references).

### Clarity for developers
Writing targets a technical audience (junior–mid software developers) and favors concrete examples, clear explanations, and minimal ambiguity.

### Reproducibility
Readers can reproduce all examples, code, and setup steps from the public GitHub repository.

### Consistency
Terminology, formatting, structure, and code style are consistent across all chapters.

### Transparent AI usage
The use of AI tools (Spec-Kit Plus, Claude Code, etc.) is acknowledged; limitations or uncertainties are not hidden.

## Key Standards

*   **Content & structure**:
    *   All chapters are written in Markdown/MDX and compatible with Docusaurus.
    *   Each chapter includes, in order when applicable:
        *   Frontmatter (title, description, sidebar label, etc.)
        *   Short abstract / overview
        *   Prerequisites
        *   Learning objectives
        *   Main content (concepts + step-by-step guidance)
        *   Code/examples or demonstrations
        *   Summary / key takeaways
        *   Optional exercises or further reading
    *   The chapter and section hierarchy matches the outline defined in /sp.outline.
*   **Technical standards**:
    *   All code samples are syntactically correct and tested against the specified tool/library versions (where practical).
    *   When describing commands, APIs, libraries, or configurations, include version numbers where they materially affect behavior.
    *   Non-trivial technical claims (performance, limitations, behavior of tools/frameworks) are traceable to official docs or widely recognized authoritative sources.
*   **Writing style**:
    *   Target Flesch-Kincaid grade level: approximately 10–12.
    *   Prefer active voice and direct language.
    *   Introduce and define new or specialized terms on first use.
    *   Maintain and reuse a consistent glossary for important terms.
*   **Citations and external material**:
    *   External ideas, definitions, diagrams, and non-trivial examples are credited via inline links or references.
    *   Direct quotations are clearly marked and attributed.
    *   Avoid copying text verbatim from sources; when necessary, quote minimally and with attribution.
*   **Tooling & workflow**:
    *   Spec-Kit Plus spec files (/sp.constitution, /sp.outline, /sp.tasks, etc.) are treated as the single source of truth for project scope and structure.
    *   Any substantial change to book structure or goals is first reflected in the relevant /sp.* spec file.
    *   Claude Code and other AI tools are used in a loop of:
        *   Draft → Review → Verify against sources → Revise
        *   AI output is never accepted as final without human verification.

## Constraints & Success Criteria

*   **Constraints**:
    *   **Delivery stack**:
        *   The book is implemented as a Docusaurus v2 site.
        *   The site builds successfully via `npm run build` (or the project’s documented build command) without errors.
        *   The site is deployed to GitHub Pages using a documented procedure in the repository (e.g., README or DEPLOY.md).
    *   **Repository requirements**:
        *   The GitHub repository contains:
            *   All Markdown/MDX content
            *   Docusaurus configuration files
            *   Build and deployment scripts/configuration
            *   Instructions for local setup and build
        *   The repository is structured so a new contributor can clone, install dependencies, run the dev server, and build the site by following documented steps.
    *   **Scope & length**:
        *   Overall scope, chapter list, and section list are defined in /sp.outline.
        *   Target word-count or page-count ranges per chapter are documented in /sp.outline and respected within a reasonable margin.
    *   **Quality gates**:
        *   No intentional TODO, “coming soon”, or placeholder sections in the published version.
        *   No known broken links, missing images, or obviously failing code snippets at release.
        *   Where link-checking or linting tools are configured, there are no critical unresolved issues in the main branch.
    *   **Licensing & IP**:
        *   The project uses a clear license (e.g., MIT, CC BY-SA, etc.), documented in LICENSE.
        *   Only content that can legally be included under the chosen license is used.
        *   Third-party code and diagrams are minimal excerpts, properly attributed, or replaced with original equivalents.
*   **Success criteria**:
    *   **Functional delivery**:
        *   The Docusaurus site builds without errors and is live on GitHub Pages.
        *   Navigation (sidebar, header links) accurately reflects the structure defined in /sp.outline.
        *   Code highlighting and basic theming are working and readable.
    *   **Content quality**:
        *   Spot-checks of technical claims against primary sources show no major inaccuracies.
        *   Example commands and code snippets run as described, given the documented environment and versions.
        *   Readers can follow end-to-end examples or mini-projects without missing steps.
    *   **Alignment with specs**:
        *   The final book structure and content align with /sp.outline; any deviations are documented and intentional.
        *   /sp.tasks reflects the major tasks required to maintain and extend the book.
    *   **Review & usability**:
        *   At least one technical peer review is completed focusing on correctness and clarity; identified major issues are fixed.
        *   At least one reader from the target audience confirms they can:
            *   Set up the project locally following the docs.
            *   Complete the primary learning outcomes and hands-on exercises defined in /sp.outline.

## Governance

*   This constitution supersedes all other project practices.
*   Amendments require documentation, approval, and a migration plan.
*   All pull requests and code reviews must verify compliance with these principles.
*   Complexity must always be justified and aligned with the principle of simplicity.
*   Refer to project-specific guidance documents for runtime development.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
