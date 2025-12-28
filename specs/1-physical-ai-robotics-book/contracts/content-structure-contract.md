# Content Structure Contract: Physical AI & Humanoid Robotics Technical Book

**Date**: 2025-12-05
**Feature**: [Link to spec.md]

## Purpose

This document defines the contract for the structure and format of all content within the technical book. It ensures consistency, readability, and adherence to Docusaurus best practices, as well as the project constitution.

## 1. Chapter File Structure

All chapters will be MDX files located within the `docs/` directory, organized by module:

```text
docs/
├── module1-ros2/
│   ├── index.mdx
│   ├── tf2.mdx
│   └── messages.mdx
├── module2-digital-twin/
│   └── index.mdx
└── ...
```

## 2. Chapter Content Contract

Each MDX chapter file MUST adhere to the following structure and conventions:

### 2.1. Frontmatter

Every chapter must start with Docusaurus frontmatter:

```markdown
---
id: <unique-chapter-id> # e.g., 'ros2-nodes-topics'
title: <Chapter Title> # e.g., 'ROS 2 Nodes and Topics'
description: <Brief summary of chapter content>
sidebar_label: <Short label for sidebar navigation>
---
```

### 2.2. Standard Sections (in order)

1.  **Abstract/Overview**: A brief (1-2 paragraphs) introduction to the chapter's topic.
2.  **Prerequisites**: List any prior knowledge or setup required for this chapter.
3.  **Learning Objectives**: Bulleted list of what the reader will be able to do after completing the chapter.
4.  **Main Content**: Detailed explanations, concepts, and step-by-step guides.
    -   Use `<h2>` for major sections, `<h3>` for sub-sections.
    -   Code examples should be clearly introduced and explained.
5.  **Code Examples**: Embedded using Markdown code blocks (e.g., ````python
    # code here
    ````) or referenced from the `code-examples/` directory.
6.  **Summary/Key Takeaways**: Bulleted list of the most important points from the chapter.
7.  **Exercises (Optional)**: Practical tasks for the reader to apply their knowledge.
8.  **Further Reading (Optional)**: Links to external resources for deeper dives.

## 3. Code Example Contract

-   **Location**: All runnable code examples MUST reside in the `code-examples/` directory, mirroring the `docs/` module structure (e.g., `code-examples/module1-ros2-examples/`).
-   **Standalone**: Each example should be runnable independently, with clear instructions in its respective chapter.
-   **File Naming**: Descriptive filenames (e.g., `publisher_node.py`, `spawn_robot.launch.py`).
-   **Readmes**: Complex examples or capstone projects may include a `README.md` in their subdirectory.
-   **Dependencies**: Explicitly state required dependencies within the code or a local `requirements.txt`.

## 4. Image and Media Contract

-   **Location**: Images and diagrams will be stored in the `static/` directory (e.g., `static/img/module1/ros_graph.png`).
-   **Embedding**: Use standard Markdown image syntax `![Alt Text](/img/module1/ros_graph.png)`.
-   **Accessibility**: All images MUST include descriptive alt text.
-   **Source**: Credit sources for any non-original images.

## 5. Terminology and Style Guide

-   **Glossary**: Maintain a consistent glossary (`docs/glossary.md`) for key terms.
-   **Voice**: Active voice, direct language.
-   **Formatting**: Consistent use of code blocks, inline code, bolding, and italics as defined in the project constitution.
-   **Citations**: All external references and ideas must be cited.
