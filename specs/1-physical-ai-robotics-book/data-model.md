# Conceptual Data Model: Physical AI & Humanoid Robotics Technical Book

**Date**: 2025-12-05
**Feature**: [Link to spec.md]

## Purpose

This document outlines the conceptual structure and relationships of the content within the technical book. It is not a traditional software data model, but rather a guide for organizing information and ensuring consistency across chapters and code examples.

## Key Entities and Relationships

### 1. Book Module
- **Description**: A high-level organizational unit, typically corresponding to a major part of the book (e.g., "ROS 2 Fundamentals", "Digital Twin Simulation").
- **Attributes**: Title, Description, Learning Objectives.
- **Relationships**: Contains multiple Chapters.

### 2. Chapter
- **Description**: A self-contained section within a Module, focusing on a specific topic (e.g., "Nodes and Topics", "URDF Modeling").
- **Attributes**: Title, Abstract, Prerequisites, Learning Objectives, Content Body (MDX), Key Takeaways, Exercises, Further Reading.
- **Relationships**: Belongs to one Module; May reference multiple Code Examples.

### 3. Content Section
- **Description**: A sub-section within a Chapter, breaking down the topic further.
- **Attributes**: Heading, Content Body (MDX).

### 4. Code Example
- **Description**: A runnable code snippet or small project illustrating a concept.
- **Attributes**: File Path, Language, Description, Dependencies, Expected Output, Explanation.
- **Relationships**: Referenced by one or more Chapters.

### 5. Glossary Term
- **Description**: A definition for a key technical term used in the book.
- **Attributes**: Term, Definition, References.
- **Relationships**: Referenced by Content Sections and Chapters.

### 6. Image/Diagram
- **Description**: Visual aids embedded in the content.
- **Attributes**: File Path, Caption, Alt Text, Source.
- **Relationships**: Embedded in Content Sections and Chapters.

## Content Flow and Structure

```mermaid
graph TD
    A[Book] --> B{Modules}
    B --> C1[Module 1: ROS 2]
    B --> C2[Module 2: Digital Twin]
    B --> C3[Module 3: NVIDIA Isaac]
    B --> C4[Module 4: VLA]
    C1 --> D1[Chapter 1.1: Nodes & Topics]
    C1 --> D2[Chapter 1.2: Services & Actions]
    D1 --> E1[Section: Publisher Node]
    D1 --> E2[Section: Subscriber Node]
    E1 --> F1[Code Example: publisher_node.py]
    E2 --> F2[Code Example: subscriber_node.py]
    D1 --> G1[Glossary Term: Node]
    D1 --> G2[Glossary Term: Topic]
    E1 --> H1[Image: ROS Graph]
    C1 --> I1[Assessment: ROS 2 Project]
```

## Data Model Decisions

- **MDX for Content**: All narrative and conceptual content will be written in MDX to allow for rich media and interactive components.
- **External Code Examples**: Code will be kept in a separate `code-examples/` directory and referenced, rather than embedded directly, to ensure reproducibility and independent testing.
- **Version Control**: Git will be used for versioning all content and code, with Docusaurus providing additional versioning capabilities for book editions.
