---
id: 0001
title: ROS 2 Python Testing Frameworks
stage: general
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 1-physical-ai-robotics-book
user: user
command: Research common testing frameworks and best practices for ROS 2 Python packages, specifically for humanoid robotics. Focus on identifying suitable frameworks for unit, integration, and contract testing. Summarize the findings, including pros and cons of each, and recommend a specific approach.
labels: ["ROS 2", "Python", "Testing", "Humanoid Robotics", "Unit Testing", "Integration Testing", "Contract Testing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - D:\spec-driven-dev\ai-native\history\prompts\general\0001-ros2-python-testing-frameworks.general.prompt.md
tests:
  - N/A
---

## Prompt

Research common testing frameworks and best practices for ROS 2 Python packages, specifically for humanoid robotics. Focus on identifying suitable frameworks for unit, integration, and contract testing. Summarize the findings, including pros and cons of each, and recommend a specific approach.

## Response snapshot

### ROS 2 Python Testing Frameworks and Best Practices for Humanoid Robotics

For developing robust ROS 2 Python packages, especially for complex systems like humanoid robots, a comprehensive testing strategy is essential. This involves a combination of unit, integration, and, implicitly, contract testing.

### 1. Unit Testing

Unit testing focuses on verifying individual components or functions in isolation, without involving ROS interfaces.

*   **`unittest` (Python's built-in framework)**
    *   **Pros:**
        *   Standard library, no external dependencies.
        *   Well-documented and widely understood by Python developers.
        *   Often used in conjunction with `launch_testing` for ROS-specific unit tests.
    *   **Cons:**
        *   Can be more verbose compared to `pytest`.
        *   Fixture management can be less flexible.

*   **`pytest` (Third-party Python framework)**
    *   **Pros:**
        *   More concise and readable test syntax.
        *   Powerful fixture system for managing test setup and teardown.
        *   Extensible with a rich plugin ecosystem.
        *   Good for general Python unit testing without ROS.
    *   **Cons:**
        *   Requires an additional installation.
        *   While `launch_testing` can use `pytest` for active tests, post-shutdown tests are not yet fully supported (as per search results).

### 2. Integration Testing

Integration testing verifies the interactions between multiple ROS nodes and the overall system behavior.

*   **`launch_testing`**
    *   **Pros:**
        *   Primary and officially supported tool for integration testing in ROS 2.
        *   Extends Python launch files to include tests that run concurrently with nodes and after shutdown.
        *   Relies on Python's `unittest` module for assertions, making it familiar to Python developers.
        *   Designed to prevent crosstalk between parallel tests using unique ROS domain IDs.
        *   Can launch multiple nodes and verify their interactions.
    *   **Cons:**
        *   Steeper learning curve due to its integration with ROS 2 launch system.
        *   Post-shutdown tests with `pytest` are not fully supported.

### 3. Physical Integration Testing

For robotics, especially humanoid, physical integration testing is crucial to validate the combined functionality of hardware and software.

*   **`ROSPIT2` (Physical Integration Testing)**
    *   **Pros:**
        *   Specifically designed for physical integration testing in ROS 2.
        *   Supports declarative test execution using XML specifications.
        *   Provides a low-level testing framework with pre- and post-conditions, and invariants.
        *   Developed by AIST, Japan, suggesting a focus on robust robotics applications.
    *   **Cons:**
        *   May have a steeper learning curve due to its specialized nature.
        *   Likely requires specific setup for hardware interaction.
        *   Less information available on general usage and community support compared to more common frameworks.

### 4. Contract Testing

While no explicit "contract testing" frameworks were identified in the ROS 2 Python context, the principles of contract testing (ensuring components adhere to agreed-upon interfaces) can be addressed through robust integration tests using `launch_testing`. These tests would validate communication patterns, message types, and data flow between ROS nodes, effectively serving as contract checks.

### Recommendation

For ROS 2 Python packages in humanoid robotics, a multi-tiered testing approach is recommended:

1.  **Unit Testing:** Use `pytest` for its conciseness, powerful features, and broad community adoption for individual Python components that do not directly interact with ROS. For components that have minimal ROS interaction but are still primarily Python logic, `pytest` can be integrated with mock objects for ROS interfaces.

2.  **Integration Testing:** Leverage `launch_testing` as the primary tool for integration tests. This framework is purpose-built for ROS 2 and allows for comprehensive testing of node interactions, communication, and overall system behavior in a simulated ROS environment. Use `unittest` assertions within `launch_testing` for consistency and broad compatibility.

3.  **Physical Integration Testing:** For hardware-in-the-loop or real-robot testing, explore `ROSPIT2`. This specialized framework is designed for physical integration and can help validate the combined functionality of your humanoid robot's hardware and software.

4.  **Continuous Integration (CI):** Integrate all tests into a CI pipeline to ensure that every code change is automatically tested, catching regressions early.

This approach provides a robust testing pyramid, starting with fast and isolated unit tests, progressing to integration tests that verify system-level interactions, and culminating in physical integration tests for real-world validation.

Sources:
- [theconstruct.ai](https://vertexaisearch.cloud.google.com/grounding-api-redirect/AUZIYQEbKr1CPuuK-jxkG00Cc_ZuSwkdJM_J1XUFS8WyrmBSUQgYLOAm9NTEZHIzg-wlJ1RauCkLxYocGlb9aa98hmkc_iO4HdcckZP61A9M3PGsnuwJ9avt6R-o_PyCJW-Q1QmoLNev3VpxUlM8-VqZoS_u1ldMJLc=)
- [arnebaeyens.com](https://vertexaisearch.cloud.google.com/grounding-api-redirect/AUZIYQGx_lSzdo4knTHwe-E4qGIxp81PQ8ZuAhKNOnk2rYi_UIbAGoXDFef0Rawf9rfgyinZFxH9RhI1icjHCs9ZlTA-HuT0t7gERwTcRzixRuJF00hmzK6etvBax904q4q3IFKG_PNw6kAWU3ABykB7rPEwOB42z8V6)
- [youtube.com](https://vertexaisearch.cloud.google.com/grounding-api-redirect/AUZIYQFIiBF62SfSyLm_qlCDC7-cssWZeqSz6SjMgWZbYSJ3NasmV66wgD8FqiN5zgr9V8sNwmokCcj5CgFhz25x5RWI0IlojvWU7t2_XSRh-4yjXYHuKiALRMWYykvkqOmnH-GZdR69H2s=)
- [github.com](https://vertexaisearch.cloud.google.com/grounding-api-redirect/AUZIYQGGrvkj-WgBWtSQ1AEPav0ba2nt5zyjhLsW2NwhpU51pzaScoDeoAbXERGwBRnGVW5FHOSTwUiFtK_SzcpOVNPyo98dmr8IMqFZB4WB2A0iLiknLfUR-77Ou6uvAaE=)

## Outcome

- ✅ Impact: Provided a detailed summary of ROS 2 Python testing frameworks and a recommended approach for humanoid robotics.
- 🧪 Tests: N/A
- 📁 Files: Created a PHR file at D:\spec-driven-dev\ai-native\history\prompts\general\0001-ros2-python-testing-frameworks.general.prompt.md
- 🔁 Next prompts: None
- 🧠 Reflection: The web search provided relevant information. The summary covers unit, integration, and physical integration testing, with a recommendation that combines different frameworks.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
