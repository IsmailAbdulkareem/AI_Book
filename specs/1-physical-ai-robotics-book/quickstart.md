# Quickstart Guide: Physical AI & Humanoid Robotics Technical Book

**Date**: 2025-12-05
**Feature**: [Link to spec.md]

## Purpose

This guide provides a rapid setup for developers and readers to get the Docusaurus book and its associated code examples up and running locally.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

-   **Git**: For cloning the repository.
    ```bash
    sudo apt update
    sudo apt install git
    ```
-   **Node.js (LTS) & npm**: For Docusaurus development.
    ```bash
    curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
    sudo apt-get install -y nodejs
    ```
-   **Python 3.x**: For ROS 2 code examples.
    ```bash
    sudo apt install python3 python3-pip
    ```
-   **Ubuntu 22.04 LTS**: The primary development environment.

## 2. Local Setup

Follow these steps to set up the project locally:

### 2.1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-ai-robotics-book.git
cd physical-ai-robotics-book
```

### 2.2. Install Docusaurus Dependencies

Navigate to the repository root and install Node.js dependencies:

```bash
npm install
```

### 2.3. Start the Docusaurus Development Server

This will launch the book locally in your browser, typically at `http://localhost:3000`.

```bash
npm run start
```

### 2.4. Build the Docusaurus Site (for deployment preview)

To generate a static build of the book:

```bash
npm run build
```

The static assets will be generated in the `build/` directory.

## 3. Running Code Examples

Code examples are located in the `code-examples/` directory. Each module will have its own subdirectory.

### 3.1. General Python Examples

To run a standalone Python script:

```bash
python code-examples/module1-ros2-examples/publisher_node.py
```

### 3.2. ROS 2 Examples

For ROS 2 examples, ensure ROS 2 Humble/Iron is installed and sourced.

```bash
source /opt/ros/humble/setup.bash # or iron
ros2 run <package_name> <node_executable>
```

Refer to the specific chapter for detailed instructions on running its associated code examples.
