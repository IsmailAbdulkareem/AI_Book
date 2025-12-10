---
id: module1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_position: 4
---

# Module 1: The Robotic Nervous System (ROS 2)

**Master the middleware that connects robotic intelligence to physical hardware**

---

## Module Overview

This module teaches you ROS 2 (Robot Operating System 2), the middleware framework that enables complex robotic systems. You'll learn how robots communicate, coordinate actions, and integrate sensors through a distributed architecture designed for real-time physical systems.

**By the end of this module, you will**:

* Understand Physical AI and how it differs from digital AI
* Build and run ROS 2 nodes using Python
* Implement publish-subscribe, service, and action communication patterns
* Create URDF models describing humanoid robot structure
* Integrate sensors and actuators through ROS 2

---

## Prerequisites

* **Python Basics**: Variables, functions, loops, classes
* **Command Line**: Basic terminal usage (cd, ls, mkdir)
* **Development Environment**: Ubuntu 22.04 or WSL2 with ROS 2 Humble/Iron installed

**No robotics knowledge required** - we start from foundational concepts.

---

## Module Structure

### Chapter 1: Introduction to Physical AI

**Type**: Theory-Only (Foundation)  
**Lessons**: 8  
**Duration**: 8-10 hours

**You will learn**:

* What Physical AI is and why it matters
* Real-world constraints that make robotics challenging
* Current humanoid robot landscape (Atlas, Optimus, Figure 01)
* Sensor systems: LIDAR, cameras, IMUs, force/torque sensors
* Why sensor fusion is critical for robust perception

→ [Start Chapter 1](./chapter-1-introduction-to-physical-ai.md)

---

### Chapter 2: ROS 2 Architecture and Core Concepts

**Type**: Theory-to-Practice  
**Lessons**: 8 (5 theory + 3 code)  
**Duration**: 12-16 hours

**You will learn**:

* ROS 2 as middleware (not an operating system)
* DDS distributed architecture and why it replaced ROS 1
* Nodes, topics, services, and actions communication patterns
* Creating your first minimal ROS 2 node
* QoS (Quality of Service) policies for reliable communication
* Launch files for multi-node orchestration

→ [Start Chapter 2](./chapter-2-ros2-architecture.md)

---

### Chapter 3: Python-ROS Integration with rclpy

**Type**: Practice-Heavy  
**Lessons**: 8  
**Duration**: 14-18 hours

**You will learn**:

* Publishers and subscribers for streaming data
* Service servers and async clients for request-response
* Parameters for runtime configuration
* Callback groups for concurrent operations
* Timers for multi-rate control loops
* Executors for system composition
* **Capstone**: 3-node sensor-filter-controller system

→ [Start Chapter 3](./chapter-3-python-ros-integration.md)

---

### Chapter 4: Robot Description with URDF and Xacro

**Type**: Theory-to-Practice  
**Lessons**: 8 (3 theory + 5 code)  
**Duration**: 16-20 hours

**You will learn**:

* URDF as the blueprint for robot structure
* Links, joints, and kinematic chains
* Visual vs collision geometry trade-offs
* Xacro for parameterized, reusable robot descriptions
* Bipedal humanoid design considerations
* Sensor integration (cameras, LIDAR, IMU) in URDF
* Validation and visualization with RViz

→ [Start Chapter 4](./chapter-4-urdf-robot-description.md)

---

## Module Learning Path

```
Chapter 1: Physical AI Foundations
    ↓ (Understand WHAT robots need and WHY)
Chapter 2: ROS 2 Architecture
    ↓ (Learn HOW robots communicate)
Chapter 3: Python/rclpy Integration
    ↓ (Build complete robot control systems)
Chapter 4: URDF Robot Modeling
    ↓ (Define robot physical structure)
───────────────────────────────────
Module 1 Complete! → Module 2: Simulation
```

---

## Estimated Time Commitment

| Component                 | Time            |
| ------------------------- | --------------- |
| **Reading & Theory**      | 20-25 hours     |
| **Hands-On Coding**       | 20-30 hours     |
| **Challenges & Projects** | 10-15 hours     |
| **Total Module 1**        | **50-70 hours** |

**Recommended Pace**: 10-15 hours per week = 4-6 weeks to complete

---

## What You'll Build

### Chapter 2 Projects

* Minimal ROS 2 "Hello World" node
* QoS-configured sensor publisher
* Multi-node system with launch file

### Chapter 3 Projects

* Temperature sensor publisher-subscriber pair
* Validation service (request-response)
* Parameter-driven configurable node
* Multi-rate sensor fusion controller
* **Capstone**: 3-node perception pipeline

### Chapter 4 Projects

* Parameterized humanoid leg URDF
* Bilateral arm macro with Xacro
* Complete humanoid with integrated sensors
* **Capstone**: Validated full humanoid model in RViz

---

## Success Criteria

You have successfully completed Module 1 when you can:

* Explain Physical AI and sensor fusion principles
* Create and run ROS 2 nodes using Python
* Implement all communication patterns (topics, services, actions)
* Design multi-node systems with launch files
* Build URDF models for humanoid robots
* Integrate sensors and validate robot descriptions

---

## Tools & Resources

### Required Software

* **ROS 2**: Humble or Iron (Ubuntu 22.04 / WSL2)
* **Python**: 3.10+
* **Colcon**: ROS 2 build tool
* **RViz**: Visualization tool (included with ROS 2)

### Installation Guides

* [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
* [Colcon Installation](https://colcon.readthedocs.io/en/released/user/installation.html)

### Reference Documentation

* [ROS 2 Documentation](https://docs.ros.org/en/humble/)
* [rclpy API Reference](https://docs.ros2.org/latest/api/rclpy/)
* [URDF Specification](http://wiki.ros.org/urdf/XML)

---

## Ready to Begin?

**Start with Chapter 1: Introduction to Physical AI**

Master the foundational concepts before diving into ROS 2 implementation.

[Start Chapter 1 →](./chapter-1-introduction-to-physical-ai.md)
