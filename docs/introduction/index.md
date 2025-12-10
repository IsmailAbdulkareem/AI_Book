---
id: introduction
title: "Introduction & Foundations"
sidebar_position: 1
---

# Introduction & Foundations

## What is Physical AI?

Physical AI represents the intersection of artificial intelligence and embodied systems—robots that exist in and interact with the physical world. Unlike "pure digital" AI that operates solely in computational spaces, Physical AI must contend with:

- **Real-world physics**: Gravity, friction, collisions, and sensor noise
- **Temporal constraints**: Real-time decision making and control loops
- **Uncertainty**: Imperfect sensors, unpredictable environments, and partial observability
- **Safety**: Physical systems can cause harm if not properly controlled

This book focuses on **humanoid robotics** as the ultimate expression of Physical AI—systems that can navigate human spaces, manipulate objects, and interact naturally with people.

## Course Overview

This technical book is structured around a 13-week course/quarter, organized into four core modules plus a capstone project:

### Module 1: The Robotic Nervous System (ROS 2)
Learn ROS 2 as the foundational communication framework that enables distributed robot systems. Topics include nodes, topics, services, actions, and message interfaces.

### Module 2: The Digital Twin (Gazebo & Unity)
Master simulation environments for developing and testing robots before deploying to physical hardware. Learn Gazebo for physics simulation and Unity for visualization.

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
Explore NVIDIA Isaac Sim and Isaac ROS for photorealistic simulation, synthetic data generation, and advanced perception pipelines including VSLAM and object detection.

### Module 4: Vision-Language-Action (VLA)
Build end-to-end systems that translate natural language commands into robot actions, combining speech recognition, LLM-based planning, and ROS 2 action execution.

### Capstone Project: The Autonomous Humanoid
Integrate all modules to create a simulated humanoid robot that can receive voice commands, plan tasks, navigate obstacles, and manipulate objects.

## Prerequisites

Before diving into this book, you should have:

- **Python proficiency**: Comfortable with Python 3.8+, including object-oriented programming
- **AI basics**: Understanding of machine learning concepts (neural networks, training/inference)
- **Linux comfort**: Ability to use command-line tools, package managers, and basic shell scripting
- **Git familiarity**: Version control basics for managing code and configurations

## Weekly Breakdown (Weeks 1-13)

### Weeks 1-3: Module 1 - ROS 2 Foundations
- Week 1: ROS 2 architecture, nodes, and topics
- Week 2: Services, actions, and launch files
- Week 3: TF2, URDF, and assessment project

### Weeks 4-6: Module 2 - Digital Twin
- Week 4: Gazebo basics and URDF/SDF modeling
- Week 5: Sensor simulation and ros2_control
- Week 6: Unity integration and assessment

### Weeks 7-9: Module 3 - NVIDIA Isaac
- Week 7: Isaac Sim setup and basic simulation
- Week 8: Isaac ROS perception pipelines
- Week 9: Sim-to-real transfer concepts and assessment

### Weeks 10-12: Module 4 - VLA
- Week 10: Speech recognition and LLM integration
- Week 11: Task planning from natural language
- Week 12: ROS 2 action execution and assessment

### Week 13: Capstone Project
- Integration of all modules
- End-to-end autonomous humanoid demonstration

## How to Use This Book

Each module follows a consistent structure:

1. **Overview**: High-level concepts and motivation
2. **Core Concepts**: Detailed explanations with examples
3. **Guided Examples**: Step-by-step tutorials
4. **Assessment**: Practical exercises to reinforce learning

You can read this book linearly, or jump to specific modules based on your interests. However, we recommend following the module order, as later modules build on earlier concepts.

## Next Steps

- [Set up your development environment](./environment-setup.md)
- [Review the glossary](../glossary.md) for key terms
- Begin with [Module 1: The Robotic Nervous System](../module1/index.md)

