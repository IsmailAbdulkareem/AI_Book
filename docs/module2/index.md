---
id: module2-digital-twin
title: "Module 2: The Digital Twin (Gazebo & Unity)"
sidebar_position: 5
---

# Module 2: The Digital Twin (Gazebo & Unity)

**Master physics simulation and visualization for robot development**

---

## Module Overview

This module teaches you to create digital twins—virtual representations of physical robots. You'll learn Gazebo for physics simulation and Unity for high-quality visualization, enabling you to develop and test robots without physical hardware.

**By the end of this module, you will**:

* Understand digital twin concepts and benefits
* Set up Gazebo simulation environments
* Spawn and control robots in Gazebo
* Integrate sensors (cameras, LiDAR, IMU) in simulation
* Use Unity for photorealistic visualization
* Build human-robot interaction scenes

---

## Prerequisites

* **Module 1 Complete**: ROS 2 fundamentals and URDF modeling
* **Ubuntu 22.04**: With ROS 2 Humble/Iron installed
* **Gazebo**: Installed via ROS 2 packages
* **Unity** (Optional): For visualization chapters

---

## Module Structure

### Chapter 1: Gazebo Simulation Basics

**Type**: Theory-to-Practice  
**Lessons**: 6  
**Duration**: 10-12 hours

**You will learn**:

* Gazebo's role in robotics development
* Setting up simulation environments
* Spawning robots from URDF/SDF
* Physics engines and simulation parameters
* Sensor simulation (cameras, LiDAR, IMU)
* Visualization and debugging tools

→ [Start Chapter 1](./chapter-1-gazebo-basics.md)

---

### Chapter 2: Robot Control in Gazebo

**Type**: Practice-Heavy  
**Lessons**: 6  
**Duration**: 12-14 hours

**You will learn**:

* Controlling robot joints via ROS 2 topics
* ros2_control for hardware abstraction
* Position, velocity, and effort control modes
* Mobile base control (differential drive)
* Manipulator arm control
* Trajectory execution

→ [Start Chapter 2](./chapter-2-robot-control.md)

---

### Chapter 3: Unity Integration for Visualization

**Type**: Practice-Heavy  
**Lessons**: 4  
**Duration**: 8-10 hours

**You will learn**:

* Unity's role in robotics visualization
* ROS-TCP-Connector setup
* Basic robot visualization in Unity
* Sensor data streaming from ROS 2
* Human-robot interaction scenes

→ [Start Chapter 3](./chapter-3-unity-integration.md)

---

## Module Learning Path

```
Chapter 1: Gazebo Basics
    ↓ (Understand simulation fundamentals)
Chapter 2: Robot Control
    ↓ (Control robots in simulation)
Chapter 3: Unity Visualization
    ↓ (High-quality visualization)
───────────────────────────────────
Module 2 Complete! → Module 3: AI-Robot Brain
```

---

## Estimated Time Commitment

| Component                 | Time            |
| ------------------------- | --------------- |
| **Reading & Theory**      | 10-12 hours     |
| **Hands-On Practice**     | 20-26 hours     |
| **Projects**              | 8-12 hours      |
| **Total Module 2**        | **38-50 hours** |

**Recommended Pace**: 10-15 hours per week = 3-4 weeks to complete

---

## What You'll Build

### Chapter 1 Projects

* Spawn simple robot in Gazebo
* Add sensors (camera, LiDAR, IMU)
* Tune physics parameters

### Chapter 2 Projects

* Control joints via ROS 2 topics
* Set up ros2_control
* Control mobile base navigation
* Control manipulator arm

### Chapter 3 Projects

* Unity robot visualization
* Sensor data streaming
* Human-robot interaction scene

---

## Success Criteria

You have successfully completed Module 2 when you can:

* Spawn robots in Gazebo from URDF/SDF
* Simulate sensors and access data via ROS 2
* Control robots using ros2_control
* Visualize robots in Unity
* Create basic human-robot interaction scenes

---

## Tools & Resources

### Required Software

* **Gazebo**: Classic (ROS 2 Humble) or Garden (ROS 2 Iron)
* **ROS 2**: Humble or Iron
* **Unity** (Optional): 2021.3 LTS or newer
* **ROS-TCP-Connector**: For Unity integration

### Installation Guides

* [Gazebo Installation](https://classic.gazebosim.org/tutorials?cat=install)
* [Unity Installation](https://unity.com/download)
* [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

### Reference Documentation

* [Gazebo Documentation](https://classic.gazebosim.org/documentation)
* [ros2_control](https://control.ros.org/)
* [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

---

## Ready to Begin?

**Start with Chapter 1: Gazebo Simulation Basics**

Learn to create and control robots in simulation.

[Start Chapter 1 →](./chapter-1-gazebo-basics.md)
