---
id: intro-environment-setup
title: "Environment Setup"
sidebar_position: 2
---

# Environment Setup

This guide provides high-level instructions for setting up your development environment for Physical AI and Humanoid Robotics development. For detailed, step-by-step instructions, always refer to the **official documentation** of each tool.

## Operating System

**Recommended: Ubuntu 22.04 LTS (Jammy Jellyfish)**

Ubuntu 22.04 LTS is the standard platform for ROS 2 Humble and most robotics tools. You have several options:

### Option 1: Native Ubuntu Installation
- Install Ubuntu 22.04 LTS as your primary OS or dual-boot
- Best performance and compatibility

### Option 2: Virtual Machine
- Use VirtualBox, VMware, or Hyper-V
- Allocate at least 4GB RAM and 50GB disk space
- Enable hardware acceleration if available

### Option 3: WSL 2 (Windows)
- Windows Subsystem for Linux 2 on Windows 10/11
- Good for development, but limited for GPU acceleration
- May require additional setup for GUI applications

## Core Software Stack

### 1. ROS 2 (Humble Hawksbill or Iron Irwini)

ROS 2 is the robotic middleware that forms the "nervous system" of your robot.

**Installation:**
- Follow the official ROS 2 installation guide: https://docs.ros.org/en/humble/Installation.html
- Choose either Humble (LTS) or Iron (latest) based on your needs
- Ensure you source the setup script in your `.bashrc`:
  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  ```

**Verification:**
```bash
ros2 --help
```

### 2. Gazebo (Classic or Garden)

Gazebo provides physics simulation for robots.

**For ROS 2 Humble:**
- Gazebo Classic (recommended for compatibility)
- Install via: `sudo apt install ros-humble-gazebo-ros-pkgs`

**For ROS 2 Iron:**
- Gazebo Garden (newer, more features)
- Follow: https://gazebosim.org/docs/garden/install

**Verification:**
```bash
gazebo --version
```

### 3. Unity (Optional)

Unity is used for visualization and human-robot interaction scenes.

**Installation:**
- Download Unity Hub: https://unity.com/download
- Install Unity Editor (2021.3 LTS or newer)
- Install ROS-TCP-Connector package for ROS 2 integration

**Note:** Unity is optional for basic development but recommended for advanced visualization.

### 4. NVIDIA Isaac Sim & Isaac ROS

NVIDIA Isaac provides photorealistic simulation and AI perception pipelines.

**Prerequisites:**
- NVIDIA GPU with CUDA support (RTX 2060 or better recommended)
- NVIDIA drivers (version 470+)
- Docker (for Isaac ROS)

**Isaac Sim Installation:**
- Follow: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html
- Requires NVIDIA Omniverse Launcher

**Isaac ROS Installation:**
- Follow: https://nvidia-isaac-ros.github.io/getting_started/
- Uses Docker containers for consistent environments

**Verification:**
```bash
docker run --rm nvcr.io/nvidia/isaac/isaac-ros:latest
```

## Development Paths

### Path 1: Local Workstation Development

**Hardware Requirements:**
- CPU: 6+ cores (Intel i7/AMD Ryzen 7 or better)
- RAM: 16GB minimum, 32GB recommended
- GPU: NVIDIA RTX 3060 or better (for Isaac Sim)
- Storage: 100GB+ free space (SSD recommended)

**Best for:**
- Learning and development
- Simulation-heavy work
- Cost-effective for individuals

### Path 2: Cloud + Edge Deployment

**Cloud Workstation (e.g., AWS g5/g6e instances):**
- GPU-enabled cloud instances for training and simulation
- Cost: ~$1-3/hour depending on instance type
- Best for: Teams, cost-sharing, or when local hardware is limited

**Edge Device (NVIDIA Jetson):**
- Jetson Orin Nano/NX for deployment
- RealSense cameras, IMU, microphone arrays
- Best for: Real robot deployment and sim-to-real transfer

**Pattern: Train-in-Cloud, Deploy-to-Jetson**
- Develop and train in cloud workstations
- Deploy optimized models to Jetson edge devices
- Avoids the "latency trap" of cloud-based robot control

## The Latency Trap

**Why controlling real robots from the cloud is dangerous:**

- Network latency (50-200ms+) makes real-time control impossible
- Safety-critical systems require sub-10ms control loops
- Connection drops can cause robot failures or safety hazards

**Solution:**
- Train models in the cloud (fast iteration, powerful GPUs)
- Deploy inference to edge devices (low latency, reliable)
- Use cloud for monitoring and data collection, not control

## Quick Setup Checklist

- [ ] Install Ubuntu 22.04 LTS
- [ ] Install ROS 2 Humble or Iron
- [ ] Install Gazebo
- [ ] (Optional) Install Unity Hub and Editor
- [ ] Install NVIDIA drivers (if using GPU)
- [ ] Install Docker (for Isaac ROS)
- [ ] Install Isaac Sim via Omniverse Launcher
- [ ] Verify all installations with test commands
- [ ] Set up ROS 2 workspace: `mkdir -p ~/ros2_ws/src`

## Getting Help

If you encounter issues:

1. **Check official documentation first** - Most tools have excellent docs
2. **ROS 2 Discourse**: https://discourse.ros.org/
3. **Gazebo Forums**: https://community.gazebosim.org/
4. **NVIDIA Isaac Forums**: https://forums.developer.nvidia.com/c/omniverse/isaac-sim/

## Next Steps

Once your environment is set up:

- [Return to Introduction](./index.md)
- [Review the Glossary](../glossary.md)
- [Start Module 1: ROS 2](../module1/index.md)

