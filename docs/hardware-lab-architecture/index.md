---
id: hardware-lab-architecture
title: "Hardware & Lab Architecture"
sidebar_position: 8
---

# Hardware & Lab Architecture

## Why Physical AI Needs Serious Compute

Physical AI systems are computationally demanding because they must:

- **Process sensor data in real-time**: Cameras, LiDAR, IMU generate massive data streams
- **Run AI models at inference speed**: Perception, planning, and control require low latency
- **Handle multiple parallel tasks**: Navigation, manipulation, and communication simultaneously
- **Support sim-to-real development**: Training models in simulation before deployment

This section describes the hardware requirements and lab setup options for Physical AI development.

## Digital Twin Workstation Requirements

Your development workstation runs simulations, trains models, and develops code before deploying to physical robots.

### Minimum Requirements

- **CPU**: 6+ cores (Intel i7/AMD Ryzen 7 or equivalent)
- **RAM**: 16GB (32GB recommended for large simulations)
- **GPU**: NVIDIA RTX 3060 or better (required for Isaac Sim)
- **Storage**: 100GB+ free space (SSD recommended for faster I/O)
- **OS**: Ubuntu 22.04 LTS

### Recommended Configuration

- **CPU**: 12+ cores (Intel i9/AMD Ryzen 9)
- **RAM**: 32GB or 64GB
- **GPU**: NVIDIA RTX 4070 or better (RTX 4090 for best performance)
- **Storage**: 500GB+ NVMe SSD
- **Network**: Gigabit Ethernet for cloud integration

### Why GPU Matters

- **Isaac Sim**: Requires RTX GPU for real-time ray tracing
- **Model Training**: GPU acceleration speeds up training by 10-100x
- **Real-time Inference**: Some models require GPU for acceptable latency

## Physical AI Edge Kit

For deploying to real robots, you need edge computing devices that can run AI models with low latency.

### NVIDIA Jetson Options

**Jetson Orin Nano** (Recommended):
- **GPU**: 1024-core NVIDIA Ampere architecture
- **RAM**: 8GB
- **Power**: 7-15W
- **Best for**: Small robots, cost-sensitive projects

**Jetson Orin NX**:
- **GPU**: 1024-core NVIDIA Ampere architecture
- **RAM**: 16GB
- **Power**: 10-25W
- **Best for**: Medium robots, more complex AI workloads

**Jetson AGX Orin**:
- **GPU**: 2048-core NVIDIA Ampere architecture
- **RAM**: 32GB or 64GB
- **Power**: 15-60W
- **Best for**: Large robots, research platforms

### Sensor Suite

**Depth Camera** (Intel RealSense D435/D455):
- RGB-D sensing for navigation and manipulation
- ROS 2 support via `realsense-ros`
- ~$200-300

**IMU** (Bosch BMI160 or similar):
- Inertial measurement for odometry and stabilization
- Integrated into many robot platforms
- ~$20-50

**Microphone Array** (ReSpeaker or similar):
- Multi-microphone array for voice commands
- Beamforming for noise reduction
- ~$50-100

**LiDAR** (Optional, for advanced navigation):
- 2D or 3D LiDAR for mapping and localization
- More expensive ($500-2000+)
- Often integrated into robot platforms

## Robot Lab Options

### Tier 1: Proxy Robot

**Options**:
- **Unitree Go2**: Quadruped robot, ~$1,600
- **Robotic Arm**: 6-DOF arm (e.g., Dynamixel-based), ~$500-2000
- **Mobile Base**: Differential drive platform, ~$300-800

**Pros**:
- Lower cost entry point
- Good for learning ROS 2 and basic behaviors
- Less risk of damage during development

**Cons**:
- Limited manipulation capabilities (for non-arm options)
- May not represent humanoid challenges
- Less impressive for demonstrations

### Tier 2: Miniature Humanoid

**Options**:
- **Hiwonder XArm**: Small humanoid, ~$300-500
- **OP3 (Open Platform 3)**: Research humanoid, ~$1,000-1,500
- **Unitree G1 Mini**: Compact humanoid, ~$3,000-5,000

**Pros**:
- Humanoid form factor (bipedal, arms, head)
- Good for manipulation and navigation research
- More realistic for humanoid AI development

**Cons**:
- Higher cost than proxy robots
- May have limited payload and stability
- Requires more careful control

### Tier 3: Premium Lab (Unitree G1)

**Unitree G1** (Full-size humanoid):
- **Cost**: ~$16,000-20,000
- **Capabilities**: Full humanoid with advanced manipulation
- **Best for**: Research labs, sim-to-real transfer, publication-quality work

**Pros**:
- Industry-leading humanoid platform
- Excellent sim-to-real transfer capabilities
- Impressive demonstrations and research outcomes

**Cons**:
- High cost (may require grant funding)
- Requires significant space and safety considerations
- May be overkill for learning/teaching

## Ether Lab (Cloud-Native) Setup

For teams or individuals without local GPU workstations, cloud-based development is an option.

### Cloud Workstation Options

**AWS EC2 Instances**:
- **g5.xlarge**: 1x A10G GPU, 4 vCPU, 16GB RAM (~$1.00/hour)
- **g5.2xlarge**: 1x A10G GPU, 8 vCPU, 32GB RAM (~$1.50/hour)
- **g6e.xlarge**: 1x L4 GPU, 4 vCPU, 32GB RAM (~$0.75/hour)

**Google Cloud Platform**:
- **n1-standard-4 + T4 GPU**: ~$0.50-1.00/hour
- **a2-highgpu-1g**: 1x A100 GPU, ~$3.00/hour

**Azure**:
- **NC6s v3**: 1x V100 GPU, ~$2.00/hour
- **NCas_T4_v3**: 1x T4 GPU, ~$0.80/hour

### Cost Considerations

**OpEx (Operational Expenditure) - Cloud**:
- Pay-as-you-go pricing
- No upfront hardware investment
- Easy to scale up/down
- Can become expensive with heavy usage

**CapEx (Capital Expenditure) - Local**:
- One-time hardware purchase
- No ongoing compute costs
- Better for long-term, intensive use
- Requires upfront investment

### When to Use Cloud

- **Limited budget**: Can't afford local GPU workstation
- **Team sharing**: Multiple developers can share instances
- **Occasional use**: Don't need GPU 24/7
- **Experimentation**: Try different GPU types before buying

### When to Use Local

- **Intensive development**: Using GPU many hours per day
- **Data privacy**: Sensitive robot data shouldn't leave premises
- **Low latency needs**: Local development is faster
- **Long-term projects**: Cost-effective over 6+ months

## The Latency Trap

### Why Cloud Control is Dangerous

**Network Latency**:
- Typical cloud latency: 50-200ms (depending on location)
- Robot control loops require: &lt;10ms for safety
- Result: Unstable, unsafe robot behavior

**Connection Reliability**:
- Network drops cause robot to lose control
- Safety-critical systems cannot tolerate interruptions
- Real robots can cause physical harm if uncontrolled

### The Solution: Train-in-Cloud, Deploy-to-Jetson

**Development Phase (Cloud)**:
1. Train AI models in cloud workstations (fast, powerful GPUs)
2. Develop and test code in cloud simulations
3. Iterate quickly without local hardware constraints

**Deployment Phase (Edge)**:
1. Deploy optimized models to Jetson edge devices
2. Run inference locally on the robot (low latency, reliable)
3. Use cloud only for monitoring and data collection

**Pattern**:
```
Cloud Workstation (Training)
    ↓ (export model)
Jetson Edge Device (Inference)
    ↓ (robot control)
Physical Robot
```

## Lab Planning Guidance

### For Instructors

**Budget Considerations**:
- **Minimal**: Cloud workstations + proxy robots (~$2,000-5,000 total)
- **Standard**: Local workstations + miniature humanoids (~$10,000-20,000)
- **Premium**: High-end workstations + Unitree G1 (~$30,000-50,000)

**Student Access**:
- Shared lab workstations for simulation
- Rotating access to physical robots
- Cloud backup for overflow capacity

### For Self-Learners

**Starting Point**:
1. **Cloud workstation** for initial learning (AWS free tier or low-cost instances)
2. **Proxy robot** (Unitree Go2 or robotic arm) for hands-on experience
3. **Jetson Orin Nano** for edge deployment

**Progression**:
- Start with simulation-only (Gazebo, Isaac Sim)
- Add proxy robot for real-world experience
- Upgrade to humanoid when ready for advanced projects

### Budget Planning Worksheet

| Item | Tier 1 (Minimal) | Tier 2 (Standard) | Tier 3 (Premium) |
|------|------------------|-------------------|------------------|
| Workstation | Cloud ($100-500/mo) | Local ($2,000-4,000) | Local ($5,000-8,000) |
| Edge Device | Jetson Orin Nano ($500) | Jetson Orin NX ($700) | Jetson AGX Orin ($2,000) |
| Sensors | RealSense ($300) | RealSense + IMU ($400) | Full suite ($1,000) |
| Robot | Proxy ($500-2,000) | Mini Humanoid ($1,000-5,000) | Unitree G1 ($16,000) |
| **Total** | **$1,300-3,300** | **$4,100-10,100** | **$24,000-27,000** |

## Next Steps

- [Capstone Project: The Autonomous Humanoid](../capstone_workflow/index.md)
- [Appendices & Resources](../appendix/resources.md)

