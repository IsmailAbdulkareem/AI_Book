---
id: module2-chapter3
title: "Chapter 3: Unity Integration for Visualization"
sidebar_position: 3
---

# Chapter 3: Unity Integration for Visualization

**Type**: Practice-Heavy  
**Lessons**: 4  
**Duration**: 8-10 hours

## Chapter Overview

This chapter introduces Unity as a visualization and interaction platform for robotics. While Gazebo handles physics, Unity excels at photorealistic rendering and human-robot interaction scenes.

**By the end of this chapter, you will**:
- Understand Unity's role in robotics visualization
- Set up ROS-TCP-Connector for ROS 2 integration
- Create basic robot visualization in Unity
- Stream sensor data from ROS 2 to Unity
- Build simple human-robot interaction scenes

## Unity for Robotics

### Why Unity?

**Strengths**:
- **Photorealistic Rendering**: High-quality graphics
- **Human-Robot Interaction**: Natural interaction scenes
- **VR/AR Support**: Immersive training environments
- **Game-Like Interfaces**: Intuitive control and monitoring

**Use Cases**:
- Visualization of robot behavior
- Human-robot interaction design
- Training data generation
- Presentation and demos

### Unity vs. Gazebo

**Gazebo**: Physics simulation, sensor models, robot control  
**Unity**: Visualization, interaction, presentation

**Best Practice**: Use both!
- Develop in Gazebo (physics accuracy)
- Visualize in Unity (presentation quality)

## ROS-TCP-Connector Setup

### Installation

1. **Install Unity Hub** and Unity Editor (2021.3 LTS or newer)

2. **Install ROS-TCP-Connector**:
   - Download from: https://github.com/Unity-Technologies/ROS-TCP-Connector
   - Import into Unity project

3. **Configure ROS 2 Endpoint**:
   ```python
   # In Unity, set ROS IP and port
   # Default: localhost:10000
   ```

### ROS 2 Bridge

Unity communicates with ROS 2 via TCP:

```
Unity Editor
    ↓ (TCP)
ROS-TCP-Endpoint (ROS 2 node)
    ↓ (ROS 2 topics)
ROS 2 System
```

## Basic Robot Visualization

### Import Robot Model

1. Import URDF or 3D model into Unity
2. Set up robot hierarchy (links, joints)
3. Configure materials and textures

### ROS 2 Integration

```csharp
// Unity C# script example
using ROS2;

public class RobotController : MonoBehaviour {
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> cmdVelPub;
    
    void Start() {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        ros2Node = ros2Unity.CreateNode("unity_robot_controller");
        cmdVelPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>("cmd_vel");
    }
    
    void Update() {
        // Publish velocity commands
        var twist = new geometry_msgs.msg.Twist();
        twist.Linear.X = 0.5f;
        cmdVelPub.Publish(twist);
    }
}
```

## Sensor Data Streaming

### Camera Images

Stream camera data from ROS 2 to Unity:

```csharp
public class CameraSubscriber : MonoBehaviour {
    private ISubscription<sensor_msgs.msg.Image> imageSub;
    
    void Start() {
        imageSub = ros2Node.CreateSubscription<sensor_msgs.msg.Image>(
            "camera/image_raw",
            msg => {
                // Update Unity texture with image data
                UpdateTexture(msg);
            }
        );
    }
}
```

### LiDAR Point Clouds

Visualize LiDAR data in Unity:

```csharp
public class LidarVisualizer : MonoBehaviour {
    private ISubscription<sensor_msgs.msg.PointCloud2> lidarSub;
    
    void Start() {
        lidarSub = ros2Node.CreateSubscription<sensor_msgs.msg.PointCloud2>(
            "lidar/points",
            msg => {
                // Render point cloud
                RenderPointCloud(msg);
            }
        );
    }
}
```

## Human-Robot Interaction Scenes

### Setting Up HRI Environment

1. **Import Human Models**: Use Unity's character assets
2. **Create Interaction Space**: Room, furniture, objects
3. **Add Robot**: Import robot model
4. **Configure Interaction**: Hand gestures, speech, etc.

### Example: Handshake Interaction

```csharp
public class HandshakeController : MonoBehaviour {
    public Transform robotHand;
    public Transform humanHand;
    
    void Update() {
        // Detect hand proximity
        float distance = Vector3.Distance(
            robotHand.position, 
            humanHand.position
        );
        
        if (distance < 0.1f) {
            // Trigger handshake animation
            TriggerHandshake();
        }
    }
}
```

## Chapter Projects

### Project 1: Basic Visualization
- Import robot model into Unity
- Set up ROS-TCP connection
- Display robot in Unity scene

### Project 2: Sensor Visualization
- Stream camera images to Unity
- Display images on screen
- Add LiDAR point cloud visualization

### Project 3: HRI Scene
- Create human-robot interaction environment
- Add basic interaction (handshake, pointing)
- Record interaction for analysis

## Chapter Summary

**Key Takeaways**:

1. **Unity** provides high-quality visualization for robotics

2. **ROS-TCP-Connector** enables ROS 2 integration

3. **Sensor streaming** brings real robot data into Unity

4. **HRI scenes** enable natural interaction design

5. **Unity + Gazebo** complement each other (physics + visualization)

## Next Steps

- [Module 3: The AI-Robot Brain](../module3/index.md)
- [Return to Module 2 Overview](../module2/index.md)

