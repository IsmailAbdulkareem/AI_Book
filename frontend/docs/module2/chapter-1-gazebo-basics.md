---
id: module2-chapter1
title: "Chapter 1: Gazebo Simulation Basics"
sidebar_position: 1
---

# Chapter 1: Gazebo Simulation Basics

**Type**: Theory-to-Practice  
**Lessons**: 6  
**Duration**: 10-12 hours

## Chapter Overview

This chapter introduces Gazebo, the physics simulation engine that enables you to develop and test robots without physical hardware. You'll learn to set up simulation environments, spawn robots, and understand physics simulation fundamentals.

**By the end of this chapter, you will**:
- Understand Gazebo's role in robotics development
- Set up Gazebo simulation environments
- Spawn robots from URDF/SDF files
- Understand physics engines and simulation parameters
- Use Gazebo's visualization and debugging tools

## What is Gazebo?

Gazebo is an open-source 3D physics simulation engine designed for robotics. It provides:

- **Physics Simulation**: Realistic dynamics, collisions, and contact forces
- **Sensor Simulation**: Cameras, LiDAR, IMUs, and more
- **Environment Modeling**: Create complex worlds with obstacles
- **Plugin System**: Extend functionality with custom plugins

## Gazebo vs. Real World

### Advantages of Simulation

- **Safety**: Test dangerous scenarios without risk
- **Speed**: Faster iteration than physical testing
- **Cost**: No hardware wear or damage
- **Reproducibility**: Exact same conditions every time
- **Scalability**: Test multiple robots simultaneously

### Limitations

- **Reality Gap**: Simulation never perfectly matches reality
- **Computational Cost**: Complex simulations are resource-intensive
- **Model Accuracy**: Requires accurate robot and environment models

## Setting Up Gazebo

### Installation

**For ROS 2 Humble**:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

**For ROS 2 Iron**:
```bash
# Gazebo Garden
sudo apt install gazebo
```

### Basic Launch

```bash
# Launch empty Gazebo world
gazebo

# Launch with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py
```

## URDF vs SDF

### URDF (Unified Robot Description Format)

- XML format for robot structure
- Used by ROS 2 and Gazebo Classic
- Simpler, less feature-rich

### SDF (Simulation Description Format)

- More powerful than URDF
- Supports environments, multiple robots
- Used by Gazebo Garden
- Can include physics, lighting, materials

### Converting URDF to SDF

```bash
# Convert URDF to SDF
gz sdf -k my_robot.urdf > my_robot.sdf
```

## Spawning Robots in Gazebo

### Using ROS 2

```bash
# Spawn robot from URDF
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -topic robot_description \
  -x 0 -y 0 -z 0.5
```

### Using Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_pkg')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        # Spawn in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-topic', 'robot_description']
        ),
    ])
```

## Physics Engines

Gazebo supports multiple physics engines:

### ODE (Open Dynamics Engine)
- Default for Gazebo Classic
- Good general-purpose physics
- Fast and stable

### Bullet
- Better for complex collisions
- More accurate contact forces

### DART (Dynamic Animation and Robotics Toolkit)
- Excellent for articulated bodies
- Good for humanoid robots

### Simbody
- High accuracy
- Slower but more precise

### Configuring Physics

```xml
<physics name="default_physics" default="0" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

## Sensor Simulation

### Camera

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
      </range>
    </ray>
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### IMU

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
      </angular_velocity>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

## Visualization and Debugging

### Gazebo GUI

- **Scene Graph**: View robot structure
- **Physics**: Monitor simulation performance
- **Sensors**: View sensor data
- **World Stats**: FPS, real-time factor

### ROS 2 Topics

```bash
# View camera images
ros2 topic echo /camera/image_raw

# View LiDAR data
ros2 topic echo /lidar/scan

# View IMU data
ros2 topic echo /imu/data
```

### RViz Integration

```bash
# Launch RViz to visualize Gazebo data
rviz2
```

In RViz, add displays for:
- RobotModel (from robot_state_publisher)
- Camera images
- LiDAR point clouds
- TF transforms

## Chapter Projects

### Project 1: Spawn Simple Robot
- Create URDF for simple mobile base
- Spawn in Gazebo
- Verify robot appears correctly

### Project 2: Add Sensors
- Add camera to robot
- Add LiDAR to robot
- Verify sensor data on ROS 2 topics

### Project 3: Physics Tuning
- Adjust physics parameters
- Test different physics engines
- Compare simulation vs. expected behavior

## Chapter Summary

**Key Takeaways**:

1. **Gazebo** enables safe, fast robot development in simulation

2. **URDF/SDF** describe robots for simulation

3. **Physics engines** offer trade-offs between speed and accuracy

4. **Sensor simulation** provides realistic sensor data

5. **Visualization tools** help debug and understand simulation

## Next Steps

- [Chapter 2: Robot Control in Gazebo](./chapter-2-robot-control.md)
- [Return to Module 2 Overview](../module2/index.md)

