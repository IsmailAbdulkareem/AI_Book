---
id: module2-chapter2
title: "Chapter 2: Robot Control in Gazebo"
sidebar_position: 2
---

# Chapter 2: Robot Control in Gazebo

**Type**: Practice-Heavy  
**Lessons**: 6  
**Duration**: 12-14 hours

## Chapter Overview

This chapter teaches you how to control robots in Gazebo using ROS 2. You'll learn to control joints, move robots, and integrate with ros2_control for advanced control systems.

**By the end of this chapter, you will**:
- Control robot joints via ROS 2 topics
- Use ros2_control for hardware abstraction
- Implement basic locomotion controllers
- Control manipulator arms
- Understand the difference between position, velocity, and effort control

## Controlling Joints

### Direct Topic Control

Simple robots can be controlled by publishing directly to joint command topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(
            Float64,
            '/joint1/command',
            10
        )
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0
    
    def timer_callback(self):
        msg = Float64()
        msg.data = self.angle
        self.publisher.publish(msg)
        self.angle += 0.1
        if self.angle > 1.57:
            self.angle = -1.57
```

### Using Command Line

```bash
# Publish joint command
ros2 topic pub /joint1/command std_msgs/msg/Float64 "data: 0.5"
```

## ros2_control Integration

ros2_control provides hardware abstraction and standardized control interfaces.

### Basic ros2_control Setup

**1. Add ros2_control tags to URDF**:

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

**2. Add Gazebo plugin**:

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**3. Create controllers configuration**:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

## Control Modes

### Position Control

Commands joint to specific angle:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

trajectory = JointTrajectory()
trajectory.joint_names = ['joint1']
point = JointTrajectoryPoint()
point.positions = [0.5]  # radians
point.time_from_start.sec = 1
trajectory.points = [point]

pub.publish(trajectory)
```

### Velocity Control

Commands joint velocity:

```python
from std_msgs.msg import Float64

msg = Float64()
msg.data = 0.5  # rad/s
pub.publish(msg)
```

### Effort Control

Commands joint torque:

```python
from std_msgs.msg import Float64

msg = Float64()
msg.data = 10.0  # N⋅m
pub.publish(msg)
```

## Mobile Base Control

### Differential Drive

Control a two-wheeled robot:

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x = 0.5   # m/s forward
cmd.angular.z = 0.2  # rad/s turn

pub.publish(cmd)
```

### Gazebo Plugin

```xml
<gazebo>
  <plugin name="differential_drive_controller" 
          filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/</namespace>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>
    <command_topic>cmd_vel</command_topic>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
  </plugin>
</gazebo>
```

## Manipulator Control

### Joint Trajectory Controller

For multi-DOF arms:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

trajectory = JointTrajectory()
trajectory.joint_names = ['shoulder', 'elbow', 'wrist']

point = JointTrajectoryPoint()
point.positions = [0.5, 1.0, 0.3]
point.velocities = [0.0, 0.0, 0.0]
point.time_from_start.sec = 2
trajectory.points = [point]

pub.publish(trajectory)
```

## Chapter Projects

### Project 1: Simple Joint Control
- Control single joint via topic
- Move joint through range of motion
- Log joint state

### Project 2: ros2_control Setup
- Configure ros2_control for robot
- Launch controllers
- Control joints via controllers

### Project 3: Mobile Base Navigation
- Control differential drive robot
- Implement simple navigation (forward, turn, stop)
- Monitor odometry

### Project 4: Arm Manipulation
- Control multi-DOF arm
- Execute pick-and-place trajectory
- Verify end-effector position

## Chapter Summary

**Key Takeaways**:

1. **Joint control** can be direct (topics) or via ros2_control

2. **ros2_control** provides standardized hardware abstraction

3. **Control modes** (position, velocity, effort) serve different purposes

4. **Mobile bases** use velocity commands (Twist messages)

5. **Manipulators** use trajectory controllers for coordinated motion

## Next Steps

- [Chapter 3: Unity Integration](./chapter-3-unity-integration.md)
- [Return to Module 2 Overview](../module2/index.md)

