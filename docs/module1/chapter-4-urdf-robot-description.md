---
id: module1-chapter4
title: "Chapter 4: Robot Description with URDF and Xacro"
sidebar_position: 4
---

# Chapter 4: Robot Description with URDF and Xacro

**Type**: Theory-to-Practice  
**Lessons**: 8 (3 theory + 5 code)  
**Duration**: 16-20 hours

## Chapter Overview

This chapter teaches you how to describe robot physical structure using URDF (Unified Robot Description Format) and Xacro (XML Macros). You'll learn to model humanoid robots, integrate sensors, and validate your models.

**By the end of this chapter, you will**:
- Understand URDF as the blueprint for robot structure
- Model links, joints, and kinematic chains
- Balance visual vs collision geometry
- Use Xacro for parameterized, reusable descriptions
- Design bipedal humanoid structures
- Integrate sensors (cameras, LiDAR, IMU) in URDF
- Validate and visualize models with RViz

## URDF: The Robot Blueprint

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML format that describes:
- **Links**: Rigid bodies (segments of the robot)
- **Joints**: Connections between links
- **Visual Geometry**: What the robot looks like
- **Collision Geometry**: For physics simulation
- **Inertial Properties**: Mass, center of mass, inertia tensors

### Why URDF Matters

URDF is used by:
- **ROS 2**: Robot modeling and TF2 transforms
- **Gazebo**: Physics simulation
- **RViz**: Visualization
- **MoveIt**: Motion planning
- **Robot State Publisher**: Coordinate frame management

## Basic URDF Structure

### Minimal URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" 
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
</robot>
```

## Links, Joints, and Kinematic Chains

### Links

**Links** are rigid bodies. Each link has:
- **Visual**: For rendering (can be simplified)
- **Collision**: For physics (should be accurate)
- **Inertial**: Mass and inertia properties

### Joints

**Joint Types**:
- **Fixed**: No movement (welded connection)
- **Revolute**: Rotation around one axis (hinge)
- **Prismatic**: Linear motion (slider)
- **Continuous**: Unlimited rotation
- **Planar**: Motion in a plane
- **Floating**: 6-DOF motion

### Kinematic Chains

A **kinematic chain** is a series of links connected by joints:
```
base_link → joint1 → link1 → joint2 → link2 → ...
```

For humanoids:
```
torso → shoulder → upper_arm → elbow → forearm → wrist → hand
```

## Visual vs Collision Geometry

### Trade-offs

**Visual Geometry**:
- Can be detailed and complex
- Used only for rendering
- Doesn't affect simulation performance

**Collision Geometry**:
- Should be simple (boxes, cylinders, spheres)
- Used for physics calculations
- Complex geometry slows simulation

### Best Practices

```xml
<link name="complex_link">
  <!-- Detailed visual for rendering -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/detailed_arm.dae"/>
    </geometry>
  </visual>
  
  <!-- Simple collision for physics -->
  <collision>
    <geometry>
      <box size="0.2 0.1 0.3"/>
    </geometry>
  </collision>
</link>
```

## Xacro: Parameterized URDF

### Why Xacro?

**Problems with raw URDF**:
- Repetitive code (left/right arms are similar)
- Hard to maintain (change one value, update many places)
- No reusability

**Xacro Benefits**:
- **Macros**: Reusable components
- **Parameters**: Variables for dimensions
- **Math**: Calculations in XML
- **Includes**: Modular file structure

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="parameterized_robot">
  
  <!-- Parameters -->
  <xacro:property name="leg_length" value="0.5"/>
  <xacro:property name="leg_width" value="0.1"/>
  
  <!-- Macro for leg -->
  <xacro:macro name="leg" params="prefix">
    <link name="${prefix}_leg">
      <visual>
        <geometry>
          <box size="${leg_width} ${leg_width} ${leg_length}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="${leg_width} ${leg_width} ${leg_length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="5"/>
        <inertia ixx="0.1" ixy="0" ixz="0" 
                 iyy="0.1" iyz="0" izz="0.1"/>
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Use macro -->
  <xacro:leg prefix="left"/>
  <xacro:leg prefix="right"/>
  
</robot>
```

## Bipedal Humanoid Design

### Humanoid Structure

**Key Components**:
1. **Torso**: Central body
2. **Head**: With cameras
3. **Arms**: Left and right (shoulder, elbow, wrist)
4. **Legs**: Left and right (hip, knee, ankle)
5. **Feet**: For balance

### Example: Humanoid Leg

```xml
<xacro:macro name="humanoid_leg" params="side">
  <!-- Upper Leg -->
  <link name="${side}_upper_leg">
    <visual>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.15 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" 
               iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Hip Joint -->
  <joint name="${side}_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="${side}_upper_leg"/>
    <origin xyz="${0.1 if side == 'left' else -0.1} 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>
  
  <!-- Lower Leg -->
  <link name="${side}_lower_leg">
    <visual>
      <geometry>
        <box size="0.12 0.12 0.35"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.12 0.12 0.35"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.05" ixy="0" ixz="0" 
               iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>
  
  <!-- Knee Joint -->
  <joint name="${side}_knee_joint" type="revolute">
    <parent link="${side}_upper_leg"/>
    <child link="${side}_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="40" velocity="2"/>
  </joint>
  
  <!-- Foot -->
  <link name="${side}_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" 
               iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Ankle Joint -->
  <joint name="${side}_ankle_joint" type="revolute">
    <parent link="${side}_lower_leg"/>
    <child link="${side}_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1"/>
  </joint>
</xacro:macro>
```

## Sensor Integration in URDF

### Camera Integration

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" 
             iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Gazebo camera plugin -->
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
  </sensor>
</gazebo>
```

### LiDAR Integration

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.01" ixy="0" ixz="0" 
             iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="torso"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
</joint>

<!-- Gazebo LiDAR plugin -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>40</update_rate>
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
        <resolution>0.01</resolution>
      </range>
    </ray>
  </sensor>
</gazebo>
```

### IMU Integration

```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" 
             iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo IMU plugin -->
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
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Validation and Visualization

### Check URDF Syntax

```bash
# Check URDF file
check_urdf my_robot.urdf

# Check Xacro file (processes Xacro first)
check_urdf my_robot.xacro
```

### Visualize in RViz

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro my_robot.xacro)"

# Launch RViz
rviz2
```

In RViz:
1. Add "RobotModel" display
2. Set Fixed Frame to "base_link"
3. Robot should appear

### Visualize in Gazebo

```bash
# Launch Gazebo with robot
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -topic robot_description
```

## Chapter Projects

### Project 1: Parameterized Humanoid Leg
- Create Xacro macro for humanoid leg
- Use parameters for dimensions
- Instantiate left and right legs

### Project 2: Bilateral Arm Macro
- Create Xacro macro for humanoid arm
- Include shoulder, elbow, wrist joints
- Use for both left and right arms

### Project 3: Complete Humanoid with Sensors
- Combine torso, head, arms, legs
- Add camera to head
- Add LiDAR to torso
- Add IMU to torso

### Project 4: Capstone - Validated Full Humanoid
- Complete humanoid model
- All sensors integrated
- Validated with `check_urdf`
- Visualized in RViz
- Documented with dimensions and joint limits

## Chapter Summary

**Key Takeaways**:

1. **URDF** describes robot physical structure (links, joints)

2. **Visual vs Collision** geometry serve different purposes

3. **Xacro** enables parameterized, reusable robot descriptions

4. **Humanoid design** requires careful joint placement and limits

5. **Sensor integration** in URDF enables simulation and visualization

6. **Validation** ensures models are correct before use

## Next Steps

- [Module 2: The Digital Twin](../module2/index.md)
- [Return to Module 1 Overview](../module1/index.md)

