---
id: module1-chapter2
title: "Chapter 2: ROS 2 Architecture and Core Concepts"
sidebar_position: 2
---

# Chapter 2: ROS 2 Architecture and Core Concepts

**Type**: Theory-to-Practice  
**Lessons**: 8 (5 theory + 3 code)  
**Duration**: 12-16 hours

## Chapter Overview

This chapter introduces ROS 2 (Robot Operating System 2) as the middleware framework that enables distributed robotic systems. You'll learn the architecture, core concepts, and build your first ROS 2 nodes.

**By the end of this chapter, you will**:
- Understand ROS 2 as middleware (not an operating system)
- Know why DDS replaced ROS 1's architecture
- Create and run ROS 2 nodes
- Implement publish-subscribe communication
- Configure QoS (Quality of Service) policies
- Create launch files for multi-node systems

## ROS 2 as Middleware

### What is Middleware?

**Middleware** is software that provides services and capabilities beyond what the operating system offers. ROS 2 is middleware specifically designed for robotics.

**ROS 2 is NOT**:
- An operating system (runs on Linux, Windows, macOS)
- A programming language (works with Python, C++, Java, etc.)
- A robot (it's software that runs on robots)

**ROS 2 IS**:
- A communication framework
- A set of tools and libraries
- A standard way to build robot software

### Why ROS 2?

**Problems ROS 2 Solves**:
1. **Distributed Systems**: Robots have many components (sensors, actuators, AI) that need to communicate
2. **Modularity**: Break complex systems into manageable nodes
3. **Reusability**: Share code and packages across projects
4. **Standardization**: Common interfaces and message types

## DDS: The Foundation of ROS 2

### Why DDS Replaced ROS 1

**ROS 1 Limitations**:
- Single master node (single point of failure)
- No real-time guarantees
- Limited security features
- Difficult to use across networks

**DDS (Data Distribution Service) Benefits**:
- **Decentralized**: No single master node
- **Real-Time**: QoS policies for deterministic communication
- **Security**: Built-in authentication and encryption
- **Network Transparency**: Works seamlessly across networks
- **Industry Standard**: Used in aerospace, automotive, medical devices

### DDS Architecture

DDS provides a **publish-subscribe** model:

```
Publisher Node          Subscriber Node
     │                        │
     │  (DDS Layer)          │
     │                        │
     └─────────┬─────────────┘
               │
         DDS Domain
```

- **No central broker**: Nodes discover each other automatically
- **Type-safe**: Message types are enforced
- **QoS Policies**: Control reliability, durability, deadline

## Core Concepts

### Nodes

**Nodes** are independent processes that perform specific tasks.

**Characteristics**:
- Single-purpose (e.g., "camera driver", "path planner")
- Communicate via topics, services, actions
- Can run on same machine or distributed across network

**Example Nodes**:
- `camera_node`: Publishes camera images
- `navigation_node`: Plans paths
- `motor_controller_node`: Controls motors

### Topics

**Topics** are named communication channels for streaming data.

**Publish-Subscribe Pattern**:
- **Publisher**: Sends messages to a topic
- **Subscriber**: Receives messages from a topic
- **Many-to-Many**: Multiple publishers/subscribers per topic

**Example Topics**:
- `/camera/image_raw`: Camera images
- `/cmd_vel`: Velocity commands
- `/odom`: Odometry data

### Services

**Services** provide request-response communication.

**Characteristics**:
- **Synchronous**: Client waits for response
- **One-to-One**: One server, one client per request
- **Use Cases**: Calibration, configuration, one-time actions

**Example Services**:
- `/calibrate_camera`: Trigger camera calibration
- `/get_robot_state`: Query current robot state

### Actions

**Actions** combine topics and services for long-running tasks.

**Characteristics**:
- **Asynchronous**: Client doesn't block
- **Feedback**: Continuous updates during execution
- **Cancellable**: Can abort long-running tasks
- **Use Cases**: Navigation, manipulation sequences

**Example Actions**:
- `/navigate_to_pose`: Navigate to a goal
- `/pick_object`: Execute pick-and-place sequence

## Your First ROS 2 Node

### Minimal "Hello World" Node

Create `hello_ros2.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Components**:
1. `rclpy.init()`: Initialize ROS 2
2. `Node`: Base class for all nodes
3. `get_logger()`: For logging messages
4. `rclpy.spin()`: Keep node alive
5. `rclpy.shutdown()`: Clean shutdown

### Running the Node

```bash
# Make executable
chmod +x hello_ros2.py

# Run
python3 hello_ros2.py
```

**Expected Output**:
```
[INFO] [hello_node]: Hello, ROS 2!
```

## Publish-Subscribe Example

### Publisher Node

Create `talker.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

Create `listener.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing

**Terminal 1** (Publisher):
```bash
python3 talker.py
```

**Terminal 2** (Subscriber):
```bash
python3 listener.py
```

**Terminal 3** (Monitor):
```bash
ros2 topic echo /chatter
```

## Quality of Service (QoS)

QoS policies control how ROS 2 handles message delivery.

### Common QoS Profiles

**Reliable** (default):
- Messages guaranteed to be delivered
- Use for: Commands, critical data

**Best Effort**:
- Messages may be dropped
- Use for: High-frequency sensor data (cameras, LiDAR)

**Example**:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable QoS
qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

# Best Effort QoS (for high-frequency data)
qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

self.publisher_ = self.create_publisher(
    Image, 'camera/image_raw', qos_best_effort
)
```

## Launch Files

Launch files start multiple nodes and configure the system.

### Basic Launch File

Create `my_launch_file.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='talker',
            name='talker_node',
        ),
        Node(
            package='my_package',
            executable='listener',
            name='listener_node',
        ),
    ])
```

### Running Launch Files

```bash
ros2 launch my_package my_launch_file.launch.py
```

## Chapter Projects

### Project 1: Minimal ROS 2 Node
- Create a node that logs a message every second
- Verify it runs and logs correctly

### Project 2: QoS-Configured Sensor Publisher
- Create a publisher for simulated sensor data (temperature, distance)
- Configure QoS for best-effort delivery
- Publish at high frequency (10+ Hz)

### Project 3: Multi-Node System
- Create publisher, subscriber, and processor nodes
- Use a launch file to start all nodes
- Verify communication between nodes

## Chapter Summary

**Key Takeaways**:

1. **ROS 2 is middleware** for building distributed robot systems

2. **DDS architecture** provides decentralized, real-time communication

3. **Core concepts**: Nodes, Topics, Services, Actions

4. **QoS policies** control message delivery reliability

5. **Launch files** orchestrate multi-node systems

## Next Steps

- [Chapter 3: Python-ROS Integration with rclpy](./chapter-3-python-ros-integration.md)
- [Return to Module 1 Overview](../module1/index.md)

