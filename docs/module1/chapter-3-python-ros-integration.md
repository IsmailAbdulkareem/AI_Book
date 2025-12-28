---
id: module1-chapter3
title: "Chapter 3: Python-ROS Integration with rclpy"
sidebar_position: 3
---

# Chapter 3: Python-ROS Integration with rclpy

**Type**: Practice-Heavy  
**Lessons**: 8  
**Duration**: 14-18 hours

## Chapter Overview

This chapter dives deep into using Python with ROS 2 through the `rclpy` library. You'll build complete robot control systems using publishers, subscribers, services, actions, and advanced features like executors and callback groups.

**By the end of this chapter, you will**:
- Implement publishers and subscribers for streaming data
- Create service servers and async clients
- Use parameters for runtime configuration
- Understand callback groups for concurrent operations
- Implement timers for multi-rate control loops
- Use executors for system composition
- Build a 3-node sensor-filter-controller system

## rclpy: Python Client Library

`rclpy` is the Python client library for ROS 2. It provides a Pythonic interface to ROS 2 functionality.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Initialize components here
    
    def cleanup(self):
        # Optional cleanup
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
```

## Publishers and Subscribers

### Publisher Example: Temperature Sensor

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(
            Float32, 
            'temperature', 
            10
        )
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Float32()
        msg.data = 20.0 + random.uniform(-2.0, 2.0)  # Simulated temp
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data}°C')
```

### Subscriber Example: Temperature Monitor

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        self.temp_history = []
    
    def temperature_callback(self, msg):
        self.temp_history.append(msg.data)
        if len(self.temp_history) > 100:
            self.temp_history.pop(0)
        
        avg_temp = sum(self.temp_history) / len(self.temp_history)
        self.get_logger().info(
            f'Current: {msg.data}°C, Average: {avg_temp:.2f}°C'
        )
```

## Services: Request-Response

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        self.get_logger().info('AddTwoInts service ready')
    
    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b}'
        )
        self.get_logger().info(f'Sending response: {response.sum}')
        return response
```

### Service Client Example (Async)

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Parameters: Runtime Configuration

Parameters allow nodes to be configured without recompiling.

### Declaring Parameters

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # Declare parameters with defaults
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('sensor_name', 'default_sensor')
        self.declare_parameter('enable_logging', True)
        
        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.sensor_name = self.get_parameter('sensor_name').value
        self.enable_logging = self.get_parameter('enable_logging').value
        
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Sensor name: {self.sensor_name}')
```

### Setting Parameters via Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='configurable_node',
            parameters=[{
                'publish_rate': 20.0,
                'sensor_name': 'camera_1',
                'enable_logging': True,
            }]
        ),
    ])
```

## Callback Groups

Callback groups control how callbacks are executed, enabling concurrent processing.

### Reentrant Callback Group

```python
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from rclpy.node import Node

class ConcurrentNode(Node):
    def __init__(self):
        super().__init__('concurrent_node')
        
        # Create reentrant callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribers with callback group
        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1, 10,
            callback_group=self.callback_group
        )
        self.sub2 = self.create_subscription(
            String, 'topic2', self.callback2, 10,
            callback_group=self.callback_group
        )
    
    def callback1(self, msg):
        # Can run concurrently with callback2
        self.get_logger().info(f'Callback 1: {msg.data}')
    
    def callback2(self, msg):
        # Can run concurrently with callback1
        self.get_logger().info(f'Callback 2: {msg.data}')
```

## Timers: Multi-Rate Control Loops

Timers enable nodes to perform periodic tasks at different rates.

### Multi-Rate Example

```python
class MultiRateNode(Node):
    def __init__(self):
        super().__init__('multi_rate_node')
        
        # Fast loop: 100 Hz (10 ms)
        self.fast_timer = self.create_timer(0.01, self.fast_callback)
        
        # Medium loop: 10 Hz (100 ms)
        self.medium_timer = self.create_timer(0.1, self.medium_callback)
        
        # Slow loop: 1 Hz (1 second)
        self.slow_timer = self.create_timer(1.0, self.slow_callback)
    
    def fast_callback(self):
        # High-frequency control (e.g., motor control)
        pass
    
    def medium_callback(self):
        # Medium-frequency processing (e.g., sensor fusion)
        pass
    
    def slow_callback(self):
        # Low-frequency tasks (e.g., status reporting)
        self.get_logger().info('Status update')
```

## Executors: System Composition

Executors control how callbacks are processed. Different executors provide different execution models.

### SingleThreadedExecutor (Default)

```python
import rclpy
from rclpy.executors import SingleThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    
    node1 = Node1()
    node2 = Node2()
    node3 = Node3()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.add_node(node3)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()
```

### MultiThreadedExecutor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
# Allows concurrent callback execution
```

## Capstone Project: 3-Node Sensor-Filter-Controller System

Build a complete system with three nodes:

### Node 1: Sensor Node
- Publishes raw sensor data (temperature, distance)
- High-frequency publishing (20 Hz)

### Node 2: Filter Node
- Subscribes to raw sensor data
- Applies filtering (moving average, outlier removal)
- Publishes filtered data

### Node 3: Controller Node
- Subscribes to filtered data
- Implements control logic (e.g., maintain temperature)
- Publishes control commands

### Implementation Structure

```
sensor_node (20 Hz)
    ↓ (raw_data topic)
filter_node (10 Hz)
    ↓ (filtered_data topic)
controller_node (5 Hz)
    ↓ (control_cmd topic)
actuator (hypothetical)
```

### Project Requirements

1. **Sensor Node**:
   - Publishes `sensor_msgs/Range` messages
   - Simulates noisy sensor data
   - Configurable publish rate via parameter

2. **Filter Node**:
   - Implements moving average filter
   - Removes outliers
   - Publishes filtered data

3. **Controller Node**:
   - Implements simple PID controller
   - Publishes control commands
   - Logs control performance

4. **Launch File**:
   - Starts all three nodes
   - Configures parameters
   - Sets up topic remapping if needed

## Chapter Projects

### Project 1: Temperature Sensor Publisher-Subscriber Pair
- Publisher simulates temperature sensor
- Subscriber logs and computes statistics

### Project 2: Validation Service
- Service validates sensor data
- Client sends data for validation
- Returns validation result

### Project 3: Parameter-Driven Configurable Node
- Node with multiple parameters
- Launch file sets parameters
- Node adapts behavior based on parameters

### Project 4: Multi-Rate Sensor Fusion Controller
- Fast loop: Sensor reading (100 Hz)
- Medium loop: Sensor fusion (10 Hz)
- Slow loop: Control output (1 Hz)

### Project 5: Capstone - 3-Node System
- Complete sensor-filter-controller pipeline
- All nodes communicate via topics
- Launch file orchestrates system

## Chapter Summary

**Key Takeaways**:

1. **rclpy** provides Pythonic interface to ROS 2

2. **Publishers/Subscribers** enable streaming data communication

3. **Services** provide request-response patterns

4. **Parameters** enable runtime configuration

5. **Callback groups** enable concurrent processing

6. **Timers** support multi-rate control loops

7. **Executors** control system-wide callback processing

## Next Steps

- [Chapter 4: Robot Description with URDF and Xacro](./chapter-4-urdf-robot-description.md)
- [Return to Module 1 Overview](../module1/index.md)

