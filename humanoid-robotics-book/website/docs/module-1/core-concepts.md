---
sidebar_position: 2
---

# Module 1 - Core Concepts

## ROS 2 Fundamentals

### Nodes
In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. Each node is designed to perform a specific task and communicates with other nodes through topics, services, and actions.

Key characteristics of ROS 2 nodes:
- Written in any of the supported languages (C++, Python, etc.)
- Use the same client library (rcl) to interface with the ROS 2 runtime
- Can be run on different machines in a distributed system
- Have unique names within a ROS 2 domain

### Topics and Message Passing
Topics enable asynchronous communication between nodes through a publish/subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decoupling allows for flexible system design.

```python
# Example: Simple publisher in Python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Services
Services provide synchronous request/response communication. A service client sends a request to a service server, which processes the request and sends back a response.

### Actions
Actions are used for long-running tasks that require feedback and goal preemption. They consist of three parts: goal, feedback, and result.

## Quality of Service (QoS)

QoS profiles define how messages are delivered in terms of reliability, durability, liveliness, and history. These settings are crucial for real-time and safety-critical applications.

Key QoS policies:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local durability
- **History**: Keep all or keep last N messages
- **Depth**: Size of the message queue

## Parameter System
ROS 2 includes a dynamic parameter system that allows configuration values to be changed at runtime. Parameters can be declared, validated, and dynamically updated.

## Launch System
ROS 2's launch system allows multiple nodes to be started and configured simultaneously using Python launch files. This replaces the XML-based launch files of ROS 1.

## TF2 (Transform Library)
TF2 is a package that keeps track of coordinate frame relationships over time. It's essential for robot navigation, manipulation, and sensor fusion.

## Package Management
ROS 2 packages are organized with ament as the build system. Each package contains a `package.xml` file that describes dependencies and metadata.

## Distributed System Architecture

### Data Distribution Service (DDS)
ROS 2's architecture is built on top of Data Distribution Service (DDS), which is a specification for real-time, scalable, dependable, and high-performance data exchange. DDS provides:

- **Decentralized communication**: No central master node required
- **Real-time capabilities**: Support for time-critical applications
- **Security**: Built-in security features for protected communication
- **Language independence**: Support for multiple programming languages

### Middleware Implementation
Different DDS implementations can be used with ROS 2:
- **Fast DDS** (formerly Fast RTPS): Default implementation
- **Cyclone DDS**: Lightweight implementation
- **RTI Connext DDS**: Commercial implementation
- **OpenSplice DDS**: Open-source implementation

## Client Libraries

### rclpy - Python Client Library
The Python client library provides Python bindings for ROS 2 functionality. It includes:

- Node creation and management
- Publisher and subscriber interfaces
- Service and action client/server implementations
- Parameter management
- Logging and lifecycle management

### rclcpp - C++ Client Library
The C++ client library provides C++ bindings with:

- Performance-optimized implementations
- Advanced memory management
- Real-time capabilities
- Direct access to low-level ROS 2 primitives

## Communication Patterns in Depth

### Topics and Publishers/Subscribers
The publish/subscribe pattern in ROS 2 offers several advantages:

- **Loose coupling**: Publishers and subscribers don't need to know about each other
- **Broadcasting**: Multiple subscribers can receive the same data
- **Asynchronous communication**: Publishers and subscribers can run at different rates
- **Scalability**: Easy to add more publishers or subscribers

### Services and Request/Response
Services provide synchronous communication with:

- **Request/response pattern**: Client sends request, server responds
- **Blocking calls**: Client waits for response before continuing
- **Reliable delivery**: Ensures request and response are delivered
- **Type safety**: Strong typing for request and response messages

### Actions and Goal-Oriented Communication
Actions are designed for long-running tasks with:

- **Goal management**: Send goals to action servers
- **Feedback**: Continuous feedback during execution
- **Result**: Final result when action completes
- **Preemption**: Ability to cancel running actions

## Advanced ROS 2 Concepts

### Lifecycle Nodes
Lifecycle nodes provide a state machine for node management:

- **Managed states**: Unconfigured, inactive, active, finalized
- **State transitions**: Controlled transitions between states
- **Error handling**: Proper error recovery and state management
- **System integration**: Better integration with system managers

### Composition
ROS 2 allows for component composition:

- **Single process**: Multiple nodes in a single process
- **Memory efficiency**: Reduced inter-process communication overhead
- **Performance**: Better performance for tightly coupled nodes
- **Deployment flexibility**: Choose between separate processes or composition

### Intra-process Communication
For nodes in the same process:

- **Zero-copy**: Direct memory access without copying
- **High performance**: Minimal overhead communication
- **Thread safety**: Proper synchronization mechanisms
- **Automatic optimization**: ROS 2 automatically uses intra-process when possible

## Security in ROS 2

### DDS Security Standard
ROS 2 implements the DDS Security standard with:

- **Authentication**: Verify identity of nodes
- **Access control**: Control what nodes can communicate
- **Encryption**: Encrypt data in transit
- **Signing**: Ensure data integrity

### Implementation
Security is implemented through:

- **Security plugins**: Pluggable security implementations
- **Configuration files**: Define security policies
- **Certificates**: PKI-based authentication
- **Secure by default**: Can be enabled/disabled as needed