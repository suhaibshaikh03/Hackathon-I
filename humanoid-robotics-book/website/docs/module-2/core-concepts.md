---
sidebar_position: 2
---

# Module 2 - Core Concepts

## ROS 2 Graph Architecture

### Distributed System Design
ROS 2 implements a distributed system architecture based on Data Distribution Service (DDS) for communication between nodes. This enables:
- Reliable message passing between processes and machines
- Language independence (C++, Python, and others)
- Distributed computing across multiple devices
- Quality of Service (QoS) policies for different communication needs

### Node Communication
In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. Each node is designed to perform a specific task and communicates with other nodes through topics, services, and actions.

Key characteristics of ROS 2 nodes:
- Written in any of the supported languages (C++, Python, etc.)
- Use the same client library (rcl) to interface with the ROS 2 runtime
- Can be run on different machines in a distributed system
- Have unique names within a ROS 2 domain

### Graph Visualization
The ROS 2 graph can be visualized using tools like `rqt_graph` which shows:
- Nodes as boxes
- Topics as ovals
- Connections between nodes and topics as arrows
- Service and action connections as dashed lines

## Communication Paradigms

### Topics - Publish/Subscribe
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

### Services - Request/Response
Services provide synchronous request/response communication. A service client sends a request to a service server, which processes the request and sends back a response.

### Actions - Long-Running Tasks
Actions are used for long-running tasks that require feedback and goal preemption. They consist of three parts: goal, feedback, and result.

## Quality of Service (QoS) Profiles

QoS profiles define how messages are delivered in terms of reliability, durability, liveliness, and history. These settings are crucial for real-time and safety-critical applications.

Key QoS policies:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local durability
- **History**: Keep all or keep last N messages
- **Depth**: Size of the message queue
- **Deadline**: Maximum time between consecutive messages
- **Lifespan**: Maximum lifetime of a message

### QoS Policy Examples
```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Reliable communication for critical data
reliable_qos = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Best effort for real-time sensor data
best_effort_qos = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

## URDF (Unified Robot Description Format)

### Kinematic Trees
URDF defines robot structure as a tree of rigid bodies connected by joints. Key components include:
- **Links**: Rigid bodies with mass and geometry
- **Joints**: Connections between links (revolute, prismatic, continuous, etc.)
- **Visual**: Meshes and colors for rendering
- **Collision**: Shapes for collision detection
- **Inertial**: Mass, center of mass, and moments of inertia

### URDF Example Structure
```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Shoulder joint and link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## Parameter System

### Dynamic Configuration
ROS 2 includes a dynamic parameter system that allows configuration values to be changed at runtime. Parameters can be declared, validated, and dynamically updated.

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptors
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.parameter import Parameter

        for param in params:
            if param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                self.max_velocity = param.value
                self.get_logger().info(f'Updated max_velocity to {self.max_velocity}')
        return SetParametersResult(successful=True)
```

## Launch System

### Composable Systems
ROS 2's launch system allows multiple nodes to be started and configured simultaneously using Python launch files. This replaces the XML-based launch files of ROS 1.

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='publisher_node',
            name='publisher',
            parameters=[
                {'param1': 'value1'},
                {'param2': 'value2'}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        ),
        Node(
            package='my_package',
            executable='subscriber_node',
            name='subscriber'
        )
    ])
```

## TF2 (Transform Library)

### Coordinate Frame Management
TF2 is a package that keeps track of coordinate frame relationships over time. It's essential for robot navigation, manipulation, and sensor fusion.

Key features:
- Automatic interpolation between transform timestamps
- Efficient lookup of transforms
- Support for both static and dynamic transforms
- Tools for debugging and visualizing transforms

```python
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lookup_transform(self, target_frame, source_frame, time):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return transform
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return None
```