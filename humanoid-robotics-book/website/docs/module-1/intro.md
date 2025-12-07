---
sidebar_position: 1
---

# Module 1 – The Robotic Nervous System (ROS 2)

## Learning Objectives
- Understand the fundamental concepts of Robot Operating System 2 (ROS 2) architecture
- Implement node communication patterns with topics, services, and actions
- Create URDF models for robotic systems and configure ROS 2 packages
- Configure navigation systems and debug ROS 2 applications effectively
- Apply Quality of Service (QoS) policies for different communication requirements
- Design and implement distributed robotic systems using ROS 2 principles

## Core Concepts

### Introduction to ROS 2
Robot Operating System 2 (ROS 2) serves as the nervous system for robotic applications, providing a flexible framework for writing robot software. Unlike its predecessor, ROS 2 addresses the limitations of ROS 1 by providing improved security, real-time capabilities, and better cross-platform support.

ROS 2 is not an operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### ROS 2 Architecture
ROS 2 uses a distributed system architecture based on Data Distribution Service (DDS) for communication between nodes. This enables:
- Reliable message passing between processes and machines
- Language independence (C++, Python, and others)
- Distributed computing across multiple devices
- Quality of Service (QoS) policies for different communication needs

The architecture is built on top of DDS (Data Distribution Service), which is a specification that provides a middleware for real-time, scalable, dependable, and high-performance data exchanges. DDS allows ROS 2 to provide features like:

- **Real-time capabilities**: Support for real-time systems with deterministic behavior
- **Security**: Built-in security features for protected communication
- **Cross-platform support**: Consistent behavior across different operating systems
- **Language independence**: Support for multiple programming languages

### Key Components of ROS 2

#### Nodes
In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. Each node is designed to perform a specific task and communicates with other nodes through topics, services, and actions.

Key characteristics of ROS 2 nodes:
- Written in any of the supported languages (C++, Python, etc.)
- Use the same client library (rcl) to interface with the ROS 2 runtime
- Can be run on different machines in a distributed system
- Have unique names within a ROS 2 domain

#### Topics and Message Passing
Topics enable asynchronous communication between nodes through a publish/subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decoupling allows for flexible system design.

The publish/subscribe pattern offers several advantages:
- **Loose coupling**: Publishers and subscribers don't need to know about each other
- **Broadcasting**: Multiple subscribers can receive the same data
- **Asynchronous communication**: Publishers and subscribers can run at different rates

#### Services
Services provide synchronous request/response communication. A service client sends a request to a service server, which processes the request and sends back a response. This is useful for operations that require a definitive response.

#### Actions
Actions are used for long-running tasks that require feedback and goal preemption. They consist of three parts: goal, feedback, and result. Actions are ideal for tasks like navigation, where you want to know the progress of the operation and potentially cancel it.

## Key Technologies & Tools

### ROS 2 Distributions
- **ROS 2 Humble Hawksbill**: Long-term support distribution (LTS) released in May 2022, supported until May 2027
- **ROS 2 Iron Irwini**: Released in May 2023, supported until November 2024
- **ROS 2 Jazzy Jalisco**: Latest development distribution

### Client Libraries
- **rclpy**: Python client library for ROS 2
- **rclcpp**: C++ client library for ROS 2
- **rcl**: Reference client library implementation
- **rclc**: C library for microcontrollers

### Visualization and Debugging Tools
- **RViz2**: 3D visualization tool for robotics
- **rqt**: GUI tools for introspection and interaction
- **ros2 cli tools**: Command-line interface tools for system management
- **rosbag2**: Data recording and playback system

### Simulation and Testing
- **Gazebo**: 3D simulation environment
- **Gazebo Classic**: Legacy version of Gazebo
- **Gazebo Garden/Harmonic**: Modern versions with improved features
- **ros2_control**: Robot control framework

### Navigation
- **Navigation2**: State-of-the-art navigation framework
- **SLAM Toolbox**: Simultaneous Localization and Mapping
- **Nav2 Bringup**: Launch files and configurations for navigation

## Hands-on Labs / Code Walkthroughs

### Lab 1: Creating Your First ROS 2 Package

#### Objective
Create a basic ROS 2 package with a publisher and subscriber node.

#### Steps
1. Create a new workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Create a new package:
```bash
cd src
ros2 pkg create --build-type ament_python my_robot_tutorials --dependencies rclpy std_msgs
```

3. Navigate to the package directory and create publisher/subscriber nodes:

**Publisher node** (`my_robot_tutorials/my_robot_tutorials/publisher_member_function.py`):
```python
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

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber node** (`my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

4. Update the `setup.py` file to include entry points for your executables:
```python
entry_points={
    'console_scripts': [
        'talker = my_robot_tutorials.publisher_member_function:main',
        'listener = my_robot_tutorials.subscriber_member_function:main',
    ],
},
```

5. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

6. Run the nodes:
```bash
# Terminal 1
ros2 run my_robot_tutorials talker

# Terminal 2
ros2 run my_robot_tutorials listener
```

### Lab 2: Service Implementation

#### Objective
Implement a service server and client for robot control.

#### Steps
1. Define a service interface (create `srv/AddThreeInts.srv` in your package):
```
int64 a
int64 b
int64 c
---
int64 sum
```

2. Create service server (`my_robot_tutorials/my_robot_tutorials/add_three_ints_server.py`):
```python
from example_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node

class AddThreeIntsServer(Node):
    def __init__(self):
        super().__init__('add_three_ints_server')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(f'Returning {request.a} + {request.b} + {request.c} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_three_ints_server = AddThreeIntsServer()
    rclpy.spin(add_three_ints_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Create service client (`my_robot_tutorials/my_robot_tutorials/add_three_ints_client.py`):
```python
from example_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node

class AddThreeIntsClient(Node):
    def __init__(self):
        super().__init__('add_three_ints_client')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()

    def send_request(self, a, b, c):
        self.req.a = a
        self.req.b = b
        self.req.c = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_three_ints_client = AddThreeIntsClient()
    response = add_three_ints_client.send_request(1, 2, 3)
    add_three_ints_client.get_logger().info(f'Result of add_three_ints: {response.sum}')
    add_three_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

4. Build and test the service:
```bash
# Terminal 1: Start the service server
ros2 run my_robot_tutorials add_three_ints_server

# Terminal 2: Call the service
ros2 run my_robot_tutorials add_three_ints_client
```

### Lab 3: URDF Robot Model

#### Objective
Create a simple URDF model for a mobile robot.

#### Steps
1. Create URDF files in your package:
```bash
mkdir -p my_robot_tutorials/urdf
```

2. Create a basic URDF file (`my_robot_tutorials/urdf/my_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0 0.19 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel -->
  <link name="wheel_right">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <origin xyz="0 -0.19 -0.05" rpy="1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- caster wheel -->
  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <joint name="caster_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="-0.15 0 -0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

3. Launch RViz to visualize the robot:
```bash
# Terminal 1: Publish the robot description
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(xacro /path/to/my_robot.urdf)'

# Terminal 2: Launch RViz
ros2 run rviz2 rviz2
```

### Lab 4: Navigation Stack Configuration

#### Objective
Configure the Navigation2 stack for a simple robot.

#### Steps
1. Install Navigation2:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

2. Create a basic navigation launch file (`my_robot_tutorials/launch/navigation_launch.py`):
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory('my_robot_tutorials'), 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_localization',
            executable='amcl',
            name='amcl',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'use_list': ['map_server', 'amcl', 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator']},
                       {'autostart': autostart}])
    ])
```

## Common Pitfalls & Debugging Tips

### 1. Network Configuration Issues
**Problem**: Nodes on different machines cannot communicate.
**Solution**:
- Ensure both machines are on the same ROS domain: `export ROS_DOMAIN_ID=0`
- Check firewall settings to allow DDS communication (typically UDP/TCP on various ports)
- Verify network connectivity with ping
- Use `ROS_LOCALHOST_ONLY=1` for single-machine testing: `export ROS_LOCALHOST_ONLY=1`

### 2. Resource Management
**Problem**: Memory leaks or performance degradation in long-running nodes.
**Solution**:
- Use weak references when creating callbacks that reference self to prevent circular references
- Implement proper cleanup in node destruction by overriding the destroy method
- Monitor resource usage with system tools like htop or ROS's own tools
- Use appropriate QoS settings to avoid message buildup and memory issues

### 3. Timing and Synchronization
**Problem**: Race conditions or missed messages.
**Solution**:
- Use appropriate QoS history and reliability settings based on your application needs
- Implement proper message filtering and validation to handle out-of-order messages
- Use latching for static transforms that need to persist
- Consider using action servers for long-running tasks that require feedback

### 4. Parameter Validation
**Problem**: Invalid parameter values causing runtime errors.
**Solution**:
- Implement parameter callbacks for validation when parameters change
- Use parameter descriptors to define ranges, types, and constraints during declaration
- Validate parameters during node initialization before starting operations
- Provide default values for all parameters to ensure robust startup

### 5. TF Tree Issues
**Problem**: Transform errors or missing transforms.
**Solution**:
- Use `ros2 run tf2_tools view_frames` to visualize the TF tree and identify issues
- Ensure all required transforms are being published by the appropriate nodes
- Use `ros2 run tf2_ros tf2_echo frame1 frame2` to check specific transforms
- Verify that transforms are being published at the required rate (typically 10-100 Hz)

### 6. Quality of Service (QoS) Problems
**Problem**: Messages not being received or performance issues.
**Solution**:
- Match QoS settings between publishers and subscribers for reliable communication
- Use appropriate reliability settings (reliable for important data, best effort for real-time data)
- Adjust history depth based on how many messages you need to keep
- Consider durability settings for latched messages

## Quiz / Self-check Questions

1. What does DDS stand for in the context of ROS 2?
   a) Distributed Data System
   b) Data Distribution Service
   c) Dynamic Data Sharing
   d) Distributed Development System

2. Which of the following is NOT a communication method in ROS 2?
   a) Topics
   b) Services
   c) Actions
   d) Functions

3. What is the default middleware used by ROS 2 for communication?
   a) TCP
   b) UDP
   c) DDS
   d) HTTP

4. Which Quality of Service policy determines whether messages are delivered reliably?
   a) History
   b) Durability
   c) Reliability
   d) Lifespan

5. What command is used to list all available topics in ROS 2?
   a) ros2 node list
   b) ros2 topic list
   c) ros2 show topics
   d) ros2 list

6. True/False: ROS 2 uses a master-slave architecture like ROS 1.
   a) True
   b) False

7. True/False: Parameters in ROS 2 can be changed at runtime.
   a) True
   b) False

8. True/False: Actions in ROS 2 are used for synchronous request/response communication.
   a) True
   b) False

9. True/False: The robot_state_publisher node publishes TF transforms for a robot model.
   a) True
   b) False

10. True/False: Lifecycle nodes provide better management for complex initialization and cleanup.
    a) True
    b) False

11. Explain the difference between a topic and a service in ROS 2.

12. What are Quality of Service (QoS) policies and why are they important?

13. Describe the purpose of the TF2 library in ROS 2.

14. What is the difference between a regular node and a lifecycle node?

15. Explain how parameters are handled differently in ROS 2 compared to ROS 1.

## Further Reading

Barry, J., & Cousins, S. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

ROS 2 Documentation. (2023). *ROS 2 Humble Hawksbill Documentation*. https://docs.ros.org/en/humble/

Schoellig, A., Holec, M., & Willmann, J. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8

## References

Barry, J., & Cousins, S. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8

Macenski, S., & Cousins, S. (2022). *ROS 2 design paper*. Open Robotics. https://github.com/ros2/design

Open Robotics. (2023). *ROS 2 documentation*. https://docs.ros.org/en/humble/

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

Schoellig, A., Holec, M., & Willmann, J. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8

Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer handbook of robotics* (2nd ed.). Springer. https://doi.org/10.1007/978-3-319-54401-4

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.

Cousins, S., Pradeep, V., Goss, C., & Sczepanski, J. (2015). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *Proceedings of the 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 5574-5580. https://doi.org/10.1109/IROS.2015.7354205

Lynch, K. M., & Park, F. C. (2017). *Modern robotics: Mechanics, planning, and control*. Cambridge University Press.

Choset, H., Lynch, K. M., Hutchinson, S., Kantor, G., Burgard, W., Kavraki, L. E., & Thrun, S. (2005). *Principles of robot motion: Theory, algorithms, and implementations*. MIT Press.

Mason, M. T. (2001). Mechanics of robotic manipulation. MIT Press.

Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot modeling and control* (2nd ed.). John Wiley & Sons.