---
sidebar_position: 3
---

# Module 1 - Hands-on Labs / Code Walkthroughs

## Lab 1: Creating Your First ROS 2 Package

### Objective
Create a basic ROS 2 package with a publisher and subscriber node.

### Prerequisites
- ROS 2 Humble Hawksbill installed
- Proper ROS 2 environment sourced

### Steps
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

3. Navigate to the package directory and create publisher node (`my_robot_tutorials/my_robot_tutorials/publisher_member_function.py`):
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

4. Create subscriber node (`my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py`):
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

5. Update the `setup.py` file to include entry points for your executables:
```python
entry_points={
    'console_scripts': [
        'talker = my_robot_tutorials.publisher_member_function:main',
        'listener = my_robot_tutorials.subscriber_member_function:main',
    ],
},
```

6. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

7. Run the nodes:
```bash
# Terminal 1
ros2 run my_robot_tutorials talker

# Terminal 2
ros2 run my_robot_tutorials listener
```

### Expected Results
- Publisher node publishes "Hello World" messages with an incrementing counter
- Subscriber node receives and prints the messages to the console

## Lab 2: Service Implementation

### Objective
Implement a service server and client for robot control.

### Steps
1. Create service server (`my_robot_tutorials/my_robot_tutorials/add_three_ints_server.py`):
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

2. Create service client (`my_robot_tutorials/my_robot_tutorials/add_three_ints_client.py`):
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

3. Build and test the service:
```bash
# Terminal 1: Start the service server
ros2 run my_robot_tutorials add_three_ints_server

# Terminal 2: Call the service
ros2 run my_robot_tutorials add_three_ints_client
```

### Expected Results
- Service server receives requests and returns the sum of three integers
- Service client sends request and prints the result

## Lab 3: URDF Robot Model

### Objective
Create a simple URDF model for a mobile robot.

### Steps
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

### Expected Results
- Robot model appears in RViz with proper visual representation
- TF tree shows the relationships between links

## Lab 4: Launch Files and System Composition

### Objective
Create launch files to manage multiple nodes and system composition.

### Steps
1. Create a launch directory in your package:
```bash
mkdir -p my_robot_tutorials/launch
```

2. Create a launch file (`my_robot_tutorials/launch/talker_listener_launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='my_robot_tutorials',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
```

3. Launch the system:
```bash
ros2 launch my_robot_tutorials talker_listener_launch.py
```

### Expected Results
- Both talker and listener nodes start simultaneously
- Communication occurs as in the individual node test

## Lab 5: Parameter Management

### Objective
Implement parameter management in ROS 2 nodes.

### Steps
1. Create a parameter node (`my_robot_tutorials/my_robot_tutorials/parameter_node.py`):
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('wheel_diameter', 0.1)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Wheel diameter: {self.wheel_diameter}')

        # Set up parameter callback for dynamic changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
            elif param.name == 'max_velocity':
                self.max_velocity = param.value
            elif param.name == 'wheel_diameter':
                self.wheel_diameter = param.value
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    rclpy.spin(parameter_node)
    parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Create a parameters file (`my_robot_tutorials/config/robot_params.yaml`):
```yaml
parameter_node:
  ros__parameters:
    robot_name: 'turtlebot4'
    max_velocity: 0.5
    wheel_diameter: 0.08
```

3. Launch with parameters:
```bash
ros2 run my_robot_tutorials parameter_node --ros-args --params-file config/robot_params.yaml
```

### Expected Results
- Node starts with parameters loaded from YAML file
- Parameters can be changed dynamically during runtime

## Lab 6: Quality of Service (QoS) Implementation

### Objective
Implement different QoS policies for various communication requirements.

### Steps
1. Create a QoS example node (`my_robot_tutorials/my_robot_tutorials/qos_example.py`):
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

class QoSExampleNode(Node):
    def __init__(self):
        super().__init__('qos_example_node')

        # Create different QoS profiles
        # Reliable communication (for important data)
        reliable_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Best effort communication (for real-time data)
        best_effort_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Create publishers with different QoS
        self.reliable_publisher = self.create_publisher(String, 'reliable_topic', reliable_qos)
        self.best_effort_publisher = self.create_publisher(String, 'best_effort_topic', best_effort_qos)

        # Create timer to send messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Message {self.counter}'

        self.reliable_publisher.publish(msg)
        self.best_effort_publisher.publish(msg)

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    qos_example_node = QoSExampleNode()
    rclpy.spin(qos_example_node)
    qos_example_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Run the QoS example:
```bash
ros2 run my_robot_tutorials qos_example
```

### Expected Results
- Messages published with different QoS policies
- Understanding of when to use different QoS settings

## Lab 7: Debugging and Monitoring Tools

### Objective
Use ROS 2 debugging and monitoring tools effectively.

### Steps
1. Monitor system with various tools:
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Monitor topic information
ros2 topic info /topic_name

# Check topic message rate
ros2 topic hz /topic_name

# Echo topic messages
ros2 topic echo /topic_name

# Check node information
ros2 node info /node_name

# Visualize the node graph
ros2 run rqt_graph rqt_graph

# Check system health
ros2 doctor

# Record data
ros2 bag record -a

# Play back recorded data
ros2 bag play /path/to/bagfile
```

2. Use rqt tools for visualization:
```bash
# Launch rqt with various plugins
ros2 run rqt_gui rqt_gui

# Specific tools
ros2 run rqt_plot rqt_plot
ros2 run rqt_console rqt_console
ros2 run rqt_tf_tree rqt_tf_tree
```

### Expected Results
- Ability to monitor and debug ROS 2 systems effectively
- Understanding of various diagnostic tools