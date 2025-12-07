---
sidebar_position: 3
---

# Module 2 - Hands-on Labs / Code Walkthroughs

## Lab 1: Creating Your First ROS 2 Package

### Objective
Create a basic ROS 2 package with publisher and subscriber nodes using rclpy.

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
1. Create service server (`my_robot_tutorials/my_robot_tutorials/add_two_ints_server.py`):
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()
    rclpy.spin(add_two_ints_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Create service client (`my_robot_tutorials/my_robot_tutorials/add_two_ints_client.py`):
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_client = AddTwoIntsClient()
    response = add_two_ints_client.send_request(1, 2)
    add_two_ints_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    add_two_ints_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Build and test the service:
```bash
# Terminal 1: Start the service server
ros2 run my_robot_tutorials add_two_ints_server

# Terminal 2: Call the service
ros2 run my_robot_tutorials add_two_ints_client
```

### Expected Results
- Service server receives requests and returns the sum of two integers
- Service client sends request and prints the result

## Lab 3: URDF Robot Model

### Objective
Create a simple URDF model for a 6-DOF humanoid arm.

### Steps
1. Create URDF files in your package:
```bash
mkdir -p my_robot_tutorials/urdf
```

2. Create a 6-DOF arm URDF file (`my_robot_tutorials/urdf/humanoid_arm.urdf`):
```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder joint and link -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_yaw" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Upper arm -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="elbow_pitch" type="revolute">
    <parent link="upper_arm"/>
    <child link="lower_arm"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Lower arm -->
  <link name="lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.035"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="wrist_pitch" type="revolute">
    <parent link="lower_arm"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.125" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="wrist_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="wrist_yaw" type="revolute">
    <parent link="wrist_link"/>
    <child link="hand"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="1"/>
  </joint>

  <!-- Hand -->
  <link name="hand">
    <visual>
      <geometry>
        <box size="0.08 0.06 0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.06 0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

3. Create a launch file to visualize the robot:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('my_robot_tutorials')
    urdf_path = os.path.join(package_dir, 'urdf', 'humanoid_arm.urdf')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'config', 'view_robot.rviz')]
        )
    ])
```

4. Launch RViz to visualize the robot:
```bash
# Terminal 1: Publish the robot description
ros2 launch my_robot_tutorials view_arm.launch.py
```

### Expected Results
- 6-DOF humanoid arm model appears in RViz with proper visual representation
- TF tree shows the relationships between links
- Joints can be manipulated using the joint_state_publisher_gui

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
            output='screen',
            parameters=[
                {'param1': 'value1'},
                {'param2': 'value2'}
            ]
        ),
        Node(
            package='my_robot_tutorials',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])
```

3. Create a more complex launch file with parameters (`my_robot_tutorials/launch/robot_demo_launch.py`):
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    background_r_arg = DeclareLaunchArgument(
        'background_r',
        default_value='50',
        description='Background red value (0-255)'
    )

    return LaunchDescription([
        background_r_arg,
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[
                {'background_b': 255},
                {'background_g': 255},
                {'background_r': LaunchConfiguration('background_r')}
            ]
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            remappings=[
                ('/turtle1/cmd_vel', '/cmd_vel')
            ]
        )
    ])
```

4. Launch the system:
```bash
ros2 launch my_robot_tutorials talker_listener_launch.py
```

### Expected Results
- Both talker and listener nodes start simultaneously
- Communication occurs as in the individual node test
- Parameters can be passed via launch arguments

## Lab 5: Quality of Service (QoS) Implementation

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

3. Monitor both topics with different tools to observe the differences:
```bash
# Monitor reliable topic
ros2 topic echo /reliable_topic std_msgs/msg/String

# Monitor best effort topic
ros2 topic echo /best_effort_topic std_msgs/msg/String

# Check message rates
ros2 topic hz /reliable_topic
ros2 topic hz /best_effort_topic
```

### Expected Results
- Messages published with different QoS policies
- Understanding of when to use different QoS settings
- Reliable topic should have guaranteed delivery, best effort may have losses

## Lab 6: Parameter Management

### Objective
Implement parameter management in ROS 2 nodes.

### Steps
1. Create a parameter node (`my_robot_tutorials/my_robot_tutorials/parameter_demo.py`):
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters

class ParameterDemoNode(Node):
    def __init__(self):
        super().__init__('parameter_demo_node')

        # Declare parameters with default values and descriptors
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(
                description='Name of the robot',
                type=ParameterType.PARAMETER_STRING
            )
        )

        self.declare_parameter(
            'max_velocity',
            1.0,
            ParameterDescriptor(
                description='Maximum velocity for robot movement',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        self.declare_parameter(
            'wheel_diameter',
            0.1,
            ParameterDescriptor(
                description='Diameter of robot wheels in meters',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

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
        from rcl_interfaces.msg import SetParametersResult
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == 'robot_name' and param.type_ == ParameterType.PARAMETER_STRING:
                self.robot_name = param.value
                self.get_logger().info(f'Updated robot_name to {self.robot_name}')
            elif param.name == 'max_velocity' and param.type_ == ParameterType.PARAMETER_DOUBLE:
                if param.value > 0.0 and param.value <= 10.0:  # Reasonable limits
                    self.max_velocity = param.value
                    self.get_logger().info(f'Updated max_velocity to {self.max_velocity}')
                else:
                    result.successful = False
                    result.reason = f'Max_velocity must be between 0.0 and 10.0, got {param.value}'
                    break
            elif param.name == 'wheel_diameter' and param.type_ == ParameterType.PARAMETER_DOUBLE:
                if param.value > 0.0:  # Positive diameter required
                    self.wheel_diameter = param.value
                    self.get_logger().info(f'Updated wheel_diameter to {self.wheel_diameter}')
                else:
                    result.successful = False
                    result.reason = f'Wheel diameter must be positive, got {param.value}'
                    break

        return result

def main(args=None):
    rclpy.init(args=args)
    parameter_demo_node = ParameterDemoNode()

    # Create a timer to periodically log parameter values
    timer = parameter_demo_node.create_timer(5.0, lambda: parameter_demo_node.get_logger().info(
        f'Current params - Robot: {parameter_demo_node.robot_name}, '
        f'Velocity: {parameter_demo_node.max_velocity}, '
        f'Wheel: {parameter_demo_node.wheel_diameter}'
    ))

    rclpy.spin(parameter_demo_node)
    parameter_demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Create a parameters file (`my_robot_tutorials/config/robot_params.yaml`):
```yaml
parameter_demo_node:
  ros__parameters:
    robot_name: 'turtlebot4'
    max_velocity: 0.5
    wheel_diameter: 0.08
```

3. Launch with parameters:
```bash
ros2 run my_robot_tutorials parameter_demo --ros-args --params-file config/robot_params.yaml
```

4. Test dynamic parameter changes:
```bash
# Change parameters at runtime
ros2 param set /parameter_demo_node robot_name "new_robot_name"
ros2 param set /parameter_demo_node max_velocity 2.0
```

### Expected Results
- Node starts with parameters loaded from YAML file
- Parameters can be changed dynamically during runtime
- Parameter validation works to prevent invalid values

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

3. Test with the nodes created in previous labs:
```bash
# Monitor the talker/listener communication
ros2 topic hz /topic
ros2 topic echo /topic

# Check node status
ros2 node info /minimal_publisher
ros2 node info /minimal_subscriber
```

### Expected Results
- Ability to monitor and debug ROS 2 systems effectively
- Understanding of various diagnostic tools
- Successful use of rqt tools for visualization