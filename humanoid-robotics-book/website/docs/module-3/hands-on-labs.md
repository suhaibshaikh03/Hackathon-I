---
sidebar_position: 3
---

# Module 3 - Hands-on Labs / Code Walkthroughs

## Lab 1: Gazebo Classic vs. Gazebo Harmonic Setup

### Objective
Compare and contrast Gazebo Classic and Gazebo Harmonic, setting up both environments for humanoid robotics simulation.

### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble
- NVIDIA GPU with RTX support for Isaac Sim
- Gazebo Classic and Harmonic installed

### Steps
1. Install both Gazebo versions:
```bash
# Install Gazebo Classic (legacy)
sudo apt update
sudo apt install ros-humble-gazebo-*
sudo apt install gazebo-classic

# Install Gazebo Harmonic (modern)
sudo apt install ros-humble-gazebo-ros
sudo apt install gazebo-harmonic
```

2. Create a basic world file (`basic_world.sdf`):
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1e6</mass>
          <inertia>
            <ixx>1e6</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1e6</iyy>
            <iyz>0</iyz>
            <izz>1e6</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Simple Box Object -->
    <model name="box_object">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 1 1</ambient>
            <diffuse>0.5 0.5 1 1</diffuse>
            <specular>0.5 0.5 1 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

3. Launch the world in both versions:
```bash
# For Gazebo Classic
gazebo basic_world.sdf

# For Gazebo Harmonic
gz sim basic_world.sdf
```

4. Compare the differences in:
   - GUI appearance and controls
   - Performance metrics
   - Plugin compatibility
   - Command-line interface

### Expected Results
- Both simulators load the world successfully
- Differences in GUI, performance, and command structure are observed
- Understanding of when to use each version is gained

## Lab 2: Isaac Sim Environment Creation

### Objective
Create a photorealistic humanoid environment in Isaac Sim with RTX ray-tracing and PhysX physics.

### Steps
1. Launch Isaac Sim:
```bash
# Using Isaac Sim launcher or directly
isaac-sim.bat  # Windows
./isaac-sim.sh  # Linux
```

2. Create a new stage with basic humanoid environment:
```python
# Python script to create environment
import omni
from pxr import Gf, Sdf, UsdGeom
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive

# Create a ground plane
plane = create_primitive(
    prim_path="/World/plane",
    primitive_type="Plane",
    scale=Gf.Vec3f(10, 10, 1),
    position=Gf.Vec3f(0, 0, 0)
)

# Add lighting
from omni.isaac.core.utils.light import create_light
create_light(
    prim_path="/World/light",
    light_type="DistantLight",
    position=Gf.Vec3f(0, 0, 10),
    intensity=3000
)

# Create textured materials for realism
from omni.isaac.core.materials import OmniPBR
material = OmniPBR("/World/materials/wood", color=(0.6, 0.4, 0.2))
```

3. Set up RTX rendering:
   - Enable ray tracing in Isaac Sim settings
   - Configure PhysX physics parameters
   - Add domain randomization elements

4. Create a basic humanoid model:
```python
# Import a simple humanoid model
add_reference_to_stage(
    usd_path="path/to/humanoid_model.usd",
    prim_path="/World/Humanoid"
)
```

### Expected Results
- Photorealistic environment with RTX rendering
- Accurate physics simulation with PhysX
- Domain randomization elements implemented
- Humanoid model properly imported and simulated

## Lab 3: URDF to SDF Conversion and Validation

### Objective
Convert a URDF model to SDF format and validate it for simulation use.

### Steps
1. Create a simple humanoid URDF model (`humanoid.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.15"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0.15 0" rpy="0 0 0.5"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.1 -0.15 0" rpy="0 0 -0.5"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

2. Convert URDF to SDF:
```bash
# Convert using xacro first if needed
ros2 run xacro xacro humanoid.urdf > humanoid_expanded.urdf

# Convert to SDF
gz sdf -p humanoid_expanded.urdf > humanoid.sdf
```

3. Validate the SDF file:
```bash
# Validate SDF syntax
gz sdf -k humanoid.sdf

# Launch in Gazebo
gz sim -r humanoid.sdf
```

4. Add Gazebo-specific extensions to the SDF:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <length>0.3</length>
            <radius>0.15</radius>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 1 1</ambient>
          <diffuse>0 0 1 1</diffuse>
          <specular>0 0 1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <length>0.3</length>
            <radius>0.15</radius>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Add Gazebo-specific extensions -->
    <gazebo reference="base_link">
      <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="head">
      <material>Gazebo/Red</material>
    </gazebo>

    <gazebo reference="left_upper_arm">
      <material>Gazebo/Green</material>
    </gazebo>

    <gazebo reference="right_upper_arm">
      <material>Gazebo/Green</material>
    </gazebo>
  </model>
</sdf>
```

### Expected Results
- URDF model successfully converted to SDF
- SDF file validates without errors
- Model loads correctly in Gazebo
- Gazebo-specific extensions properly applied

## Lab 4: Unity-ROS Integration for HRI Testing

### Objective
Set up Unity-ROS integration using the ROS-TCP-Connector for high-fidelity visualization and human-robot interaction testing.

### Steps
1. Install Unity Robotics packages:
```bash
# In Unity Package Manager:
# 1. Add package from git URL: com.unity.robotics.ros-tcp-connector
# 2. Add package from git URL: com.unity.robotics.urdf-importer
```

2. Create a Unity scene with ROS connection:
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string rosIP = "127.0.0.1";
    int rosPort = 10000;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);

        // Start publishing messages
        InvokeRepeating("SendRobotPose", 0.0f, 0.1f); // Every 0.1 seconds
    }

    void SendRobotPose()
    {
        // Create and send a message
        var poseMsg = new RosMessageTypes.Geometry.PoseMsg();
        poseMsg.position.x = transform.position.x;
        poseMsg.position.y = transform.position.y;
        poseMsg.position.z = transform.position.z;

        ros.Publish("robot_pose", poseMsg);
    }
}
```

3. Set up ROS bridge:
```bash
# Terminal 1: Start ROS bridge
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="127.0.0.1" -p ROS_TCP_PORT:="10000"

# Terminal 2: Test connection
ros2 topic echo /robot_pose geometry_msgs/Pose
```

4. Create HRI test scenario:
   - Design interactive environment in Unity
   - Implement gesture recognition
   - Create visual feedback for robot responses
   - Test human-robot interaction patterns

### Expected Results
- Unity connects successfully to ROS network
- Bidirectional communication established
- Robot pose data transmitted from Unity to ROS
- HRI testing environment operational

## Lab 5: Sensor Simulation and Calibration

### Objective
Implement and calibrate various sensor models in simulation to match real-world characteristics.

### Steps
1. Add sensors to your robot model in SDF:
```xml
<!-- Depth Camera -->
<sensor name="depth_camera" type="depth">
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <visualize>true</visualize>
  <topic>depth_camera/image_raw</topic>
</sensor>

<!-- IMU Sensor -->
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>

<!-- LiDAR Sensor -->
<sensor name="lidar_2d" type="ray">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <visualize>true</visualize>
  <topic>lidar_2d_scan</topic>
</sensor>
```

2. Validate sensor data:
```bash
# Monitor camera data
ros2 run image_view image_view --ros-args -r image:=/depth_camera/image_raw

# Monitor IMU data
ros2 topic echo /imu/data

# Monitor LiDAR data
ros2 run rviz2 rviz2  # Add LaserScan display for /lidar_2d_scan
```

3. Calibrate sensor noise models to match real hardware:
```python
# Example Python script to compare simulated vs. real sensor data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
import numpy as np

class SensorCalibrator(Node):
    def __init__(self):
        super().__init__('sensor_calibrator')

        # Subscribe to simulated sensor data
        self.sim_imu_sub = self.create_subscription(
            Imu, '/sim_imu/data', self.sim_imu_callback, 10)

        # Subscribe to real sensor data (when available)
        self.real_imu_sub = self.create_subscription(
            Imu, '/real_imu/data', self.real_imu_callback, 10)

        # Compare statistics
        self.sim_data = []
        self.real_data = []

    def sim_imu_callback(self, msg):
        # Store simulated data for analysis
        self.sim_data.append([msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z])

    def real_imu_callback(self, msg):
        # Store real data for comparison
        self.real_data.append([msg.linear_acceleration.x,
                               msg.linear_acceleration.y,
                               msg.linear_acceleration.z])

    def compare_statistics(self):
        # Compare mean, variance, and frequency characteristics
        if len(self.sim_data) > 100 and len(self.real_data) > 100:
            sim_acc = np.array(self.sim_data)
            real_acc = np.array(self.real_data)

            print("Simulated IMU Statistics:")
            print(f"Mean: {np.mean(sim_acc, axis=0)}")
            print(f"Std: {np.std(sim_acc, axis=0)}")

            print("Real IMU Statistics:")
            print(f"Mean: {np.mean(real_acc, axis=0)}")
            print(f"Std: {np.std(real_acc, axis=0)}")
```

### Expected Results
- All sensors properly configured in SDF
- Sensor data published with correct topics
- Noise characteristics calibrated to match real hardware
- Sensor validation performed and documented

## Lab 6: Domain Randomization Implementation

### Objective
Implement domain randomization techniques to improve sim-to-real transfer.

### Steps
1. Create domain randomization in Isaac Sim:
```python
# Example Python script for domain randomization
import omni
from omni.isaac.core import World
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.materials import OmniPBR
import numpy as np
import random

class DomainRandomization:
    def __init__(self):
        self.world = World()
        self.scene = self.world.scene

    def randomize_textures(self):
        """Randomize visual properties of objects"""
        materials = self.scene.get_materials()
        for material in materials:
            # Randomize color
            color = [random.uniform(0, 1) for _ in range(3)]
            material.set_color(color)

            # Randomize roughness
            roughness = random.uniform(0.1, 0.9)
            material.set_roughness(roughness)

            # Randomize metallic
            metallic = random.uniform(0.0, 0.5)
            material.set_metallic(metallic)

    def randomize_lighting(self):
        """Randomize lighting conditions"""
        lights = self.scene.get_lights()
        for light in lights:
            # Randomize intensity
            intensity = random.uniform(1000, 5000)
            light.set_intensity(intensity)

            # Randomize color temperature
            temp = random.uniform(3000, 6500)
            light.set_color_temperature(temp)

    def randomize_physics(self):
        """Randomize physical properties"""
        prims = self.scene.get_all_usd_objects()
        for prim in prims:
            if "collision" in prim.prim_type:
                # Randomize friction coefficients
                friction_static = random.uniform(0.1, 1.0)
                friction_dynamic = random.uniform(0.1, 0.9)

                # Apply to physics properties
                # This would involve setting USD physics attributes

    def randomize_object_poses(self):
        """Randomize object positions and orientations"""
        objects = self.scene.get_world_poses()
        for obj in objects:
            # Add small random offsets to positions
            pos_offset = [random.uniform(-0.1, 0.1) for _ in range(3)]
            rot_offset = [random.uniform(-0.1, 0.1) for _ in range(3)]

            # Apply offsets to object transforms

    def apply_randomization(self):
        """Apply all randomization techniques"""
        self.randomize_textures()
        self.randomize_lighting()
        self.randomize_physics()
        self.randomize_object_poses()

        # Reset physics simulation to apply changes
        self.world.reset()
```

2. Implement domain randomization in Gazebo:
```xml
<!-- Example of domain randomization parameters in SDF -->
<model name="randomizable_box">
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 1 1</ambient>
        <diffuse>0.5 0.5 1 1</diffuse>
        <specular>0.5 0.5 1 1</specular>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.1667</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1667</iyy>
        <iyz>0</iyz>
        <izz>0.1667</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Gazebo-specific extensions for domain randomization -->
  <gazebo reference="link">
    <material>Gazebo/Blue</material>
    <!-- Physics parameters that can be randomized -->
    <mu1>0.1</mu1>  <!-- Friction coefficient -->
    <mu2>0.1</mu2>  <!-- Secondary friction coefficient -->
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>100.0</kd>  <!-- Contact damping -->
  </gazebo>
</model>
```

3. Create a training script that applies domain randomization:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import time

class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization_node')

        # Timer to periodically randomize domain parameters
        self.randomization_timer = self.create_timer(5.0, self.randomize_domain)

        # Counter for randomization cycles
        self.cycle_count = 0

    def randomize_domain(self):
        """Randomize domain parameters"""
        # Randomize lighting conditions
        lighting_factor = random.uniform(0.5, 2.0)

        # Randomize object properties
        friction_range = (random.uniform(0.1, 0.5), random.uniform(0.1, 0.5))

        # Randomize sensor noise
        noise_factor = random.uniform(0.8, 1.2)

        self.get_logger().info(f'Domain randomization cycle {self.cycle_count}: '
                              f'lighting={lighting_factor:.2f}, '
                              f'friction=({friction_range[0]:.2f}, {friction_range[1]:.2f}), '
                              f'noise={noise_factor:.2f}')

        self.cycle_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = DomainRandomizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Expected Results
- Domain randomization parameters successfully implemented
- Simulation environment varies appropriately during training
- Performance improvements observed in sim-to-real transfer
- Randomization parameters documented and validated

## Lab 7: Physics Validation and Simulation-to-Reality Transfer

### Objective
Validate physics simulation accuracy and implement techniques for improving sim-to-real transfer.

### Steps
1. Create physics validation experiments:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import time
import math

class PhysicsValidationNode(Node):
    def __init__(self):
        super().__init__('physics_validation_node')

        # Publishers for commanding physical behaviors
        self.command_pub = self.create_publisher(Float64, '/validation_command', 10)

        # Subscribers for measuring responses
        self.response_sub = self.create_subscription(
            Vector3, '/validation_response', self.response_callback, 10)

        # Validation parameters
        self.validation_phase = 0
        self.validation_data = []
        self.start_time = time.time()

        # Timer for validation sequence
        self.validation_timer = self.create_timer(0.1, self.validation_step)

    def response_callback(self, msg):
        """Record response data"""
        current_time = time.time() - self.start_time
        self.validation_data.append((current_time, msg.x, msg.y, msg.z))

    def validation_step(self):
        """Execute physics validation sequence"""
        if self.validation_phase == 0:
            # Free fall validation
            cmd_msg = Float64()
            cmd_msg.data = 0.0  # No external force, just gravity
            self.command_pub.publish(cmd_msg)

            if len(self.validation_data) > 100:  # Collect enough data
                self.validate_free_fall()
                self.validation_phase = 1

        elif self.validation_phase == 1:
            # Pendulum motion validation
            cmd_msg = Float64()
            cmd_msg.data = 9.81 * math.sin(0.1)  # Simple pendulum force
            self.command_pub.publish(cmd_msg)

            if len(self.validation_data) > 200:
                self.validate_pendulum_motion()
                self.validation_phase = 2

    def validate_free_fall(self):
        """Validate free fall acceleration matches 9.81 m/s^2"""
        # Analyze collected data for free fall acceleration
        # Compare to theoretical value of 9.81 m/s^2
        self.get_logger().info('Free fall validation completed')

    def validate_pendulum_motion(self):
        """Validate pendulum motion follows expected physics"""
        # Analyze collected data for pendulum motion
        # Compare to theoretical pendulum equations
        self.get_logger().info('Pendulum motion validation completed')

def main(args=None):
    rclpy.init(args=args)
    node = PhysicsValidationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. Implement system identification for sim-to-real transfer:
```python
#!/usr/bin/env python3
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

class SystemIdentification:
    def __init__(self):
        self.sim_params = {
            'mass': 1.0,
            'friction_coefficient': 0.1,
            'damping': 0.01
        }
        self.real_params = {}  # To be identified
        self.residual_model = None

    def collect_data(self, sim_data, real_data):
        """Collect paired simulation and real-world data"""
        # Ensure data is properly synchronized
        assert len(sim_data) == len(real_data), "Data length mismatch"

        self.sim_inputs = sim_data['inputs']
        self.sim_outputs = sim_data['outputs']
        self.real_outputs = real_data['outputs']

    def residual_function(self, params, sim_output, real_output):
        """Calculate the difference between sim and real outputs"""
        # Adjust sim_output based on identified parameters
        adjusted_output = self.apply_parameter_adjustment(sim_output, params)

        # Calculate residual (difference)
        residual = real_output - adjusted_output

        return np.mean(residual ** 2)  # Mean squared error

    def apply_parameter_adjustment(self, sim_output, params):
        """Apply parameter adjustments to simulation output"""
        # Example: adjust for unmodeled dynamics
        adjusted = sim_output.copy()

        # Apply adjustments based on identified parameters
        for i in range(len(adjusted)):
            # Add residual dynamics model
            if self.residual_model:
                adjusted[i] += self.residual_model.predict(i)

        return adjusted

    def identify_parameters(self):
        """Identify parameters that minimize sim-to-real gap"""
        def objective(params):
            return self.residual_function(params, self.sim_outputs, self.real_outputs)

        # Initial guess for parameters
        initial_guess = [0.1, 0.01, 0.001]  # mass_factor, friction_factor, damping_factor

        # Optimize parameters
        result = minimize(objective, initial_guess, method='BFGS')

        self.real_params = {
            'mass_factor': result.x[0],
            'friction_factor': result.x[1],
            'damping_factor': result.x[2]
        }

        return result

    def validate_transfer(self):
        """Validate sim-to-real transfer with identified parameters"""
        # Apply identified parameters to simulation
        corrected_sim = self.apply_parameter_adjustment(
            self.sim_outputs,
            [self.real_params['mass_factor'],
             self.real_params['friction_factor'],
             self.real_params['damping_factor']]
        )

        # Calculate improvement metric
        original_error = np.mean((self.sim_outputs - self.real_outputs) ** 2)
        corrected_error = np.mean((corrected_sim - self.real_outputs) ** 2)

        improvement = (original_error - corrected_error) / original_error * 100

        print(f"Original error: {original_error:.4f}")
        print(f"Corrected error: {corrected_error:.4f}")
        print(f"Improvement: {improvement:.2f}%")

        return improvement > 10  # Require >10% improvement

def main():
    # Example usage
    sys_id = SystemIdentification()

    # In practice, you would collect real and sim data
    # For this example, we'll use synthetic data
    sim_data = {
        'inputs': np.random.rand(1000),
        'outputs': np.random.rand(1000) * 2
    }
    real_data = {
        'inputs': np.random.rand(1000),
        'outputs': np.random.rand(1000) * 2 + np.random.normal(0, 0.1, 1000)
    }

    sys_id.collect_data(sim_data, real_data)
    result = sys_id.identify_parameters()
    success = sys_id.validate_transfer()

    print(f"Parameter identification {'successful' if success else 'failed'}")
    print(f"Identified parameters: {sys_id.real_params}")

if __name__ == "__main__":
    main()
```

3. Test sim-to-real transfer with validation metrics:
```bash
# Compare simulation and real-world performance
# 1. Run experiment in simulation
# 2. Run same experiment on real hardware
# 3. Compare key metrics:
#    - Success rate
#    - Execution time
#    - Energy consumption
#    - Safety incidents
# 4. Calculate sim-to-real gap
```

### Expected Results
- Physics validation confirms simulation accuracy
- System identification reduces sim-to-real gap
- Transfer learning techniques validated
- Performance metrics show improvement over baseline

## Lab 8: Complete Digital Twin Integration

### Objective
Integrate all components into a complete digital twin system with validation.

### Steps
1. Create a complete system launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Launch Isaac Sim components (if available)
    isaac_sim_nodes = [
        Node(
            package='isaac_sim_nodes',
            executable='isaac_sim_bridge',
            name='isaac_sim_bridge',
            parameters=[
                {'enable_rt': True},
                {'domain_randomization': True}
            ]
        )
    ]

    # Launch Unity bridge
    unity_bridge = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_bridge',
        parameters=[
            {'ROS_IP': '127.0.0.1'},
            {'ROS_TCP_PORT': 10000}
        ]
    )

    # Launch robot controllers
    robot_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller', '-c', '/controller_manager']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['position_controllers', '-c', '/controller_manager']
        )
    ]

    # Launch perception pipeline
    perception_pipeline = [
        Node(
            package='perception_package',
            executable='object_detector',
            name='object_detector',
            parameters=[
                {'detection_threshold': 0.5},
                {'enable_tracking': True}
            ]
        )
    ]

    return LaunchDescription([
        gazebo_launch,
        unity_bridge
    ] + isaac_sim_nodes + robot_controllers + perception_pipeline)
```

2. Validate complete system integration:
```bash
# Launch the complete system
ros2 launch my_digital_twin complete_system.launch.py

# Monitor all system components
ros2 topic list
ros2 node list
ros2 param list

# Test end-to-end functionality
# 1. Send command from Unity
# 2. Process through ROS 2
# 3. Execute in simulation
# 4. Verify behavior matches expectation
```

3. Document performance metrics and validation results:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import csv

class SystemValidator(Node):
    def __init__(self):
        super().__init__('system_validator')

        # Performance tracking
        self.metrics = {
            'start_time': time.time(),
            'message_counts': {},
            'response_times': [],
            'error_rates': []
        }

        # Publishers for validation results
        self.results_pub = self.create_publisher(String, '/validation_results', 10)

        # Timer for periodic validation
        self.validation_timer = self.create_timer(1.0, self.validate_system)

    def validate_system(self):
        """Validate complete system performance"""
        # Check message rates
        # Check response times
        # Check error rates
        # Generate validation report

        report = f"System Validation Report:\n"
        report += f"Runtime: {time.time() - self.metrics['start_time']:.2f}s\n"
        report += f"Active connections validated\n"
        report += f"Performance within acceptable ranges\n"

        msg = String()
        msg.data = report
        self.results_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    validator = SystemValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Expected Results
- Complete digital twin system operational
- All components communicating properly
- Performance metrics validated against requirements
- Comprehensive documentation of integration process