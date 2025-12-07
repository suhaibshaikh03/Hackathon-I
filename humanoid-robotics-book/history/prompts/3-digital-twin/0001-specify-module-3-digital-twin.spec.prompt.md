---
id: 0001
title: Specify Module 3 Digital Twin
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 3-digital-twin
branch: 2-ros2-nervous-system
user: unknown
command: /sp.phr
labels: ["spec", "module-3"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/3-digital-twin/spec.md
tests:
 - none
---

## Prompt

Module 3: The Digital Twin (Gazebo & Unity) ### /sp.spec – Module 3 Detailed Specification ("WHAT" – not "how")

#### 3. Module 3 Specification – The Digital Twin (Gazebo & Unity)
**WHAT** this module must teach and prove the student can do:

This module focuses on creating virtual replicas—or "digital twins"—of humanoid robots and their environments using Gazebo for physics-based simulation and Unity for photorealistic rendering. Students will learn to construct simulated worlds that mimic real-world physics, integrate robot models, and equip them with sensors to generate realistic data streams compatible with ROS 2. Emphasis is placed on transitioning from URDF (ROS-native) to SDF (Gazebo-optimized) formats for enhanced simulation fidelity, while bridging to Unity for human-robot interaction (HRI) testing. By the end, students must build and launch a complete Gazebo world featuring a humanoid robot that loads under gravity, maintains balance against collisions, and publishes sensor data to ROS 2 topics, verifiable in RViz.

- **Understand Gazebo's Role in Physics Simulation and ROS 2 Integration**
  - Define Gazebo as a 3D simulator for robots, environments, and sensors, supporting accurate physics (gravity, friction, collisions via Bullet/ODE engines) and real-time data generation.
  - Compare Gazebo Classic (legacy, URDF-focused) with Gazebo Harmonic (modern, SDF-native, ROS 2 Jazzy/Rolling compatible) for side-by-side usage, including migration basics like plugin updates and world file adaptations.
  - Explain world building: SDF files for scenes (ground planes, lights, obstacles) with physics tags (e.g., `<gravity>`, `<surface><friction>` for realistic interactions).
  - Student must launch a basic empty world in both Gazebo versions via ROS 2 commands (`ros2 launch gazebo_ros gz_sim.launch.py world:=empty.sdf`), inspect physics parameters with `gz sim -v 4`, and confirm no errors in terminal output.

- **Model Humanoid Robots in URDF and SDF for Gazebo**
  - Convert URDF kinematic descriptions (links, joints, visuals) to SDF equivalents, adding Gazebo-specific extensions like `<inertial>` for mass/dynamics and `<collision>` geometries for physics interactions.
  - Incorporate humanoid-specific elements: Bipedal structure with multi-DOF legs/arms, balance via joint damping, and static/dynamic properties for standing/walking under gravity.
  - Use tools like `xacro` for parametric URDF generation and `gz sdf -p` for SDF validation.
  - Student must author/modify an SDF file for a simple humanoid (e.g., 7-DOF upper body + legs), spawn it in Gazebo (`ros2 launch gazebo_ros spawn_entity.py sdf:=humanoid.sdf`), and demonstrate it responding to physics (e.g., falling/recovering from a push via applied force).

- **Simulate and Calibrate Sensors for Humanoid Perception**
  - Add ROS 2-compatible sensors to SDF models: LiDAR (gpu_lidar plugin for 360° scans, configurable rays/resolution/range/noise), depth cameras (rgbd_camera for RGB-D data, intrinsics like FOV/clipping), IMUs (for orientation/angular velocity/linear acceleration with Gaussian noise models), and joint encoders (via joint_state_publisher).
  - Calibrate via SDF tags (e.g., `<update_rate>`, `<noise><type>gaussian</type></noise>`) and ROS 2 bridges (e.g., `ros_gz_bridge` for topic mapping like `/scan` to `sensor_msgs/LaserScan`).
  - Visualize and verify data: Use RViz to display point clouds, images, and IMU arrows; tune parameters to match real hardware (e.g., RealSense D435i specs).
  - Student must equip a humanoid model with at least three sensors (LiDAR on torso, depth camera on head, IMU on base), launch the simulation, and echo topics (`ros2 topic echo /imu`) to confirm calibrated data streams during motion.

- **Integrate Unity for High-Fidelity Visualization and HRI Testing**
  - Import Gazebo-exported scenes/models (via USD/SDF export) into Unity using ROS-TCP-Connector or ros2_for_unity for bidirectional ROS 2 communication (e.g., subscribing to TF/joint states, publishing commands).
  - Leverage Unity's rendering for photorealistic HRI: Add avatars, lighting/shadows, and XR support to test natural interactions like gesture recognition or multi-modal feedback.
  - Enable visualizations: Use Unity Robotics Visualizations package for runtime display of ROS topics (e.g., 3D LiDAR clouds, markers for paths).
  - Student must export a Gazebo humanoid scene to Unity, set up a simple HRI demo (e.g., robot waves in response to a virtual human gesture via `/cmd_vel`), and run it with synchronized ROS 2 data visible in both simulators.

**Module Deliverables and Proof of Learning:**
- A ROS 2 workspace with SDF worlds, sensor-equipped humanoid models, and launch files for Gazebo (Classic/Harmonic) and Unity integration.
- Runnable demo: Launch a physics-responsive humanoid in Gazebo that stands stably, detects obstacles via sensors, and exports to Unity for HRI visualization; capture via video/log showing topic data (e.g., `/joint_states`, `/camera/depth/image_raw`) and physics interactions.
- All examples must be Ubuntu 22.04 + ROS 2 Humble/Iron + Gazebo Harmonic compatible, with inline code snippets and APA-cited references to official docs (e.g., Gazebo Tutorials, Unity Robotics Hub).

This specification ensures hands-on mastery of digital twins for Physical AI, emphasizing verifiable simulations that bridge to real humanoid deployment.

## Response snapshot

The detailed specification for Module 3: The Digital Twin (Gazebo & Unity) has been written.

## Outcome

- ✅ Impact: Module 3 specification created, detailing learning objectives and deliverables for digital twin implementation in Gazebo and Unity.
- 🧪 Tests: none
- 📁 Files: specs/3-digital-twin/spec.md
- 🔁 Next prompts: Proceed with planning the implementation for Module 3.
- 🧠 Reflection: Successfully created a detailed spec document for a complex module using the provided template.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring a fallback to agent-native PHR creation.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate why `create-phr.sh` was not found and ensure its availability or provide a robust agent-native alternative for future PHR creation.
