---
sidebar_position: 1
---

# Module 3 – The Digital Twin (Gazebo & Unity)

## Learning Objectives
- Build photorealistic, physics-accurate humanoid environments in Isaac Sim 5.0+
- Use RTX ray-tracing and PhysX 5.4+ for accurate physics simulation
- Create domain-randomized assets (textures, lighting, object poses, distractors)
- Define Gazebo as a 3D simulator for robots, environments, and sensors
- Support accurate physics (gravity, friction, collisions via Bullet/ODE engines) and real-time data generation
- Compare Gazebo Classic (legacy, URDF-focused) with Gazebo Harmonic (modern, SDF-native, ROS 2 Jazzy/Rolling compatible)
- Explain world building: SDF files for scenes (ground planes, lights, obstacles) with physics tags
- Launch basic empty world in both Gazebo versions via ROS 2 commands
- Import and rig production-grade humanoid models (Unitree G1/H1, Figure 02, Agility Digit, etc.) via OpenUSD
- Configure correct joint limits, torque curves, and soft-body contacts
- Convert URDF kinematic descriptions (links, joints, visuals) to SDF equivalents
- Add Gazebo-specific extensions like `<inertial>` for mass/dynamics and `<collision>` geometries for physics interactions
- Incorporate humanoid-specific elements: Bipedal structure with multi-DOF legs/arms
- Configure balance via joint damping, and static/dynamic properties for standing/walking under gravity
- Use tools like `xacro` for parametric URDF generation
- Use `gz sdf -p` for SDF validation
- Author/modify an SDF file for a simple humanoid (e.g., 7-DOF upper body + legs)
- Spawn it in Gazebo using `ros2 launch gazebo_ros spawn_entity.py`
- Demonstrate it responding to physics (e.g., falling/recovering from a push via applied force)

## Core Concepts

### Digital Twin Architecture
A digital twin is a virtual replica of a physical robot or system that exists simultaneously in the physical and digital worlds. In robotics, digital twins bridge the gap between simulation and reality, allowing for safer and more cost-effective development.

### Simulation Platforms Comparison
This module covers two major simulation platforms:
- **Gazebo**: Traditional physics simulation with strong ROS integration
- **Unity**: High-fidelity visualization and HRI testing platform
- **Isaac Sim**: NVIDIA's advanced simulation with RTX rendering and PhysX physics

### Physics Simulation Fundamentals
Modern simulation engines provide accurate physics modeling essential for robotics development:
- Gravity and environmental forces
- Friction and contact dynamics
- Collision detection and response
- Realistic sensor simulation

## Key Technologies & Tools
- **Gazebo Classic**: Legacy version with strong URDF support
- **Gazebo Harmonic**: Modern version with native SDF support
- **Isaac Sim**: NVIDIA's high-fidelity simulation platform
- **Unity Robotics**: High-fidelity visualization and HRI testing
- **ROS-TCP-Connector**: Bridge between ROS 2 and Unity
- **SDF (Simulation Description Format)**: XML-based simulation format
- **URDF (Unified Robot Description Format)**: Robot description format
- **OpenUSD**: Universal Scene Description for asset interchange

## Hands-on Labs / Code Walkthroughs
In this module, you'll implement:
1. Gazebo Classic and Harmonic environment creation
2. SDF world building with physics configuration
3. URDF to SDF conversion and validation
4. Humanoid model import and rigging with OpenUSD
5. Sensor simulation and calibration
6. Unity-ROS integration for visualization
7. Physics validation and simulation-to-reality transfer

## Common Pitfalls & Debugging Tips
- Ensure proper physics parameters match real-world values for accurate simulation
- Validate SDF files using `gz sdf -p` command
- Check coordinate frame relationships with `ros2 run tf2_tools view_frames`
- Verify sensor noise models match real hardware characteristics
- Use appropriate domain randomization for sim-to-real transfer

## Quiz / Self-check Questions
1. What are the key differences between Gazebo Classic and Gazebo Harmonic?
2. How does RTX ray-tracing enhance simulation quality in Isaac Sim?
3. What is the purpose of domain randomization in digital twin systems?
4. How do you validate SDF files for simulation?
5. What are the advantages of using Unity for HRI testing?

## Further Reading
Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *Proceedings 2004 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 3, 2149-2154. https://doi.org/10.1109/IROS.2004.1389862

Unity Technologies. (2023). *Unity Robotics Hub Documentation*. https://github.com/Unity-Technologies/Unity-Robotics-Hub

NVIDIA Corporation. (2023). *Isaac Sim Documentation*. https://docs.omniverse.nvidia.com/isaacsim/

Open Robotics. (2023). *Gazebo Documentation*. https://gazebosim.org/

## References

Cousins, S., Pradeep, V., Goss, C., & Sczepanski, J. (2015). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *Proceedings of the 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 5574-5580. https://doi.org/10.1109/IROS.2015.7354205

Unity Technologies. (2023). *Unity robotics simulation*. https://github.com/Unity-Technologies/Unity-Robotics-Hub

NVIDIA Corporation. (2023). *Isaac Sim documentation*. https://docs.omniverse.nvidia.com/isaacsim/

Open Robotics. (2023). *Gazebo documentation*. https://gazebosim.org/

Coumans, E., & Bai, Y. (2016). Mujoco: A physics engine for model-based control. *IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots*, 285-290. https://doi.org/10.1109/CIMSim.2016.7755424

Barry, J., & Cousins, S. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

Schoellig, A., Holec, M., & Willmann, J. (2022). The Robot Operating System 2: Design, challenges, and lessons learned. In *New Results in Control Theory and Its Applications* (pp. 153-173). Springer. https://doi.org/10.1007/978-3-030-97062-3_8