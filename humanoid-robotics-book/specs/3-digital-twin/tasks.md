# Implementation Checklist for Module 3: The Digital Twin (Gazebo & Unity)

## Phase 1: Gazebo Physics Simulation and ROS 2 Integration
- [ ] Set up Gazebo Classic and Gazebo Harmonic environments
- [ ] Configure ROS 2 Humble/Iron with Gazebo integration
- [ ] Create basic empty world SDF file for testing
- [ ] Launch basic empty world in both Gazebo versions via ROS 2 commands
- [ ] Inspect physics parameters with `gz sim -v 4`
- [ ] Verify no errors in terminal output
- [ ] Compare differences between Gazebo Classic and Harmonic
- [ ] Document migration basics for plugin updates and world file adaptations

## Phase 2: SDF World Building and Physics Configuration
- [ ] Create SDF files for scenes with physics tags (`<gravity>`, `<surface><friction>`)
- [ ] Configure ground planes with appropriate physics properties
- [ ] Add lighting configurations to SDF scenes
- [ ] Set up obstacle models in simulation environments
- [ ] Test realistic interactions with physics parameters
- [ ] Validate SDF files using `gz sdf -p` command

## Phase 3: Humanoid Robot Modeling in SDF
- [ ] Convert URDF kinematic descriptions to SDF equivalents
- [ ] Add Gazebo-specific extensions (`<inertial>` for mass/dynamics)
- [ ] Add collision geometries for physics interactions
- [ ] Incorporate humanoid-specific elements (bipedal structure)
- [ ] Configure multi-DOF legs and arms for humanoid
- [ ] Set up balance via joint damping
- [ ] Configure static and dynamic properties for standing/walking under gravity
- [ ] Use `xacro` for parametric URDF generation
- [ ] Author/modify SDF file for simple humanoid (7-DOF upper body + legs)

## Phase 4: Spawning and Physics Testing
- [ ] Spawn humanoid model in Gazebo using `ros2 launch gazebo_ros spawn_entity.py`
- [ ] Test humanoid responding to physics (falling from pushes)
- [ ] Verify humanoid recovering from applied forces
- [ ] Test humanoid standing under gravity
- [ ] Validate joint damping parameters
- [ ] Test physics interactions with environment

## Phase 5: Sensor Simulation and Integration
- [ ] Add LiDAR sensor to SDF model (gpu_lidar plugin)
- [ ] Configure LiDAR parameters (rays, resolution, range, noise)
- [ ] Add depth camera to SDF model (rgbd_camera)
- [ ] Configure depth camera parameters (FOV, clipping)
- [ ] Add IMU to SDF model
- [ ] Configure IMU parameters (orientation, angular velocity, linear acceleration with Gaussian noise)
- [ ] Add joint encoders via joint_state_publisher
- [ ] Configure sensor update rates
- [ ] Set up noise models for sensors

## Phase 6: ROS 2 Bridge Configuration
- [ ] Configure SDF tags for sensor calibration
- [ ] Set up `<noise><type>gaussian</type></noise>` for sensors
- [ ] Configure ROS 2 bridges (e.g., `ros_gz_bridge`)
- [ ] Map topics like `/scan` to `sensor_msgs/LaserScan`
- [ ] Test topic mapping for all sensors
- [ ] Validate sensor data streams

## Phase 7: Visualization and Data Verification
- [ ] Set up RViz for displaying point clouds
- [ ] Configure RViz for displaying images
- [ ] Set up RViz for displaying IMU arrows
- [ ] Tune parameters to match real hardware (e.g., RealSense D435i specs)
- [ ] Equip humanoid model with at least three sensors (LiDAR on torso, depth camera on head, IMU on base)
- [ ] Launch simulation with all sensors
- [ ] Echo topics using `ros2 topic echo /imu`
- [ ] Confirm calibrated data streams during motion

## Phase 8: Unity Integration Setup
- [ ] Install Unity and ROS-TCP-Connector
- [ ] Install ros2_for_unity package
- [ ] Set up bidirectional ROS 2 communication
- [ ] Configure subscription to TF/joint states
- [ ] Set up command publishing capability
- [ ] Test basic ROS 2 communication with Unity

## Phase 9: High-Fidelity Visualization
- [ ] Export Gazebo humanoid scene to Unity using USD/SDF export
- [ ] Import scenes/models into Unity
- [ ] Add avatars to Unity environment
- [ ] Configure lighting and shadows in Unity
- [ ] Set up XR support for HRI testing
- [ ] Implement gesture recognition capabilities
- [ ] Add multi-modal feedback systems

## Phase 10: Unity Robotics Visualizations
- [ ] Install Unity Robotics Visualizations package
- [ ] Set up runtime display of ROS topics
- [ ] Configure 3D LiDAR clouds visualization
- [ ] Set up markers for paths
- [ ] Test visualization of ROS data in Unity

## Phase 11: HRI Demo Implementation
- [ ] Export Gazebo humanoid scene to Unity
- [ ] Set up simple HRI demo (robot waves in response to virtual human gesture)
- [ ] Implement `/cmd_vel` command handling
- [ ] Run demo with synchronized ROS 2 data visible in both simulators
- [ ] Test bidirectional communication between Gazebo and Unity
- [ ] Validate synchronized data streams

## Phase 12: Workspace and Launch Files
- [ ] Create ROS 2 workspace with SDF worlds
- [ ] Add sensor-equipped humanoid models to workspace
- [ ] Create launch files for Gazebo Classic integration
- [ ] Create launch files for Gazebo Harmonic integration
- [ ] Create launch files for Unity integration
- [ ] Test complete launch system
- [ ] Document workspace structure

## Phase 13: Deliverables and Proof of Learning
- [ ] Create runnable demo launching physics-responsive humanoid in Gazebo
- [ ] Verify humanoid stands stably in simulation
- [ ] Test obstacle detection via sensors
- [ ] Export to Unity for HRI visualization
- [ ] Capture video/log showing topic data (e.g., `/joint_states`, `/camera/depth/image_raw`)
- [ ] Document physics interactions
- [ ] Verify Ubuntu 22.04 + ROS 2 Humble/Iron + Gazebo Harmonic compatibility
- [ ] Add inline code snippets to documentation
- [ ] Include APA-cited references to official docs

## Phase 14: Testing and Validation
- [ ] Test complete Gazebo world with humanoid robot
- [ ] Verify robot loads under gravity properly
- [ ] Test balance against collisions
- [ ] Validate sensor data publishing to ROS 2 topics
- [ ] Verify data streams in RViz
- [ ] Test Unity export functionality
- [ ] Validate HRI capabilities
- [ ] Document any issues and resolutions

## Phase 15: Documentation and Completion
- [ ] Document all code snippets and configurations
- [ ] Create user guides for each component
- [ ] Verify all examples work on target platforms
- [ ] Complete APA citations for official documentation
- [ ] Prepare final deliverable package
- [ ] Validate module meets learning objectives