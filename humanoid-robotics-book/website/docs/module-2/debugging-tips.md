---
sidebar_position: 4
---

# Common Pitfalls & Debugging Tips - Module 2

## Common Gazebo Simulation Pitfalls

### 1. Physics Instability
**Problem**: Robot model explodes or behaves erratically in simulation.
**Solution**:
- Check mass and inertia values in URDF/SDF
- Ensure proper joint limits and damping
- Adjust physics engine parameters (step size, solver iterations)
- Verify collision geometries don't overlap

### 2. Sensor Data Issues
**Problem**: Sensor data is unrealistic or missing.
**Solution**:
- Verify sensor topics are being published: `ros2 topic list | grep sensor`
- Check sensor configuration in URDF/SDF
- Validate sensor noise parameters
- Confirm sensor mounting position and orientation

### 3. Performance Problems
**Problem**: Simulation runs slowly or lags.
**Solution**:
- Reduce visual complexity in the scene
- Lower sensor update rates during development
- Use simpler collision geometries (boxes instead of meshes)
- Adjust physics engine parameters for speed vs. accuracy

### 4. Coordinate Frame Issues
**Problem**: Robot components appear in wrong positions or orientations.
**Solution**:
- Use `ros2 run tf2_tools view_frames` to check TF tree
- Verify joint origins in URDF/SDF
- Check joint axis directions and limits
- Use RViz to visualize TF frames

## Unity Integration Troubleshooting

### 1. Connection Problems
**Problem**: Unity cannot connect to ROS 2 network.
**Solution**:
- Verify ROS_DOMAIN_ID matches between Unity and ROS 2
- Check network connectivity between systems
- Ensure firewall allows TCP connections on configured ports
- Validate ROS-TCP-Connector configuration

### 2. Data Synchronization Issues
**Problem**: Unity visualization doesn't match Gazebo simulation.
**Solution**:
- Check message publishing rates and buffer sizes
- Verify coordinate frame conventions (Unity uses left-handed, ROS uses right-handed)
- Validate transform conversions between systems
- Monitor network latency and packet loss

### 3. Rendering Performance
**Problem**: Unity scene runs slowly or has visual artifacts.
**Solution**:
- Optimize 3D models (reduce polygon count)
- Use appropriate texture resolutions
- Adjust rendering quality settings
- Implement Level of Detail (LOD) systems

## Debugging Strategies

### 1. Using Gazebo Tools
```bash
# Check simulation status
gz topic -i

# List all available topics
gz topic -l

# Echo a specific topic
gz topic -e -t /world/default/model/robot/joint_state

# Check model properties
gz model -m robot_name -i
```

### 2. ROS 2 Integration Debugging
```bash
# Monitor TF tree
ros2 run tf2_tools view_frames

# Check ROS topics related to Gazebo
ros2 topic list | grep gazebo

# Echo joint states
ros2 topic echo /joint_states

# Check robot description
ros2 param get /robot_state_publisher robot_description
```

### 3. Sensor Data Validation
```bash
# Monitor camera data
ros2 run image_view image_view

# Check laser scan data
ros2 run rviz2 rviz2  # Add LaserScan display

# Monitor IMU data
ros2 topic echo /imu/data
```

### 4. Physics Debugging
- Enable contact visualization in Gazebo GUI
- Check joint forces and velocities
- Monitor simulation real-time factor (RTF)
- Use Gazebo's built-in physics debugger

## Performance Optimization

### 1. Simulation Optimization
- Use appropriate collision geometries (primitives over meshes)
- Set realistic update rates for sensors
- Adjust physics parameters for your specific use case
- Consider using multi-threaded physics if available

### 2. Sensor Optimization
- Use compressed image transport for camera data
- Implement throttling for high-frequency sensors
- Adjust sensor resolution to minimum required
- Use appropriate Quality of Service (QoS) settings

### 3. Model Optimization
- Simplify meshes for collision detection
- Use appropriate level of detail
- Optimize URDF/SDF with minimal but complete definitions
- Implement proper joint damping and limits

## Sim-to-Real Transfer Debugging

### 1. Domain Randomization Validation
- Monitor which parameters are being randomized
- Validate randomization ranges are realistic
- Compare simulation vs. real-world performance metrics
- Gradually reduce randomization as transfer improves

### 2. System Identification
- Collect data from both simulation and reality
- Identify systematic differences in dynamics
- Validate identified parameters improve transfer
- Use system identification tools to refine models

### 3. Validation Techniques
- Test in simulation before real-world deployment
- Start with simple tasks and increase complexity
- Monitor safety margins during transfer
- Use safety boundaries in both sim and real systems

## Troubleshooting Examples

### Example 1: Robot Falls Through Ground
**Symptoms**: Robot model falls through the ground plane
**Diagnosis**:
- Check mass values in URDF/SDF
- Verify collision geometries are properly defined
- Ensure gravity is enabled in the world file
- Check that links have proper inertial properties

### Example 2: Joint Limits Ignored
**Symptoms**: Robot joints exceed physical limits
**Diagnosis**:
- Verify joint limits are properly set in URDF/SDF
- Check that controllers respect joint limits
- Validate joint type (revolute vs. continuous)
- Monitor joint position feedback

### Example 3: Unity-ROS Communication Fails
**Symptoms**: Unity doesn't receive ROS messages or vice versa
**Diagnosis**:
- Check ROS_DOMAIN_ID environment variable
- Verify ROS-TCP-Connector IP and port configuration
- Confirm firewall settings allow connections
- Test basic network connectivity between systems

## Best Practices for Debugging

1. **Start Simple**: Begin with basic models and gradually add complexity
2. **Validate Models**: Use `gz sdf -k` to validate SDF files
3. **Monitor Performance**: Keep track of real-time factor and update rates
4. **Use Visualization**: Leverage RViz and Gazebo GUI for debugging
5. **Log Data**: Record simulation runs for analysis
6. **Version Control**: Track changes to URDF/SDF files
7. **Documentation**: Keep notes on successful configurations