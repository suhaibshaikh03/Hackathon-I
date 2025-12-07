---
sidebar_position: 2
---

# Appendix B: Troubleshooting Guide

## Common ROS 2 Issues

### 1. Node Communication Problems
**Symptoms**: Nodes cannot communicate or topics are not visible across machines
**Solutions**:
- Check ROS_DOMAIN_ID: `echo $ROS_DOMAIN_ID` (should be same on all machines)
- Verify network configuration: `export ROS_LOCALHOST_ONLY=1` for single-machine testing
- Check firewall settings for DDS communication ports (UDP/TCP on various ports)
- Use `ros2 doctor` to diagnose system issues
- Verify nodes are on same network: `ping other_machine_ip`

### 2. Performance Issues
**Symptoms**: High CPU usage, memory leaks, slow response times
**Solutions**:
- Check timer frequencies in nodes (avoid too-high frequencies)
- Use appropriate QoS settings for communication
- Implement proper resource cleanup in node destruction
- Monitor with `htop` or `ros2 doctor`
- Check for circular references preventing garbage collection

### 3. Parameter Issues
**Symptoms**: Parameters not loading from launch files, runtime changes not taking effect
**Solutions**:
- Verify parameter file syntax with YAML linter
- Check parameter names match exactly between declaration and usage
- Use `ros2 param list` to see loaded parameters
- Ensure parameters are declared before getting/setting
- Check namespace in launch files

## Isaac Platform Troubleshooting

### 1. Isaac Sim Issues
**Symptoms**: Isaac Sim fails to launch, crashes, or runs slowly
**Solutions**:
- Verify NVIDIA GPU drivers and RTX support: `nvidia-smi`
- Check CUDA installation: `nvcc --version`
- Ensure sufficient VRAM for scene complexity
- Close other GPU-intensive applications
- Verify Isaac Sim installation and license activation

### 2. Isaac Lab Training Issues
**Symptoms**: Training fails to start, slow training, or NaN losses
**Solutions**:
- Check GPU memory usage: `nvidia-smi`
- Verify Isaac Lab installation with test scripts
- Ensure sufficient system RAM for parallel environments
- Check Python virtual environment activation
- Validate robot and environment configurations

### 3. Isaac ROS Perception Issues
**Symptoms**: Perception nodes crash, high latency, or inaccurate results
**Solutions**:
- Verify Isaac ROS package installation
- Check GPU acceleration for perception nodes
- Monitor input data rates and formats
- Validate sensor calibration parameters
- Use appropriate QoS settings for real-time performance

## Simulation Issues

### 1. Gazebo Problems
**Symptoms**: Gazebo fails to launch, physics instability, or rendering issues
**Solutions**:
- Check Gazebo installation: `gazebo --version`
- Verify graphics drivers and OpenGL support
- Check SDF file syntax: `gz sdf -k model.sdf`
- Validate physics parameters and joint limits
- Monitor system resources during simulation

### 2. Unity Integration Issues
**Symptoms**: Unity-ROS connection fails, high latency, or data synchronization problems
**Solutions**:
- Verify ROS-TCP-Connector installation and configuration
- Check IP addresses and port configurations
- Ensure firewall allows TCP connections
- Validate Unity and ROS network settings
- Check ROS_DOMAIN_ID consistency

### 3. URDF/SDF Conversion Problems
**Symptoms**: Models don't load correctly, joint limits ignored, or visual artifacts
**Solutions**:
- Validate URDF syntax: `check_urdf robot.urdf`
- Verify joint types and limits in SDF
- Check coordinate frame conventions
- Validate mass and inertia properties
- Use `gz sdf -p urdf_file.urdf > sdf_file.sdf` for conversion

## Vision-Language-Action Pipeline Issues

### 1. Voice Recognition Problems
**Symptoms**: Whisper fails to transcribe, high error rate, or audio issues
**Solutions**:
- Check microphone permissions and configuration
- Verify audio format compatibility
- Test with different Whisper model sizes
- Apply audio preprocessing for noise reduction
- Check audio input/output device selection

### 2. LLM Integration Issues
**Symptoms**: LLM fails to generate plans, incorrect responses, or API errors
**Solutions**:
- Verify API key configuration and rate limits
- Check prompt formatting and structure
- Validate JSON parsing in responses
- Implement proper error handling and retries
- Test with simple commands first

### 3. Perception Pipeline Problems
**Symptoms**: Object detection fails, incorrect classifications, or slow processing
**Solutions**:
- Verify camera calibration parameters
- Check lighting conditions and image quality
- Validate detection model and weights
- Ensure proper coordinate frame transformations
- Monitor GPU memory and compute usage

## Hardware Integration Issues

### 1. Sensor Problems
**Symptoms**: Sensor data missing, incorrect values, or high noise
**Solutions**:
- Check sensor driver installation and configuration
- Verify sensor calibration files
- Test sensors independently before integration
- Check cable connections and power supply
- Validate sensor frame alignment and transforms

### 2. Actuator Control Issues
**Symptoms**: Joints not responding, erratic movements, or safety limits triggered
**Solutions**:
- Verify actuator driver installation
- Check joint limits and safety parameters
- Validate control loop timing and frequencies
- Test with simple position commands first
- Check for mechanical issues or obstructions

### 3. Network Communication Problems
**Symptoms**: Lost connections, high latency, or data corruption
**Solutions**:
- Check network cable connections and quality
- Verify network configuration and IP settings
- Monitor network bandwidth usage
- Use wired connections instead of Wi-Fi for critical data
- Check for network interference or congestion

## Debugging Strategies

### 1. Systematic Debugging Approach
1. **Reproduce the issue**: Ensure you can consistently reproduce the problem
2. **Isolate the component**: Identify which specific component is failing
3. **Check prerequisites**: Verify all dependencies and configurations
4. **Review logs**: Examine system logs and error messages
5. **Test incrementally**: Break down the problem into smaller parts
6. **Document the solution**: Record the fix for future reference

### 2. Common Debugging Commands
```bash
# ROS 2 debugging
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 param list
ros2 node info /node_name
ros2 topic info /topic_name
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# System monitoring
htop
nvidia-smi
df -h
free -h

# Isaac Sim debugging
isaac-sim.sh --/renderer/ogl=4  # Force OpenGL renderer
isaac-sim.sh --summary-cache-dir /tmp/isaac_cache  # Use temp cache

# TF debugging
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo frame1 frame2
```

### 3. Logging Best Practices
```python
# Use appropriate logging levels
self.get_logger().debug("Detailed debugging info")
self.get_logger().info("General information")
self.get_logger().warn("Warning message")
self.get_logger().error("Error occurred")
self.get_logger().fatal("Fatal error")

# Include contextual information
self.get_logger().info(f'Processing command {command_id} for target {target_object}')
```

## Performance Optimization

### 1. Identify Bottlenecks
- Use profiling tools to identify slow components
- Monitor CPU, GPU, and memory usage
- Check communication latencies between nodes
- Analyze message rates and queue depths

### 2. Optimization Strategies
- Use appropriate QoS settings for different data types
- Implement efficient data structures and algorithms
- Consider multi-threading for I/O operations
- Optimize sensor data rates and resolutions
- Use intra-process communication when possible

## Safety and Error Handling

### 1. Emergency Procedures
- Implement emergency stop functionality
- Use safety boundaries and geofencing
- Monitor system health continuously
- Implement graceful degradation

### 2. Error Recovery
- Design systems to handle errors gracefully
- Implement retry mechanisms for transient failures
- Use timeouts to prevent indefinite waiting
- Log errors for diagnostic purposes

## Validation and Testing

### 1. Component Testing
- Test each component independently before integration
- Use mock objects for external dependencies
- Validate input/output behavior
- Test error conditions and edge cases

### 2. Integration Testing
- Test component interactions
- Validate system behavior as a whole
- Check performance under load
- Verify safety systems function correctly

## Hardware-Specific Troubleshooting

### 1. NVIDIA Jetson Issues
**Symptoms**: Performance issues, thermal throttling, or power problems
**Solutions**:
- Check power supply adequacy (use recommended barrel jack for full power)
- Monitor thermal status: `sudo tegrastats`
- Adjust power modes: `sudo nvpmodel -m 0` for maximum performance
- Verify JetPack version compatibility

### 2. RTX GPU Issues
**Symptoms**: Rendering problems, CUDA errors, or memory issues
**Solutions**:
- Update GPU drivers to latest version
- Check CUDA installation and version compatibility
- Monitor GPU memory usage
- Verify ray tracing and tensor core capabilities

## Common Error Messages and Solutions

### ROS 2 Errors
- **"Failed to contact master"**: Check if ROS 2 daemon is running and network configuration
- **"Topic not found"**: Verify topic names and node connections
- **"Service not available"**: Check if service server is running and accessible
- **"Parameter not declared"**: Declare parameters before getting/setting them

### Isaac Platform Errors
- **"CUDA initialization error"**: Verify GPU drivers and CUDA installation
- **"License not activated"**: Check Isaac Sim license activation
- **"Model loading failed"**: Verify model paths and permissions
- **"Simulation instability"**: Check physics parameters and timestep settings

### Audio/Video Errors
- **"Microphone not accessible"**: Check audio permissions and device selection
- **"Camera not found"**: Verify camera connections and driver installation
- **"Audio device busy"**: Check if other processes are using audio devices
- **"Video encoding error"**: Check codec availability and permissions

## Checklist for Common Issues

### Before Running Complex Systems
- [ ] All required nodes are running
- [ ] Network configuration is correct
- [ ] Parameters are properly loaded
- [ ] Sensor data is flowing correctly
- [ ] Safety systems are active
- [ ] Emergency stops are functional

### When Troubleshooting Performance
- [ ] Monitor CPU and GPU usage
- [ ] Check memory allocation and leaks
- [ ] Verify network bandwidth and latency
- [ ] Analyze message rates and queue sizes
- [ ] Profile critical code sections
- [ ] Validate real-time constraints

### For Safety Validation
- [ ] Emergency stop functionality tested
- [ ] Safety boundaries verified
- [ ] Collision detection active
- [ ] Force/torque limits enforced
- [ ] Human safety protocols active
- [ ] Error recovery mechanisms tested