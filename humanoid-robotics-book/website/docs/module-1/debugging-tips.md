---
sidebar_position: 4
---

# Module 1 - Common Pitfalls & Debugging Tips

## Node Communication Issues

### 1. Topic Connection Problems
**Problem**: Publishers and subscribers cannot communicate.
**Solution**:
- Verify topic names match exactly between publisher and subscriber
- Check that message types are compatible
- Ensure both nodes are on the same ROS domain ID
- Use `ros2 topic list` to verify topic existence
- Check QoS profile compatibility between publisher and subscriber

### 2. Service Connection Issues
**Problem**: Service clients cannot connect to service servers.
**Solution**:
- Verify service names match exactly
- Ensure service server is running before client calls
- Check message types are compatible
- Use `ros2 service list` to verify service existence
- Verify network configuration for multi-machine setups

### 3. Node Discovery Issues
**Problem**: Nodes cannot discover each other in the ROS graph.
**Solution**:
- Check network configuration and firewall settings
- Verify ROS_DOMAIN_ID environment variable
- Use `ros2 node list` to verify node existence
- Check for multiple DDS implementations conflicts
- Ensure proper multicast traffic is allowed through firewalls

### 4. Action Connection Issues
**Problem**: Action clients cannot connect to action servers.
**Solution**:
- Verify action names match exactly
- Ensure action server is running before client connects
- Check action message types are compatible
- Use `ros2 action list` to verify action existence
- Monitor action feedback and goal status properly

## Performance Issues

### 1. Memory Leaks
**Problem**: Nodes consume increasing amounts of memory over time.
**Solution**:
- Properly destroy nodes and associated resources
- Use weak references where appropriate to prevent circular references
- Monitor memory usage with system tools
- Implement proper cleanup in node destruction
- Use memory profilers to identify leak sources

### 2. High CPU Usage
**Problem**: Nodes consume excessive CPU resources.
**Solution**:
- Adjust timer frequencies appropriately
- Use appropriate QoS settings to reduce unnecessary processing
- Implement efficient message processing and filtering
- Profile code to identify bottlenecks
- Consider using threading for I/O operations

### 3. Network Bandwidth Issues
**Problem**: High network usage affecting real-time performance.
**Solution**:
- Use appropriate QoS settings (reliability, history, depth)
- Implement message throttling for high-frequency topics
- Use compressed message formats when possible
- Monitor network usage with tools like `iftop` or `nethogs`
- Consider using intra-process communication for co-located nodes

## Resource Management

### 1. Parameter Validation
**Problem**: Invalid parameter values causing runtime errors.
**Solution**:
- Implement parameter callbacks for validation
- Use parameter descriptors to define ranges, types, and constraints
- Validate parameters during node initialization
- Provide sensible default values
- Use parameter change callbacks to react to dynamic changes

### 2. Lifecycle Management
**Problem**: Nodes don't initialize or shut down properly.
**Solution**:
- Implement proper node lifecycle methods
- Use lifecycle nodes for complex initialization and cleanup
- Handle shutdown signals gracefully
- Clean up resources in destructors
- Implement error recovery and state management

### 3. Timer and Callback Management
**Problem**: Callbacks not executing at expected rates or causing blocking.
**Solution**:
- Use appropriate timer periods for different tasks
- Avoid blocking operations in callbacks
- Use multi-threaded executors when needed
- Monitor callback execution times
- Implement proper callback grouping for related operations

## Quality of Service (QoS) Issues

### 1. Message Loss
**Problem**: Messages are being lost despite expectations.
**Solution**:
- Check QoS profile compatibility between publisher and subscriber
- Adjust reliability settings (RELIABLE vs BEST_EFFORT)
- Modify history settings (KEEP_ALL vs KEEP_LAST)
- Increase queue depth for high-frequency topics
- Consider durability settings for latched messages

### 2. Performance Degradation
**Problem**: System performance decreases with certain QoS settings.
**Solution**:
- Match QoS settings to application requirements
- Use BEST_EFFORT for real-time sensor data
- Use RELIABLE for critical control messages
- Adjust history depth based on message importance
- Consider deadline and lifespan QoS policies for time-sensitive data

## TF2 (Transform Library) Issues

### 1. Transform Lookup Failures
**Problem**: Unable to find transforms between coordinate frames.
**Solution**:
- Use `ros2 run tf2_tools view_frames` to visualize the TF tree
- Verify all required transforms are being published
- Check transform publishing frequency (should be 10-100 Hz)
- Use appropriate timeout values in transform lookups
- Ensure proper frame naming conventions

### 2. Transform Staleness
**Problem**: Received transforms are too old for current processing.
**Solution**:
- Increase timeout values appropriately
- Verify transform publishing frequency
- Check for dropped transforms due to network issues
- Use latest available transform when appropriate
- Monitor transform publication rates

## Debugging Strategies

### 1. Using ROS 2 Command Line Tools
```bash
# Monitor topics and their rates
ros2 topic list
ros2 topic info /topic_name
ros2 topic hz /topic_name
ros2 topic echo /topic_name

# Monitor nodes and their status
ros2 node list
ros2 node info /node_name
ros2 lifecycle list node_name  # for lifecycle nodes

# Parameter inspection and modification
ros2 param list
ros2 param get /node_name param_name
ros2 param set /node_name param_name value

# Service and action inspection
ros2 service list
ros2 action list
ros2 action send_goal /action_name action_type "goal_data"

# System health checks
ros2 doctor
ros2 run tf2_tools view_frames
```

### 2. Logging Best Practices
```python
# Use appropriate logging levels
self.get_logger().debug("Detailed debugging info for development")
self.get_logger().info("General information about normal operation")
self.get_logger().warn("Warning message about potential issues")
self.get_logger().error("Error occurred but node can continue")
self.get_logger().fatal("Critical error requiring node shutdown")

# Include contextual information in logs
self.get_logger().info(f'Processing message {msg.id} from topic {msg.source}')
```

### 3. Using rqt Tools for Visualization
- `rqt_graph`: Visualize the node graph and connections
- `rqt_plot`: Plot numeric values over time
- `rqt_console`: Monitor log messages from all nodes
- `rqt_bag`: Record and replay data
- `rqt_tf_tree`: Visualize the transform tree
- `rqt_reconfigure`: Dynamically adjust node parameters

### 4. Performance Profiling
- Use `ros2 run topic_tools throttle` to control message rates for testing
- Monitor CPU and memory usage with `htop` or `top`
- Use `ros2 topic delay` and `ros2 topic hz` to measure latency
- Profile Python code with `cProfile` for performance bottlenecks
- Use `ros2 run tf2_ros tf2_monitor` to check transform quality

## Launch and System Management

### 1. Launch File Issues
**Problem**: Nodes not starting correctly or in wrong order.
**Solution**:
- Use appropriate conditions and events in launch files
- Implement proper node startup ordering with conditions
- Use launch arguments for configuration flexibility
- Add launch configuration for debugging
- Implement error handling and node restart policies

### 2. Parameter Loading Issues
**Problem**: Parameters not loading from YAML files correctly.
**Solution**:
- Verify YAML file syntax and structure
- Check parameter file paths and permissions
- Use correct namespace in parameter files
- Validate parameter names match declarations
- Test parameter loading with `ros2 param list`

## Common Error Messages and Solutions

- **"Failed to contact master"**: ROS 1 terminology; in ROS 2, check DDS configuration and network settings
- **"Topic not found"**: Verify topic names, node connections, and QoS settings
- **"Service not available"**: Check if service server is running and accessible
- **"Parameter not declared"**: Declare parameters before getting/setting them
- **"Could not find the graph resource"**: Check ROS domain ID and network configuration
- **"Timeout exceeded"**: Increase timeout values or verify transform publishing rates
- **"Subscription already registered"**: Check for duplicate subscription attempts

## Best Practices for Debugging

1. **Start Simple**: Begin with basic publisher/subscriber examples before adding complexity
2. **Isolate Issues**: Test components independently before integration
3. **Use Logging**: Add comprehensive logging to track execution flow and state
4. **Monitor Resources**: Check CPU, memory, network usage, and transform rates
5. **Version Control**: Use git to track changes and revert when needed
6. **Documentation**: Keep notes on configurations that work and issues encountered
7. **Systematic Testing**: Test incrementally with defined success criteria
8. **Community Resources**: Use ROS Discourse, answers.ros.org, and GitHub issues for help
9. **Reproducible Issues**: Create minimal examples that reproduce problems
10. **Safety First**: Always test with safety mechanisms in place

## Testing and Validation

### 1. Unit Testing
- Write tests for individual node functionality
- Mock ROS interfaces for isolated testing
- Test parameter validation and callback functions
- Verify message handling and state transitions

### 2. Integration Testing
- Test node communication patterns
- Validate system behavior with multiple nodes
- Test parameter loading and dynamic reconfiguration
- Verify proper shutdown and cleanup procedures

### 3. Performance Testing
- Measure message latency and throughput
- Monitor CPU and memory usage under load
- Test system behavior under high-frequency messaging
- Validate real-time performance requirements