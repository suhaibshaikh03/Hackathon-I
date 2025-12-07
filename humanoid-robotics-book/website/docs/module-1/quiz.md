---
sidebar_position: 5
---

# Module 1 - Quiz / Self-check Questions

## Multiple Choice Questions

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

6. Which client library is used for Python in ROS 2?
   a) rclcpp
   b) rclpy
   c) rclc
   d) rcl

7. What is the purpose of the robot_state_publisher node?
   a) To publish sensor data
   b) To publish TF transforms for a robot model
   c) To control robot actuators
   d) To manage robot parameters

8. Which command is used to visualize the TF tree in ROS 2?
   a) ros2 run tf2_ros view_frames
   b) ros2 run tf2_tools view_frames
   c) ros2 run rqt_tf_tree rqt_tf_tree
   d) ros2 run tf2_ros tf2_echo

9. What is the default DDS implementation used by ROS 2?
   a) Cyclone DDS
   b) RTI Connext DDS
   c) Fast DDS
   d) OpenSplice DDS

10. Which launch file format is used in ROS 2?
    a) XML files
    b) YAML files
    c) Python files
    d) JSON files

## True/False Questions

11. ROS 2 uses a master-slave architecture like ROS 1.
    a) True
    b) False

12. Parameters in ROS 2 can be changed at runtime.
    a) True
    b) False

13. Actions in ROS 2 are used for synchronous request/response communication.
    a) True
    b) False

14. The robot_state_publisher node publishes TF transforms for a robot model.
    a) True
    b) False

15. Lifecycle nodes provide better management for complex initialization and cleanup.
    a) True
    b) False

16. QoS settings must match exactly between publishers and subscribers for communication to work.
    a) True
    b) False

17. The default ROS domain ID is 0.
    a) True
    b) False

18. TF2 provides automatic interpolation between transform timestamps.
    a) True
    b) False

19. ROS 2 supports real-time systems with deterministic behavior.
    a) True
    b) False

20. Services in ROS 2 can return multiple responses to a single request.
    a) True
    b) False

## Short Answer Questions

21. Explain the difference between a topic and a service in ROS 2, including their use cases and communication patterns.

22. What are Quality of Service (QoS) policies and why are they important in ROS 2? Give examples of when you would use different QoS settings.

23. Describe the purpose of the TF2 library in ROS 2 and explain how it differs from the original TF library in ROS 1.

24. What is the difference between a regular node and a lifecycle node? When would you use each type?

25. Explain how parameters are handled differently in ROS 2 compared to ROS 1, including the benefits of the new approach.

26. What is the purpose of the launch system in ROS 2 and how does it differ from ROS 1 launch files?

27. Describe the publish/subscribe communication pattern in ROS 2 and explain its advantages and disadvantages.

28. What is the role of the DDS middleware in ROS 2 and how does it enable distributed computing?

29. Explain the concept of actions in ROS 2 and provide examples of when you would use actions instead of services.

30. What are the key differences between ROS 1 and ROS 2 in terms of architecture and design?

## Hands-on Questions

31. Write a simple ROS 2 publisher node in Python that publishes "Hello World" messages to a topic called "greetings" at 1 Hz. Include proper node lifecycle and error handling.

32. Create a launch file that starts both the publisher and a subscriber node from question 31. Include parameter loading and node configuration.

33. Explain how you would debug a situation where two ROS 2 nodes on different machines cannot communicate. Include specific commands and tools you would use.

34. Design a URDF model for a simple differential drive robot with two wheels and a caster. Include visual, collision, and inertial properties.

35. Implement a service server that accepts two integers and returns their sum, difference, product, and quotient. Include proper error handling for division by zero.

36. Create a node that uses parameters with validation callbacks. The node should accept parameters for robot velocity limits and validate them against predefined ranges.

37. Write a simple action server that simulates a navigation task with feedback. The action should accept a goal position and provide feedback on progress.

38. Implement a QoS example that demonstrates the difference between RELIABLE and BEST_EFFORT communication in a real scenario.

39. Create a launch file that demonstrates the use of launch arguments and conditions for conditional node startup.

40. Write a node that subscribes to a sensor topic, processes the data, and publishes the result to another topic with appropriate QoS settings.

## Scenario-Based Questions

41. You're developing a robot that needs to operate in a noisy factory environment with real-time requirements. Which QoS policies would you choose for sensor data vs. control commands? Justify your answer.

42. A team member is struggling with transform lookup failures in their navigation node. What debugging steps would you recommend and what tools would you use?

43. You need to integrate a legacy ROS 1 node with a new ROS 2 system. What approaches would you consider and what are the trade-offs of each?

44. Your robot is experiencing high CPU usage from TF lookups. How would you optimize the transform usage and what alternatives might you consider?

45. You're designing a multi-robot system where robots need to share information. How would you structure the ROS 2 network and what considerations would you make for scalability?

## Answers

1. b) Data Distribution Service
2. d) Functions
3. c) DDS
4. c) Reliability
5. b) ros2 topic list
6. b) rclpy
7. b) To publish TF transforms for a robot model
8. b) ros2 run tf2_tools view_frames
9. c) Fast DDS
10. c) Python files
11. b) False (ROS 2 uses a decentralized architecture)
12. a) True
13. b) False (Actions are for long-running tasks with feedback)
14. a) True
15. a) True
16. b) False (They should be compatible but don't need to match exactly)
17. a) True
18. a) True
19. a) True
20. b) False (Services return one response per request)