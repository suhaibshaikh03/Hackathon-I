---
sidebar_position: 5
---

# Quiz / Self-check Questions - Module 2

## Multiple Choice Questions

1. What does SDF stand for in Gazebo?
   a) Simulation Development Framework
   b) Simulation Description Format
   c) System Definition File
   d) Sensor Data Format

2. Which physics engines are supported by Gazebo? (Select all that apply)
   a) ODE
   b) Bullet
   c) PhysX
   d) Simbody

3. What is the primary purpose of domain randomization?
   a) To make simulations run faster
   b) To improve sim-to-real transfer by randomizing parameters
   c) To reduce computational requirements
   d) To simplify robot models

4. Which Unity package enables communication with ROS 2?
   a) Unity Robotics Toolkit
   b) ROS-TCP-Connector
   c) Unity ML-Agents
   d) Unity Perception

5. What is the default physics engine used by Gazebo?
   a) Bullet
   b) PhysX
   c) ODE
   d) Simbody

## True/False Questions

6. SDF files are based on JSON format.
   a) True
   b) False

7. The robot_state_publisher node is responsible for publishing TF transforms.
   a) True
   b) False

8. Unity uses a left-handed coordinate system while ROS uses a right-handed system.
   a) True
   b) False

9. Gazebo can simulate various sensor types including cameras, LiDAR, and IMUs.
   a) True
   b) False

10. Sim-to-real transfer requires identical simulation and real-world parameters.
    a) True
    b) False

## Short Answer Questions

11. Explain the concept of a digital twin in robotics and its benefits.

12. What are the key differences between URDF and SDF formats?

13. Describe the process of converting a URDF model to SDF for Gazebo simulation.

14. What is the role of sensor calibration in digital twin implementations?

15. Explain how domain randomization helps with sim-to-real transfer.

## Hands-on Questions

16. Write an SDF snippet to define a simple box obstacle with appropriate collision and visual properties.

17. Create a URDF joint definition for a revolute joint with proper limits and safety controllers.

18. Explain how you would debug a situation where a robot model falls through the ground in Gazebo.

19. Design a simple sensor configuration for a mobile robot with a camera and LiDAR.

20. Describe the steps to integrate a Unity visualization with a ROS 2 system.

## Answers

1. b) Simulation Description Format
2. a) ODE, b) Bullet, d) Simbody (all except PhysX)
3. b) To improve sim-to-real transfer by randomizing parameters
4. b) ROS-TCP-Connector
5. c) ODE
6. b) False (SDF is XML-based)
7. a) True
8. a) True
9. a) True
10. b) False (domain randomization actually helps with different parameters)

11. A digital twin is a virtual replica of a physical robot/system. Benefits include risk reduction, cost efficiency, optimization, and training capabilities.

12. URDF is primarily for robot description in ROS, while SDF describes full simulation environments including worlds, objects, and sensors.

13. Use the `gz sdf -p urdf_file.urdf > sdf_file.sdf` command to convert, then validate with `gz sdf -k`.

14. Sensor calibration ensures simulated sensors match real-world characteristics for accurate sim-to-real transfer.

15. Domain randomization trains policies across varied conditions, making them more robust to sim-to-real differences.