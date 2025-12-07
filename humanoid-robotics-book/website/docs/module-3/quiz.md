---
sidebar_position: 5
---

# Quiz / Self-check Questions - Module 3

## Multiple Choice Questions

1. What does NITROS stand for in the Isaac ROS ecosystem?
   a) NVIDIA Isaac Transport for ROS
   b) Network Interface for Transport and ROS Systems
   c) NVIDIA Integrated Transport and Optimization System
   d) Native Isaac Transport and Optimization for ROS

2. Which Isaac component is primarily responsible for reinforcement learning?
   a) Isaac Sim
   b) Isaac Lab
   c) Isaac ROS
   d) Isaac GR00T

3. What is the main purpose of domain randomization?
   a) To reduce computational requirements
   b) To improve sim-to-real transfer by randomizing simulation parameters
   c) To simplify robot models
   d) To increase simulation speed

4. Which physics engine does Isaac Sim use?
   a) ODE
   b) Bullet
   c) PhysX 5.4+
   d) Simbody

5. What is Isaac GR00T primarily designed for?
   a) Simulation
   b) Perception
   c) Vision-Language-Action reasoning
   d) Navigation

## True/False Questions

6. Isaac Sim is built on the NVIDIA Omniverse platform.
   a) True
   b) False

7. Isaac Lab can train policies with over 1000 parallel simulation instances.
   a) True
   b) False

8. Isaac ROS only works with NVIDIA Jetson platforms.
   a) True
   b) False

9. RTX ray tracing provides photorealistic rendering in Isaac Sim.
   a) True
   b) False

10. Automatic Domain Randomization (ADR) requires manual tuning of randomization parameters.
    a) True
    b) False

## Short Answer Questions

11. Explain the role of Isaac Sim in the NVIDIA Isaac ecosystem.

12. What are the benefits of synthetic data generation for robotics?

13. Describe the concept of vectorized environments in Isaac Lab.

14. What is the purpose of the NITROS framework in Isaac ROS?

15. How does Isaac GR00T enable vision-language-action reasoning?

## Hands-on Questions

16. Describe the steps to set up domain randomization in Isaac Sim for a manipulation task.

17. Explain how to configure a reinforcement learning environment in Isaac Lab with custom rewards.

18. What are the key considerations when deploying Isaac ROS perception on a Jetson platform?

19. Design a simple VLA (Vision-Language-Action) pipeline using Isaac GR00T components.

20. Explain the process of validating sim-to-real transfer for a trained locomotion policy.

## Answers

1. a) NVIDIA Isaac Transport for ROS
2. b) Isaac Lab
3. b) To improve sim-to-real transfer by randomizing simulation parameters
4. c) PhysX 5.4+
5. c) Vision-Language-Action reasoning
6. a) True
7. a) True
8. b) False (It can work on other NVIDIA GPU platforms too)
9. a) True
10. b) False (ADR adapts parameters automatically during training)

11. Isaac Sim provides high-fidelity simulation with photorealistic rendering and accurate physics for training and testing robotic systems.

12. Synthetic data provides unlimited training data with perfect ground truth annotations at no cost, enabling robust model training.

13. Vectorized environments run thousands of parallel simulation instances simultaneously, dramatically accelerating robot learning.

14. NITROS optimizes data transport between ROS nodes, reducing memory copies and latency for GPU-accelerated processing.

15. Isaac GR00T integrates vision and language processing to understand commands and generate executable action plans.