# Multi-Drone-PX4-RL
A toolkit to spawn multiple PX4 simulated drones, design formations with a simple drawing canvas, and control the swarm either manually (keyboard) or automatically with a trained RL leader.

## Features

### Formation Flight 
- Draw custom formations using an interactive OpenCV canvas.
- Convert drawings into 3D drone positions (drone_points.json).
- Automatically distribute drones into those positions.

### Reinforcement Learning Integration
- Deploy trained models into Gazebo/ROS 2 for realistic testing.

### ROS 2 + PX4 Integration

- Uses px4_ros_com and px4_msgs for communication.
- Supports multi-drone simulation with PX4 SITL and Gazebo Harmonic.

### Visualization Tools

- LiDAR data bridging from Gazebo → ROS 2 topics.
- RViz integration to inspect pointclouds.

### Control Modes

- Manual mode: WASD / IJKL keyboard control via sshkeyboard.
- Autonomous mode: RL-based agent navigation.

### Pre-requisites
- Make sure you have PX4-Autopilot, Gazebo and ROS2 installed. If not done, you can follow this guide here which also includes some examples for flying x500 PX4 Drones in this repository [px4_ros2_ws](https://github.com/DevashishHarsh/px4_ros2_ws/tree/main).

Scripts are OS-agnostic (Linux/Windows paths handled).
