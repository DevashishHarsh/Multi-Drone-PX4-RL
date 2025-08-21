# Multi-Drone-PX4-RL
A toolkit to spawn multiple PX4 simulated drones, design formations with a simple drawing canvas, and control the swarm either manually (keyboard) or automatically with a trained RL leader.

## Features

### Formation Flight 
- Draw custom formations using an interactive OpenCV canvas.
- Convert drawings into 3D drone positions (drone_points.json).
- Automatically distribute drones into those positions.
![Formation Flight](assets/formation.gif)

### Reinforcement Learning Integration
- Deploy trained models into Gazebo/ROS 2 for realistic testing.
![Reinforce](assets/reinforcement.gif)

### ROS 2 + PX4 Integration

- Uses px4_ros_com and px4_msgs for communication.
- Supports multi-drone simulation with PX4 SITL and Gazebo Harmonic.
![Integrate](assets/integration.gif)

### Visualization Tools

- LiDAR data bridging from Gazebo → ROS 2 topics.
- RViz integration to inspect pointclouds.
![Visualize](assets/visual.gif)

### Control Modes

- Manual mode: WASD / IJKL keyboard control via sshkeyboard.
- Autonomous mode: RL-based agent navigation.
![Modes](assets/modes.gif)

### Pre-requisites
- Make sure you have PX4-Autopilot, Gazebo and ROS2 installed. If not done, you can follow this guide here which also includes some examples for flying x500 PX4 Drones in this repository [px4_ros2_ws](https://github.com/DevashishHarsh/px4_ros2_ws/tree/main).
> I have also included a [setup.sh](setup.sh) file which installs everything that is required but I would suggest you install everything manually.
```
chmod +x setup.sh
./setup.sh
source ~/.bashrc
```
- 


