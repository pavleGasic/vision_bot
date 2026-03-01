# VisionBot

**VisionBot** is a mobile robot platform developed as part of a Master's thesis project.

The goal of this project is to design and implement an intelligent rover-like robot capable of AI-based vision processing (e.g., face recognition).

## Requirements

- Ubuntu 24.04 (recommended)
- ROS 2 Jazzy
- Gazebo Harmonic
- colcon
- RViz2

Make sure ROS 2 is sourced:

```bash
source /opt/ros/jazzy/setup.bash
```

## Build Instructions

Navigate to your ROS 2 workspace and build:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

After successful build, source the workspace:

```bash
source install/setup.bash
```

## Running the Simulation

After building and sourcing your workspace, you can launch the full Gazebo simulation of VisionBot using:

```bash
ros2 launch vision_bot sim.launch.py
```

### Visualize in RViz2

To open RViz2 separately:

```bash
rviz2
```

## Author

Pavle Gasic
Master Thesis Project – Vision-Based Mobile Robot Platform
